#include "controller.h"

#include <string.h>  // memcpy
#include <stm32f4xx.h>

#include "display.h"
#include "afg.h"
#include "leds_step.h"
#include "expander.h"
#include "adc_pots_selector.h"
#include "MAX5135.h"
#include "delays.h"
#include "eprom.h"

// Step selected for editing (0-31)
volatile uint8_t edit_mode_step_num = 0;
volatile uint8_t edit_mode_section = 0;

// Jobs
volatile ControllerJobFlags controller_job_flags;

inline static void adc_pause(void) {
  NVIC_DisableIRQ(ADC_IRQn);
};

inline static void adc_resume(void) {
  NVIC_EnableIRQ(ADC_IRQn);
};

// Start Timer 6 for clear switch measurement
void InitClear_Timer() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  TIM6->PSC = 21000;
  TIM6->ARR = 200;
  TIM6->CNT = 0;
  TIM6->DIER = TIM_DIER_UIE;
  TIM6->CR1 |= TIM_CR1_CEN;

  NVIC_EnableIRQ(TIM6_DAC_IRQn);
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
};

// Main Loop. Does not return.

void ControllerMainLoop() {
  uButtons switches;

  controller_job_flags.adc_pot_sel = 0;
  controller_job_flags.adc_mux_shift_out = 0;
  controller_job_flags.inhibit_adc = 0;
  controller_job_flags.afg1_tick = 0;
  controller_job_flags.afg2_tick = 2;

  // Stable switches state, post debouncing
  volatile uint64_t stable_switches_state;

  // Previous stable switches state
  volatile uint64_t previous_switches_state;

  // Raw new switches reading
  volatile uint64_t new_switches_state;

  // Time at which the switches were last scanned
  uint32_t switch_last_read_time = 0;

  // Time at which the display was updated
  uint32_t leds_update_time = 0;

  // Down counter to debounce switches
  uint16_t switch_debounce_counter = KEY_DEBOUNCE_COUNT;

  // Initial read
  stable_switches_state = previous_switches_state = HC165_ReadSwitches();
  switches.value = stable_switches_state;

  AfgAllInitialize();

  // Main loop. Does not return.
  while (1) {

    // Compute continuous stage address
    ComputeContinuousStep1();
    ComputeContinuousStep2();

    // Expensive to recalculate every tick, so do it here
    afg1_step_width = GetStepWidth(afg1_section, afg1_step_num, GetTimeMultiplier1());;
    afg2_step_width = GetStepWidth(afg2_section, afg2_step_num, GetTimeMultiplier2());;

    // Scan switches every 5ms
    if (get_millis() - switch_last_read_time > 5) {
      new_switches_state = HC165_ReadSwitches(); // SLOOOOOOOOOOOW right here ..
      switch_last_read_time = get_millis();
      if (new_switches_state == stable_switches_state) {
        // Nothing is happening
        switch_debounce_counter = KEY_DEBOUNCE_COUNT;
      } else {
        // Switch state is changing
        switch_debounce_counter -=1;
      }

      // Navigation switches have their own debouncing logic.
      // Apply it every 5ms tick.
      ControllerProcessNavigationSwitches(&switches);
    }
    if (!switch_debounce_counter) {
      // Now switches are stable
      switch_debounce_counter = KEY_DEBOUNCE_COUNT;
      previous_switches_state = stable_switches_state;
      stable_switches_state = new_switches_state;

      switches.value = stable_switches_state;

      // Apply switches that should only be applied after debounce
      ControllerProcessStageAddressSwitches(&switches);
    }

    // Step programming can be applied continuously to be more responsive,
    // eg when the clock is running fast.
    ControllerApplyProgrammingSwitches(&switches);

    // Update panel state
    if (display_update_flags.b.MainDisplay) {
      UpdateModeSectionLeds();
      display_update_flags.b.MainDisplay = 0;
    };
    if (display_update_flags.b.StepsDisplay) {
      UpdateStepSectionLeds();
      display_update_flags.b.StepsDisplay = 0;
    };

    // Flush LEDs every 20ms.
    // Shifting out to the leds is kind of slow, so rate limit the update to 50 Hz.
    if (get_millis() - leds_update_time > 20) {
      FlushLedUpdates();
      leds_update_time = get_millis();
    }

    // Process Stop and Start signals.
    // Stop and start have their own debouncing/edge detection logic reading GPIO directly.
    // This bypasses the controller and goes straight to afg.
    ProcessStopStart(GPIO_ReadInputData(GPIOB));

    // Shift adc mux if time
    if (controller_job_flags.adc_mux_shift_out) {
      // Disable conversion during shift
      controller_job_flags.inhibit_adc = 1;
      if (Is_Expander_Present()) {
        // Increment the slider, including expander sliders
        controller_job_flags.adc_pot_sel = AdcMuxAdvanceExpanded(controller_job_flags.adc_pot_sel);
      } else {
        // Increments the slider
        controller_job_flags.adc_pot_sel = AdcMuxAdvance(controller_job_flags.adc_pot_sel);
      }
      controller_job_flags.adc_mux_shift_out = 0;
      // Wait for settle
      delay_us(10);
      // Reenable conversion again
      controller_job_flags.inhibit_adc = 0;
    }

    if (controller_job_flags.afg1_tick) {
      // Send data to external dac
      MAX5135_DAC_send(MAX5135_DAC_CH_0, controller_job_flags.afg1_outputs.time);
      MAX5135_DAC_send(MAX5135_DAC_CH_1, controller_job_flags.afg1_outputs.ref);
      controller_job_flags.afg1_tick = 0;
    }
    if (controller_job_flags.afg2_tick) {
      // Send data to external dac
      MAX5135_DAC_send(MAX5135_DAC_CH_2, controller_job_flags.afg2_outputs.time);
      MAX5135_DAC_send(MAX5135_DAC_CH_3, controller_job_flags.afg2_outputs.ref);
      controller_job_flags.afg2_tick = 0;
    }

    // Go into modal loops if called for

    if (controller_job_flags.modal_loop == CONTROLLER_MODAL_LOAD) {
      // Sub loop for load program
      ControllerLoadProgramLoop(); // only exits when done
      controller_job_flags.modal_loop = CONTROLLER_MODAL_NONE;
    } else if (controller_job_flags.modal_loop == CONTROLLER_MODAL_SAVE) {
      ControllerSaveProgramLoop(); // only exits when done
      controller_job_flags.modal_loop = CONTROLLER_MODAL_NONE;
    }

  }; // end main loop
}

void ControllerApplyProgrammingSwitches(uButtons * key) {
  unsigned char step_num = 0, section = 0;

  // Determine step num for different display modes
  if (display_mode == DISPLAY_MODE_VIEW_1) {
    step_num = afg1_step_num;
    section = afg1_section;
  } else if (display_mode == DISPLAY_MODE_VIEW_2) {
    step_num = afg2_step_num;
    section = afg2_section;
  } else if (display_mode == DISPLAY_MODE_EDIT_1 || display_mode == DISPLAY_MODE_EDIT_2) {
    step_num = edit_mode_step_num;
    section = edit_mode_section;
  };

  // Apply programming from switches to active step
  ApplyProgrammingSwitches(section, step_num, key);
}

void ControllerProcessStageAddressSwitches(uButtons * key) {

  // Only do one of reset, strobe or advance
  if (!key->b.StageAddress1Reset) {
    DoReset1();
    update_display();
  } else if (!key->b.StageAddress1PulseSelect) {
    DoStrobe1();
    update_display();
  } else if (!key->b.StageAddress1Advance) {
    DoAdvance1();
    update_display();
  }

  if (!key->b.StageAddress2Reset) {
    DoReset2();
    update_display();
  } else if ( (!key->b.StageAddress2PulseSelect)) {
    DoStrobe2();
    update_display();
  } else if (!key->b.StageAddress2Advance) {
    DoAdvance2();
    update_display();
  };

  if (!key->b.StageAddress1ContiniousSelect) {
    EnableContinuousStageAddress1();
  } else {
    DisableContinuousStageAddress1();
  }

  if (!key->b.StageAddress2ContiniousSelect) {
    EnableContinuousStageAddress2();
  } else {
    DisableContinuousStageAddress2();
  }

  if (!key->b.ClearUp || !key->b.ClearDown) {
    // Wait for long press on clear
    InitClear_Timer();
  }
}

void ControllerProcessNavigationSwitches(uButtons* key) {
  // Down counters which track the number of ticks of this method
  // before left and right switches should be processed again.
  // This enables some debouncing but also long press and hold to scroll.
  static uint16_t left_counter = SHORT_COUNTER_TICKS;
  static uint16_t right_counter = SHORT_COUNTER_TICKS;

  // Display/edit mode changes
  if (!key->b.StageAddress1Display && display_mode != DISPLAY_MODE_VIEW_1) {
    display_mode = DISPLAY_MODE_VIEW_1;
  }
  if (!key->b.StageAddress2Display && display_mode != DISPLAY_MODE_VIEW_2) {
    display_mode = DISPLAY_MODE_VIEW_2;
  }
  if (!key->b.StepLeft || !key->b.StepRight) {
    if (display_mode == DISPLAY_MODE_VIEW_1) {
      display_mode = DISPLAY_MODE_EDIT_1;
      edit_mode_section = afg1_section;
      edit_mode_step_num = 0;
      right_counter = SCROLL_WAIT_COUNTER;
      left_counter = SCROLL_WAIT_COUNTER;
      update_display();
    }
    else if (display_mode == DISPLAY_MODE_VIEW_2) {
      display_mode = DISPLAY_MODE_EDIT_2;
      edit_mode_section = afg2_section;
      edit_mode_step_num = 0;
      right_counter = SCROLL_WAIT_COUNTER;
      left_counter = SCROLL_WAIT_COUNTER;
      update_display();
    }
  }

  // Section shift for each afg in 16 slider mode
  if (!Is_Expander_Present()) {
    if (!key->b.StepLeft && !key->b.StageAddress1Display) {
      afg1_section = 0;
      display_mode = DISPLAY_MODE_VIEW_1;
      update_display();
    } else if (!key->b.StepRight && !key->b.StageAddress1Display) {
      afg1_section = 1;
      display_mode = DISPLAY_MODE_VIEW_1;
      update_display();
    } else if (!key->b.StepLeft && !key->b.StageAddress2Display) {
      afg2_section = 0;
      display_mode = DISPLAY_MODE_VIEW_2;
      update_display();
    } else if (!key->b.StepRight && !key->b.StageAddress2Display) {
      afg2_section = 1;
      display_mode = DISPLAY_MODE_VIEW_2;
      update_display();
    }
  }

  // Decrement counters
  if (!key->b.StepLeft && left_counter) {
    // Long hold
    left_counter -= 1;
  } else if (key->b.StepLeft) {
    // Release
    left_counter = SHORT_COUNTER_TICKS;
  }

  if (!key->b.StepRight && right_counter) {
    right_counter -= 1;
  } else if (key->b.StepRight) {
    right_counter = SHORT_COUNTER_TICKS;
  }

  // Left counter expired, do step left
  if (!left_counter) {
    update_display();
    if (edit_mode_step_num == 0) {
      // Wrap around to max step
      edit_mode_step_num = get_max_step();
    } else {
      // Decrement edit step
      edit_mode_step_num -= 1;
    }
    // Long count when held down
    left_counter = LONG_COUNTER_TICKS;
  }

  // Right counter expired, do step right
  if (!right_counter) {
    update_display();
    if (edit_mode_step_num == get_max_step()) {
      // Wrap around to 0
      edit_mode_step_num = 0;
    } else {
      // Increment edit step
      edit_mode_step_num += 1;
    }
    // Long count when held down
    right_counter = LONG_COUNTER_TICKS;
  }
}

void ControllerCheckClear() {
  uButtons myButtons;
  static uint8_t clear_counter1 = 0, clear_counter2 = 0;

  TIM6->SR = (uint16_t) ~TIM_IT_Update;

  myButtons.value = HC165_ReadSwitches();

  if (clear_counter1 < 30 && clear_counter2 < 30) {
    if (!myButtons.b.ClearUp || !myButtons.b.ClearDown) {
      if (!myButtons.b.ClearUp) {
        clear_counter1++;
      } else {
        clear_counter1 = 0;
      }
      if (!myButtons.b.ClearDown) {
        clear_counter2++;
      } else {
        clear_counter2 = 0;
      }
    } else {
      if (clear_counter1) {
        // Go into LOAD modal in main loop
        controller_job_flags.modal_loop = CONTROLLER_MODAL_LOAD;
      } else if (clear_counter2) {
        // Go into SAVE modal in main loop
        controller_job_flags.modal_loop = CONTROLLER_MODAL_SAVE;
      }
      // Reset counters
      clear_counter1 = 0;
      clear_counter2 = 0;
      TIM_SetCounter(TIM6, 0x00);
      TIM6->CR1 &= ~TIM_CR1_CEN;
    }
  }
  else if (clear_counter1 >= 30 || clear_counter2 >= 30) {
    // CLEAR
    // Signal by flashing step leds
    RunClearAnimation();

    TIM_SetCounter(TIM6, 0x00);
    TIM6->CR1 &= ~TIM_CR1_CEN;

    if (clear_counter1 == 30 || clear_counter2 == 30) {
      InitProgram();
      HardStop1();
      HardStop2();
    };

    clear_counter1 = 0;
    clear_counter2 = 0;
  };
}

void ControllerCalibrationLoop() {
    uButtons switches;
    switches.value = HC165_ReadSwitches();

    DISPLAY_LED_I_OFF;
    DISPLAY_LED_II_OFF;

    // Run animation, and continue doing adc reads until stage 2 advance is pressed
    while (switches.b.StageAddress2Advance) {
      RunCalibrationAnimation();
      switches.value = HC165_ReadSwitches();

      if (!switches.b.Pulse1On) {
        // Choose normal behavior
        swapped_pulses = 0;
      } else if (!switches.b.Pulse2On) {
        // Choose swapped pulse leds behavior
        swapped_pulses = 1;
      }

      // Shift adc mux if time
      if (controller_job_flags.adc_mux_shift_out) {
        // Disable conversion during shift
        controller_job_flags.inhibit_adc = 1;
        if (Is_Expander_Present()) {
          // Increment the slider, including expander sliders
          controller_job_flags.adc_pot_sel = AdcMuxAdvanceExpanded(controller_job_flags.adc_pot_sel);
        } else {
          // Increments the slider
          controller_job_flags.adc_pot_sel = AdcMuxAdvance(controller_job_flags.adc_pot_sel);
        }
        controller_job_flags.adc_mux_shift_out = 0;
        // Wait for settle
        delay_ms(10);
        // Reenable conversion again
        controller_job_flags.inhibit_adc = 0;
      }
    } // End read loop

    // Read calibration constants
    adc_pause();
    for (uint8_t c = 0; c < 8 ; c++) {
      cal_constants[c] = add_data[c];
      if (cal_constants[c] < 500) {
        // Extremely low reading. Probably wrong. Discard it.
        cal_constants[c] = 4095;
      }
    };
    __disable_irq();

    // Erase EPROM
    CAT25512_erase();

    // Store calibration constants to eprom
    CAT25512_write_block(
        eprom_memory.analog_cal_data.start,
        (unsigned char *) cal_constants,
        eprom_memory.analog_cal_data.size);

    // Store swapped pulses
    CAT25512_write_block(
        eprom_memory.pulse_leds_switched.start,
        (unsigned char *) &swapped_pulses,
        eprom_memory.pulse_leds_switched.size);

    __enable_irq();
    adc_resume();
}

void ControllerLoadCalibration() {
  // Read analog calibration data
  CAT25512_read_block(
      eprom_memory.analog_cal_data.start,
      (unsigned char *) cal_constants,
      eprom_memory.analog_cal_data.size);

  // Read swapped pulses choice
  CAT25512_read_block(
      eprom_memory.pulse_leds_switched.start,
      (unsigned char *) &swapped_pulses,
      eprom_memory.pulse_leds_switched.size);

  // Precompute scalers from calibration data
  PrecomputeCalibration();
}

// Load program loop
void ControllerLoadProgramLoop() {
  SavedProgram saved_program = {};
  uint8_t program_num = 0;
  uButtons previous_switches, switches;
  previous_switches.value = HC165_ReadSwitches();

  HardStop1();
  HardStop2();
  StepLedsLightSingleStep(0);

  while (1) {
    switches.value = HC165_ReadSwitches();
    if (switches.value != previous_switches.value) {
      if (!switches.b.StepRight) {
        if (program_num >= 15) {
          program_num = 0;
        } else {
          program_num += 1;
        }
        StepLedsLightSingleStep(program_num);
      } else if (!switches.b.StepLeft) {
        if (program_num == 0) {
          program_num = 15;
        } else {
          program_num -= 1;
        }
        StepLedsLightSingleStep(program_num);
      } else if (!switches.b.ClearUp) {
        // Load program. Takes a little while.
        __disable_irq();

        // Read from EPROM into temporary saved_program
        CAT25512_read_block(
            eprom_memory.programs[program_num].start,
            (unsigned char *) &saved_program,
            eprom_memory.programs[program_num].size);

        // Copy from saved_program to steps and sliders
        memcpy((void *) steps, (void *) saved_program.steps, sizeof(steps));
        memcpy((void *) sliders, (void *) saved_program.sliders, sizeof(sliders));
        pin_all_sliders();

        // Cute little animation
        RunLoadProgramAnimation();
        display_mode = DISPLAY_MODE_VIEW_1;
        __enable_irq();
        return;
      } else if (!switches.b.Pulse1On
          || !switches.b.Pulse2On
          || !switches.b.Pulse1Off
          || !switches.b.Pulse2Off) {
        // Abort load
        display_mode = DISPLAY_MODE_VIEW_1;
        return;
      }
      previous_switches.value = switches.value;
    } else {
      delay_ms(15);
      RunWaitingLoadSaveAnimation();
    }
  }
}

// Save program loop
void ControllerSaveProgramLoop() {
  uint8_t program_num = 0;
  SavedProgram saved_program = {};
  uButtons previous_switches, switches;
  previous_switches.value = HC165_ReadSwitches();

  HardStop1();
  HardStop2();
  StepLedsLightSingleStep(0);

  while (1) {
    switches.value = HC165_ReadSwitches();
    if (switches.value != previous_switches.value) {
      if (!switches.b.StepRight) {
        if (program_num >= 15) {
          program_num = 0;
        } else {
          program_num += 1;
        }
        StepLedsLightSingleStep(program_num);
      } else if (!switches.b.StepLeft) {
        if (program_num == 0) {
          program_num = 15;
        } else {
          program_num -= 1;
        }
        StepLedsLightSingleStep(program_num);
      } else if (!switches.b.ClearDown) {
        // Save program. Takes a little while.
        __disable_irq();

        // Copy from steps and sliders to temp saved_program
        memcpy((void *) saved_program.steps, (void *) steps, sizeof(steps));
        memcpy((void *) saved_program.sliders, (void *) sliders, sizeof(sliders));

        // Write the temp saved program to the EPROM
        CAT25512_write_block(
            eprom_memory.programs[program_num].start,
            (unsigned char *) &saved_program,
            eprom_memory.programs[program_num].size);

        // Run cute animation
        RunSaveProgramAnimation();
        display_mode = DISPLAY_MODE_VIEW_1;
        __enable_irq();
        return;
      } else if (!switches.b.Pulse1On
          || !switches.b.Pulse2On
          || !switches.b.Pulse1Off
          || !switches.b.Pulse2Off) {
        // Abort save
        display_mode = DISPLAY_MODE_VIEW_1;
        return;
      }
      previous_switches.value = switches.value;
    } else {
      delay_ms(15);
      RunWaitingLoadSaveAnimation();
    }
  }
}
