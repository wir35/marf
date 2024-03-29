#include "controller.h"

#include <stddef.h>
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
#include "analog_data.h"

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


// Functionality that needs to run both the main and modal loops
void ControllerCommonAllLoops() {
  // Expensive to recalculate every tick, so do it here
  AfgRecalculateStepWidths();

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
}

// Main Loop. Does not return.

void ControllerMainLoop() {
  uButtons switches;

  controller_job_flags.adc_pot_sel = 0;
  controller_job_flags.adc_mux_shift_out = 0;
  controller_job_flags.inhibit_adc = 0;

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
  AfgTick(AFG1, get_afg1_pulse_inputs(), 1);
  AfgTick(AFG2, get_afg2_pulse_inputs(), 1);

  // Main loop. Does not return.
  while (1) {
    ControllerCommonAllLoops();

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

      // Clear switch might send us into another mode
      if (!switches.b.ClearUp || !switches.b.ClearDown) {
        // Wait for long press on clear
        InitClear_Timer();
      }
    }

    // Step programming can be applied continuously to be more responsive,
    // eg when the clock is running fast.
    ControllerApplyProgrammingSwitches(&switches);

    // Update panel state
    UpdateModeSectionLeds(AfgGetControllerState(AFG1), AfgGetControllerState(AFG2), edit_mode_section, edit_mode_step_num);
    display_update_flags.b.MainDisplay = 0;
    UpdateStepSectionLeds(AfgGetControllerState(AFG1), AfgGetControllerState(AFG2), edit_mode_step_num);
    display_update_flags.b.StepsDisplay = 0;


    // Flush LEDs every 20ms.
    // Shifting out to the leds is kind of slow, so rate limit the update to 50 Hz.
    if (get_millis() - leds_update_time > 20) {
      FlushLedUpdates();
      leds_update_time = get_millis();
    }

    // Go into modal loops if called for

    if (controller_job_flags.modal_loop == CONTROLLER_MODAL_LOAD) {
      // Sub loop for load program
      ControllerLoadProgramLoop(); // only exits when done
      delay_ms(500);
    } else if (controller_job_flags.modal_loop == CONTROLLER_MODAL_SAVE) {
      // Save program
      ControllerSaveProgramLoop(); // only exits when done
      delay_ms(500);
    } else if (controller_job_flags.modal_loop == CONTROLLER_MODAL_SCAN) {
      // Scan the adc2 inputs before processing a pulse in
      ControllerScanAdcLoop(); // only exits when done
    }

  }; // end main loop
}

void ControllerApplyProgrammingSwitches(uButtons * key) {
  AfgControllerState afg1 = AfgGetControllerState(AFG1);
  AfgControllerState afg2 = AfgGetControllerState(AFG2);
  uint8_t step_num = 0, section = 0;

  // Determine step num for different display modes
  if (display_mode == DISPLAY_MODE_VIEW_1) {
    step_num = afg1.step_num;
    section = afg1.section;
  } else if (display_mode == DISPLAY_MODE_VIEW_2) {
    step_num = afg2.step_num;
    section = afg2.section;
  } else if (display_mode == DISPLAY_MODE_EDIT_1 || display_mode == DISPLAY_MODE_EDIT_2) {
    step_num = edit_mode_step_num;
    section = edit_mode_section;
  };

  // Apply programming from switches to active step
  ApplyProgrammingSwitches(section, step_num, key);
}

void ControllerProcessStageAddressSwitches(uButtons * key) {
  PulseInputs signals1 = {}, signals2 = {};

  // Only do one of reset, strobe or advance
  if (!key->b.StageAddress1Reset) {
    AfgReset(AFG1);
    update_display();
  } else if (!key->b.StageAddress1PulseSelect) {
    signals1.strobe = 1;
    AfgProcessModeChanges(AFG1, signals1);
  } else if (!key->b.StageAddress1Advance) {
    signals1.start = 1;
    signals1.stop = 1;
    AfgProcessModeChanges(AFG1, signals1);
  }

  if (!key->b.StageAddress2Reset) {
    AfgReset(AFG2);
    update_display();
  } else if ( (!key->b.StageAddress2PulseSelect)) {
    signals2.strobe = 1;
    AfgProcessModeChanges(AFG2, signals2);
  } else if (!key->b.StageAddress2Advance) {
    signals2.start = 1;
    signals2.stop = 1;
    AfgProcessModeChanges(AFG2, signals2);
  };

  if (!key->b.StageAddress1ContiniousSelect) {
    EnableContinuousStageAddress(AFG1);
  } else {
    DisableContinuousStageAddress(AFG1);
  }

  if (!key->b.StageAddress2ContiniousSelect) {
    EnableContinuousStageAddress(AFG2);
  } else {
    DisableContinuousStageAddress(AFG2);
  }
}

void ControllerProcessNavigationSwitches(uButtons* key) {
  AfgControllerState afg1 = AfgGetControllerState(AFG1);
  AfgControllerState afg2 = AfgGetControllerState(AFG2);

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
      edit_mode_section = afg1.section;
      edit_mode_step_num = 0;
      right_counter = SCROLL_WAIT_COUNTER;
      left_counter = SCROLL_WAIT_COUNTER;
      update_display();
    }
    else if (display_mode == DISPLAY_MODE_VIEW_2) {
      display_mode = DISPLAY_MODE_EDIT_2;
      edit_mode_section = afg2.section;
      edit_mode_step_num = 0;
      right_counter = SCROLL_WAIT_COUNTER;
      left_counter = SCROLL_WAIT_COUNTER;
      update_display();
    }
  }

  // Section shift for each afg in 16 slider mode
  if (!Is_Expander_Present()) {
    if (!key->b.StepLeft && !key->b.StageAddress1Display) {
      AfgSetSection(AFG1, 0);
      display_mode = DISPLAY_MODE_VIEW_1;
      update_display();
    } else if (!key->b.StepRight && !key->b.StageAddress1Display) {
      AfgSetSection(AFG1, 1);
      display_mode = DISPLAY_MODE_VIEW_1;
      update_display();
    } else if (!key->b.StepLeft && !key->b.StageAddress2Display) {
      AfgSetSection(AFG2, 0);
      display_mode = DISPLAY_MODE_VIEW_2;
      update_display();
    } else if (!key->b.StepRight && !key->b.StageAddress2Display) {
      AfgSetSection(AFG2, 1);
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
  static uint8_t inhibit_modal = 0;

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
      if (clear_counter1 && !inhibit_modal) {
        // Go into LOAD modal in main loop
        controller_job_flags.modal_loop = CONTROLLER_MODAL_LOAD;
        inhibit_modal = 1;
      } else if (clear_counter2 && !inhibit_modal) {
        // Go into SAVE modal in main loop
        controller_job_flags.modal_loop = CONTROLLER_MODAL_SAVE;
        inhibit_modal = 1;
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
      AfgHardStop(AFG1);
      AfgHardStop(AFG2);
    };

    clear_counter1 = 0;
    clear_counter2 = 0;
  };

  if (myButtons.b.ClearUp && myButtons.b.ClearDown) {
    // Means clear switch is in the middle
    inhibit_modal = 0;
  }
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
  uint32_t switch_last_read_time = 0;
  uint32_t now = 0;
  previous_switches.value = HC165_ReadSwitches();

  StepLedsLightSingleStep(0);

  while (1) {
    now = get_millis();
    ControllerCommonAllLoops();

    if (now - switch_last_read_time > 5) {
      switches.value = HC165_ReadSwitches();
      switch_last_read_time = now;
    } else {
      switches.value = previous_switches.value;
    }

    if (switches.value != previous_switches.value) {
      // Allow stage address to continue to function in modal
      ControllerProcessStageAddressSwitches(&switches);

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
        // Load program.
        // Read from EPROM into temporary saved_program
        CAT25512_read_block(
            eprom_memory.programs[program_num].start,
            (unsigned char *) &saved_program,
            eprom_memory.programs[program_num].size);

        // Copy from saved_program to steps and sliders
        memcpy((void *) steps, (void *) saved_program.steps, sizeof(steps));
        memcpy((void *) sliders, (void *) saved_program.sliders, sizeof(sliders));

        // Pin sliders and reset to step 1
        pin_all_sliders();
        AfgReset(AFG1);
        AfgReset(AFG2);

        // Cute little animation
        RunLoadProgramAnimation();
        display_mode = DISPLAY_MODE_VIEW_1;
        controller_job_flags.modal_loop = CONTROLLER_MODAL_NONE;
        return;
      } else if (!switches.b.StageAddress1Display || !switches.b.StageAddress2Display) {
        // Abort load
        display_mode = DISPLAY_MODE_VIEW_1;
        controller_job_flags.modal_loop = CONTROLLER_MODAL_NONE;
        return;
      }
      previous_switches.value = switches.value;
    } else {
      RunWaitingLoadSaveAnimation(AfgGetControllerState(AFG1), AfgGetControllerState(AFG2));
      delay_us(500);
    }
  }
}

// Save program loop
void ControllerSaveProgramLoop() {
  uint8_t program_num = 0;
  SavedProgram saved_program = {};
  uButtons previous_switches, switches;
  uint32_t switch_last_read_time = 0;
  uint32_t now = 0;
  previous_switches.value = HC165_ReadSwitches();

  StepLedsLightSingleStep(0);

  while (1) {
    now = get_millis();
    ControllerCommonAllLoops();

    if (now - switch_last_read_time > 5) {
      switches.value = HC165_ReadSwitches();
      switch_last_read_time = now;
    } else {
      switches.value = previous_switches.value;
    }

    if (switches.value != previous_switches.value) {
      // Allow stage address to continue to function in modal
      ControllerProcessStageAddressSwitches(&switches);

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
        // Save program
        // Copy from steps and sliders to temp saved_program.
        // Prevent updates to slider values during this time.
        controller_job_flags.inhibit_adc = 1;
        memcpy((void *) saved_program.steps, (void *) steps, sizeof(steps));
        memcpy((void *) saved_program.sliders, (void *) sliders, sizeof(sliders));
        controller_job_flags.inhibit_adc = 0;

        // Write the temp saved program to the EPROM
        CAT25512_write_block(
            eprom_memory.programs[program_num].start,
            (unsigned char *) &saved_program,
            eprom_memory.programs[program_num].size);

        // Run cute animation
        RunSaveProgramAnimation();
        display_mode = DISPLAY_MODE_VIEW_1;
        controller_job_flags.modal_loop = CONTROLLER_MODAL_NONE;
        return;
      } else if (!switches.b.StageAddress1Display || !switches.b.StageAddress2Display) {
        // Abort save
        display_mode = DISPLAY_MODE_VIEW_1;
        controller_job_flags.modal_loop = CONTROLLER_MODAL_NONE;
        return;
      }
      previous_switches.value = switches.value;
    } else {
      RunWaitingLoadSaveAnimation(AfgGetControllerState(AFG1), AfgGetControllerState(AFG2));
      delay_us(500);
    }
  }
}

// Immediately scan all the adc2 lines, and then process mode changes from the input pulses.
// This guarantees that we acquire the new values present on external inputs and stage address
// before we attempt to utilize them. This prevents a couple ms of glitching on state transitions.
// New readings are double buffered and applied at the end so that the current state being held
// also does not glitch.
void ControllerScanAdcLoop() {
  // Double buffer the new adc readings and apply them all together
  uint16_t new_readings[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  adc_pause();
  controller_job_flags.inhibit_adc = 1;
  AdcMuxResetAllOff();

  // Scan all of the ADC2 channels
  for (uint8_t i = 0; i < 8; i++) {
    AdcMuxSelectAdc2(i);  // Shift the mux
    delay_us(10);         // Settling time

    // Start injected conversion
    ADC_SoftwareStartInjectedConv(ADC2);
    while (ADC_GetFlagStatus(ADC2, ADC_FLAG_JEOC) == RESET) {}
    new_readings[i] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
    ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);
  }

  // Apply all the new readings immediately without filtering
  for (uint8_t i = 0; i < 8; i++) {
    WriteOtherCvWithoutSmoothing(i, new_readings[i]);
  }

  // Now process the pending events
  // Disable all irq including the function generators
  __disable_irq();
  if (any_pulses_high(controller_job_flags.afg1_interrupts)) {
    AfgProcessModeChanges(AFG1, controller_job_flags.afg1_interrupts);
    controller_job_flags.afg1_interrupts = PULSE_INPUTS_NONE;
  }
  if (any_pulses_high(controller_job_flags.afg2_interrupts)) {
    AfgProcessModeChanges(AFG2, controller_job_flags.afg2_interrupts);
    controller_job_flags.afg2_interrupts = PULSE_INPUTS_NONE;
  }

  // Restore normal operation
  AdcMuxReset();
  delay_us(10);
  controller_job_flags.adc_pot_sel = 0;
  controller_job_flags.adc_mux_shift_out = 1;
  controller_job_flags.inhibit_adc = 1;
  controller_job_flags.modal_loop = CONTROLLER_MODAL_NONE;
  __enable_irq();
  adc_resume();
}

