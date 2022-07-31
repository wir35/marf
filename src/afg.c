#include "afg.h"

#include <stm32f4xx.h>

#include "analog_data.h"
#include "display.h"
#include "MAX5135.h"
#include "program.h"
#include "cycle_counter.h"
#include "delays.h"

// Current step number
volatile uint8_t afg1_step_num = 0, afg2_step_num = 0;

// Step tick counters
volatile uint32_t afg1_step_cnt = 0, afg2_step_cnt = 0;
volatile uint32_t afg1_step_width = 0, afg2_step_width = 0;

// AFG modes
volatile unsigned char afg1_mode = MODE_RUN;
volatile unsigned char afg2_mode = MODE_RUN;
volatile unsigned char afg1_prev_mode = MODE_RUN;
volatile unsigned char afg2_prev_mode = MODE_RUN;

// The voltage level of the current step
volatile unsigned int afg1_step_level = 0;
volatile unsigned int afg2_step_level = 0;

// The voltage level of the previous step
volatile unsigned int afg1_prev_step_level = 0;
volatile unsigned int afg2_prev_step_level = 0;

// The stage address selection step
volatile uint8_t afg1_stage_address = 0;
volatile uint8_t afg2_stage_address = 0;

// The offset into step programming for each afg [0-15][16-31] 0 or 1
volatile uint8_t afg1_section = 0;
volatile uint8_t afg2_section = 0;

#define START_STOP_WINDOW 4

// Start Timer 3 for AFG1 start pulse duration measurement
// Only used when going into enable/sustain
void InitStart_1_SignalTimer() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM3->PSC = AFG_TIMER_PRESCALER;
  TIM3->ARR = 1;
  TIM3->CNT = 0;
  TIM3->DIER = TIM_DIER_UIE;
  TIM3->CR1 |= TIM_CR1_CEN;

  NVIC_EnableIRQ(TIM3_IRQn);
};

// Start Timer 7 for AFG2 start pulse duration measurement
// Only used when going into enable/sustain
void InitStart_2_SignalTimer() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

  TIM7->PSC = AFG_TIMER_PRESCALER;
  TIM7->ARR = 1;
  TIM7->CNT = 0;
  TIM7->DIER = TIM_DIER_UIE;
  TIM7->CR1 |= TIM_CR1_CEN;

  NVIC_EnableIRQ(TIM7_IRQn);
};

void AfgAllInitialize() {
  afg1_section = 0;
  afg1_step_num = 0;
  afg1_step_cnt = 0xFFFFFFFF;
  afg1_mode = MODE_STOP;

  afg2_section = 0;
  afg2_step_num = 0;
  afg2_step_cnt = 0xFFFFFFFF;
  afg2_mode = MODE_STOP;
}

// Triggered by timer when waiting on sustain or enable step.
// Returns 1 when starting running again
uint8_t CheckStart1() {
  uint8_t start_signal = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 1;
  uint8_t run_again = 0;

  if (afg1_mode == MODE_WAIT_HI_Z && start_signal) {
    // Enable with start high, start running
    run_again = 1;
  }
  if (afg1_mode == MODE_STAY_HI_Z && !start_signal) {
    // Sustain step with start low, start running again
    run_again = 1;
  }
  if (run_again) {
    afg1_mode = MODE_RUN;
    afg1_step_num = GetNextStep(afg1_section, afg1_step_num);
    afg1_step_cnt = 0;
    return 1;
  } else {
    return 0;
  }
}

// Triggered by timer when waiting on sustain or enable step.
// Returns 1 when starting running again
uint8_t CheckStart2() {
  uint8_t start_signal = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 1;
  uint8_t run_again = 0;

  if (afg2_mode == MODE_WAIT_HI_Z && start_signal) {
    // Enable with start high, start running
    run_again = 1;
  }
  if (afg2_mode == MODE_STAY_HI_Z && !start_signal) {
    // Sustain step with start low, start running again
    run_again = 1;
  }
  if (run_again) {
    afg2_mode = MODE_RUN;
    afg2_step_num = GetNextStep(afg2_section, afg2_step_num);
    afg2_step_cnt = 0;
    return 1;
  } else {
    return 0;
  }
}

void JumpToStep1(unsigned int step) {
  // unsigned int OutputVoltage = 0;

  // Sample and hold current output voltage value.
  afg1_prev_step_level = afg1_step_level;

  // Then update the step number to where ever we are strobing to
  afg1_step_num = step;

  // Reset step counter
  afg1_step_cnt = 0;

  // Break out of some modes
  if (afg1_mode == MODE_WAIT_HI_Z || afg1_mode == MODE_STAY_HI_Z) {
    afg1_mode = afg1_prev_mode;
  }

  update_display();
}

/* Handle jumping to new stage. Keep in sync with 1. */
void JumpToStep2(unsigned int step) {
  // unsigned int OutputVoltage = 0;

  afg2_prev_step_level = afg2_step_level;
  afg2_step_num = step;
  afg2_step_cnt = 0;

  if (afg2_mode == MODE_WAIT_HI_Z || afg2_mode == MODE_STAY_HI_Z) {
    afg2_mode = afg2_prev_mode;
  }
  update_display();
}

// Process start, stop and strobe pulse inputs in reaction to an interrupt on any of them.
// Since simultaneous pulses are meaningful, we process them all together.
void ProcessModeChanges1(PulseInputs pulses) {
  if (pulses.strobe) {
    // Strobe to a step
    JumpToStep1(afg1_stage_address);
    if (pulses.start) {
      // And start running
      afg1_mode = MODE_RUN;
    }
  } else if (pulses.start && pulses.stop && afg1_mode != MODE_WAIT) {
    // Stop and start together are an advance.
    // Do not change the mode, but jump immediately to the next step.
    JumpToStep1(GetNextStep(afg1_section, afg1_step_num));
  } else if (pulses.start) {
    if (afg1_mode == MODE_STOP) {
      // Start running
      afg1_mode = MODE_RUN;
      afg1_step_num = GetNextStep(afg1_section, afg1_step_num);
      afg1_step_cnt = 0;
      update_display();
    } else if (afg1_mode == MODE_WAIT_HI_Z || afg1_mode == MODE_STAY_HI_Z) {
      // If on sustain or enable step, check after timer
      InitStart_1_SignalTimer(); // Calls CheckStart1() later
    }
  } else if (pulses.stop && afg1_mode == MODE_RUN) {
    afg1_mode = MODE_STOP;
    update_display();
  }
}

void ProcessModeChanges2(PulseInputs pulses) {
  if (pulses.strobe) {
    // Strobe to a step
    JumpToStep2(afg2_stage_address);
    if (pulses.start) {
      // And start running
      afg2_mode = MODE_RUN;
    }
    update_display();
  } else if (pulses.start && pulses.stop && afg2_mode != MODE_WAIT) {
    // Stop and start together are an advance.
    // Do not change the mode, but jump immediately to the next step.
    JumpToStep2(GetNextStep(afg2_section, afg2_step_num));
  } else if (pulses.start) {
    if (afg2_mode == MODE_STOP) {
      // Start running
      afg2_mode = MODE_RUN;
      afg2_step_num = GetNextStep(afg2_section, afg2_step_num);
      afg2_step_cnt = 0;
      update_display();
    } else if (afg2_mode == MODE_WAIT_HI_Z || afg2_mode == MODE_STAY_HI_Z) {
      // If on sustain or enable step, check after timer
      InitStart_2_SignalTimer(); // Calls CheckStart2() later
    }
  } else if (pulses.stop && afg2_mode == MODE_RUN) {
    afg2_mode = MODE_STOP;
    update_display();
  }
}


// Every tick triggers new output voltages and a check if the step has ended.
ProgrammedOutputs AfgTick1() {

  float delta_voltage = 0.0;
  uint16_t output_voltage = 0;
  uint8_t do_recalculate_step_width = 1;
  ProgrammedOutputs outputs;

  // Compute continuous stage address
  ComputeContinuousStep1();

  if (afg1_step_cnt < afg1_step_width) {
    afg1_step_cnt += 1;
  }

  // Check if we're at the end of the step
  if (afg1_mode == MODE_WAIT) {
      // Continuous step address mode. Check if the step has changed by the stage address, not the timer.
      if (afg1_step_num != (unsigned int) (afg1_stage_address)) {
        // Sample and hold current voltage output value
        afg1_prev_step_level = afg1_step_level;
        afg1_step_num = (unsigned int) (afg1_stage_address);
        // Reset step counter
        afg1_step_cnt = 0;
        do_recalculate_step_width = 1;
      }
  } else if ((afg1_step_cnt >= afg1_step_width)) {
    // Sample and hold current step value into PreviousStep for next step slope computation
    afg1_prev_step_level = afg1_step_level;

    // Pin step count to max value to stop ref
    afg1_step_cnt = 0xFFFFFFFF;

    if (get_step_programming(afg1_section, afg1_step_num).b.OpModeSTOP) {
      // Stop step
      afg1_mode = MODE_STOP;
      // Don't reset step counter
    };

    if (get_step_programming(afg1_section, afg1_step_num).b.OpModeENABLE
        && afg1_mode != MODE_WAIT_HI_Z)  {
      // Enable step, check start banana
      if ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 0)) {
        // Go into enable mode
        afg1_prev_mode = afg1_mode;
        afg1_mode = MODE_WAIT_HI_Z;
      };
      afg1_step_cnt = 0; // Reset step counter
    };

    if (get_step_programming(afg1_section, afg1_step_num).b.OpModeSUSTAIN
        && afg1_mode != MODE_STAY_HI_Z) {
      // Sustain step, check start banana
      if ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 1)) {
        // Go into sustain mode
        afg1_prev_mode = afg1_mode;
        afg1_mode = MODE_STAY_HI_Z;
        InitStart_1_SignalTimer();
      };
      // Don't reset step counter
    };

    if (afg1_mode == MODE_RUN) {
      // Advance to the next step
      afg1_step_num = GetNextStep(afg1_section, afg1_step_num);
      afg1_step_cnt = 0; // Reset step counter
      do_recalculate_step_width = 1;
    };
  };

  // Now set output voltages
  // Compute the current step's programmed voltage output
  output_voltage = GetStepVoltage(afg1_section, afg1_step_num);

  // Set AFG1 time out value
  outputs.time = get_time_slider_level(afg1_step_num) >> 2;

  if (afg1_step_cnt < afg1_step_width) {
    // Set AFG1 reference out value
    // (Slopes down from 1023 to 0 over the course of the step)
    outputs.ref = 1023 - (uint16_t) ((afg1_step_cnt << 10) / afg1_step_width);

    // If the step is sloped, then slope from PreviousStep to the new output value
    if (get_step_programming(afg1_section, afg1_step_num).b.Sloped ) {
      if (afg1_prev_step_level >= output_voltage) {
        // Slope down
        delta_voltage = (float) (afg1_prev_step_level - output_voltage) / afg1_step_width;
        output_voltage = afg1_prev_step_level - (uint16_t) (delta_voltage * afg1_step_cnt);
      } else if (output_voltage > afg1_prev_step_level) {
        // Slope up
        delta_voltage = (float) (output_voltage - afg1_prev_step_level) / afg1_step_width;
        output_voltage = afg1_prev_step_level + (uint16_t) (delta_voltage * afg1_step_cnt);
      }
    }
  } else {
    // No reference output when not running
    outputs.ref = 0;
  }

  afg1_step_level = output_voltage;
  outputs.voltage = output_voltage;

  if (afg1_step_cnt > PULSE_ACTIVE_STEP_WIDTH) {
    outputs.all_pulses = 0;
    outputs.pulse1 = 0;
    outputs.pulse2 = 0;
  } else {
    outputs.all_pulses = 1;
    outputs.pulse1 = get_step_programming(afg1_section, afg1_step_num).b.OutputPulse1;
    outputs.pulse2 = get_step_programming(afg1_section, afg1_step_num).b.OutputPulse2;
  }

  if (do_recalculate_step_width) {
    afg1_step_width = GetStepWidth(afg1_section, afg1_step_num, GetTimeMultiplier1());
    if (display_mode == DISPLAY_MODE_VIEW_1) update_display();
  }

  return outputs;
};

// Keep duplicated logic in sync with above
ProgrammedOutputs AfgTick2() {

  float delta_voltage = 0.0;
  uint16_t output_voltage = 0;
  uint8_t do_recalculate_step_width = 1;
  ProgrammedOutputs outputs;

  ComputeContinuousStep2();

  if (afg2_step_cnt < afg2_step_width) {
    afg2_step_cnt += 1;
  }

  if (afg2_mode == MODE_WAIT) {
    if (afg2_step_num != (unsigned int) (afg2_stage_address)) {
      // Sample and hold current voltage output value
      afg2_prev_step_level = afg2_step_level;
      afg2_step_num = (unsigned int) (afg2_stage_address);
      // Reset step counter
      afg2_step_cnt = 0;
      do_recalculate_step_width = 1;
    }
  } else if (afg2_step_cnt >= afg2_step_width) {
    afg2_step_cnt = 0xFFFFFFFF;
    afg2_prev_step_level = afg2_step_level;

    if (get_step_programming(afg2_section, afg2_step_num).b.OpModeSTOP) {
      afg2_mode = MODE_STOP;
    }

    if (get_step_programming(afg2_section, afg2_step_num).b.OpModeENABLE
        && afg2_mode != MODE_WAIT_HI_Z) {
      if ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 0)) {
        afg2_prev_mode = afg2_mode;
        afg2_mode = MODE_WAIT_HI_Z;
      };
      afg2_step_cnt = 0;
    }

    if ((get_step_programming(afg2_section, afg2_step_num).b.OpModeSUSTAIN
        && afg2_mode != MODE_STAY_HI_Z)) {
      if ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 1)) {
        afg2_prev_mode = afg2_mode;
        afg2_mode = MODE_STAY_HI_Z;
        InitStart_2_SignalTimer();
      }
    }

    if (afg2_mode == MODE_RUN) {
      afg2_step_num = GetNextStep(afg2_section, afg2_step_num);
      afg2_step_cnt = 0;
      do_recalculate_step_width = 1;
    }
  }

  output_voltage = GetStepVoltage(afg2_section, afg2_step_num);
  outputs.time = get_time_slider_level(afg2_step_num) >> 2;

  if (afg2_step_cnt < afg2_step_width) {
    outputs.ref = 1023 - (uint16_t) ((afg2_step_cnt << 10) / afg2_step_width);

    if (get_step_programming(afg2_section, afg2_step_num).b.Sloped ) {
      if (afg2_prev_step_level >= output_voltage) {
        // Slope down
        delta_voltage = (float) (afg2_prev_step_level - output_voltage) / afg2_step_width;
        output_voltage = afg2_prev_step_level - (unsigned int) (delta_voltage * afg2_step_cnt);
      } else if (output_voltage > afg2_prev_step_level) {
        // Slope up
        delta_voltage =  (float) (output_voltage - afg2_prev_step_level) / afg2_step_width;
        output_voltage = afg2_prev_step_level + (unsigned int) (delta_voltage * afg2_step_cnt);
      }
    }
  } else {
    outputs.ref = 0;
  }

  afg2_step_level = output_voltage;
  outputs.voltage = output_voltage;

  if (afg2_step_cnt > PULSE_ACTIVE_STEP_WIDTH) {
    outputs.all_pulses = 0;
    outputs.pulse1 = 0;
    outputs.pulse2 = 0;
  } else {
    outputs.all_pulses = 1;
    outputs.pulse1 = get_step_programming(afg2_section, afg2_step_num).b.OutputPulse1;
    outputs.pulse2 = get_step_programming(afg2_section, afg2_step_num).b.OutputPulse2;
  }

  if (do_recalculate_step_width) {
    afg2_step_width = GetStepWidth(afg2_section, afg2_step_num, GetTimeMultiplier2());
    if (display_mode == DISPLAY_MODE_VIEW_2) update_display();
  }

  return outputs;
};

#define TIME_MULTIPLIER_SCALER 0.0009766

float GetTimeMultiplier1() {
  return scale_time_fake_log2(read_calibrated_add_data_float(ADC_TIMEMULTIPLY_Ch_1)) * TIME_MULTIPLIER_SCALER;
}

float GetTimeMultiplier2() {
  return scale_time_fake_log2(read_calibrated_add_data_float(ADC_TIMEMULTIPLY_Ch_2)) * TIME_MULTIPLIER_SCALER;
}



