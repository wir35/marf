#include "afg.h"

#include <stm32f4xx.h>

#include "analog_data.h"
#include "display.h"
#include "MAX5135.h"
#include "program.h"
#include "cycle_counter.h"
#include "delays.h"

// Afg state. Volatile since control is frequently exchanged between interrupts and main loop.
volatile AfgState afg1;
volatile AfgState afg2;

// Convenience for addressing the two afgs from a uint8_t that is 0 or 1
volatile AfgState *afgs[2] = { &afg1, &afg2 };

void AfgAllInitialize() {
  afg1.section = 0;
  afg1.step_num = 0;
  afg1.step_width = 1;
  afg1.step_cnt = 0xFFFFFFFF;
  afg1.mode = MODE_STOP;
  afg1.prev_mode = MODE_STOP;
  afg1.stage_address = 0;
  afg1.step_level = 0;
  afg1.prev_step_level = 0;

  afg2.section = 0;
  afg2.step_num = 0;
  afg2.step_width = 1;
  afg2.step_cnt = 0xFFFFFFFF;
  afg2.mode = MODE_STOP;
  afg2.prev_mode = MODE_STOP;
  afg2.stage_address = 0;
  afg2.step_level = 0;
  afg2.prev_step_level = 0;
}

AfgControllerState AfgGetControllerState(uint8_t afg_num) {
  AfgState *afg = (AfgState *) afgs[afg_num];
  AfgControllerState state;
  state.mode = afg->mode;
  state.section = afg->section;
  state.step_num = afg->step_num;
  return state;
}

void AfgSetSection(uint8_t afg_num, uint8_t section) {
  AfgState *afg = (AfgState *) afgs[afg_num];
  afg->section = section;
}

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

void InitStartSignalTimer(uint8_t afg_num) {
  if (afg_num == AFG1) InitStart_1_SignalTimer();
  if (afg_num == AFG2) InitStart_2_SignalTimer();
}

#define TIME_MULTIPLIER_SCALER 0.0009766

float GetTimeMultiplier(uint8_t afg_num) {
  if (afg_num == AFG1) {
    return scale_time_fake_log2(read_calibrated_add_data_float(ADC_TIMEMULTIPLY_Ch_1)) * TIME_MULTIPLIER_SCALER;
  } else if (afg_num == AFG2) {
    return scale_time_fake_log2(read_calibrated_add_data_float(ADC_TIMEMULTIPLY_Ch_2)) * TIME_MULTIPLIER_SCALER;
  } else {
    return 0;
  }
}

// Compute continuous step stage selection

inline void ComputeContinuousStep(uint8_t afg_num) {
  if (afg_num == AFG1) {
    afg1.stage_address = read_calibrated_add_data_uint16(ADC_STAGEADDRESS_Ch_1) >> get_max_step_shift12();
    if (afg1.stage_address > get_max_step()) afg1.stage_address = get_max_step();
  } else if (afg_num == AFG2) {
    afg2.stage_address = read_calibrated_add_data_uint16(ADC_STAGEADDRESS_Ch_2) >> get_max_step_shift12();
    if (afg2.stage_address > get_max_step()) afg2.stage_address = get_max_step();
  }
}

void AfgRecalculateStepWidths() {
  afg1.step_width = GetStepWidth(afg1.section, afg1.step_num, GetTimeMultiplier(AFG1));
  afg2.step_width = GetStepWidth(afg2.section, afg2.step_num, GetTimeMultiplier(AFG2));
}

// Triggered by timer when waiting on sustain or enable step.
// Returns 1 when starting running again
uint8_t AfgCheckStart(uint8_t afg_num, uint8_t start_signal) {
  AfgState *afg = (AfgState *) afgs[afg_num];
  uint8_t run_again = 0;

  if (afg->mode == MODE_WAIT_HI_Z && start_signal) {
    // Enable with start high, start running
    run_again = 1;
  }
  if (afg->mode == MODE_STAY_HI_Z && !start_signal) {
    // Sustain step with start low, start running again
    run_again = 1;
  }
  if (run_again) {
    afg->mode = MODE_RUN;
    afg->step_num = GetNextStep(afg->section, afg->step_num);
    afg->step_cnt = 0;
    return 1;
  } else {
    return 0;
  }
}

void AfgJumpToStep(uint8_t afg_num, uint8_t step) {
  AfgState *afg = (AfgState *) afgs[afg_num];

  // Sample and hold current output voltage value.
  afg->prev_step_level = afg->step_level;

  // Then update the step number to where ever we are strobing to
  afg->step_num = step;

  // Reset step counter
  afg->step_cnt = 0;

  // Break out of some modes
  if (afg->mode == MODE_WAIT_HI_Z || afg->mode == MODE_STAY_HI_Z) {
    afg->mode = afg->prev_mode;
  }

  update_display();
}

void AfgHardStop(uint8_t afg_num) {
  AfgState *afg = (AfgState *) afgs[afg_num];

  afg->mode = MODE_STOP;
  afg->step_num = 0;
  afg->step_cnt = 0xFFFFFFFF;
}

void AfgReset(uint8_t afg_num) {
  AfgState *afg = (AfgState *) afgs[afg_num];

  if (afg->mode != MODE_WAIT) {
    AfgJumpToStep(afg_num, 0);
  }
}

// Process start, stop and strobe pulse inputs in reaction to an interrupt on any of them.
// Since simultaneous pulses are meaningful, we process them all together.
void AfgProcessModeChanges(uint8_t afg_num, PulseInputs pulses) {
  AfgState *afg = (AfgState *) afgs[afg_num];

  if (pulses.strobe) {
    // Strobe to a step
    AfgJumpToStep(afg_num, afg->stage_address);
    if (pulses.start) {
      // And start running
      afg->mode = MODE_RUN;
    }
  } else if (pulses.start && pulses.stop && afg->mode != MODE_WAIT) {
    // Stop and start together are an advance.
    // Do not change the mode, but jump immediately to the next step.
    AfgJumpToStep(afg_num, GetNextStep(afg->section, afg->step_num));
  } else if (pulses.start) {
    if (afg->mode == MODE_STOP) {
      // Start running
      afg->mode = MODE_RUN;
      afg->step_num = GetNextStep(afg->section, afg->step_num);
      afg->step_cnt = 0;
      update_display();
    } else if (afg->mode == MODE_WAIT_HI_Z || afg->mode == MODE_STAY_HI_Z) {
      // If on sustain or enable step, check after timer
      InitStartSignalTimer(afg_num); // Calls AfgCheckStart() later
    }
  } else if (pulses.stop && afg->mode == MODE_RUN) {
    afg->mode = MODE_STOP;
    update_display();
  }
}

// Every tick triggers new output voltages and a check if the step has ended.
ProgrammedOutputs AfgTick(uint8_t afg_num, PulseInputs pulses) {
  AfgState *afg = (AfgState *) afgs[afg_num];

  uStep step = get_step_programming(afg->section, afg->step_num);
  float delta_voltage = 0.0;
  uint16_t output_voltage = 0;
  uint8_t do_recalculate_step_width = 1;
  ProgrammedOutputs outputs;

  // Compute continuous stage address
  ComputeContinuousStep(afg_num);

  if (afg->step_cnt < afg->step_width) {
    afg->step_cnt += 1;
  }

  // Check if we're at the end of the step
  if (afg->mode == MODE_WAIT) {
      // Continuous step address mode. Check if the step has changed by the stage address, not the timer.
      if (afg->step_num != afg->stage_address) {
        // Sample and hold current voltage output value
        afg->prev_step_level = afg->step_level;
        afg->step_num = afg->stage_address;
        // Reset step counter
        afg->step_cnt = 0;
        do_recalculate_step_width = 1;
      }
  } else if (afg->step_cnt >= afg->step_width) {
    // Sample and hold current step value into PreviousStep for next step slope computation
    afg->prev_step_level = afg->step_level;

    // Pin step count to max value to stop the reference
    afg->step_cnt = 0xFFFFFFFF;

    if (step.b.OpModeSTOP) {
      // Stop step
      afg->mode = MODE_STOP;
      // Don't reset step counter
    };

    if (step.b.OpModeENABLE && afg->mode != MODE_WAIT_HI_Z)  {
      // Enable step, check start banana
      if (pulses.start == 0) {
        // Go into enable mode
        afg->prev_mode = afg->mode;
        afg->mode = MODE_WAIT_HI_Z;
      };
      afg->step_cnt = 0; // Reset step counter
    };

    if (step.b.OpModeSUSTAIN && afg->mode != MODE_STAY_HI_Z) {
      // Sustain step, check start banana
      if (pulses.start == 1) {
        // Go into sustain mode
        afg->prev_mode = afg->mode;
        afg->mode = MODE_STAY_HI_Z;
        InitStartSignalTimer(afg_num); // Check continuously by timer for start to go low
      };
      // Don't reset step counter
    };

    if (afg->mode == MODE_RUN) {
      // Advance to the next step
      afg->step_num = GetNextStep(afg->section, afg->step_num);
      afg->step_cnt = 0; // Reset step counter
      do_recalculate_step_width = 1;
    };
  };

  // Now set output voltages
  // Compute the current step's programmed voltage output
  output_voltage = GetStepVoltage(afg->section, afg->step_num);

  // Set AFG time out value
  outputs.time = get_time_slider_level(afg->step_num) >> 2;

  if (afg->step_cnt < afg->step_width) {
    // Set AFG1 reference out value
    // (Slopes down from 1023 to 0 over the course of the step)
    outputs.ref = 1023 - (uint16_t) ((afg->step_cnt << 10) / afg->step_width);

    // If the step is sloped, then slope from prev_step_level to the new output value
    if (step.b.Sloped ) {
      if (afg->prev_step_level >= output_voltage) {
        // Slope down
        delta_voltage = (float) (afg->prev_step_level - output_voltage) / afg->step_width;
        output_voltage = afg->prev_step_level - (uint16_t) (delta_voltage * afg->step_cnt);
      } else if (output_voltage > afg->prev_step_level) {
        // Slope up
        delta_voltage = (float) (output_voltage - afg->prev_step_level) / afg->step_width;
        output_voltage = afg->prev_step_level + (uint16_t) (delta_voltage * afg->step_cnt);
      }
    }
  } else {
    // No reference output when not running
    outputs.ref = 0;
  }

  afg->step_level = output_voltage;
  outputs.voltage = output_voltage;

  if (afg->step_cnt > PULSE_ACTIVE_STEP_WIDTH) {
    outputs.all_pulses = 0;
    outputs.pulse1 = 0;
    outputs.pulse2 = 0;
  } else {
    outputs.all_pulses = 1;
    outputs.pulse1 = step.b.OutputPulse1;
    outputs.pulse2 = step.b.OutputPulse2;
  }

  if (do_recalculate_step_width) {
    afg->step_width = GetStepWidth(afg->section, afg->step_num, GetTimeMultiplier(afg_num));
    update_display();
  }

  return outputs;
};

void EnableContinuousStageAddress(uint8_t afg_num) {
  AfgState *afg = (AfgState *) afgs[afg_num];

  if (afg->mode != MODE_WAIT) {
    afg->prev_mode = afg->mode;
    afg->mode = MODE_WAIT;
    update_main_display();
  }
}

void DisableContinuousStageAddress(uint8_t afg_num) {
  AfgState *afg = (AfgState *) afgs[afg_num];

  if (afg->mode == MODE_WAIT) {
    afg->mode = afg->prev_mode;
    afg->prev_mode = MODE_WAIT;
    update_display();
  }
}


