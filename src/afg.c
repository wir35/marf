#include "afg.h"

#include <stm32f4xx.h>

#include "analog_data.h"
#include "display.h"
#include "MAX5135.h"
#include "program.h"

// Current step number
volatile uint8_t afg1_step_num = 0, afg2_step_num = 0;

// Step counters
volatile uint32_t afg1_step_cnt = 0, afg2_step_cnt = 0;

// Sequencer modes
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

// State for processing start and stop signals
uint16_t previous_gpiob_data = 0;
uint8_t start1_counter = 0;
uint8_t stop1_counter = 0;
uint8_t start2_counter = 0;
uint8_t stop2_counter = 0;

// Start Timer 3 for AFG1 start pulse duration measurement
// Only used when going into enable/sustain
void InitStart_1_SignalTimer() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM3->PSC = STEP_TIMER_PRESCALER;
  TIM3->ARR = START_TIMER_SUSTAIN;
  TIM3->CNT = 0;
  TIM3->DIER = TIM_DIER_UIE;
  TIM3->CR1 |= TIM_CR1_CEN;

  NVIC_EnableIRQ(TIM3_IRQn);
};

// Start Timer 7 for AFG2 start pulse duration measurement
// Only used when going into enable/sustain
void InitStart_2_SignalTimer() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

  TIM7->PSC = STEP_TIMER_PRESCALER;
  TIM7->ARR = START_TIMER_SUSTAIN;
  TIM7->CNT = 0;
  TIM7->DIER = TIM_DIER_UIE;
  TIM7->CR1 |= TIM_CR1_CEN;

  NVIC_EnableIRQ(TIM7_IRQn);
};

void DoStop1() {
  if (afg1_mode == MODE_RUN) {
    afg1_prev_mode = MODE_RUN;
    afg1_mode = MODE_STOP;
    update_display();
  };
}

void DoStop2() {
  if (afg2_mode == MODE_RUN) {
    afg2_prev_mode = MODE_RUN;
    afg2_mode = MODE_STOP;
    update_display();
  };
}

void DoStart1() {
  if (afg1_mode == MODE_STOP || afg1_mode == MODE_ADVANCE) {
    // Start run
    afg1_mode = MODE_RUN;
    afg1_step_num = GetNextStep(afg1_section, afg1_step_num);
    afg1_step_cnt = 0;
    update_display();
    DoStepOutputPulses1();
  }

  if (afg1_mode == MODE_WAIT_HI_Z || afg1_mode == MODE_STAY_HI_Z) {
    // If on sustain or enable step, check after timer
    InitStart_1_SignalTimer();
    // Calls CheckStart1() later
  }
}

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
    DoStepOutputPulses1();
    return 1;
  } else {
    return 0;
  }
}

void DoStart2() {
  if (afg2_mode == MODE_STOP || afg2_mode == MODE_ADVANCE) {
    afg2_mode = MODE_RUN;
    afg2_step_num = GetNextStep(afg2_section, afg2_step_num);
    afg2_step_cnt = 0;
    update_display();
    DoStepOutputPulses2();
  }
  if(afg2_mode == MODE_WAIT_HI_Z || afg2_mode == MODE_STAY_HI_Z) {
    InitStart_2_SignalTimer();
  }
}

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
    DoStepOutputPulses2();
    return 1;
  } else {
    return 0;
  }
}

void JumpToStep1(unsigned int step) {
  unsigned int OutputVoltage = 0;

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

  if (display_mode == DISPLAY_MODE_VIEW_1) update_display();

  if (get_step_programming(afg1_section, afg1_step_num).b.Sloped) {
    // Sloped step, hold the value
    OutputVoltage = afg1_prev_step_level;
  } else {
    // Stepped, immediately jump
    OutputVoltage = GetStepVoltage(afg1_section, afg1_step_num);
  }

  // Set DAC channel 1 to AFG1 voltage out value
  afg1_step_level = OutputVoltage;
  DAC_SetChannel1Data(DAC_Align_12b_R, OutputVoltage);

  // Set AFG1 time out value
  MAX5135_DAC_send(MAX5135_DAC_CH_0,
      GetStepTime(afg1_section, afg1_step_num) >> 2);

  // Set AFG1 reference out value
  // (Slopes down from 1023 to 0 over the course of the step)
  MAX5135_DAC_send(MAX5135_DAC_CH_1, 1023);

  DoStepOutputPulses1();
}

/* Handle jumping to new stage. Keep in sync with 1. */
void JumpToStep2(unsigned int step) {
  unsigned int OutputVoltage = 0;

  afg2_prev_step_level = afg2_step_level;
  afg2_step_num = step;
  afg2_step_cnt = 0;

  if (afg1_mode == MODE_WAIT_HI_Z || afg1_mode == MODE_STAY_HI_Z) {
    afg1_mode = afg1_prev_mode;
  }
  if (display_mode == DISPLAY_MODE_VIEW_2) update_display();

  if (get_step_programming(afg2_section, afg2_step_num).b.Sloped) {
    OutputVoltage = afg2_prev_step_level;
  } else {
    OutputVoltage = GetStepVoltage(afg2_section, afg2_step_num);
  }

  afg2_step_level = OutputVoltage;
  DAC_SetChannel2Data(DAC_Align_12b_R, OutputVoltage);

  MAX5135_DAC_send(MAX5135_DAC_CH_2,
      GetStepTime(afg2_section, afg2_step_num) >> 2);
  MAX5135_DAC_send(MAX5135_DAC_CH_3, 1023);

  DoStepOutputPulses2();
}

// Take the GPIO data directly, detect rising edges and trigger stop, start and advance
void ProcessStopStart(uint16_t gpiob_data) {
  // Detect rising edge on stop and start signals and set down counter
  if (!(previous_gpiob_data & 1) && (gpiob_data & 1)) stop1_counter = START_STOP_WINDOW;
  if (!(previous_gpiob_data & (1<<8)) && (gpiob_data & (1<<8))) start1_counter = START_STOP_WINDOW;
  if (stop1_counter && start1_counter) {
    // Both start + stop high triggers an advance, same as the advance switch
    DoAdvance1();
    stop1_counter = start1_counter = 0;
  }
  else if (stop1_counter && --stop1_counter == 0) {
    // Stop1 window timed out
    DoStop1();
  } else if (start1_counter && --start1_counter == 0) {
    // Start1 window timed out
    DoStart1();
  }

  if (!(previous_gpiob_data & (1<<1)) && (gpiob_data & (1<<1))) stop2_counter = START_STOP_WINDOW; // stop jack rising edge
  if (!(previous_gpiob_data & (1<<6)) && (gpiob_data & (1<<6))) start2_counter = START_STOP_WINDOW; // start jack rising edge
  if (stop2_counter && start2_counter) {
    DoAdvance2();
    stop2_counter = start2_counter = 0;
  } else if (stop2_counter && --stop2_counter == 0) {
    DoStop2();
  }
  else if (start2_counter && --start2_counter == 0) {
    DoStart2();
  }
  previous_gpiob_data = gpiob_data;
}

void DoStrobe1() {
  uint8_t start_signal = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 1;
  JumpToStep1(afg1_stage_address);
  if (start_signal) {
    // Handle a special case where start + strobe are high together
    // If we let the start logic run normally then we'll end up on step n+1
    // So go directly into run mode here, and the subsequent start handler will do nothing
    afg1_mode = MODE_RUN;
  }
  update_display();
}

void DoStrobe2() {
  uint8_t start_signal = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 1;
  JumpToStep2(afg2_stage_address);
  if (start_signal) {
    afg2_mode = MODE_RUN;
  }
  update_display();
}

/*
  Every tick triggers new output voltages and a check if the step has ended.
 */
ProgrammedOutputs AfgTick1() {

  uint32_t step_width = 0; // Step width = number of timer ticks
  float delta_voltage = 0.0;
  uint16_t output_voltage = 0;
  uint8_t do_pulses = 0; // 1 if pulses should fire
  ProgrammedOutputs outputs;

  // Calculate step duration
  step_width = GetStepWidth(afg1_section, afg1_step_num, GetTimeMultiplier1());

  if (afg1_step_cnt < step_width) {
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
        do_pulses = 1;
      }
  } else if ((afg1_step_cnt >= step_width)) {
    // Sample and hold current step value into PreviousStep for next step slope computation
    afg1_prev_step_level = afg1_step_level;

    // Pin step count to max value to stop ref
    afg1_step_cnt = 0xFFFFFFFF;

    // Resolve mode change for step end

    if ((afg1_mode == MODE_ADVANCE)) {
      // Stop after advance
      afg1_mode =  MODE_STOP;
      // Don't reset
    };

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
      do_pulses = 1;
      afg1_step_cnt = 0; // Reset step counter
    };
  };

  if (display_mode == DISPLAY_MODE_VIEW_1) update_display();

  // Now set output voltages
  // Compute the current step's programmed voltage output
  output_voltage = GetStepVoltage(afg1_section, afg1_step_num);

  // Set AFG1 time out value
  // MAX5135_DAC_send(MAX5135_DAC_CH_0, GetStepTime(afg1_section, afg1_step_num) >> 2);
  outputs.time = GetStepTime(afg1_section, afg1_step_num) >> 2;

  if (afg1_step_cnt < step_width) {
    // Set AFG1 reference out value
    // (Slopes down from 1023 to 0 over the course of the step)
    outputs.ref = 1023 - (uint16_t) ((1023.0 / (float) step_width) * ((float) afg1_step_cnt));
    // MAX5135_DAC_send(MAX5135_DAC_CH_1, 1023 - (uint16_t) ((1023.0 / (float) step_width) * ((float) afg1_step_cnt)));

    // If the step is sloped, then slope from PreviousStep to the new output value
    if (get_step_programming(afg1_section, afg1_step_num).b.Sloped ) {
      if (afg1_prev_step_level >= output_voltage) {
        // Slope down
        delta_voltage = (float) (afg1_prev_step_level - output_voltage) / step_width;
        output_voltage = afg1_prev_step_level - (unsigned int) (delta_voltage * afg1_step_cnt);
      } else if (output_voltage > afg1_prev_step_level) {
        // Slope up
        delta_voltage = (float) (output_voltage - afg1_prev_step_level) / step_width;
        output_voltage = afg1_prev_step_level + (unsigned int) (delta_voltage * afg1_step_cnt);
      }
    }
  } else {
    // No reference output when not running
    outputs.ref = 0;
    //MAX5135_DAC_send(MAX5135_DAC_CH_1, 0);
  }

  // Set DAC channel 1 to AFG1 voltage out value
  afg1_step_level = output_voltage;
  outputs.voltage = output_voltage;
  // DAC_SetChannel1Data(DAC_Align_12b_R, output_voltage);

  // Now that output voltages are set, pulses can fire now
  if (do_pulses) DoStepOutputPulses1();
  return outputs;
};

// Keep duplicated logic in sync with above
ProgrammedOutputs AfgTick2() {

  uint16_t step_width = 0;
  float delta_voltage = 0.0;
  uint16_t output_voltage = 0;
  uint8_t do_pulses = 0;
  ProgrammedOutputs outputs;

  step_width = GetStepWidth(afg2_section, afg2_step_num, GetTimeMultiplier2());

  if (afg2_step_cnt < step_width) {
    afg2_step_cnt += 1;
  }

  if (afg2_mode == MODE_WAIT) {
    if (afg2_step_num != (unsigned int) (afg2_stage_address)) {
      // Sample and hold current voltage output value
      afg2_prev_step_level = afg2_step_level;
      afg2_step_num = (unsigned int) (afg2_stage_address);
      // Reset step counter
      afg2_step_cnt = 0;
      do_pulses = 1;
    }
  } else if (afg2_step_cnt >= step_width) {
    afg2_step_cnt = 0xFFFFFFFF;
    afg2_prev_step_level = afg2_step_level;

    if ((afg2_mode == MODE_ADVANCE)) {
      afg2_prev_step_level = afg2_step_level;
      afg2_mode =  MODE_STOP;
    };

    if (get_step_programming(afg2_section, afg2_step_num).b.OpModeSTOP) {
      afg2_mode = MODE_STOP;
    };

    if (get_step_programming(afg2_section, afg2_step_num).b.OpModeENABLE
        && afg2_mode != MODE_WAIT_HI_Z) {
      if ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 0)) {
        afg2_prev_mode = afg2_mode;
        afg2_mode = MODE_WAIT_HI_Z;
      };
      afg2_step_cnt = 0;
    };

    if ((get_step_programming(afg2_section, afg2_step_num).b.OpModeSUSTAIN
        && afg2_mode != MODE_STAY_HI_Z)) {
      if ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 1)) {
        afg2_prev_mode = afg2_mode;
        afg2_mode = MODE_STAY_HI_Z;
        InitStart_2_SignalTimer();
      }
      // Don't reset
    };

    if (afg2_mode == MODE_RUN) {
      afg2_step_num = GetNextStep(afg2_section, afg2_step_num);
      do_pulses = 1;
      afg2_step_cnt = 0;
    };
  }

  if (display_mode == DISPLAY_MODE_VIEW_2) update_display();

  output_voltage = GetStepVoltage(afg2_section, afg2_step_num);

  outputs.time = GetStepTime(afg2_section, afg2_step_num) >> 2;

  // MAX5135_DAC_send(MAX5135_DAC_CH_2, GetStepTime(afg2_section, afg2_step_num) >> 2);

  if (afg2_step_cnt < step_width) {
    outputs.ref = 1023 - (unsigned int) ((1023.0 / (float) step_width) * ((float) afg2_step_cnt));

    // MAX5135_DAC_send(MAX5135_DAC_CH_3,1023 - (unsigned int) ((1023.0 / (float) step_width) * ((float) afg2_step_cnt)));

    if (get_step_programming(afg2_section, afg2_step_num).b.Sloped ) {
      if (afg2_prev_step_level >= output_voltage) {
        delta_voltage = (float) (afg2_prev_step_level - output_voltage) / step_width;
        output_voltage = afg2_prev_step_level - (unsigned int) (delta_voltage * afg2_step_cnt);
      } else if (output_voltage > afg2_prev_step_level) {
        delta_voltage =  (float) (output_voltage - afg2_prev_step_level) / step_width;
        output_voltage = afg2_prev_step_level + (unsigned int) (delta_voltage * afg2_step_cnt);
      }
    }
  } else {
    // MAX5135_DAC_send(MAX5135_DAC_CH_3, 0);
    outputs.ref = 0;
  }

  afg2_step_level = output_voltage;
  outputs.voltage = output_voltage;

  // DAC_SetChannel2Data(DAC_Align_12b_R, output_voltage);

  if (do_pulses) DoStepOutputPulses2();

  return outputs;
};

#define TIME_MULTIPLIER_SCALER 0.0009766

// The panel is marked for log scale (0.5, 1, 2, 4) but linear pots are used.
// Scale the time multipliers to more closely match the panel.
// Use a linear interpolation between the points instead of log2.

inline float fake_log2(float linear_val) {
  if (linear_val < 1365.0) {
    // 512 - 1024 or 0.5 - 1
    return linear_val * 0.375 + 512.0;
  } else if (linear_val < 2730) {
    // 1024 - 2048 or 1 - 2
    return (linear_val - 1365) * 0.882 + 1024.0;
  } else {
    // 2048 - 4095 or 2 - 4
    return (linear_val - 2730) * 1.5 + 2048.0;
  }
}

float GetTimeMultiplier1() {
  return fake_log2(read_calibrated_add_data_float(ADC_TIMEMULTIPLY_Ch_1)) * TIME_MULTIPLIER_SCALER;
}

float GetTimeMultiplier2() {
  return fake_log2(read_calibrated_add_data_float(ADC_TIMEMULTIPLY_Ch_2)) * TIME_MULTIPLIER_SCALER;
}



