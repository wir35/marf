#include "afg.h"

#include <stm32f4xx.h>

#include "analog_data.h"
#include "display.h"
#include "MAX5135.h"
#include "program.h"

// Current step number
volatile uint8_t afg1_step_num = 0, afg2_step_num = 0;

// Length of the current step in timer "ticks"
volatile uint32_t afg1_step_width = 0, afg2_step_width = 0;

// Step counters
volatile uint32_t afg1_step_cnt = 0, afg2_step_cnt = 0;

// Sequencer modes
volatile unsigned char afg1_mode = MODE_RUN;
volatile unsigned char afg2_mode = MODE_RUN;
volatile unsigned char afg1_prev_mode = MODE_RUN;
volatile unsigned char afg2_prev_mode = MODE_RUN;
volatile unsigned char afg1_advance = 0;
volatile unsigned char afg2_advance = 0;

// Current mode for start condition
volatile unsigned char afg1_start_mode = START_MODE_ZERO;
volatile unsigned char afg2_start_mode = START_MODE_ZERO;

// The voltage level of the current step
volatile unsigned int afg1_step_level = 0;
volatile unsigned int afg2_step_level = 0;

// The voltage level of the previous step
volatile unsigned int afg1_prev_step_level = 0;
volatile unsigned int afg2_prev_step_level = 0;

// The stage address selection step
volatile uint8_t afg1_stage_address = 0;
volatile uint8_t afg2_stage_address = 0;

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
  if ((afg1_mode != MODE_WAIT && afg1_mode != MODE_WAIT_HI_Z && afg1_mode != MODE_STAY_HI_Z)) {
    afg1_prev_mode = MODE_RUN;
    afg1_mode = MODE_STOP;
    update_display();
  };
}

void DoStop2() {
  if (afg2_mode != MODE_WAIT && afg2_mode != MODE_WAIT_HI_Z && afg2_mode != MODE_STAY_HI_Z) {
    afg2_prev_mode = MODE_RUN;
    afg2_mode = MODE_STOP;
    update_display();
  };
}

void DoStart1() {
  if (afg1_mode != MODE_STAY_HI_Z
      && afg1_mode != MODE_WAIT_HI_Z
      && afg1_mode != MODE_WAIT
      && afg1_mode != MODE_RUN) {
    // Go into run
    afg1_mode = MODE_RUN;
    afg1_step_num = GetNextStep(0, afg1_step_num);
    DoStepOutputPulses1();
  }

  if (afg1_mode == MODE_WAIT_HI_Z) {
    // If waiting on enable step, start running again
    InitStart_1_SignalTimer();
    afg1_mode = MODE_RUN;
    afg1_step_num = GetNextStep(0, afg1_step_num);
    DoStepOutputPulses1();
  }

  if (afg1_mode == MODE_STAY_HI_Z) {
    InitStart_1_SignalTimer();
  };
}

void DoStart2() {
  if (afg2_mode != MODE_STAY_HI_Z
      && afg2_mode != MODE_WAIT_HI_Z
      && afg2_mode != MODE_WAIT
      && afg2_mode != MODE_RUN) {
    afg2_mode = MODE_RUN;
    afg2_step_num = GetNextStep(1, afg2_step_num);
    DoStepOutputPulses2();
  }
  if(afg2_mode == MODE_WAIT_HI_Z) {
    InitStart_2_SignalTimer();
    afg2_mode = MODE_RUN;
    afg2_step_num = GetNextStep(1, afg2_step_num);
    DoStepOutputPulses2();
  }

  if(afg2_mode == MODE_STAY_HI_Z) {
    InitStart_2_SignalTimer();
  }
}

void JumpToStep1(unsigned int step) {
  unsigned int OutputVoltage = 0;

  // Sample and hold current output voltage value.
  afg1_prev_step_level = afg1_step_level;

  // Then update the step number to where ever we are strobing to
  afg1_step_num = step;

  // Reset step width
  afg1_step_cnt = 0;

  // Break out of some modes
  if (afg1_mode == MODE_WAIT_HI_Z || afg1_mode == MODE_STAY_HI_Z) {
    afg1_mode = afg1_prev_mode;
  }

  if (display_mode == DISPLAY_MODE_VIEW_1) update_display();

  if (steps[0][afg1_step_num].b.Sloped ) {
    // Sloped step, hold the value
    OutputVoltage = afg1_prev_step_level;
  } else {
    // Stepped, immediately jump
    OutputVoltage = GetStepVoltage(0, afg1_step_num);
  }

  // Set DAC channel 1 to AFG1 voltage out value
  afg1_step_level = OutputVoltage;
  DAC_SetChannel1Data(DAC_Align_12b_R, OutputVoltage);

  // Set AFG1 time out value
  MAX5135_DAC_send(MAX5135_DAC_CH_0, steps[0][afg1_step_num].b.TLevel >> 2);

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

  if (steps[1][afg2_step_num].b.Sloped ) {
    OutputVoltage = afg2_prev_step_level;
  } else {
    OutputVoltage = GetStepVoltage(1, afg2_step_num);
  }

  afg2_step_level = OutputVoltage;
  DAC_SetChannel2Data(DAC_Align_12b_R, OutputVoltage);

  MAX5135_DAC_send(MAX5135_DAC_CH_2, steps[1][afg2_step_num].b.TLevel >> 2);
  MAX5135_DAC_send(MAX5135_DAC_CH_3, 1023);

  DoStepOutputPulses2();
}

/*
  Every tick triggers new output voltages and a check if the step has ended.
 */
void AfgTick1() {

  // TODO(maxl0rd): rename locals
  unsigned long int StepWidth_1 = 0; // Step width = number of timer ticks
  float deltaVoltage = 0.0;
  unsigned long OutputVoltage = 0;
  unsigned char doPulses = 0; // 1 if pulses should fire

  // TODO(maxl0rd): maybe move this to IRQ
  // TODO(maxl0rd): use precomputed calibration

  // Calculate step duration and scaler for Timer 4.
  StepWidth_1 = GetStepWidth(0, afg1_step_num);
  TIM4->PSC = (uint16_t) (
      (((((float) add_data[ADC_TIMEMULTIPLY_Ch_1]) * 3.5f)
          / cal_constants[ADC_TIMEMULTIPLY_Ch_1]) + 0.5f)
          * STEP_TIMER_PRESCALER);

  if (afg1_step_cnt < StepWidth_1) {
    afg1_step_cnt += 1;
  };

  // Check if we're at the end of the step
  if ((afg1_step_cnt >= StepWidth_1)) {
    // Sample and hold current step value into PreviousStep for next step slope computation
    afg1_prev_step_level = afg1_step_level;

    // Reset step width
    afg1_step_cnt = 0;

    // Resolve mode change for step end

    if ((afg1_mode == MODE_ADVANCE)) {
      // Stop after advance
      afg1_mode =  MODE_STOP;
    };

    if (steps[0][afg1_step_num].b.OpModeSTOP) {
      // Stop step
      afg1_prev_mode = afg1_mode;
      afg1_mode = MODE_STOP;
    };

    if (steps[0][afg1_step_num].b.OpModeENABLE
        && afg1_mode != MODE_WAIT_HI_Z)  {
      // Enable step, check start banana
      if ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 0)) {
        // Go into enable mode
        afg1_prev_mode = afg1_mode;
        afg1_mode = MODE_WAIT_HI_Z;
      };
    };

    if (steps[0][afg1_step_num].b.OpModeSUSTAIN
        && afg1_mode != MODE_STAY_HI_Z) {
      // Sustain step, check start banana
      if ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 1)) {
        // Go into sustain mode
        afg1_prev_mode = afg1_mode;
        afg1_mode = MODE_STAY_HI_Z;
        InitStart_1_SignalTimer();
      };
    };

    if (afg1_mode == MODE_RUN) {
      // Advance to the next step
      afg1_step_num = GetNextStep(0, afg1_step_num);
      doPulses = 1;
    };
  };


  if (afg1_mode == MODE_WAIT) {
    // Continuous step address mode. Check if the step has changed.
    if (afg1_step_num != (unsigned int) (afg1_stage_address)) {
      // Sample and hold current voltage output value
      afg1_prev_step_level = afg1_step_level;
      afg1_step_num = (unsigned int) (afg1_stage_address);
      // Reset step width
      afg1_step_cnt = 0;
      doPulses = 1;
    }
  };

  if (afg1_mode == MODE_WAIT_STROBE) {
    // What does this do? Is it ever in this mode?
    afg1_step_num = (unsigned int) (afg1_stage_address);
    afg1_mode = afg1_prev_mode;
  }

  if (display_mode == DISPLAY_MODE_VIEW_1) {
    display_update_flags.b.MainDisplay = 1;
    display_update_flags.b.StepsDisplay = 1;
  };

  // Now set output voltages
  // Compute the current step's programmed voltage output
  OutputVoltage = GetStepVoltage(0, afg1_step_num);

  // If the step is sloped, then slope from PreviousStep to the new output value
  if (steps[0][afg1_step_num].b.Sloped ) {
    if (afg1_prev_step_level >= OutputVoltage) {
      // Slope down
      deltaVoltage = (float) (afg1_prev_step_level - OutputVoltage) / StepWidth_1;
      OutputVoltage = afg1_prev_step_level - (unsigned int) (deltaVoltage * afg1_step_cnt);
    } else if (OutputVoltage > afg1_prev_step_level) {
      // Slope up
      deltaVoltage =  (float) (OutputVoltage - afg1_prev_step_level) / StepWidth_1;
      OutputVoltage = afg1_prev_step_level + (unsigned int) (deltaVoltage * afg1_step_cnt);
    }
  }

  // Set DAC channel 1 to AFG1 voltage out value
  afg1_step_level = OutputVoltage;
  DAC_SetChannel1Data(DAC_Align_12b_R, OutputVoltage);

  // Set AFG1 time out value
  MAX5135_DAC_send(MAX5135_DAC_CH_0, steps[0][afg1_step_num].b.TLevel >> 2);

  // Set AFG1 reference out value
  // TODO(maxl0rd): check that MODE_ADVANCE is being set correctly in every case and that JumpToStep1() is called
  if (afg1_mode == MODE_RUN || afg1_mode == MODE_ADVANCE) {
    // (Slopes down from 1023 to 0 over the course of the step)
    MAX5135_DAC_send(MAX5135_DAC_CH_1,
        1023 - (unsigned int) ((1023.0 / (float) StepWidth_1) * ((float) afg1_step_cnt)));
  } else {
    // No reference output when not running
    MAX5135_DAC_send(MAX5135_DAC_CH_1, 0);
  }

  // Now that output voltages are set, pulses can fire now
  if (doPulses) DoStepOutputPulses1();
};

// Keep duplicated logic in sync with above
void AfgTick2() {

  // TODO(maxl0rd): rename locals
  unsigned long int StepWidth_2 = 0;
  float deltaVoltage = 0.0;
  unsigned long OutputVoltage = 0;
  unsigned char doPulses = 0;

  StepWidth_2 = GetStepWidth(1, afg2_step_num);

  // TODO(maxl0rd): use precomputed calibration
  TIM5->PSC = (uint16_t) (
      ((((add_data[ADC_TIMEMULTIPLY_Ch_2])*3.5)
          / cal_constants[ADC_TIMEMULTIPLY_Ch_2])+0.5)
          * STEP_TIMER_PRESCALER);

  if (afg2_step_cnt < StepWidth_2) {
    afg2_step_cnt += 1;
  };

  if ((afg2_step_cnt >= StepWidth_2)) {
    afg2_prev_step_level = afg2_step_level;
    afg2_step_cnt = 0;

    if ((afg2_mode == MODE_ADVANCE)) {
      afg2_mode =  MODE_STOP;
    };

    if (steps[1][afg2_step_num].b.OpModeSTOP) {
      afg2_prev_mode = afg2_mode;
      afg2_mode = MODE_STOP;
    };

    if (steps[1][afg2_step_num].b.OpModeENABLE
        && afg2_mode != MODE_WAIT_HI_Z) {
      if ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 0)) {
        afg2_prev_mode = afg2_mode;
        afg2_mode = MODE_WAIT_HI_Z;
      };
    };

    if ((steps[1][afg2_step_num].b.OpModeSUSTAIN
        && afg2_mode != MODE_STAY_HI_Z)) {
      if ((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 1)) {
        afg2_prev_mode = afg2_mode;
        afg2_mode = MODE_STAY_HI_Z;
        InitStart_2_SignalTimer();
      }
    };

    if (afg2_mode == MODE_RUN) {
      afg2_step_num = GetNextStep(1, afg2_step_num);
      doPulses = 1;
    };
  }

  if (afg2_mode == MODE_WAIT) {
    if (afg2_step_num != (unsigned int) (afg2_stage_address)) {
      // Sample and hold current voltage output value
      afg2_prev_step_level = afg2_step_level;
      afg2_step_num = (unsigned int) (afg2_stage_address);
      // Reset step width
      afg2_step_cnt = 0;
      doPulses = 1;
    }
  };

  if (afg2_mode == MODE_WAIT_STROBE) {
    // What does this do?
    afg2_step_num = (unsigned int) (afg2_stage_address);
    afg2_mode = afg2_prev_mode;
  }

  if (display_mode == DISPLAY_MODE_VIEW_2) {
    display_update_flags.b.MainDisplay = 1;
    display_update_flags.b.StepsDisplay = 1;
  };

  OutputVoltage = GetStepVoltage(1, afg2_step_num);

  if (steps[1][afg2_step_num].b.Sloped ) {
    if (afg2_prev_step_level >= OutputVoltage) {
      deltaVoltage = (float) (afg2_prev_step_level - OutputVoltage) / StepWidth_2;
      OutputVoltage = afg2_prev_step_level - (unsigned int) (deltaVoltage * afg2_step_cnt);
    } else if (OutputVoltage > afg2_prev_step_level) {
      // Slope up
      deltaVoltage =  (float) (OutputVoltage - afg2_prev_step_level) / StepWidth_2;
      OutputVoltage = afg2_prev_step_level + (unsigned int) (deltaVoltage * afg2_step_cnt);
    }
  }

  afg2_step_level = OutputVoltage;
  DAC_SetChannel2Data(DAC_Align_12b_R, OutputVoltage);

  MAX5135_DAC_send(MAX5135_DAC_CH_2, steps[1][afg2_step_num].b.TLevel >> 2);

  // TODO(maxl0rd): check that MODE_ADVANCE is being set correctly in every case and that JumpToStep2() is called
  if (afg2_mode == MODE_RUN || afg2_mode == MODE_ADVANCE) {
    MAX5135_DAC_send(MAX5135_DAC_CH_3,
        1023 - (unsigned int) (((float) 0x3FF/ (float) StepWidth_2) * ((float) afg2_step_cnt)));
  } else {
    MAX5135_DAC_send(MAX5135_DAC_CH_3, 0);
  }

  // Now that output voltages are set, pulses can fire now
  if (doPulses) DoStepOutputPulses2();
};
