#include "display.h"

#include <stm32f4xx.h>

#include "leds_modes.h"
#include "leds_step.h"
#include "program.h"
#include "afg.h"

// Flags which can be set by anything to update the led display
volatile uDisplayUpdateFlag display_update_flags;

// Current display mode state
volatile uint8_t display_mode = DISPLAY_MODE_VIEW_1;

// Do the pulse LEDs need to be swapped?
uint8_t swapped_pulses = 0;

volatile uint32_t save_counter = 0, load_counter = 0;


// Update mode and programming LEDs
void UpdateModeSectionLeds(uint8_t edit_mode_step_num, uint8_t bank) {
  uint8_t step_num = 0, section = 0;
  uLeds mLeds;
  uStep* mStep;

  // Initialize all leds off
  // Low value (0) indicates an LED is lit.
  mLeds.value[0] = 0xFF;
  mLeds.value[1] = 0xFF;
  mLeds.value[2] = 0xFF;
  mLeds.value[3] = 0xFF;

  // AFG1 mode LEDs
  if (afg1_mode == MODE_RUN || afg1_mode == MODE_ADVANCE) {
    mLeds.b.Seq1Run = 0;
  } else if (afg1_mode == MODE_WAIT  || afg1_mode == MODE_WAIT_HI_Z || afg1_mode == MODE_STAY_HI_Z) {
    mLeds.b.Seq1Wait = 0;
  } else if (afg1_mode == MODE_STOP) {
    mLeds.b.Seq1Stop = 0;
  };

  // AFG2 mode LEDs
  if (afg2_mode == MODE_RUN || afg2_mode == MODE_ADVANCE) {
    mLeds.b.Seq2Run = 0;
  } else if (afg2_mode == MODE_WAIT || afg2_mode == MODE_WAIT_HI_Z  || afg2_mode == MODE_STAY_HI_Z) {
    mLeds.b.Seq2Wait = 0;
  } else if (afg2_mode == MODE_STOP) {
    mLeds.b.Seq2Stop = 0;
  };

  if ((display_mode == DISPLAY_MODE_VIEW_1) || (display_mode == DISPLAY_MODE_EDIT_1) ) {
    DISPLAY_LED_I_ON;
    DISPLAY_LED_II_OFF;
  } else if ((display_mode == DISPLAY_MODE_VIEW_2) || (display_mode == DISPLAY_MODE_EDIT_2) ) {
    DISPLAY_LED_II_ON;
    DISPLAY_LED_I_OFF;
  };

  // Determine step for different DisplayModes
  if ( display_mode == DISPLAY_MODE_VIEW_1 ) {
    step_num = afg1_step_num;
    section = 0;
  } else if ( display_mode == DISPLAY_MODE_VIEW_2 ) {
    step_num = afg2_step_num;
    section = 1;
  } else if ( display_mode == DISPLAY_MODE_EDIT_1 ) {
    step_num = edit_mode_step_num;
    section = 0;
  } else if ( display_mode == DISPLAY_MODE_EDIT_2 ) {
    step_num = edit_mode_step_num;
    section = 1;
  };

  mStep = (uStep*) &steps[section][step_num];

  mLeds.b.VoltageFull   = ~mStep->b.FullRange;
  mLeds.b.Voltage0      = ~mStep->b.Voltage0;
  mLeds.b.Voltage2      = ~mStep->b.Voltage2;
  mLeds.b.Voltage4      = ~mStep->b.Voltage4;
  mLeds.b.Voltage6      = ~mStep->b.Voltage6;
  mLeds.b.Voltage8      = ~mStep->b.Voltage8;
  if (swapped_pulses) {
    mLeds.b.Pulse1        = ~mStep->b.OutputPulse2;
    mLeds.b.Pulse2        = ~mStep->b.OutputPulse1;
  } else {
    mLeds.b.Pulse1        = ~mStep->b.OutputPulse1;
    mLeds.b.Pulse2        = ~mStep->b.OutputPulse2;
  }
  mLeds.b.CycleFirst    = ~mStep->b.CycleFirst;
  mLeds.b.CycleLast     = ~mStep->b.CycleLast;
  mLeds.b.VoltageSource = ~mStep->b.VoltageSource;
  mLeds.b.Integration   = ~mStep->b.Sloped;
  mLeds.b.Quantization  = ~mStep->b.Quantize;
  mLeds.b.TimeRange0    = ~mStep->b.TimeRange_p03;
  mLeds.b.TimeRange1    = ~mStep->b.TimeRange_p3;
  mLeds.b.TimeRange2    = ~mStep->b.TimeRange_3;
  mLeds.b.TimeRange3    = ~mStep->b.TimeRange_30;
  mLeds.b.TimeSource    = ~mStep->b.TimeSource;
  mLeds.b.OPStop        = ~mStep->b.OpModeSTOP;
  mLeds.b.OPSustain     = ~mStep->b.OpModeSUSTAIN;
  mLeds.b.OPEnable      = ~mStep->b.OpModeENABLE;

  // TODO(maxl0rd): Move different display modes into different functions yea

  if ((display_mode == DISPLAY_MODE_SAVE_1) || (display_mode == DISPLAY_MODE_SAVE_2) ||
      (display_mode == DISPLAY_MODE_LOAD_1) || (display_mode == DISPLAY_MODE_LOAD_2)) {
    mLeds.value[0] = 0xFF;
    mLeds.value[1] = 0xFF;
    mLeds.value[2] = 0xFF;
    mLeds.value[3] = 0xFF;

    if((display_mode == DISPLAY_MODE_SAVE_1) || (display_mode == DISPLAY_MODE_SAVE_2))
    {
      mLeds.b.Seq2Wait = 1;
      save_counter++;
      if(save_counter < 1500)
      {
        mLeds.b.Seq1Wait = 0;
      }
      else if(save_counter < 3000)
      {
        mLeds.b.Seq1Wait = 1;
      }
      else save_counter = 0;
    }
    else if((display_mode == DISPLAY_MODE_LOAD_1) || (display_mode == DISPLAY_MODE_LOAD_2))
    {
      mLeds.b.Seq1Wait = 1;
      load_counter++;
      if(load_counter < 1500)
      {
        mLeds.b.Seq2Wait = 0;
      }
      else if(load_counter < 3000)
      {
        mLeds.b.Seq2Wait = 1;
      }
      else load_counter = 0;


    }

    if(!Is_Expander_Present())
    {
      if(bank == 1) mLeds.value[0] &= ~(1 << 6);
      else mLeds.value[0] &= ~(1 << 7);
    }
  };

  LEDS_modes_SendStruct(&mLeds);
};

// Update steps leds
void UpdateStepSection(uint8_t edit_mode_step_num)
{
  if ( display_mode == DISPLAY_MODE_VIEW_1 ) {
    LED_STEP_LightStep(afg1_step_num);
  };
  if ( display_mode == DISPLAY_MODE_VIEW_2 ) {
    LED_STEP_LightStep(afg2_step_num);
  };
  if ( ( display_mode == DISPLAY_MODE_EDIT_1 ) ||
      ( display_mode == DISPLAY_MODE_EDIT_2 ) ||
      ( display_mode == DISPLAY_MODE_SAVE_1 ) ||
      ( display_mode == DISPLAY_MODE_SAVE_2 ) ||
      (display_mode == DISPLAY_MODE_LOAD_1) ||
      (display_mode == DISPLAY_MODE_LOAD_2)
  ) {
    LED_STEP_LightStep(edit_mode_step_num);
  };
};
