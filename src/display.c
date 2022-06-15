#include "display.h"

#include <stm32f4xx.h>

#include "leds_modes.h"
#include "leds_step.h"
#include "program.h"
#include "afg.h"
#include "controller.h"

// Flags which can be set by anything to update the led display
volatile uDisplayUpdateFlag display_update_flags;

// Current display mode state
volatile uint8_t display_mode = DISPLAY_MODE_VIEW_1;

// Do the pulse LEDs need to be swapped?
uint8_t swapped_pulses = 0;

uint32_t save_counter = 0, load_counter = 0;

// Update the led data for view and edit modes
void UpdateLedsProgramMode(uLeds* mLeds, uStep* step) {
  mLeds->b.VoltageFull   = ~step->b.FullRange;
  mLeds->b.Voltage0      = ~step->b.Voltage0;
  mLeds->b.Voltage2      = ~step->b.Voltage2;
  mLeds->b.Voltage4      = ~step->b.Voltage4;
  mLeds->b.Voltage6      = ~step->b.Voltage6;
  mLeds->b.Voltage8      = ~step->b.Voltage8;
  if (swapped_pulses) {
    mLeds->b.Pulse1        = ~step->b.OutputPulse2;
    mLeds->b.Pulse2        = ~step->b.OutputPulse1;
  } else {
    mLeds->b.Pulse1        = ~step->b.OutputPulse1;
    mLeds->b.Pulse2        = ~step->b.OutputPulse2;
  }
  mLeds->b.CycleFirst    = ~step->b.CycleFirst;
  mLeds->b.CycleLast     = ~step->b.CycleLast;
  mLeds->b.VoltageSource = ~step->b.VoltageSource;
  mLeds->b.Integration   = ~step->b.Sloped;
  mLeds->b.Quantization  = ~step->b.Quantize;
  mLeds->b.TimeRange0    = ~step->b.TimeRange_p03;
  mLeds->b.TimeRange1    = ~step->b.TimeRange_p3;
  mLeds->b.TimeRange2    = ~step->b.TimeRange_3;
  mLeds->b.TimeRange3    = ~step->b.TimeRange_30;
  mLeds->b.TimeSource    = ~step->b.TimeSource;
  mLeds->b.OPStop        = ~step->b.OpModeSTOP;
  mLeds->b.OPSustain     = ~step->b.OpModeSUSTAIN;
  mLeds->b.OPEnable      = ~step->b.OpModeENABLE;
}

// Update mode and programming LEDs
void UpdateModeSectionLeds() {
  uLeds mLeds;

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

  uStep led_step;

  switch (display_mode) {
  case DISPLAY_MODE_VIEW_1:
    led_step = get_step_programming(0, afg1_step_num);
    UpdateLedsProgramMode(&mLeds, &led_step);
    UpdateStepSection(afg1_step_num);
  case DISPLAY_MODE_EDIT_1:
    led_step = get_step_programming(edit_mode_section, edit_mode_step_num);
    UpdateLedsProgramMode(&mLeds, &led_step);
    UpdateStepSection(edit_mode_step_num);
    break;
  case DISPLAY_MODE_VIEW_2:
    led_step = get_step_programming(1, afg2_step_num);
    UpdateLedsProgramMode(&mLeds, &led_step);
    UpdateStepSection(afg2_step_num);
  case DISPLAY_MODE_EDIT_2:
    led_step = get_step_programming(edit_mode_section, edit_mode_step_num);
    UpdateLedsProgramMode(&mLeds, &led_step);
    UpdateStepSection(edit_mode_step_num);
    break;
  case DISPLAY_MODE_SAVE_1:
  case DISPLAY_MODE_LOAD_1:
  case DISPLAY_MODE_SAVE_2:
  case DISPLAY_MODE_LOAD_2:
    break;
  }

  // And send out the led data
  LEDS_modes_SendStruct(&mLeds);
}

void UpdateStepSection() {
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
  }
}

void RunClearAnimation() {
  if (Is_Expander_Present()) {
    for (uint8_t i = 0; i < 8; i++) {
      LED_STEP_SendWordExpanded(0x00000000);
      delay_ms(60);
      LED_STEP_SendWordExpanded(0xFFFFFFFF);
      delay_ms(60);
    }
  } else {
    for (uint8_t i = 0; i < 8; i++) {
      LED_STEP_SendWord(0x0000);
      delay_ms(60);
      LED_STEP_SendWord(0xFFFF);
      delay_ms(60);
    }
  }
}
