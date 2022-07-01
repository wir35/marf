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

uint32_t steps_leds_lit = 0xFFFFFFFF;

uLeds mode_leds_lit;

void DisplayAllInitialize() {
  steps_leds_lit = 0xFFFFFFFF;
  mode_leds_lit.value[0] = 0xFF;
  mode_leds_lit.value[1] = 0xFF;
  mode_leds_lit.value[2] = 0xFF;
  mode_leds_lit.value[3] = 0xFF;
  mode_leds_lit.b.Seq1Stop = 0;
  mode_leds_lit.b.Seq2Stop = 0;

  display_update_flags.value = 0x00;
  display_update_flags.b.MainDisplay  = 1;
  display_update_flags.b.StepsDisplay = 1;

  display_mode = DISPLAY_MODE_VIEW_1;

  if (Is_Expander_Present()) {
    LED_STEP_SendWordExpanded(steps_leds_lit);
  } else {
    LED_STEP_SendWord(steps_leds_lit);
  }
  LEDS_modes_SendStruct(&mode_leds_lit);
}

// Update the led data for view and edit modes
void UpdateLedsProgramMode(uLeds* mLeds, uStep* step) {
  mLeds->b.VoltageFull   &= ~step->b.FullRange;
  mLeds->b.Voltage0      &= ~step->b.Voltage0;
  mLeds->b.Voltage2      &= ~step->b.Voltage2;
  mLeds->b.Voltage4      &= ~step->b.Voltage4;
  mLeds->b.Voltage6      &= ~step->b.Voltage6;
  mLeds->b.Voltage8      &= ~step->b.Voltage8;
  if (swapped_pulses) {
    mLeds->b.Pulse1        &= ~step->b.OutputPulse2;
    mLeds->b.Pulse2        &= ~step->b.OutputPulse1;
  } else {
    mLeds->b.Pulse1        &= ~step->b.OutputPulse1;
    mLeds->b.Pulse2        &= ~step->b.OutputPulse2;
  }
  mLeds->b.CycleFirst    &= ~step->b.CycleFirst;
  mLeds->b.CycleLast     &= ~step->b.CycleLast;
  mLeds->b.VoltageSource &= ~step->b.VoltageSource;
  mLeds->b.Integration   &= ~step->b.Sloped;
  mLeds->b.Quantization  &= ~step->b.Quantize;
  mLeds->b.TimeRange0    &= ~step->b.TimeRange_p03;
  mLeds->b.TimeRange1    &= ~step->b.TimeRange_p3;
  mLeds->b.TimeRange2    &= ~step->b.TimeRange_3;
  mLeds->b.TimeRange3    &= ~step->b.TimeRange_30;
  mLeds->b.TimeSource    &= ~step->b.TimeSource;
  mLeds->b.OPStop        &= ~step->b.OpModeSTOP;
  mLeds->b.OPSustain     &= ~step->b.OpModeSUSTAIN;
  mLeds->b.OPEnable      &= ~step->b.OpModeENABLE;
}

// Update mode and programming LEDs
void UpdateModeSectionLeds() {

  // AFG1 mode LEDs
  if (afg1_mode == MODE_RUN || afg1_mode == MODE_ADVANCE) {
    mode_leds_lit.b.Seq1Run &= 0;
  } else if (afg1_mode == MODE_WAIT  || afg1_mode == MODE_WAIT_HI_Z || afg1_mode == MODE_STAY_HI_Z) {
    mode_leds_lit.b.Seq1Wait &= 0;
  } else if (afg1_mode == MODE_STOP) {
    mode_leds_lit.b.Seq1Stop &= 0;
  };

  // AFG2 mode LEDs
  if (afg2_mode == MODE_RUN || afg2_mode == MODE_ADVANCE) {
    mode_leds_lit.b.Seq2Run &= 0;
  } else if (afg2_mode == MODE_WAIT || afg2_mode == MODE_WAIT_HI_Z  || afg2_mode == MODE_STAY_HI_Z) {
    mode_leds_lit.b.Seq2Wait &= 0;
  } else if (afg2_mode == MODE_STOP) {
    mode_leds_lit.b.Seq2Stop &= 0;
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
    led_step = get_step_programming(afg1_section, afg1_step_num);
    UpdateLedsProgramMode(&mode_leds_lit, &led_step);
    break;
  case DISPLAY_MODE_EDIT_1:
    led_step = get_step_programming(edit_mode_section, edit_mode_step_num);
    UpdateLedsProgramMode(&mode_leds_lit, &led_step);
    break;
  case DISPLAY_MODE_VIEW_2:
    led_step = get_step_programming(afg2_section, afg2_step_num);
    UpdateLedsProgramMode(&mode_leds_lit, &led_step);
    break;
  case DISPLAY_MODE_EDIT_2:
    led_step = get_step_programming(edit_mode_section, edit_mode_step_num);
    UpdateLedsProgramMode(&mode_leds_lit, &led_step);
    break;
  }
}

// Mark the led lit/dirty and it will be lit in the next shift
void UpdateStepSectionLeds() {
  if ( display_mode == DISPLAY_MODE_VIEW_1 ) {
    steps_leds_lit &= ~(1UL << afg1_step_num);
  };
  if ( display_mode == DISPLAY_MODE_VIEW_2 ) {
    steps_leds_lit &= ~(1UL << afg2_step_num);
  };
  if ( ( display_mode == DISPLAY_MODE_EDIT_1 ) ||
      ( display_mode == DISPLAY_MODE_EDIT_2 )) {
    steps_leds_lit &= ~(1UL << edit_mode_step_num);
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

// Actually shift the lit leds out via the two shift registers
// and then reset everything.
void FlushLedUpdates() {
  UpdateModeSectionLeds();
  UpdateStepSectionLeds();

  if (Is_Expander_Present()) {
    LED_STEP_SendWordExpanded(steps_leds_lit);
  } else {
    LED_STEP_SendWord(steps_leds_lit);
  }
  steps_leds_lit = 0xFFFFFFFF;

  LEDS_modes_SendStruct(&mode_leds_lit);
  mode_leds_lit.value[0] = 0xFF;
  mode_leds_lit.value[1] = 0xFF;
  mode_leds_lit.value[2] = 0xFF;
  mode_leds_lit.value[3] = 0xFF;
}

// Run/wait/stop leds cycling during calibration
// Called every 10ms
void RunCalibrationAnimation() {
  static uint16_t counter = 0x0000;

  if (counter < 20) {
    mode_leds_lit.b.Seq1Run = 1;
    mode_leds_lit.b.Seq1Wait = 1;
    mode_leds_lit.b.Seq1Stop = 0;
    mode_leds_lit.b.Seq2Run = 1;
    mode_leds_lit.b.Seq2Wait = 1;
    mode_leds_lit.b.Seq2Stop = 0;
  } else if (counter < 40) {
    mode_leds_lit.b.Seq1Run = 0;
    mode_leds_lit.b.Seq1Wait = 1;
    mode_leds_lit.b.Seq1Stop = 1;
    mode_leds_lit.b.Seq2Run = 0;
    mode_leds_lit.b.Seq2Wait = 1;
    mode_leds_lit.b.Seq2Stop = 1;
  } else if (counter < 60) {
    mode_leds_lit.b.Seq1Run = 1;
    mode_leds_lit.b.Seq1Wait = 0;
    mode_leds_lit.b.Seq1Stop = 1;
    mode_leds_lit.b.Seq2Run = 1;
    mode_leds_lit.b.Seq2Wait = 0;
    mode_leds_lit.b.Seq2Stop = 1;
  } else {
    counter = 0;
  }

  mode_leds_lit.b.Pulse1 = swapped_pulses;
  mode_leds_lit.b.Pulse2 = ~swapped_pulses;

  LEDS_modes_SendStruct(&mode_leds_lit);
  counter += 1;
}

void RunSaveProgramAnimation() {
  steps_leds_lit = 0xFFFF;
  for (uint8_t l = 0; l < 16; l++) {
    LED_STEP_SendWord(steps_leds_lit);
    steps_leds_lit &= ~(1UL << l);
    delay_ms(10);
  }
}

void RunLoadProgramAnimation() {
  steps_leds_lit = 0xFFFF;
  for (uint8_t l = 15; l > 0; l--) {
    LED_STEP_SendWord(steps_leds_lit);
    steps_leds_lit &= ~(1UL << l);
    delay_ms(10);
  }
}

void StepLedsLightSingleStep(uint8_t step) {
  steps_leds_lit = 0xFFFF;
  steps_leds_lit &= ~(1UL << step);
  LED_STEP_SendWord(steps_leds_lit);
}

// Called approx every 15ms
// Just toggles pulse leds while waiting for step selection.

void RunWaitingLoadSaveAnimation() {
  static uint8_t counter = 0;

  mode_leds_lit.value[0] = 0xFF;
  mode_leds_lit.value[1] = 0xFF;
  mode_leds_lit.value[2] = 0xFF;
  mode_leds_lit.value[3] = 0xFF;

  if (counter < 3) {
    mode_leds_lit.b.Pulse1 = 1;
    mode_leds_lit.b.Pulse2 = 0;
    counter += 1;
  } else if (counter < 6) {
    mode_leds_lit.b.Pulse1 = 0;
    mode_leds_lit.b.Pulse2 = 1;
    counter += 1;
  } else {
    counter = 0;
  }
  LEDS_modes_SendStruct(&mode_leds_lit);
}
