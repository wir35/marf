#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <stm32f4xx.h>

#include "HC165.h"
#include "afg.h"

typedef struct  {
  uint8_t adc_pot_sel;
  uint8_t adc_mux_shift_out;
  uint8_t inhibit_adc;
  uint8_t afg1_tick;
  ProgrammedOutputs afg1_outputs;
  uint8_t afg2_tick;
  ProgrammedOutputs afg2_outputs;
} ControllerJobFlags;

extern volatile uint8_t edit_mode_step_num;
extern volatile uint8_t edit_mode_section;

extern volatile ControllerJobFlags controller_job_flags;

#define KEY_DEBOUNCE_COUNT 3  // 3 bounces
#define KEY_TIMER 5  // scan switches every 5ms

#define LONG_COUNTER_TICKS  30;
#define SHORT_COUNTER_TICKS 10;
#define SCROLL_WAIT_COUNTER 120;

void ControllerMainLoop();

void ControllerApplyProgrammingSwitches(uButtons * key);

void ControllerProcessStageAddressSwitches(uButtons * key);

void ControllerProcessNavigationSwitches(uButtons* key);

void ControllerCheckClear();



#endif
