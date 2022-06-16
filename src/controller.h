#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <stm32f4xx.h>

#include "HC165.h"

extern volatile uint8_t edit_mode_step_num;
extern volatile uint8_t edit_mode_section;

#define KEY_DEBOUNCE_COUNT 3  // 3 bounces
#define KEY_TIMER 5  // scan switches every 5ms

#define LONG_COUNTER_TICKS 20;
#define SHORT_COUNTER_TICKS 3;

void ControllerMainLoop();

void ControllerApplyProgrammingSwitches(uButtons * key);

void ControllerProcessStageAddressSwitches(uButtons * key);

void ControllerProcessNavigationSwitches(uButtons* key);

void ControllerCheckClear();

#endif
