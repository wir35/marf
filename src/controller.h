#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <stm32f4xx.h>

#include "HC165.h"

extern volatile uint8_t edit_mode_step_num;
extern volatile uint8_t edit_mode_section;


// Variable used for key lock during the VIEW_MODE key changes steps options
extern volatile uint8_t key_locked;
extern volatile uint8_t keys_not_valid;

// Current patches bank
volatile uint8_t bank;

uint16_t counterL;
uint16_t counterR;

inline uint8_t get_edit_mode_step_num() {
  return edit_mode_step_num + (edit_mode_section << 4);
}

void ControllerProcessSwitches(uButtons* key);

void ControllerCheckClear();

#endif
