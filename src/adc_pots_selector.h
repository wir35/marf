#ifndef __ADC_POTS_SELECTOR_H_
#define __ADC_POTS_SELECTOR_H_

#include <stdint.h>

void AdcMuxGpioInitialize(void);

void AdcMuxResetAllOff(void);

// Advances the mux to the next pot and returns its index
uint8_t AdcMuxAdvance(uint8_t pot);
uint8_t AdcMuxAdvanceExpanded(uint8_t pot);

#endif
