#ifndef LEDS_STEP_H_
#define LEDS_STEP_H_

#include <stdint.h>

void LED_STEP_init(void);

void LED_STEP_SendWord(uint16_t data);

void LED_STEP_SendWordExpanded(uint32_t data);

#endif /* LEDS_STEP_H_ */
