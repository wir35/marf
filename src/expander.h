#ifndef __EXPANDER_H_
#define __EXPANDER_H_

#include <stm32f4xx.h>

#define EXPANDER_PIN GPIO_Pin_14
#define EXPANDER_GPIO GPIOB

extern uint8_t has_expander;

// Returns 1 if expander is connected
inline uint8_t Is_Expander_Present(void) {
  return has_expander;
}

// Reads the DIP setting and stores whether expander is present.
// Must restart module after adding expander. Good idea anyway...
void Init_Expander(void);

#endif
