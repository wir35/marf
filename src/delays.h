#ifndef _DELAYS_H
#define _DELAYS_H

#include <stm32f4xx.h>

#define DELAY_NOPS_120NS() ({asm("nop"); asm("nop");asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");})

#define DELAY_CLOCK_10() ({ for (uint32_t d = 10; d != 0; d--) { asm("nop");} })
#define DELAY_CLOCK_20() ({ for (uint32_t d = 20; d != 0; d--) { asm("nop");} })

extern volatile uint32_t millis;

inline uint32_t get_millis() {
  return millis;
}

void delay_ms(unsigned int ms);

void delay_us(unsigned int us);

void systickInit(uint16_t frequency);

void SysTick_Handler (void);

#endif
