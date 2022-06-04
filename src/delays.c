#include "delays.h"

#include <stm32f4xx.h>

// Systick
volatile uint32_t millis;

void systickInit(uint16_t frequency) {
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  (void) SysTick_Config(RCC_Clocks.HCLK_Frequency / frequency);
}

void SysTick_Handler (void) {
  millis++;
}

void delay_ms(unsigned int ms)
{
  volatile uint32_t nCount;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq (&RCC_Clocks);

  nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
  for (; nCount!=0; nCount--);
}

void delay_us(unsigned int us)
{
  volatile uint32_t nCount;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq (&RCC_Clocks);

  nCount=(RCC_Clocks.HCLK_Frequency/10000000)*us;
  for (; nCount!=0; nCount--);
}

void delay_ns(unsigned int ns)
{
  volatile uint32_t nCount;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq (&RCC_Clocks);

  nCount=(RCC_Clocks.HCLK_Frequency/10000000000)*ns;
  for (; nCount!=0; nCount--);
}
