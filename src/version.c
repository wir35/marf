#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>

#include "version.h"

void versionInit() {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Pin = GPIO_Pin_14;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_25MHz;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio_init);
}

unsigned char versionRevised() {
  return !GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
}


