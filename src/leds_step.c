#include <stm32f4xx_gpio.h>
#include <stdlib.h>
#include <string.h>
#include "leds_step.h"
#include "expander.h"
#include "delays.h"

#define LED_STEP_SHIFT_HIGH			GPIO_SetBits(GPIOC, GPIO_Pin_7)
#define LED_STEP_SHIFT_LOW			GPIO_ResetBits(GPIOC, GPIO_Pin_7)

#define LED_STEP_STORAGE_HIGH	  GPIO_SetBits(GPIOC, GPIO_Pin_9)
#define LED_STEP_STORAGE_LOW	  GPIO_ResetBits(GPIOC, GPIO_Pin_9)

#define LED_STEP_DATA_HIGH			GPIO_SetBits(GPIOC, GPIO_Pin_8)
#define LED_STEP_DATA_LOW				GPIO_ResetBits(GPIOC, GPIO_Pin_8)

/* Init GPIO for LEDs control via HC595 shift registers */
void LED_STEP_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitStructure));
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100mhz default
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;	 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

inline static void led_step_send_byte(uint8_t data) {
	uint8_t dat = data;

	for(uint8_t cnt = 0; cnt < 8; cnt++) {
		if ((dat & 0x80) > 0) {
			LED_STEP_DATA_HIGH;
		} else {
			LED_STEP_DATA_LOW;
		}
		LED_STEP_SHIFT_LOW;
		DELAY_NOPS_120NS(); 
		LED_STEP_SHIFT_HIGH;
		DELAY_NOPS_120NS(); 
		dat = dat << 1;
	}
	LED_STEP_DATA_LOW;
}

void LED_STEP_SendWord(uint16_t data) {
	LED_STEP_STORAGE_LOW;

	led_step_send_byte((uint8_t) (data >>8));
	led_step_send_byte((uint8_t) (data & 0x00FF));

	LED_STEP_STORAGE_HIGH;
}

void LED_STEP_SendWordExpanded(uint32_t dat) {

  LED_STEP_STORAGE_LOW;
  led_step_send_byte((uint8_t) (dat >> 16)); // Verify if these two expander sections are reversed
  led_step_send_byte((uint8_t) (dat >> 24)); // Seems backwards
  led_step_send_byte((uint8_t) (dat >> 8));
  led_step_send_byte((uint8_t) (dat) );

  LED_STEP_STORAGE_HIGH;
}

