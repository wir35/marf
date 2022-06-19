#include "leds_modes.h"

#include <stdint.h>
#include <string.h>
#include <stm32f4xx_gpio.h>

#include "delays.h"

#define LEDS_MODES_SHIFT_HIGH		GPIO_SetBits(GPIOC, GPIO_Pin_4)
#define LEDS_MODES_SHIFT_LOW		GPIO_ResetBits(GPIOC, GPIO_Pin_4)

#define LEDS_MODES_STORAGE_HIGH		GPIO_SetBits(GPIOC, GPIO_Pin_6)
#define LEDS_MODES_STORAGE_LOW		GPIO_ResetBits(GPIOC, GPIO_Pin_6)

#define LEDS_MODES_DATA_HIGH		GPIO_SetBits(GPIOC, GPIO_Pin_5)
#define LEDS_MODES_DATA_LOW			GPIO_ResetBits(GPIOC, GPIO_Pin_5)

void LEDS_modes_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitStructure));
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

inline static void leds_modes_send_byte(uint8_t data) {
	uint8_t dat = data;

	for (uint8_t cnt = 0; cnt < 8; cnt++) {
		if ((dat & 0x80) > 0) {
			LEDS_MODES_DATA_HIGH;
		} else {
			LEDS_MODES_DATA_LOW;
		}
		LEDS_MODES_SHIFT_LOW;		
		DELAY_NOPS_120NS();
		LEDS_MODES_SHIFT_HIGH;
		DELAY_NOPS_120NS(); 
		dat = dat << 1;
	}
	LEDS_MODES_DATA_LOW;
}

void LEDS_modes_SendStruct(uLeds *_Leds) {
	LEDS_MODES_STORAGE_LOW;
	leds_modes_send_byte(_Leds->value[0]);
	leds_modes_send_byte(_Leds->value[1]);
	leds_modes_send_byte(_Leds->value[2]);
	leds_modes_send_byte(_Leds->value[3]);

	LEDS_MODES_STORAGE_HIGH;
}
