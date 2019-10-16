#include <stm32f4xx_gpio.h>

#include <stdlib.h>
#include <string.h>
#include "leds_modes.h"

#define LEDS_MODES_SHIFT_HIGH		GPIO_SetBits(GPIOC, GPIO_Pin_4)
#define LEDS_MODES_SHIFT_LOW		GPIO_ResetBits(GPIOC, GPIO_Pin_4)

#define LEDS_MODES_STORAGE_HIGH		GPIO_SetBits(GPIOC, GPIO_Pin_6)
#define LEDS_MODES_STORAGE_LOW		GPIO_ResetBits(GPIOC, GPIO_Pin_6)

#define LEDS_MODES_DATA_HIGH		GPIO_SetBits(GPIOC, GPIO_Pin_5)
#define LEDS_MODES_DATA_LOW			GPIO_ResetBits(GPIOC, GPIO_Pin_5)

void LEDS_modes_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Setting up peripherial */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitStructure));
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void LEDS_modes_SendByte(unsigned char data)
{
	unsigned char dat, cnt;
	dat = data;
	for(cnt=0; cnt<8; cnt++)
	{
		if ((dat & 0x80) > 0) {
			LEDS_MODES_DATA_HIGH;
		} else {
			LEDS_MODES_DATA_LOW;
		}
		
		LEDS_MODES_SHIFT_LOW;		
		LEDS_MODES_SHIFT_HIGH;

		dat = dat << 1;
	}
	LEDS_MODES_DATA_LOW;
	
}

void LEDS_modes_SendDWord(unsigned long int data)
{
	LEDS_modes_SendByte( data&0x000000FF);
	LEDS_modes_SendByte((data&0x0000FF00)>>8);
	LEDS_modes_SendByte((data&0x00FF0000)>>16);
	LEDS_modes_SendByte((data&0xFF000000)>>24);

	LEDS_MODES_STORAGE_LOW;
	LEDS_MODES_STORAGE_HIGH;
}


void LEDS_modes_SendStruct(uLeds *_Leds)
{
	LEDS_modes_SendByte(_Leds->value[0]);
	LEDS_modes_SendByte(_Leds->value[1]);
	LEDS_modes_SendByte(_Leds->value[2]);
	LEDS_modes_SendByte(_Leds->value[3]);

	LEDS_MODES_STORAGE_LOW;
	LEDS_MODES_STORAGE_HIGH;
}
