#include <stm32f4xx_gpio.h>
#include <stdlib.h>
#include <string.h>
#include "leds_step.h"
#include "expander.h"

#define LED_STEP_SHIFT_HIGH			GPIO_SetBits(GPIOC, GPIO_Pin_7)
#define LED_STEP_SHIFT_LOW			GPIO_ResetBits(GPIOC, GPIO_Pin_7)

#define LED_STEP_STORAGE_HIGH	  GPIO_SetBits(GPIOC, GPIO_Pin_9)
#define LED_STEP_STORAGE_LOW	  GPIO_ResetBits(GPIOC, GPIO_Pin_9)

#define LED_STEP_DATA_HIGH			GPIO_SetBits(GPIOC, GPIO_Pin_8)
#define LED_STEP_DATA_LOW				GPIO_ResetBits(GPIOC, GPIO_Pin_8)

/*Init GPIO for LEDs control via HC595 shift registers*/
void LED_STEP_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Setting up peripherial */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitStructure));
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;	 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*Shift one byte to HC595 registers which control LEDs*/
void LED_STEP_SendByte(unsigned char data)
{
	unsigned char dat, cnt;
	dat = data;
	for(cnt=0; cnt<8; cnt++)
	{
		if ((dat & 0x80) > 0) {
			LED_STEP_DATA_HIGH;
		} else {
			LED_STEP_DATA_LOW;
		}

		LED_STEP_SHIFT_LOW;
		LED_STEP_SHIFT_HIGH;

		dat = dat << 1;
	}
	LED_STEP_DATA_LOW;
}

/*Shift two bytes to HC595 registers which control LEDs*/
void LED_STEP_SendWord(unsigned long int data)
{
	LED_STEP_SendByte((unsigned char) ((data&0xFF00)>>8) );
	LED_STEP_SendByte((unsigned char) (data&0x00FF) );

	LED_STEP_STORAGE_LOW;
	LED_STEP_STORAGE_HIGH;
}

/*Turn on the LED which indicates the step number StepNum*/
void LED_STEP_LightStep(unsigned int StepNum)
{
	unsigned long dat = 0xFFFFFFFF;
	unsigned char cnt, tmp1, tmp2;
	

	dat &= ~(1<<0);
	for(cnt=0;cnt<StepNum;cnt++)
	{		
		dat = dat<<1;
		dat |= (1<<0);		
	};
		
	if(!Is_Expander_Present())
	{	
		LED_STEP_SendWord(dat & 0xFFFF);
	}
	else
	{
		//if expander is presented we should control 32 LEDs instead of 16
		tmp1 = dat >> 24;
		tmp2 = dat >> 16;
		
		LED_STEP_SendByte((unsigned char) (tmp2) );
		LED_STEP_SendByte((unsigned char) (tmp1) );
		LED_STEP_SendByte((unsigned char) (dat >> 8) );
		LED_STEP_SendByte((unsigned char) (dat) );
		
		LED_STEP_STORAGE_LOW;
		LED_STEP_STORAGE_HIGH;
		
	}
};

