#include <stdlib.h>
#include <string.h>
#include <stm32f4xx_gpio.h>
#include "HC165.h"

#define SW_DAT	GPIO_Pin_0
#define SW_CP		GPIO_Pin_1
#define SW_CE		GPIO_Pin_2
#define SW_PL		GPIO_Pin_3

/* SW_CP PIN */
#define CLK_HIGH	GPIO_SetBits(GPIOC, SW_CP)
#define CLK_LOW		GPIO_ResetBits(GPIOC, SW_CP)

/* SW_PL PIN*/
#define SS_HIGH		GPIO_SetBits(GPIOC, SW_PL)
#define SS_LOW		GPIO_ResetBits(GPIOC, SW_PL)

/* SW_CE PIN*/
#define CE_HIGH		GPIO_SetBits(GPIOC, SW_CE)
#define CE_LOW		GPIO_ResetBits(GPIOC, SW_CE)

#define _BV(x)  (1<<x);

///Setting up PORTs IO
void init_HC165(void)
{
	/* init pins struct*/
	GPIO_InitTypeDef mGPIO_InitStructure;
	GPIO_InitTypeDef mGPIO_InitStructure2;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	memset(&mGPIO_InitStructure, 0, sizeof(mGPIO_InitStructure));
	mGPIO_InitStructure.GPIO_Pin 		= SW_CP|SW_PL|SW_CE;
	mGPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	mGPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	mGPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	mGPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	
	GPIO_Init(GPIOC, &mGPIO_InitStructure);
	
	memset(&mGPIO_InitStructure2, 0, sizeof(mGPIO_InitStructure2));
	mGPIO_InitStructure2.GPIO_Pin 		= SW_DAT;
	mGPIO_InitStructure2.GPIO_Mode 		= GPIO_Mode_IN;
	
	GPIO_Init(GPIOC, &mGPIO_InitStructure2);
	
	SS_HIGH;
	CE_LOW;
	CLK_LOW;
}

//Close latch and load data to internal registers 165
void HC165_LatchUp(void)
{
	CE_LOW;
	CLK_LOW;

	SS_LOW;
	delay_us(1);
	SS_HIGH;	
}

unsigned char HC165_GetByte(void)
{
	unsigned char data = 0x00, cnt = 0;	

	if (  GPIO_ReadInputDataBit(GPIOC, SW_DAT) == 1 ) {
		data = data | (0x01);
	};

	CLK_HIGH;	

	for(cnt=0; cnt<7;cnt++)
	{
		CLK_LOW;
		delay_us(1);

		data = data << 1;		
		if (  GPIO_ReadInputDataBit(GPIOC, SW_DAT) == 1 ) {
			data = data | (0x01);
		};
		CLK_HIGH;		
		delay_us(1);		
	}

	CLK_LOW;
	return data;
}

unsigned long int HC165_GetDWord(void)
{
	unsigned char tmp[4] = {0,0,0,0};
	

	tmp[0] = HC165_GetByte();
	tmp[1] = HC165_GetByte();
	tmp[2] = HC165_GetByte();
	tmp[3] = HC165_GetByte();

	return (unsigned long int) ((((unsigned long int) (tmp[3]))<<24)|(((unsigned long int) (tmp[2]))<<16)|((unsigned long int) (tmp[1])<<8)|(unsigned long int) (tmp[0]));
}

unsigned long int HC165_GetDWord1(void)
{
	unsigned char tmp[4] = {0,0,0,0};

	tmp[0] = HC165_GetByte();
	tmp[1] = HC165_GetByte();
	tmp[2] = HC165_GetByte();
	
	return (unsigned long int) ((((unsigned long int) (tmp[3]))<<24)|(((unsigned long int) (tmp[2]))<<16)|((unsigned long int) (tmp[1])<<8)|(unsigned long int) (tmp[0]));
}

//Get buttons state
unsigned long long int GetButton(void)
{
	unsigned long int keys_state_1, keys_state_2;
	
	CE_LOW;
	delay_us(10);
	HC165_LatchUp();
	
	keys_state_1 = HC165_GetDWord();
	keys_state_2 = HC165_GetDWord1();
	CE_HIGH;

	return (unsigned long long int) ( ((unsigned long long int) (keys_state_1)) |  ( ((unsigned long long int) keys_state_2)<<32) );
}
