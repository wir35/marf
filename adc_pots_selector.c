#include <stm32f4xx_gpio.h>
#include <stdlib.h>
#include <string.h>
#include "adc_pots_selector.h"

#define ADC_PS_SH_PIN	GPIO_Pin_13
#define ADC_PS_DS_PIN	GPIO_Pin_14
#define ADC_PS_ST_PIN	GPIO_Pin_15

#define ADC_POTS_SELECTOR_SHIFT_HIGH		GPIOC->BSRRL = ADC_PS_SH_PIN
#define ADC_POTS_SELECTOR_SHIFT_LOW			GPIOC->BSRRH = ADC_PS_SH_PIN

#define ADC_POTS_SELECTOR_STORAGE_HIGH	GPIOC->BSRRL = ADC_PS_ST_PIN
#define ADC_POTS_SELECTOR_STORAGE_LOW		GPIOC->BSRRH = ADC_PS_ST_PIN

#define ADC_POTS_SELECTOR_DATA_HIGH			GPIOC->BSRRL = ADC_PS_DS_PIN
#define ADC_POTS_SELECTOR_DATA_LOW			GPIOC->BSRRH = ADC_PS_DS_PIN

#define DELAY 2

//Masks for adc channels selection
unsigned long long int ChSelData[72] = {
	0xFFFFFFFFF0, //Time1 CH
	0xFFFFFFFFF1, //Time2 CH
	0xFFFFFFFFF2,	//Time3 CH
	0xFFFFFFFFF3, //Time4 CH
	0xFFFFFFFFF4, //Time5 CH
	0xFFFFFFFFF5, //Time6 CH
	0xFFFFFFFFF6, //Time7 CH
	0xFFFFFFFFF7, //Time8 CH
	0xFFFFFFFF0F, //Time9 CH
	0xFFFFFFFF1F, //Time10 CH
	0xFFFFFFFF2F, //Time11 CH
	0xFFFFFFFF3F, //Time12 CH
	0xFFFFFFFF4F, //Time13 CH
	0xFFFFFFFF5F, //Time14 CH
	0xFFFFFFFF6F, //Time15 CH
	0xFFFFFFFF7F, //Time16 CH
	0xFFFFFFF0FF, //EXT INPUT A
	0xFFFFFFF1FF, //EXT INPUT B
	0xFFFFFFF2FF, //EXT INPUT C
	0xFFFFFFF3FF, //EXT INPUT D
	0xFFFFFFF4FF, //TIME MULT 1
	0xFFFFFFF5FF, //TIME MULT 2
	0xFFFFFFF6FF, //EXT STAGE 1
	0xFFFFFFF7FF, //EXT STAGE 2
	0xFFFFF0FFFF, //Volt1 CH
	0xFFFFF1FFFF, //Volt2 CH
	0xFFFFF2FFFF, //Volt3 CH
	0xFFFFF3FFFF, //Volt4 CH
	0xFFFFF4FFFF, //Volt5 CH
	0xFFFFF5FFFF, //Volt6 CH
	0xFFFFF6FFFF, //Volt7 CH
	0xFFFFF7FFFF, //Volt8 CH
	0xFFFF0FFFFF, //Volt9 CH
	0xFFFF1FFFFF, //Volt10 CH
	0xFFFF2FFFFF, //Volt11 CH
	0xFFFF3FFFFF, //Volt12 CH
	0xFFFF4FFFFF, //Volt13 CH
	0xFFFF5FFFFF, //Volt14 CH
	0xFFFF6FFFFF, //Volt15 CH
	0xFFFF7FFFFF, //Volt16 CH
	0xFFF0FFFFFF,
	0xFFF1FFFFFF,
	0xFFF2FFFFFF,
	0xFFF3FFFFFF,
	0xFFF4FFFFFF,
	0xFFF5FFFFFF,
	0xFFF6FFFFFF,
	0xFFF7FFFFFF,
	0xFF0FFFFFFF,
	0xFF1FFFFFFF,
	0xFF2FFFFFFF,
	0xFF3FFFFFFF,
	0xFF4FFFFFFF,
	0xFF5FFFFFFF,
	0xFF6FFFFFFF,
	0xFF7FFFFFFF,
	0xF0FFFFFFFF,
	0xF1FFFFFFFF,
	0xF2FFFFFFFF,
	0xF3FFFFFFFF,
	0xF4FFFFFFFF,
	0xF5FFFFFFFF,
	0xF6FFFFFFFF,
	0xF7FFFFFFFF,
	0x0FFFFFFFFF,
	0x1FFFFFFFFF,
	0x2FFFFFFFFF,
	0x3FFFFFFFFF,
	0x4FFFFFFFFF,
	0x5FFFFFFFFF,
	0x6FFFFFFFFF,
	0x7FFFFFFFFF
	
	//0xFFFFFFFF	//ALL CHANNELS OFF
};

//Init GPIOs for ADC channels multiplexers
void ADC_POTS_selector_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Setting up peripherial */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitStructure));
	GPIO_InitStructure.GPIO_Pin 	= ADC_PS_SH_PIN|ADC_PS_ST_PIN|ADC_PS_DS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

//Send one byte to ADC channels multiplexers
void ADC_POTS_selector_SendByte(unsigned char data)
{
	unsigned char dat, cnt;
	
	dat = data;
	for(cnt=0; cnt<8; cnt++)
	{
		if ((dat & 0x80) > 0) {
			ADC_POTS_SELECTOR_DATA_HIGH;
		} else {
			ADC_POTS_SELECTOR_DATA_LOW;
		}

		ADC_POTS_SELECTOR_SHIFT_LOW;
 
		ADC_POTS_SELECTOR_SHIFT_HIGH;
   
		dat = dat << 1;
	}
	
	ADC_POTS_SELECTOR_DATA_LOW;

}

//Send 5 bytes to ADC channels multiplexers
void ADC_POTS_selector_SendDWord(unsigned long long int data)
{	
	ADC_POTS_selector_SendByte((unsigned char) ((data&0xFF00000000)>>32));
	ADC_POTS_selector_SendByte((unsigned char) ((data&0xFF000000)>>24));
	ADC_POTS_selector_SendByte((unsigned char) ( data&0x000000FF));
	ADC_POTS_selector_SendByte((unsigned char) ((data&0x0000FF00)>>8));
	ADC_POTS_selector_SendByte((unsigned char) ((data&0x00FF0000)>>16));

	
	ADC_POTS_SELECTOR_STORAGE_LOW;
	
	ADC_POTS_SELECTOR_STORAGE_HIGH;

}

//Select ADC channel
void ADC_POTS_selector_Ch(unsigned char Ch)
{
	ADC_POTS_selector_SendDWord((unsigned long long int) ChSelData[Ch]);
}
