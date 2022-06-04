#include <stm32f4xx_gpio.h>
#include <stdlib.h>
#include <string.h>
#include "adc_pots_selector.h"
#include "delays.h"

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
	0xFFFFFFF0, //Time1 CH
	0xFFFFFFF1, //Time2 CH
	0xFFFFFFF2, //Time3 CH
	0xFFFFFFF3, //Time4 CH
	0xFFFFFFF4, //Time5 CH
	0xFFFFFFF5, //Time6 CH
	0xFFFFFFF6, //Time7 CH
	0xFFFFFFF7, //Time8 CH
	0xFFFFFF0F, //Time9 CH
	0xFFFFFF1F, //Time10 CH
	0xFFFFFF2F, //Time11 CH
	0xFFFFFF3F, //Time12 CH
	0xFFFFFF4F, //Time13 CH
	0xFFFFFF5F, //Time14 CH
	0xFFFFFF6F, //Time15 CH
	0xFFFFFF7F, //Time16 CH
	0xFFFFF0F0, //EXT INPUT A
	0xFFFFF1F1, //EXT INPUT B
	0xFFFFF2F2, //EXT INPUT C
	0xFFFFF3F3, //EXT INPUT D
	0xFFFFF4F4, //TIME MULT 1
	0xFFFFF5F5, //TIME MULT 2
	0xFFFFF6F6, //EXT STAGE 1
	0xFFFFF7F7, //EXT STAGE 2
	/* 0xFFFFF0FF, //EXT INPUT A */
	/* 0xFFFFF1FF, //EXT INPUT B */
	/* 0xFFFFF2FF, //EXT INPUT C */
	/* 0xFFFFF3FF, //EXT INPUT D */
	/* 0xFFFFF4FF, //TIME MULT 1 */
	/* 0xFFFFF5FF, //TIME MULT 2 */
	/* 0xFFFFF6FF, //EXT STAGE 1 */
	/* 0xFFFFF7FF, //EXT STAGE 2 */
	0xFFF0FFFF, //Volt1 CH
	0xFFF1FFFF, //Volt2 CH
	0xFFF2FFFF, //Volt3 CH
	0xFFF3FFFF, //Volt4 CH
	0xFFF4FFFF, //Volt5 CH
	0xFFF5FFFF, //Volt6 CH
	0xFFF6FFFF, //Volt7 CH
	0xFFF7FFFF, //Volt8 CH
	0xFF0FFFFF, //Volt9 CH
	0xFF1FFFFF, //Volt10 CH
	0xFF2FFFFF, //Volt11 CH
	0xFF3FFFFF, //Volt12 CH
	0xFF4FFFFF, //Volt13 CH
	0xFF5FFFFF, //Volt14 CH
	0xFF6FFFFF, //Volt15 CH
	0xFF7FFFFF, //Volt16 CH
	0xF0FFFFFF,
	0xF1FFFFFF,
	0xF2FFFFFF,
	0xF3FFFFFF,
	0xF4FFFFFF,
	0xF5FFFFFF,
	0xF6FFFFFF,
	0xF7FFFFFF,
	0x0FFFFFFF,
	0x1FFFFFFF,
	0x2FFFFFFF,
	0x3FFFFFFF,
	0x4FFFFFFF,
	0x5FFFFFFF,
	0x6FFFFFFF,
	0x7FFFFFFF,
	0x0FFFFFFFF,
	0x1FFFFFFFF,
	0x2FFFFFFFF,
	0x3FFFFFFFF,
	0x4FFFFFFFF,
	0x5FFFFFFFF,
	0x6FFFFFFFF,
	0x7FFFFFFFF,
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
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
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
		/* DELAY_NOPS(); // data setup to serial clock time, 100ns target */
		/* ADC_POTS_SELECTOR_SHIFT_LOW; */
		/* DELAY_NOPS();  // serial clock pulse should be 80ns */
		/* ADC_POTS_SELECTOR_SHIFT_HIGH; */
		ADC_POTS_SELECTOR_SHIFT_LOW;
		DELAY_NOPS();
		ADC_POTS_SELECTOR_SHIFT_HIGH;
		DELAY_NOPS(); 
		dat = dat << 1;
	}
	
	ADC_POTS_SELECTOR_DATA_LOW;
}

void ADC_POTS_selector_SendHalfByte(unsigned char data)
{
  	unsigned char dat, cnt;
	
	dat = data;
	for(cnt=0; cnt<4; cnt++)
	{
		if ((dat & 0x8) > 0) {
			ADC_POTS_SELECTOR_DATA_HIGH;
		} else {
			ADC_POTS_SELECTOR_DATA_LOW;
		}
		/* DELAY_CLOCK(); // data setup to serial clock time, 100ns target */
		/* ADC_POTS_SELECTOR_SHIFT_LOW; */
		/* DELAY_CLOCK();  // serial clock pulse should be 80ns */
		/* ADC_POTS_SELECTOR_SHIFT_HIGH; */
		ADC_POTS_SELECTOR_SHIFT_LOW;
		DELAY_NOPS(); // clock pulse width 120ns or so; also gives long enough for data to settle
		ADC_POTS_SELECTOR_SHIFT_HIGH;
		DELAY_NOPS(); 
		dat = dat << 1;
	}
	
		ADC_POTS_SELECTOR_DATA_LOW;

}

//Send 5 bytes to ADC channels multiplexers
void ADC_POTS_selector_SendDWord(unsigned long long int data)
{	
  ADC_POTS_SELECTOR_STORAGE_LOW; 
  ADC_POTS_selector_SendByte((unsigned char) ((data&0xFF00000000)>>32));
  ADC_POTS_selector_SendByte((unsigned char) ((data&0xFF000000)>>24));
  ADC_POTS_selector_SendByte((unsigned char) ( data&0x000000FF));
  ADC_POTS_selector_SendByte((unsigned char) ((data&0x0000FF00)>>8));
  ADC_POTS_selector_SendByte((unsigned char) ((data&0x00FF0000)>>16));
  
	
  //	ADC_POTS_SELECTOR_STORAGE_LOW;
  //	DELAY_CLOCK();  
  ADC_POTS_SELECTOR_STORAGE_HIGH; // sendbyte had enough delay at the end
}

//Select ADC channel
void ADC_POTS_selector_Ch(unsigned char Ch)
{
	ADC_POTS_selector_SendDWord((unsigned long long int) ChSelData[Ch]);
}

//Select next ADC channel and tell which it is 
unsigned char ADC_inc(unsigned char pot)
{
  ADC_POTS_SELECTOR_STORAGE_LOW;
  /* Muxes are addressed by three shift registers in series as follows: */
  /*   1. time sliders aka pots 24-39 */
  /*   2. (first half) external voltages aka pots 16-23 */
  /*   2. (unused half a shift register) */
  /*   3. voltage sliders  aka pots 0 - 15 */
  unsigned char _pot, channel;
  channel = pot & 0x7; // last three bits of the pot number tell which channel of the mux is live
  if (pot >= 8 && pot < 16) {// we're on the final mux
    channel = (channel + 1) & 0x7;    // increment the channel and wrap
    ADC_POTS_selector_SendHalfByte(channel);
    _pot = 24+channel;  // time slider 1-8 are pots 24 - 31. 
  }
  else if (pot >=16 && pot < 24) { // we're on the external mux, need to shift twice to get to the voltage slider
    ADC_POTS_selector_SendByte(0xFF);
    _pot = channel; // voltage sliders are pots 0-7
  }
  else {
    ADC_POTS_selector_SendHalfByte(0xF); // just shift to next mux
    if (pot >= 32 && pot < 40) // final time sliders, external voltages are next
      {
	_pot = pot - 16; 
      }
    else {
      _pot = pot + 8; // otherwise shift just moves pot count along by 8
    }
  }
  /* // activate the shift register with the new data */
  /* ADC_POTS_SELECTOR_STORAGE_LOW; */
  ADC_POTS_SELECTOR_STORAGE_HIGH;// think the sendbyte and sendhalfbyte have enough delay at the end
  DELAY_NOPS(); 
  // tell the main interrupt which pot we're on now
  return _pot; 
}

unsigned char ADC_inc_expanded(unsigned char pot)
{
  /* Hypothesis: Muxes are addressed by five shift registers in series as follows: */
  /*   1. time sliders aka pots 24-39 */
  /*   2. (first half) external voltages aka pots 16-23 */
  /*   2. (unused half a shift register) */
  /*   3. voltage sliders  aka pots 0 - 15 */
  /*   4. expander voltage sliders aka pots 40 - 55 */
  /*   5. expander time sliders aka pots 56 - 71 */
  unsigned char _pot, channel;
  channel = pot & 0x7; // last three bits of the pot number tell which channel of the mux is live
  if (pot >= 64 && pot < 72) {// we're on the final mux
    channel = (channel + 1) & 0x7;    // increment the channel and wrap
    ADC_POTS_selector_SendHalfByte(channel);
    _pot = 24+channel;  // time slider 1-8 are pots 24 - 31. 
  }
  else if (pot >=16 && pot < 24) { // we're on the external mux, need to shift twice to get to the voltage slider
    ADC_POTS_selector_SendByte(0xFF);
    _pot = channel; // voltage sliders are pots 0-7
  }
  else {
    ADC_POTS_selector_SendHalfByte(0xF); // just shift to next mux
    if (pot >= 32 && pot < 40) // final time sliders, external voltages are next
      {
	_pot = pot - 16; 
      }
    else if (pot >=8 && pot < 16) { // from mux 3 to mux 4
      _pot = pot + 32; 
    }
    else 
      {
      _pot = pot + 8; // otherwise shift just moves pot count along by 8
      }
  }
  // activate the shift register with the new data
  ADC_POTS_SELECTOR_STORAGE_LOW;
  DELAY_CLOCK_20();  // serial clock pulse should be 80ns
  ADC_POTS_SELECTOR_STORAGE_HIGH;
  // tell the main interrupt which pot we're on now
  return _pot; 
}

