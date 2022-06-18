#include "HC165.h"

#include <stdint.h>
#include <string.h>
#include <stm32f4xx_gpio.h>

#include "delays.h"

void HC165_InitializeGPIO(void) {

	GPIO_InitTypeDef mGPIO_InitStructure;
	GPIO_InitTypeDef mGPIO_InitStructure2;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	memset(&mGPIO_InitStructure, 0, sizeof(mGPIO_InitStructure));
	mGPIO_InitStructure.GPIO_Pin 		= HC165_GPIO_PIN_CP|HC165_GPIO_PIN_PL|HC165_GPIO_PIN_CE;
	mGPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	mGPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	mGPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	mGPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	
	GPIO_Init(GPIOC, &mGPIO_InitStructure);
	
	memset(&mGPIO_InitStructure2, 0, sizeof(mGPIO_InitStructure2));
	mGPIO_InitStructure2.GPIO_Pin 		= HC165_GPIO_PIN_DAT;
	mGPIO_InitStructure2.GPIO_Mode 		= GPIO_Mode_IN;

	GPIO_Init(GPIOC, &mGPIO_InitStructure2);
	
	HC165_SS_HIGH;
	HC165_CE_LOW;
	HC165_CLOCK_LOW;
}

// Inlined library functions

inline static uint8_t GPIO_ReadInputDataBit_Fast(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
  return ((GPIOx->IDR & GPIO_Pin) != (uint32_t)Bit_RESET) ? Bit_SET : Bit_RESET;
}

// Close latch and load data to internal registers
static inline void HC165_LatchUp(void) {
	HC165_CE_LOW;
	HC165_CLOCK_LOW;
	HC165_SS_LOW;
	DELAY_NOPS_120NS();

	HC165_SS_HIGH;
	DELAY_NOPS_120NS(); 
}

static inline uint8_t HC165_GetByte(void) {
	uint8_t data = 0x00;

	DELAY_NOPS_120NS(); 

	if (GPIO_ReadInputDataBit_Fast(GPIOC, HC165_GPIO_PIN_DAT)) {
		data = data | 0x01;
	};
	HC165_CLOCK_HIGH;	
	DELAY_NOPS_120NS();

	for (uint8_t cnt = 0; cnt < 7; cnt++) {
	  HC165_CLOCK_LOW;
	  DELAY_NOPS_120NS(); 
		data = data << 1;		
		if (GPIO_ReadInputDataBit_Fast(GPIOC, HC165_GPIO_PIN_DAT)) {
			data = data | (0x01);
		};
		HC165_CLOCK_HIGH;		
		DELAY_NOPS_120NS();
	}
	HC165_CLOCK_LOW;
	DELAY_NOPS_120NS(); 

	return data;
}

// Read all switches and return all of the switch data as a 64bit bit field
uint64_t HC165_ReadSwitches(void) {
  uint8_t switches[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	HC165_CE_LOW;
	delay_us(10);
	HC165_LatchUp();

	for (uint8_t cnt = 0; cnt < 7; cnt++) {
	  // So HC165_GetByte will only be inlined once
	  switches[cnt] = HC165_GetByte();
	}
	HC165_CE_HIGH;

	// Just cast the 8 byte array pointer to a uint64_t pointer and deference it
	return *((uint64_t*) switches);
}
