#include <stm32f4xx_gpio.h>

#include "dip_config.h"


#define DipConfigPin1	GPIO_Pin_11
#define DipConfigPin2 GPIO_Pin_10
#define DipConfigPin3 GPIO_Pin_9
#define DipConfigPin4 GPIO_Pin_8

/*Init GPIOs for configuration dip switch*/
void DipConfig_init(void)
{
	GPIO_InitTypeDef mGPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	mGPIO_InitStructure.GPIO_Pin 		= DipConfigPin1|DipConfigPin2|DipConfigPin3|DipConfigPin4;
	mGPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
	mGPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	mGPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOA, &mGPIO_InitStructure);
};

/*Returns the state of dip switch*/
uDipConfig GetDipConfig(void)
{
	uDipConfig lDipConfig;
	
	lDipConfig.b.V_OUT_1V2 		= ~GPIO_ReadInputDataBit(GPIOA, DipConfigPin2);
	lDipConfig.b.SAVE_V_LEVEL = ~GPIO_ReadInputDataBit(GPIOA, DipConfigPin1);
	lDipConfig.b.V_OUT_1V 		= ~GPIO_ReadInputDataBit(GPIOA, DipConfigPin3);
	lDipConfig.b.EXPANDER_ON 	= ~GPIO_ReadInputDataBit(GPIOA, DipConfigPin4);
	
	return lDipConfig;
}
