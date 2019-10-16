#include <stm32f4xx.h>
#include "expander.h"
#include "dip_config.h"

void Init_Expander_GPIO(void)
{
	GPIO_InitTypeDef mGPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	mGPIO_InitStructure.GPIO_Pin 		= EXPANDER_PIN;
	mGPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
	mGPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	mGPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	
	GPIO_Init(EXPANDER_GPIO, &mGPIO_InitStructure);
}

/*Returns 1 if expander is connected, otherwise returns 0*/
uint8_t Is_Expander_Present(void)
{
	uDipConfig DipConfig;
	
	DipConfig = GetDipConfig();
	if(DipConfig.b.EXPANDER_ON) return 1;
	else return 0;
}
