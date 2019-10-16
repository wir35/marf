#include <stm32f4xx_rcc.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_gpio.h>

#include "MAX5135.h"

void SendData(unsigned char mData)
{
	SPI_SendData(SPI2, mData);
	//Wait for transmitter clear
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET)
	;
};

void MAX5135_SendPack(unsigned char _data1, unsigned char _data2, unsigned char _data3 )
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	
	SendData(_data1);
	SendData(_data2);
	SendData(_data3);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET)
	;	
	
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

void MAX5135init(void)
{
	/* SPI2 setting up*/
	
	GPIO_InitTypeDef mGPIO_InitStructure;
	SPI_InitTypeDef mSPI;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	
	mGPIO_InitStructure.GPIO_Pin 		= /*GPIO_Pin_12|*/GPIO_Pin_13|GPIO_Pin_15;
	mGPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	mGPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	mGPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	mGPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &mGPIO_InitStructure);
	
	mGPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_12/*|GPIO_Pin_0*/;
	mGPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	mGPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	mGPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	mGPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &mGPIO_InitStructure);
	
	//GPIO_SetBits(GPIOB, GPIO_Pin_0);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	SPI_I2S_DeInit(SPI2);
	SPI_StructInit(&mSPI);
	
	mSPI.SPI_Direction = SPI_Direction_Tx;
	mSPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	mSPI.SPI_Mode = SPI_Mode_Master;
	mSPI.SPI_DataSize = 8;
	mSPI.SPI_CPOL = SPI_CPOL_High;
	mSPI.SPI_CPHA = SPI_CPHA_1Edge;
	mSPI.SPI_FirstBit = SPI_FirstBit_MSB;	
	mSPI.SPI_NSS = SPI_NSS_Soft;
	
	SPI_Init(SPI2, &mSPI);
	SPI_Cmd(SPI2, ENABLE);
	SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Set);
	
	//Rpowerdown not used
	MAX5135_SendPack(MAX5135_CMD_PWR_CONTROL, MAX5135_DATA_NONE, MAX5135_DATA_NONE/*MAX5135_READY_PIN*/);

	//Reset MAX5135
	MAX5135_SendPack(MAX5135_CMD_SOFTWARE_CLEAR, MAX5135_DATA_NONE, MAX5135_DATA_NONE);

	//Linearity calibration
	MAX5135_SendPack(MAX5135_CMD_LINEARITY, MAX5135_LINEARITY_BIT, MAX5135_DATA_NONE);
	//Waiting stabilization
	delay_ms(10);
	//Clr linearity bit
	MAX5135_SendPack(MAX5135_CMD_LINEARITY, MAX5135_DATA_NONE, MAX5135_DATA_NONE);
	
	//init END
}


void MAX5135_DAC_send(unsigned char DAC_Ch, unsigned int DAC_val)
{
	MAX5135_SendPack(0x30|DAC_Ch, (uint8_t) (((DAC_val<<4)&0xFF00)>>8), (uint8_t) (((DAC_val<<4)&0x00FF)));
}
