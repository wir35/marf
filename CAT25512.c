#include <stm32f4xx_rcc.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_gpio.h>
#include "CAT25512.h"


/*
Send one byte to eeprom via SPI
*/
void CAT25512_SendByte(unsigned char mData)
{
	while(!SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE)); 
	SPI_I2S_SendData(SPI3, mData);
	while(!SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI3);
};


/*
Receive one byte from eeprom via SPI
*/
unsigned short int CAT25512_RecieveData(void)
{
	while(!SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE)){}; 
	SPI_I2S_SendData(SPI3, 0x00); 
	while(!SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE)){};
	return SPI_ReceiveData(SPI3);
};


/*
Initialization of CAT25512 eeprom
*/
void CAT25512_init(void)
{
	/* SPI2 setting up*/	
	GPIO_InitTypeDef mGPIO_InitStructure;
	SPI_InitTypeDef mSPI;
	
	/*GPIO Init*/	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_StructInit(&mGPIO_InitStructure);
	mGPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	mGPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	mGPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	mGPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	mGPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &mGPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_StructInit(&mGPIO_InitStructure);
	mGPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_2;
	mGPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	mGPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	mGPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	mGPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &mGPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3 );
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);	
		
	CAT25512_CS_CLEAR;
	
	/*SPI init*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	SPI_I2S_DeInit(SPI3);
	SPI_StructInit(&mSPI);
	
	mSPI.SPI_Direction 					= SPI_Direction_2Lines_FullDuplex;
	mSPI.SPI_BaudRatePrescaler 	= SPI_BaudRatePrescaler_8;
	mSPI.SPI_Mode 							= SPI_Mode_Master;
	mSPI.SPI_DataSize 					= SPI_DataSize_8b;
	mSPI.SPI_CPOL 							= SPI_CPOL_High;
	mSPI.SPI_CPHA 							= SPI_CPHA_2Edge;
	mSPI.SPI_FirstBit 					= SPI_FirstBit_MSB;	
	mSPI.SPI_NSS 								= SPI_NSS_Soft;
	
	SPI_Init(SPI3, &mSPI);
	SPI_Cmd(SPI3, ENABLE);
	SPI_NSSInternalSoftwareConfig(SPI3, SPI_NSSInternalSoft_Set);
};

/*Returns the contents of eeprom status register*/
unsigned char CAT25512_ReadStatusRegister()
{
	unsigned char mData = 0;
	CAT25512_CS_SET;
	CAT25512_SendByte( INTSRUCTION_RDSR );
	delay_ns(2);
	mData = CAT25512_RecieveData();
	delay_ns(200);
	CAT25512_CS_CLEAR;
	return mData;
};

/*Writes a byte to eeprom status register*/
void CAT25512_WriteStatusRegister(unsigned char mData)
{
	CAT25512_CS_SET;
	CAT25512_SendByte(INTSRUCTION_WRSR);
	delay_ns(2);
	CAT25512_SendByte(mData);
	delay_ns(200);
	CAT25512_CS_CLEAR;
};

/*Write enable command*/
void CAT25512_WREN(void)
{
	CAT25512_CS_SET;
	CAT25512_SendByte(INTSRUCTION_WREN);
	delay_ns(200);
	CAT25512_CS_CLEAR;
};

/*Write disable command*/
void CAT25512_WRDI(void)
{
	CAT25512_CS_SET;
	CAT25512_SendByte(INTSRUCTION_WRDI);
	delay_ns(200);
	CAT25512_CS_CLEAR;
};

/*Writes one byte of Data to CAT25512 memory*/
void CAT25512_WriteByte(unsigned short int Address, unsigned char Data)
{
	CAT25512_CS_SET;
	CAT25512_SendByte(INTSRUCTION_WRITE);
	CAT25512_SendByte((Address&0xFF00)>>8);
	CAT25512_SendByte((Address&0x00FF));
	CAT25512_SendByte(Data);
	delay_ns(200);
	CAT25512_CS_CLEAR;
};

/*Reads one byte of Data from CAT25512 memory*/
unsigned char CAT25512_ReadByte(unsigned short int Address)
{
	unsigned char Data;
	CAT25512_CS_SET;
	CAT25512_SendByte(INTSRUCTION_READ);
	CAT25512_SendByte((Address&0xFF00)>>8);
	CAT25512_SendByte((Address&0x00FF));
	Data = CAT25512_RecieveData();
	delay_ns(200);
	CAT25512_CS_CLEAR;
	return Data;
};

/*Writes the Length bytes of Data started from certain Address*/
void CAT25512_write_block(unsigned short int Address, unsigned char *Data, unsigned short int length)
{
	unsigned short int totalcnt=0;
	
	CAT25512_WREN();
	
	CAT25512_CS_SET;
	CAT25512_SendByte(INTSRUCTION_WRITE);
	CAT25512_SendByte((Address&0xFF00)>>8);
	CAT25512_SendByte((Address&0x00FF));
	
	while(totalcnt<length) {
		if ( (((/*Address+*/totalcnt)&0x007F) == 0) && (totalcnt != 0) ) {
			//Stop data send
			delay_ns(200);
			CAT25512_CS_CLEAR;
			
			//read status register
			delay_ms(1);
			while ((CAT25512_ReadStatusRegister() & (SR_RDY) ) != 0) {
				delay_ms(1);
			};
			
			delay_ms(1);	

			CAT25512_WREN();			
			
			CAT25512_CS_SET;
			CAT25512_SendByte(INTSRUCTION_WRITE);
			CAT25512_SendByte(((Address+totalcnt)&0xFF00)>>8);
			CAT25512_SendByte(((Address+totalcnt)&0x00FF));			
		};
		
		CAT25512_SendByte(Data[totalcnt]);	
		
		totalcnt++;
	};
	
	delay_ns(200);
	CAT25512_CS_CLEAR;
	
	delay_ms(1);
	while ((CAT25512_ReadStatusRegister() & (SR_RDY) ) != 0) {
		delay_ms(1);
	};
};

/*Reads the Length bytes of Data started from certain Address*/
void CAT25512_read_block(unsigned short int Address, unsigned char *Data, unsigned short int length)
{
	unsigned short int cnt=0, totalcnt=0;
	
	CAT25512_CS_SET;
	CAT25512_SendByte(INTSRUCTION_READ);
	CAT25512_SendByte((Address&0xFF00)>>8);
	CAT25512_SendByte((Address&0x00FF));
	while(totalcnt<length) {
		Data[totalcnt] = CAT25512_RecieveData();
		totalcnt++;
		cnt++;
	}
	
	delay_ns(200);
	CAT25512_CS_CLEAR;
	
};
