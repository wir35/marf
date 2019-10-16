#ifndef _MAX5135_H
#define _MAX5135_H

////  Frame format for DAC exchange
////  C7 C6 C5 C4 C3 C2 C1 C0    D15 D14 D13 ...... D5 D4 D3 D2 D0
////  -----------------------    ---------------------------------
////  |                          Data - bytes 2
////  Command - 1 byte
//// Frame length is 24 bits always
//// CS PIN used (CS connected to READY pin in chain)
//// SPI module used for communication


//CMD byte
#define MAX5135_CMD_NONE						0x00
#define MAX5135_CMD_LDAC						0x01
#define MAX5135_CMD_SOFTWARE_CLEAR	0x02
#define MAX5135_CMD_PWR_CONTROL			0x03
#define MAX5135_CMD_LINEARITY				0x05
#define MAX5135_CMD_WRITE						0x10 //OR with DAC channel
#define MAX5135_CMD_WRITE_THRU			0x20 //OR with DAC channel

//DAC channel in CMD byte
#define MAX5135_DAC_CH0							0x01
#define MAX5135_DAC_CH1							0x02
#define MAX5135_DAC_CH2							0x04
#define MAX5135_DAC_CH3							0x08

//Other data
#define MAX5135_READY_PIN						0x80
#define MAX5135_LINEARITY_BIT				0x02

//DAC_CH selection
#define MAX5135_DAC_CH_0						0x01
#define MAX5135_DAC_CH_1						0x02
#define MAX5135_DAC_CH_2						0x04
#define MAX5135_DAC_CH_3						0x08


#define MAX5135_DATA_NONE						0x00



void SendData(unsigned char mData);
void MAX5135init(void);
extern void delay_ms(unsigned int ms);
extern void delay_us(unsigned int us);
void MAX5135_DAC_send(unsigned char DAC1_Ch, unsigned int DAC1_val);

#endif

