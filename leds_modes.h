#ifndef LEDS_MODES_H_
#define LEDS_MODES_H_
//Union which allows to control each led on panel separately
typedef union
{
	struct {
		unsigned char Seq1Stop:1;
		unsigned char Seq1Wait:1;
		unsigned char Seq1Run:1;
		unsigned char Seq2Stop:1;
		unsigned char Seq2Wait:1;
		unsigned char Seq2Run:1;
		unsigned char Pulse1:1;
		unsigned char Pulse2:1;
		unsigned char TimeRange2:1;
		unsigned char TimeRange3:1;
		unsigned char TimeSource:1;
		unsigned char LedEmpty11:1;
		unsigned char LedEmpty14:1;
		unsigned char LedEmpty13:1;
		unsigned char LedEmpty15:1;
		unsigned char LedEmpty18:1;
		unsigned char Voltage8:1;
		unsigned char OPStop:1;
		unsigned char OPSustain:1;
		unsigned char OPEnable:1;
		unsigned char CycleFirst:1;
		unsigned char CycleLast:1;
		unsigned char TimeRange0:1;
		unsigned char TimeRange1:1;
		unsigned char Quantization:1;
		unsigned char Integration:1;
		unsigned char VoltageFull:1;
		unsigned char VoltageSource:1;
		unsigned char Voltage0:1;
		unsigned char Voltage2:1;
		unsigned char Voltage4:1;
		unsigned char Voltage6:1;
	} b;
	unsigned char value[4];
} uLeds;


void LEDS_modes_init(void);
void LEDS_modes_SendByte(unsigned char data);
void LEDS_modes_SendDWord(unsigned long int data);
void LEDS_modes_SendStruct(uLeds *_Leds);
extern void delay_us(unsigned int us);
#endif /* LEDS_MODES_H_ */
