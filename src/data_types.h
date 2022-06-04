#ifndef __DATA_TYPES_H
#define __DATA_TYPES_H

/*
	Main structure for step data type
*/
typedef union
{
	struct {		
		unsigned int VLevel:12;
		unsigned int TLevel:12;
		unsigned int Quantize:1;
		unsigned int Sloped:1;
		unsigned int FullRange:1;
		unsigned int VoltageSource:1;
		unsigned int Voltage0:1;
		unsigned int Voltage2:1;
		unsigned int Voltage4:1;
		unsigned int Voltage6:1;
		unsigned int Voltage8:1;
		unsigned int OpModeSTOP:1;
		unsigned int OpModeSUSTAIN:1;
		unsigned int OpModeENABLE:1;
		unsigned int CycleFirst:1;
		unsigned int CycleLast:1;
		unsigned int TimeRange_p03:1;
		unsigned int TimeRange_p3:1;
		unsigned int TimeRange_3:1;
		unsigned int TimeRange_30:1;
		unsigned int TimeSource:1;
		unsigned int OutputPulse1:1;
		unsigned int OutputPulse2:1;
		unsigned int WaitVoltageSlider:1;
		unsigned int WaitTimeSlider:1;
		unsigned int Swing:1;
		unsigned int NU4:1;
	} b;
	unsigned char val[6];
} uStep;



/*
	GPIO defines for device
*/
#define DISPLAY_LED_I				GPIO_Pin_6
#define DISPLAY_LED_I_ON		GPIO_SetBits(GPIOA, DISPLAY_LED_I)
#define DISPLAY_LED_I_OFF		GPIO_ResetBits(GPIOA, DISPLAY_LED_I)

#define	DISPLAY_LED_II			GPIO_Pin_7
#define DISPLAY_LED_II_ON		GPIO_SetBits(GPIOA, DISPLAY_LED_II)
#define DISPLAY_LED_II_OFF	GPIO_ResetBits(GPIOA, DISPLAY_LED_II)

#define PULSE_LED_I_ALL			GPIO_Pin_9
#define PULSE_LED_I_1				GPIO_Pin_11
#define PULSE_LED_I_2				GPIO_Pin_10

#define PULSE_LED_I_ALL_OFF		GPIO_SetBits(GPIOB, 	PULSE_LED_I_ALL)
#define PULSE_LED_I_ALL_ON		GPIO_ResetBits(GPIOB, PULSE_LED_I_ALL)

#define PULSE_LED_I_1_OFF		GPIO_SetBits(GPIOB, 	PULSE_LED_I_1)
#define PULSE_LED_I_1_ON		GPIO_ResetBits(GPIOB, PULSE_LED_I_1)

#define PULSE_LED_I_2_OFF		GPIO_SetBits(GPIOB, 	PULSE_LED_I_2)
#define PULSE_LED_I_2_ON		GPIO_ResetBits(GPIOB, PULSE_LED_I_2)

#define PULSE_LED_II_ALL		GPIO_Pin_2
#define PULSE_LED_II_1			GPIO_Pin_3
#define PULSE_LED_II_2			GPIO_Pin_12

#define PULSE_LED_II_ALL_OFF			GPIO_SetBits(GPIOA, 	PULSE_LED_II_ALL)
#define PULSE_LED_II_ALL_ON				GPIO_ResetBits(GPIOA, PULSE_LED_II_ALL)

#define PULSE_LED_II_1_OFF			GPIO_SetBits(GPIOA, 	PULSE_LED_II_1)
#define PULSE_LED_II_1_ON				GPIO_ResetBits(GPIOA, PULSE_LED_II_1)

#define PULSE_LED_II_2_OFF			GPIO_SetBits(GPIOA, 	PULSE_LED_II_2)
#define PULSE_LED_II_2_ON				GPIO_ResetBits(GPIOA, PULSE_LED_II_2)

#endif
