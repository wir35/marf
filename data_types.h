/*
	Main structure for step data type
*/
typedef union
{
	struct {		
		int VLevel:12;
		int TLevel:12;
		int Quantize:1;
		int Sloped:1;
		int FullRange:1;
		int VoltageSource:1;
		int Voltage0:1;
		int Voltage2:1;
		int Voltage4:1;
		int Voltage6:1;
		int Voltage8:1;
		int OpModeSTOP:1;
		int OpModeSUSTAIN:1;
		int OpModeENABLE:1;
		int CycleFirst:1;
		int CycleLast:1;
		int TimeRange_p03:1;
		int TimeRange_p3:1;
		int TimeRange_3:1;
		int TimeRange_30:1;
		int TimeSource:1;
		int OutputPulse1:1;
		int OutputPulse2:1;
		int WaitVoltageSlider:1;
		int WaitTimeSlider:1;
		int NU3:1;
		int NU4:1;
	} b;
	unsigned char val[6];
} uStep;



/*
	PIO defines for device
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


void UpdateModeSection(void);
