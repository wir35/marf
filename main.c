#include <stm32f4xx.h>
#include "stm32f4xx_syscfg.h"
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_tim.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "sin.h"
#include "MAX5135.h"
#include "HC165.h"
#include "adc_pots_selector.h"
#include "CAT25512.h"
#include "leds_step.h"
#include "leds_modes.h"
#include "data_types.h"
#include "dip_config.h"
#include "expander.h"

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

volatile unsigned int 	ADC_result;
volatile unsigned char 	ADC_POT_sel_cnt = 0;

#define MANUAL_ADDR_MAX_VAL		3472	
#define MANUAL_ADDR_STEPS			16
#define MANUAL_ADDR_WIDE			(MANUAL_ADDR_MAX_VAL/MANUAL_ADDR_STEPS)

#define KEY_MIDDLE_SECTION_MASK					0x7F0000F003FFFEFC
#define KEY_MIDDLE_SECTION_ALL_OFF_MASK 		0x070000F003FE0EFC			

#define EXT_DAC_CH_0	0x01
#define EXT_DAC_CH_1	0x02
#define EXT_DAC_CH_2	0x04
#define EXT_DAC_CH_3	0x08


#define ADC_EXT_VOLTAGE_A			0x00
#define ADC_EXT_VOLTAGE_B			0x01
#define ADC_EXT_VOLTAGE_C			0x02
#define ADC_EXT_VOLTAGE_D			0x03
#define ADC_TIMEMULTIPLY_Ch_1	0x04
#define ADC_TIMEMULTIPLY_Ch_2	0x05
#define ADC_STAGEADDRESS_Ch_1	0x06
#define ADC_STAGEADDRESS_Ch_2	0x07
	
//Union with flags which allows to update different parts of panel
typedef union
{
	struct {
		unsigned char MainDisplay:1;	
		unsigned char StepsDisplay:1;
		unsigned char OT1:1;
		unsigned char OT2:1;
		unsigned char OT3:1;
		unsigned char OT4:1;
		unsigned char OT5:1;
		unsigned char OT6:1;		
	} b;
	unsigned char value;
} uDisplayUpdateFlag;

volatile uDisplayUpdateFlag DisplayUpdateFlags;

volatile uStep Steps[2][32];				//Main steps array data
#define SEQUENCER_DATA_SIZE	(6*2*32)
volatile unsigned int AddData[8];		//Additional analog data
volatile unsigned int CalConstants[8] = {0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF};		//Additional analog data

//Display modes
#define DISPLAY_MODE_VIEW_1				0
#define DISPLAY_MODE_VIEW_2				1
#define DISPLAY_MODE_EDIT_1				2
#define DISPLAY_MODE_EDIT_2				3
#define DISPLAY_MODE_SAVE_1				4
#define DISPLAY_MODE_SAVE_2				5
#define DISPLAY_MODE_LOAD_1				6
#define DISPLAY_MODE_LOAD_2				7

volatile unsigned char 			gDisplayMode = DISPLAY_MODE_VIEW_1;		//Current display mode
volatile unsigned char 			gSequenceStepNumber_1	= 0, gSequenceStepNumber_2	= 0; //Current step number

volatile static unsigned long int 	gFullStepWidth_1 = 0, 
									gFullStepWidth_2 = 0;			//Length of the current step in timer "ticks"
volatile static unsigned long int 	gStepWidth_1 = 0, 
									gStepWidth_2 = 0;							//Step counter
unsigned char gEditModeStepNum = 0;

//Sequencer modes
#define SEQUENCER_MODE_RUN	0
#define SEQUENCER_MODE_WAIT	1
#define SEQUENCER_MODE_STOP	2
#define SEQUENCER_MODE_WAIT_STROBE 	3
#define SEQUENCER_MODE_WAIT_HI_Z		4
#define SEQUENCER_MODE_STAY_HI_Z		5
#define SEQUENCER_MODE_ADVANCE			6

volatile unsigned char gSequencerMode_1 = SEQUENCER_MODE_RUN;
volatile unsigned char gSequencerMode_2 = SEQUENCER_MODE_RUN;

//Modes for start condition
#define START_MODE_ZERO				0
#define START_MODE_WAIT_HI_Z	1
#define START_MODE_HI_Z				2

//Current mode for start condition
volatile unsigned char gStartMode_1 = START_MODE_ZERO;
volatile unsigned char gStartMode_2 = START_MODE_ZERO;

//Previous sequencer mode
volatile unsigned char gPrevSequencerMode_1 = SEQUENCER_MODE_RUN;
volatile unsigned char gPrevSequencerMode_2 = SEQUENCER_MODE_RUN;
volatile unsigned char gSequencer1_AdvanceStep = 0;
volatile unsigned char gSequencer2_AdvanceStep = 0;

volatile unsigned char gStrobeKey = 0;

//Variable used for key lock during the VIEW_MODE key changes steps options
volatile unsigned char key_locked = 0;			//Variable used for key lock during the VIEW_MODE key changes steps options
volatile unsigned char gKeysNotValid = 0;


#define STEP_TIMER_FREQ_OUT		8000			//250uSec per timer period
#define STEP_TIMER_PRESCALER	(168000000/2/1/STEP_TIMER_FREQ_OUT)

#define VOLTAGE_FULL_RANGE		4095
#define VOLTAGE_0_OFFSET			0
#define VOLTAGE_2_OFFSET			(4095/5)*1
#define VOLTAGE_4_OFFSET			(4095/5)*2
#define VOLTAGE_6_OFFSET			(4095/5)*3
#define VOLTAGE_8_OFFSET			(4095/5)*4

#define START_SIGNAL_DURATION					500

//Counter for start signal debounce
volatile uint8_t StartFilter_1	= 0;
volatile uint8_t StartFilter_2	= 0;

volatile uint8_t gSequenceReadvance_1 = 0;
volatile uint8_t gSequenceReadvance_2 = 0;

#define START_TIMER_SUSTAIN 	40				//1 = 250 uSec, 2 = 500 uSec, 3 = 0.75 mSec, 4 = 1 mSec, etc...

//Dip switch state
volatile uDipConfig gDipConfig;

volatile unsigned long PreviousStep = 0;
volatile unsigned long PreviousStep_2 = 0;

volatile uint8_t pots_step[2] = {1,1};
volatile uint8_t previous_step[2] ={1,1};
volatile unsigned int offset;
float divider;

//Variables for adc scan
#define NUMS 10
volatile uint16_t average_array[2][32][NUMS];
volatile uint8_t average_index[2][32];
volatile long long acc;

unsigned char GetNextStep(unsigned char _Section, unsigned char _StepNum);
	
//Current patches bank
volatile unsigned char bank = 1;
volatile unsigned char strobe_banana_flag1 = 0, strobe_banana_flag2 = 0;
volatile unsigned int save_counter = 0, load_counter = 0;
volatile unsigned char advanced_counter_1 = 0, advanced_counter_2 = 0; 
	
//ADC interrupt handler
void ADC_IRQHandler()
{
	unsigned char NeedInc = 0, i;
	
	//If expander is connected we should scan its sliders
	if(Is_Expander_Present())
	{
		if (ADC_POT_sel_cnt >= 71) {
			ADC_POTS_selector_Ch(0);
		} else {
			ADC_POTS_selector_Ch(ADC_POT_sel_cnt+1);
		};
	}
	else
	{
		if (ADC_POT_sel_cnt >= 39) {
			ADC_POTS_selector_Ch(0);
		} else {
			ADC_POTS_selector_Ch(ADC_POT_sel_cnt+1);
		};
	}
		
	//ADC1 conversation complete
	if ( ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == SET) {				
		
		//Calculate average of 10 measurements for voltage sliders
		if (  (ADC_POT_sel_cnt<=15)) {
			if ( (Steps[1][ADC_POT_sel_cnt].b.WaitVoltageSlider == 1) ) {
					if ( (unsigned int) (ADC1->DR) == (unsigned int) Steps[1][ADC_POT_sel_cnt].b.VLevel ) {
						Steps[1][ADC_POT_sel_cnt].b.WaitVoltageSlider = 0;
					};
			} else {
				average_array[1][ADC_POT_sel_cnt][average_index[1][ADC_POT_sel_cnt]] = (uint16_t) (ADC1->DR);
				
				average_index[1][ADC_POT_sel_cnt]++;
				if(average_index[1][ADC_POT_sel_cnt] == NUMS) average_index[1][ADC_POT_sel_cnt] = 0;
				
				acc = 0;
				for(i = 0; i < NUMS; i++)
				{
					acc += average_array[1][ADC_POT_sel_cnt][i];
				}
				Steps[1][ADC_POT_sel_cnt].b.VLevel = acc/NUMS;
				
				//Steps[1][ADC_POT_sel_cnt].b.VLevel = ((unsigned int) (ADC1->DR)+(unsigned int) Steps[1][ADC_POT_sel_cnt].b.VLevel)/2;
			};
			if ( (Steps[0][ADC_POT_sel_cnt].b.WaitVoltageSlider == 1) ) {
			if ( (unsigned int) (ADC1->DR) == (unsigned int) Steps[0][ADC_POT_sel_cnt].b.VLevel ) {
						Steps[0][ADC_POT_sel_cnt].b.WaitVoltageSlider = 0;
			};
			} else {
								average_array[0][ADC_POT_sel_cnt][average_index[0][ADC_POT_sel_cnt]] = (uint16_t) (ADC1->DR);
				
				average_index[0][ADC_POT_sel_cnt]++;
				if(average_index[0][ADC_POT_sel_cnt] == NUMS) average_index[0][ADC_POT_sel_cnt] = 0;
				
				acc = 0;
				for(i = 0; i < NUMS; i++)
				{
					acc += average_array[0][ADC_POT_sel_cnt][i];
				}
				Steps[0][ADC_POT_sel_cnt].b.VLevel = acc/NUMS;
				//Steps[0][ADC_POT_sel_cnt].b.VLevel = ((unsigned int) (ADC1->DR)+(unsigned int) Steps[0][ADC_POT_sel_cnt].b.VLevel)/2;
			};
			NeedInc = 1;
		};	

		//If expander is connected calculate additional voltages
		if(Is_Expander_Present())
		{
			if (  (ADC_POT_sel_cnt>=40 && (ADC_POT_sel_cnt<=55))) {
				if ( (Steps[1][ADC_POT_sel_cnt-24].b.WaitVoltageSlider == 1) ) {
						if ( (unsigned int) (ADC1->DR) == (unsigned int) Steps[1][ADC_POT_sel_cnt-24].b.VLevel ) {
							Steps[1][ADC_POT_sel_cnt-24].b.WaitVoltageSlider = 0;
						};
				} else {
					Steps[1][ADC_POT_sel_cnt-24].b.VLevel = ((unsigned int) (ADC1->DR)+(unsigned int) Steps[1][ADC_POT_sel_cnt-24].b.VLevel)/2;
				};
				if ( (Steps[0][ADC_POT_sel_cnt-24].b.WaitVoltageSlider == 1) ) {
				if ( (unsigned int) (ADC1->DR) == (unsigned int) Steps[0][ADC_POT_sel_cnt-24].b.VLevel ) {
							Steps[0][ADC_POT_sel_cnt-24].b.WaitVoltageSlider = 0;
				};
				} else {
					Steps[0][ADC_POT_sel_cnt-24].b.VLevel = ((unsigned int) (ADC1->DR)+(unsigned int) Steps[0][ADC_POT_sel_cnt-24].b.VLevel)/2;
				};
				NeedInc = 1;
			}
			
			//Calculate average of 2 measurements for time sliders
				if ((ADC_POT_sel_cnt>=56) && (ADC_POT_sel_cnt<=71)) {
				Steps[0][ADC_POT_sel_cnt-40].b.TLevel = (Steps[0][ADC_POT_sel_cnt-40].b.TLevel+(unsigned int)(ADC1->DR))/2;
				Steps[1][ADC_POT_sel_cnt-40].b.TLevel = (Steps[1][ADC_POT_sel_cnt-40].b.TLevel+(unsigned int)(ADC1->DR))/2;			
				NeedInc = 1;
			};
		}
		if ((ADC_POT_sel_cnt>=24) && (ADC_POT_sel_cnt<=39)) {
			Steps[0][ADC_POT_sel_cnt-24].b.TLevel = (Steps[0][ADC_POT_sel_cnt-24].b.TLevel+(unsigned int)(ADC1->DR))/2;
			Steps[1][ADC_POT_sel_cnt-24].b.TLevel = (Steps[1][ADC_POT_sel_cnt-24].b.TLevel+(unsigned int)(ADC1->DR))/2;			
			NeedInc = 1;
		};
		
	//Clear end of conversion flag
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
		
	}
	
	//ADC2 used for external inputs conversion
		if ( ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == SET ) {
		if ((ADC_POT_sel_cnt>=16) && (ADC_POT_sel_cnt<=23)) {
 			AddData[ADC_POT_sel_cnt-16] = (unsigned int)(ADC2->DR);
			NeedInc = 1;			
		};
		ADC_ClearFlag(ADC2, ADC_FLAG_EOC);
	
	};
	
	//Calculate next channel to measure
	if(Is_Expander_Present())
	{
	if (NeedInc) {
		ADC_POT_sel_cnt++;
		if (ADC_POT_sel_cnt >= 72) {//40
			ADC_POT_sel_cnt = 0;
		};
	};
}
	else
	{
			if (NeedInc) {
		ADC_POT_sel_cnt++;
		if (ADC_POT_sel_cnt >= 40) {//40
			ADC_POT_sel_cnt = 0;
			delay_us(10);
		};
	};
	}
};

/*
	Setting up acts Timer 2 as source for ADC start conversation
	with 40 ksamples per second.
*/
void mADC_init(void)
{
	GPIO_InitTypeDef GPIO_Init_user;
	ADC_InitTypeDef ADC_InitType;
	TIM_TimeBaseInitTypeDef TimeBaseInit;
	NVIC_InitTypeDef nvicStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
		
	TIM_TimeBaseStructInit(&TimeBaseInit);
  TimeBaseInit.TIM_Prescaler 			= 0;
	TimeBaseInit.TIM_CounterMode 		= TIM_CounterMode_Up;
  TimeBaseInit.TIM_Period 				= 4200-1;// for 40kHz
	TimeBaseInit.TIM_ClockDivision 	= TIM_CKD_DIV1;	
  TIM_TimeBaseInit(TIM2, &TimeBaseInit); 

	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_OC2Ref);
	TIM_CCxCmd(TIM2, TIM_Channel_2, TIM_CCx_Enable);
	TIM_SetCompare2(TIM2, 1);	
	TIM2->CCMR1 |= TIM_CCMR1_OC2M;
  TIM_Cmd(TIM2, ENABLE); 
	
	//ADC Init
	NVIC_SetPriority (ADC_IRQn, 1);  
	
	//ADC GPIO Init
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	memset(&GPIO_Init_user, 0, sizeof(GPIO_Init_user));
	GPIO_Init_user.GPIO_Pin 	= GPIO_Pin_0|GPIO_Pin_1;
	GPIO_Init_user.GPIO_Mode 	= GPIO_Mode_AN;
	GPIO_Init(GPIOA, & GPIO_Init_user);	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	
	ADC_StructInit(&ADC_InitType);
	ADC_InitType.ADC_ContinuousConvMode 	= DISABLE;
	ADC_InitType.ADC_DataAlign 						= ADC_DataAlign_Right;
	ADC_InitType.ADC_ExternalTrigConv 		= ADC_ExternalTrigConv_T2_TRGO;
	ADC_InitType.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitType.ADC_NbrOfConversion 			= 1;
	ADC_InitType.ADC_Resolution 					= ADC_Resolution_12b;
	ADC_InitType.ADC_ScanConvMode 				= DISABLE;

	ADC_Init(ADC1, &ADC_InitType);
	ADC_Init(ADC2, &ADC_InitType);	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0 ,1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1 ,1, ADC_SampleTime_480Cycles);

	//ADC interrupts init	
	nvicStructure.NVIC_IRQChannel = ADC_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 1;
	nvicStructure.NVIC_IRQChannelSubPriority = 0;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
	
	ADC_POT_sel_cnt = 0;
	ADC_POTS_selector_Ch(0);
	
	NVIC_EnableIRQ(ADC_IRQn);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);
	ADC_Cmd(ADC1, ENABLE);	
	ADC_Cmd(ADC2, ENABLE);	
};



void ADCPause(void)
{
	NVIC_DisableIRQ(ADC_IRQn);
};

void ADCResume(void)
{
	NVIC_EnableIRQ(ADC_IRQn);
};

//External interrupts init for start and stop switches
void mInterruptInit(void)
{
	GPIO_InitTypeDef mGPIO;
	EXTI_InitTypeDef mInt;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	mGPIO.GPIO_Mode = GPIO_Mode_IN;
	mGPIO.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_5|/*GPIO_Pin_6|*/GPIO_Pin_7/*|GPIO_Pin_8*/;
	mGPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	mGPIO.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_Init(GPIOB, &mGPIO);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource1);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource7);
	
	//START-STOP LINE INIT Interrupt
	EXTI_DeInit();
	mInt.EXTI_Line = EXTI_Line0|EXTI_Line1|EXTI_Line5|/*EXTI_Line6|*/EXTI_Line7/*|EXTI_Line8*/;
	mInt.EXTI_Mode = EXTI_Mode_Interrupt;
	mInt.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	mInt.EXTI_LineCmd = ENABLE;	
	EXTI_Init(&mInt);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; 						
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0x00; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority 				= 0x00; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; 						
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0x00; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority 				= 0x00; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; 						
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0x00; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority 				= 0x00; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure);
	
	EXTI_ClearITPendingBit(EXTI_Line0);	
	EXTI_ClearITPendingBit(EXTI_Line1);	
	EXTI_ClearITPendingBit(EXTI_Line5);	
	EXTI_ClearITPendingBit(EXTI_Line6);	
	EXTI_ClearITPendingBit(EXTI_Line7);	
	
};


//STOP KEY-BANANA Interrupt handler
//1 SECTION
void EXTI0_IRQHandler()
{
	if ( !(GPIOB->IDR & GPIO_IDR_IDR_0) && 
		(gSequencerMode_1 != SEQUENCER_MODE_WAIT && gSequencerMode_1 != SEQUENCER_MODE_WAIT_HI_Z && gSequencerMode_1 != SEQUENCER_MODE_STAY_HI_Z)
	) {
		gPrevSequencerMode_1 = SEQUENCER_MODE_RUN;
		gSequencerMode_1 = SEQUENCER_MODE_STOP;	
		
		DisplayUpdateFlags.b.MainDisplay 	= 1;
		DisplayUpdateFlags.b.StepsDisplay = 1;
	};
	
	EXTI_ClearITPendingBit(EXTI_Line0);	
};


/*
	Init timer for start pulse (section 1) duration measurement 
*/
void InitStart_1_SignalTimer()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM3->PSC = STEP_TIMER_PRESCALER;
	TIM3->ARR = START_TIMER_SUSTAIN;
	TIM3->CNT = 0;
	TIM3->DIER = TIM_DIER_UIE;
	TIM3->CR1 |= TIM_CR1_CEN;
	
	NVIC_EnableIRQ(TIM3_IRQn);	
};

/*
	Init timer for start pulse (section 2) duration measurement 
*/
void InitStart_2_SignalTimer()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	
	TIM7->PSC = STEP_TIMER_PRESCALER;
	TIM7->ARR = START_TIMER_SUSTAIN;
	TIM7->CNT = 0;
	TIM7->DIER = TIM_DIER_UIE;
	TIM7->CR1 |= TIM_CR1_CEN;
	
	NVIC_EnableIRQ(TIM7_IRQn);	
};

void InitClear_Timer()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	
	TIM6->PSC = 21000;
	TIM6->ARR = 200;
	TIM6->CNT = 0;
	TIM6->DIER = TIM_DIER_UIE;
	TIM6->CR1 |= TIM_CR1_CEN;
	
	NVIC_EnableIRQ(TIM6_DAC_IRQn);	
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
};

//STOP KEY-BANANA Interrupt handler
//2 SECTION
void EXTI1_IRQHandler()
{
	//if we are not in wait condition then stop the sequenser
	if ( !(GPIOB->IDR & GPIO_IDR_IDR_1)  && 
		(gSequencerMode_2 != SEQUENCER_MODE_WAIT && gSequencerMode_2 != SEQUENCER_MODE_WAIT_HI_Z && gSequencerMode_2 != SEQUENCER_MODE_STAY_HI_Z)
	) {
		gPrevSequencerMode_2 = SEQUENCER_MODE_RUN;
		gSequencerMode_2 = SEQUENCER_MODE_STOP;	
		
		DisplayUpdateFlags.b.MainDisplay = 1;
		DisplayUpdateFlags.b.StepsDisplay = 1;
	};
	
	EXTI_ClearITPendingBit(EXTI_Line1);
};



//START KEY-BANANA Interrupt handler
//1 & 2 SECTION
void EXTI9_5_IRQHandler()
{
	//1 Section
	//1 LH
	if (EXTI->PR & (1<<7)) {	 
		  
		if((gSequencerMode_1 != SEQUENCER_MODE_STAY_HI_Z && gSequencerMode_1 != SEQUENCER_MODE_WAIT_HI_Z) && (gSequencerMode_1 != SEQUENCER_MODE_WAIT) && (gSequencerMode_1 != SEQUENCER_MODE_RUN))
		{
			gSequencerMode_1 = SEQUENCER_MODE_RUN;
			gSequenceStepNumber_1 = GetNextStep(0, gSequenceStepNumber_1);
		}
		if(gSequencerMode_1 == SEQUENCER_MODE_WAIT_HI_Z)
		{
			InitStart_1_SignalTimer();
		}

		EXTI_ClearITPendingBit(EXTI_Line7);
	};
	 
	 //2 Section
	 //LH
	 if (EXTI->PR & (1<<5)) {
		 /*if (gSequencerMode_2 == SEQUENCER_MODE_STOP) {
			gPrevSequencerMode_2 = gSequencerMode_2;
			gSequencerMode_2 = SEQUENCER_MODE_RUN;	
		 };	*/	 
		 
		 if((gSequencerMode_2 != SEQUENCER_MODE_STAY_HI_Z && gSequencerMode_2 != SEQUENCER_MODE_WAIT_HI_Z) && (gSequencerMode_2 != SEQUENCER_MODE_WAIT) && (gSequencerMode_2 != SEQUENCER_MODE_RUN))
		{
			gSequencerMode_2 = SEQUENCER_MODE_RUN;
			gSequenceStepNumber_2 = GetNextStep(1, gSequenceStepNumber_2);
		}
		if(gSequencerMode_2 == SEQUENCER_MODE_WAIT_HI_Z)
		{
			InitStart_2_SignalTimer();
		}
		
		EXTI_ClearITPendingBit(EXTI_Line5);
	 };
	 
	 //LL	 
	 if (EXTI->PR & (1<<6)) {
		 if ( (gSequencerMode_2 == SEQUENCER_MODE_STOP) && 
				(!(GPIOB->IDR & GPIO_IDR_IDR_6)) )			 {
			gPrevSequencerMode_2 = gSequencerMode_2;
			gSequencerMode_2 = SEQUENCER_MODE_RUN;	
		 };		 
				
		EXTI_ClearITPendingBit(EXTI_Line6);
	 };
	 
	 
	DisplayUpdateFlags.b.MainDisplay = 1;
	DisplayUpdateFlags.b.StepsDisplay = 1;	 
};

/*
	Save current sequence to memory
*/
unsigned char SaveSequence(unsigned char SequenceCell)
{	
	ADCPause();
	if(!Is_Expander_Present())
	{
		CAT25512_write_block(bank*SequenceCell*sizeof(Steps), (unsigned char *) Steps[0], sizeof(Steps[0]));
		CAT25512_write_block(bank*SequenceCell*sizeof(Steps)+sizeof(Steps[0]), (unsigned char *) Steps[1], sizeof(Steps[1]));
	}
	else
	{
		CAT25512_write_block((SequenceCell+32)*sizeof(Steps), (unsigned char *) Steps[0], sizeof(Steps[0]));
		CAT25512_write_block((SequenceCell+32)*sizeof(Steps)+sizeof(Steps[0]), (unsigned char *) Steps[1], sizeof(Steps[1]));		
	}
	mADC_init();
	return 0;
};

void LoadSequence(unsigned char SequenceCell)
{
	unsigned char cnt;

	ADCPause();
	if(!Is_Expander_Present())
	{
		CAT25512_read_block(bank*SequenceCell*sizeof(Steps), (unsigned char *) Steps[0], sizeof(Steps[0]));
		CAT25512_read_block(bank*SequenceCell*sizeof(Steps)+sizeof(Steps[0]), (unsigned char *) Steps[1], sizeof(Steps[1]));
	}
	else 
	{
		CAT25512_read_block((SequenceCell+32)*sizeof(Steps), (unsigned char *) Steps[0], sizeof(Steps[0]));
		CAT25512_read_block((SequenceCell+32)*sizeof(Steps)+sizeof(Steps[0]), (unsigned char *) Steps[1], sizeof(Steps[1]));
	}

	//Block sliders scanning while voltages from slider and preset aren't equal
	if (gDipConfig.b.SAVE_V_LEVEL == 1) {
		for(cnt=0; cnt<16; cnt++)
		{
			Steps[0][cnt].b.WaitVoltageSlider = 1;
			Steps[0][cnt].b.WaitTimeSlider = 1;
			Steps[1][cnt].b.WaitVoltageSlider = 1;
			Steps[1][cnt].b.WaitTimeSlider = 1;	
			Steps[0][cnt+16].b.WaitVoltageSlider = 1;
			Steps[0][cnt+16].b.WaitTimeSlider = 1;
			Steps[1][cnt+16].b.WaitVoltageSlider = 1;
			Steps[1][cnt+16].b.WaitTimeSlider = 1;				
		};		
	};
	
	gSequencerMode_1 = SEQUENCER_MODE_STOP;
	gSequencerMode_2 = SEQUENCER_MODE_STOP;
	
	mADC_init();
};

/*
	Returns the duration of step number _StepNum in section _Section
*/
	#define EXT_VOLTAGE_STEP_SELECT	1023

unsigned long int GetStepWidth(unsigned char _Section, unsigned char _StepNum)
{
	unsigned long int ret_val = 0;
	unsigned long int time_level = 0;
	unsigned char ext_ban_num = 0;

	if (Steps[_Section][_StepNum].b.TimeSource) {
		//Step time is set externally
		ext_ban_num = Steps[_Section][_StepNum].b.TLevel/EXT_VOLTAGE_STEP_SELECT;
		time_level = AddData[ext_ban_num]*(4095.0/((float)CalConstants[ext_ban_num]));
	} else {
		//Step time is set on panel
		time_level = Steps[_Section][_StepNum].b.TLevel;
	};
	
	if (Steps[_Section][_StepNum].b.TimeRange_p03 == 1) {
		ret_val = (unsigned long int) ((((float) time_level * 112)/4095) +8);
	};
	
	if (Steps[_Section][_StepNum].b.TimeRange_p3 == 1) {
		ret_val = (unsigned long int) ((((float) time_level * 1120)/4095) +80);
	};
	
	if (Steps[_Section][_StepNum].b.TimeRange_3 == 1) {
		ret_val = (unsigned long int) ((((float) time_level * 11200)/4095) +800);
	};
	
	if (Steps[_Section][_StepNum].b.TimeRange_30 == 1) {
		ret_val = (unsigned long int) ((((float) time_level * 112000)/4095) +8000);
	};
	
	return ret_val;
};


/*
	Return the voltage for step number _StepNum in section _Section
*/
	#define MAX_DAC_VALUE			0xFFF 	//Максимальное значение ДАКа
	#define FULL_RANGE_STEPS	60		//Количество шагов на полный диапазон ДАКа
	#define QUANTIZE_DIVIDER	MAX_DAC_VALUE/FULL_RANGE_STEPS //1 шаг для диапазона ДАКа
	
unsigned int GetStepVoltage(unsigned char _Section, unsigned char _StepNum)
{
	unsigned int ret_val = 0;
	unsigned int voltage_level = 0;
	unsigned char ext_ban_num = 0;

	if (Steps[_Section][_StepNum].b.VoltageSource) {
		//Step voltage is set externally
		ext_ban_num = Steps[_Section][_StepNum].b.VLevel/EXT_VOLTAGE_STEP_SELECT;
		if(ext_ban_num > 3) ext_ban_num = 3;
		
		voltage_level = AddData[ext_ban_num]*(4095.0/((float)CalConstants[ext_ban_num]));
	} else {
		//Step voltage is set on panel
		voltage_level = Steps[_Section][_StepNum].b.VLevel;
	};
	
	if(voltage_level > 4095) voltage_level = 4095;

	if (Steps[_Section][_StepNum].b.FullRange) {		
		ret_val = voltage_level;
				// if quantization mode is on
		if (Steps[_Section][_StepNum].b.Quantize) {
		//Calculate the quantized voltage
			ret_val =  (ret_val / (12*divider)) ;
			ret_val = ret_val * (12*divider);
		};
	};
	
	if (Steps[_Section][_StepNum].b.Voltage0) {	
		ret_val = (voltage_level/divider);
		if (Steps[_Section][_StepNum].b.Quantize) {
			ret_val =  (ret_val / (offset/12)) ;
			ret_val = ret_val * (offset/12);
		};
	};
	
	if (Steps[_Section][_StepNum].b.Voltage2) {		
		ret_val = (voltage_level/divider);
		if (Steps[_Section][_StepNum].b.Quantize) {
			ret_val =  (ret_val / (offset/12)) ;
			ret_val = ret_val * (offset/12);
		};
		ret_val +=offset;
	};
	
	if (Steps[_Section][_StepNum].b.Voltage4) {		
		ret_val = (voltage_level/divider);
		if (Steps[_Section][_StepNum].b.Quantize) {
			ret_val =  (ret_val / (offset/12)) ;
			ret_val = ret_val * (offset/12);
		};
		ret_val +=offset*2;
	};
	
	if (Steps[_Section][_StepNum].b.Voltage6) {		
		ret_val = (voltage_level/divider);
		if (Steps[_Section][_StepNum].b.Quantize) {
			ret_val =  (ret_val / (offset/12)) ;
			ret_val = ret_val * (offset/12);
		};
		ret_val +=offset*3;
	};
	
	if (Steps[_Section][_StepNum].b.Voltage8) {		
		ret_val = (voltage_level/divider);
		if (Steps[_Section][_StepNum].b.Quantize) {
			ret_val =  (ret_val / (offset/12)) ;
			ret_val = ret_val * (offset/12);
		};
		ret_val +=offset*4;
	};

	


	
	return ret_val;
};

/*
	Calculate the number of next step
	_StepNum - current step number in section _Section
*/
unsigned char GetNextStep(unsigned char _Section, unsigned char _StepNum)
{
	unsigned char ret_val = 0;
	unsigned char isLastStage = 0;
	unsigned char tmp=0;
	unsigned char max_step;

	if(Is_Expander_Present()) max_step = 31;
	else 
	{
		max_step = 15;
		if(_StepNum > 15) return 0;
	}
	
	
	isLastStage = Steps[_Section][_StepNum].b.CycleLast;
	if (isLastStage != 0) {    
	//Current step is last step  
		for(tmp = 0; tmp <= max_step; tmp++)
		{
			if (tmp<=_StepNum) {
			  if (Steps[_Section][_StepNum-tmp].b.CycleFirst) {
			    ret_val = _StepNum-tmp;
			    break;
			  };
			} else {
			  if (Steps[_Section][max_step-(tmp-_StepNum)].b.CycleFirst) {
			    ret_val = max_step-(tmp-_StepNum);
			    break;
			  };
			};

			if (tmp == max_step) {
			  isLastStage = 0;
			};
		};
} 

    if (isLastStage == 0) {		
		if ( _StepNum >= max_step ) {
			ret_val = 0;
		} else {
			ret_val = _StepNum+1;
		};
	};
	return ret_val;
};

/*
	Timer interrupt handler for 1 section steps
*/
void TIM4_IRQHandler()
{

	unsigned long int StepWidth_1=0;	
	float deltaVoltage;
	unsigned long CurStep;
	
	
	/* Clear interrupt flag */
	TIM4->SR = (uint16_t) ~TIM_IT_Update;

	/* Calculate next step duration */		
	StepWidth_1 = GetStepWidth(0, gSequenceStepNumber_1);
					
					
		/* Calculate prescaler/multiplier*/
		TIM4->PSC = (uint16_t) ((((((float) AddData[ADC_TIMEMULTIPLY_Ch_1])*3.5)/CalConstants[ADC_TIMEMULTIPLY_Ch_1])+0.5)*STEP_TIMER_PRESCALER);
		if (Steps[0][gSequenceStepNumber_1].b.Sloped) {
			//Calculate the voltage in slope mode
			CurStep = GetStepVoltage(0, gSequenceStepNumber_1);
			if(gSequencerMode_1 == SEQUENCER_MODE_RUN || gSequencerMode_1 == SEQUENCER_MODE_ADVANCE)
			{
				if (PreviousStep>CurStep) {
					deltaVoltage =  (float) (PreviousStep-CurStep) / StepWidth_1;
					DAC_SetChannel1Data(DAC_Align_12b_R, PreviousStep - (unsigned int) (deltaVoltage*gStepWidth_1) );
				};
				if (CurStep>PreviousStep) {
					deltaVoltage =  (float) (CurStep-PreviousStep) / StepWidth_1;
					DAC_SetChannel1Data(DAC_Align_12b_R, PreviousStep + (unsigned int) (deltaVoltage*gStepWidth_1) );
				};
			}			
			else 
			{
				DAC_SetChannel1Data(DAC_Align_12b_R, GetStepVoltage(0, gSequenceStepNumber_1));
			}
		} else {
			//Calculate the voltage in not slope mode
			DAC_SetChannel1Data(DAC_Align_12b_R, GetStepVoltage(0, gSequenceStepNumber_1));
		};
		
			/*additional channels - ref and time*/
			MAX5135_DAC_send(EXT_DAC_CH_0, Steps[0][gSequenceStepNumber_1].b.TLevel);
			MAX5135_DAC_send(EXT_DAC_CH_1, 0xFFF - (unsigned int) (((double) 0xFFF/ (double) StepWidth_1)*((double) gStepWidth_1)) );
		
		/* Increment step counter */  
		if ( (gSequencerMode_1 == SEQUENCER_MODE_RUN ) || 
		(	(gSequencerMode_1 == SEQUENCER_MODE_ADVANCE) 	) //&& (gSequencer1_AdvanceStep == gSequenceStepNumber_1)
		)
		{
			gStepWidth_1++;
		};

		if ((gStepWidth_1 < StepWidth_1)) {
	
		} 
		else
		{		
			//Calculate next step		
			PreviousStep = GetStepVoltage(0, gSequenceStepNumber_1);

			if ( (gSequencerMode_1 == SEQUENCER_MODE_ADVANCE)   ) { //&& (gSequencer1_AdvanceStep == gSequenceStepNumber_1)
				gSequencerMode_1 = gPrevSequencerMode_1;
			};
			

			
			if (Steps[0][gSequenceStepNumber_1].b.OpModeSTOP) {
				gPrevSequencerMode_1 = gSequencerMode_1;
				gSequencerMode_1 = SEQUENCER_MODE_STOP;				
			};
			
			if ( (Steps[0][gSequenceStepNumber_1].b.OpModeENABLE) )  {
				if( (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == 1)) 
				{
				if(gSequencerMode_1 != SEQUENCER_MODE_WAIT_HI_Z)
				{	
					gPrevSequencerMode_1 = gSequencerMode_1;
					gSequencerMode_1 = SEQUENCER_MODE_WAIT_HI_Z;
				}
				}
			};
				
			if ( (Steps[0][gSequenceStepNumber_1].b.OpModeSUSTAIN) )  {
				if( (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == 0)) 
				{
					if(gSequencerMode_1 != SEQUENCER_MODE_STAY_HI_Z)
					{	
						gPrevSequencerMode_1 = gSequencerMode_1;
						gSequencerMode_1 = SEQUENCER_MODE_STAY_HI_Z;
						InitStart_1_SignalTimer();
					}
				}
					
			};
				
			if ( (!(Steps[0][gSequenceStepNumber_1].b.OpModeSTOP)) &&
					(!(Steps[0][gSequenceStepNumber_1].b.OpModeENABLE)) &&
					(!(Steps[0][gSequenceStepNumber_1].b.OpModeSUSTAIN)) ) {
				};

			// Reset step width
			gStepWidth_1 = 0;

			/* Generate pulse at the end of every step */
			if (gSequencerMode_1 == SEQUENCER_MODE_RUN) {		
				gSequenceStepNumber_1 = GetNextStep(0, gSequenceStepNumber_1);
				PULSE_LED_I_ALL_ON;
				
				if (Steps[0][gSequenceStepNumber_1].b.OutputPulse1) {
					PULSE_LED_I_1_ON;
				};
				if (Steps[0][gSequenceStepNumber_1].b.OutputPulse2) {
					PULSE_LED_I_2_ON;
				};	
				
				TIM_Cmd(TIM14, ENABLE);
				TIM_SetCounter(TIM14, 0x00);
			};
		};
	
	if (gSequencerMode_1 == SEQUENCER_MODE_WAIT) {
		if(gSequenceStepNumber_1 != (unsigned int) (pots_step[0]-1))
		{
			gSequenceStepNumber_1 = (unsigned int) (pots_step[0]-1); //(float) AddData[6]/(float) (CalConstants[6]/15.0)
			DAC_SetChannel1Data(DAC_Align_12b_R, GetStepVoltage(0, gSequenceStepNumber_1));
			
			PULSE_LED_I_ALL_ON;
			
			if (Steps[0][gSequenceStepNumber_1].b.OutputPulse1) {
				PULSE_LED_I_1_ON;
			};
			if (Steps[0][gSequenceStepNumber_1].b.OutputPulse2) {
				PULSE_LED_I_2_ON;
			};	
			
			TIM_Cmd(TIM14, ENABLE);
			TIM_SetCounter(TIM14, 0x00);
		}
	};	

	if (gSequencerMode_1 == SEQUENCER_MODE_WAIT_STROBE) {

			gSequenceStepNumber_1 = (unsigned int) (pots_step[0]-1); //(float) AddData[6]/(float) (CalConstants[6]/15.0)
			DAC_SetChannel1Data(DAC_Align_12b_R, GetStepVoltage(0, gSequenceStepNumber_1));
		gSequencerMode_1 = gPrevSequencerMode_1;
	}
		
	if ( gDisplayMode == DISPLAY_MODE_VIEW_1 ) {
		DisplayUpdateFlags.b.MainDisplay = 1;
		DisplayUpdateFlags.b.StepsDisplay = 1;
	};		
	
		return;
};

/*
 Timer interrupt handler for 1 section steps
*/
void TIM5_IRQHandler()
{
	unsigned long int StepWidth_2=0;	// В данной переменной хранится пересчитанное значение длительности шага 
	float deltaVoltage;
	unsigned long CurStep;
	
	/* Clear interrupt flag */
	TIM5->SR = (uint16_t) ~TIM_IT_Update;
	/* Calculate next step duration */		
		StepWidth_2 = GetStepWidth(1, gSequenceStepNumber_2);

		/* Calculate prescaler/multiplier*/
		TIM5->PSC = (uint16_t) (((((AddData[ADC_TIMEMULTIPLY_Ch_2])*3.5)/CalConstants[ADC_TIMEMULTIPLY_Ch_2])+0.5)*STEP_TIMER_PRESCALER);
		
		if (Steps[1][gSequenceStepNumber_2].b.Sloped) {
			//Calculate the voltage in slope mode
			CurStep = GetStepVoltage(1, gSequenceStepNumber_2);
			if(gSequencerMode_2 == SEQUENCER_MODE_RUN || gSequencerMode_2 == SEQUENCER_MODE_ADVANCE)
			{
				if (PreviousStep_2>CurStep) {
					deltaVoltage =  (float) (PreviousStep_2-CurStep) / StepWidth_2;
					DAC_SetChannel2Data(DAC_Align_12b_R, PreviousStep_2 - (unsigned int) (deltaVoltage*gStepWidth_2) );
				};
				if (CurStep>PreviousStep_2) {
					deltaVoltage =  (float) (CurStep-PreviousStep_2) / StepWidth_2;
					DAC_SetChannel2Data(DAC_Align_12b_R, PreviousStep_2 + (unsigned int) (deltaVoltage*gStepWidth_2) );
				};	
			}
			else
			{
				DAC_SetChannel2Data(DAC_Align_12b_R, GetStepVoltage(1, gSequenceStepNumber_2));
			}
			
		} else {
			//Calculate the voltage in not slope mode
			DAC_SetChannel2Data(DAC_Align_12b_R, GetStepVoltage(1, gSequenceStepNumber_2));
		};

			MAX5135_DAC_send(EXT_DAC_CH_2, Steps[1][gSequenceStepNumber_2].b.TLevel);
			MAX5135_DAC_send(EXT_DAC_CH_3, 0xFFF - (unsigned int) (((double) 0xFFF/ (double) StepWidth_2)*((double) gStepWidth_2)) );
	
	/* Increment step counter */
				if ( (gSequencerMode_2 == SEQUENCER_MODE_RUN ) || 
			(	(gSequencerMode_2 == SEQUENCER_MODE_ADVANCE)	) // && (gSequencer2_AdvanceStep == gSequenceStepNumber_2)
		)
		{
			gStepWidth_2++;
		};

		if ((gStepWidth_2 < StepWidth_2)) {
	
		} else {
			PreviousStep_2 = GetStepVoltage(1, gSequenceStepNumber_2);	

			if ( (gSequencerMode_2 == SEQUENCER_MODE_ADVANCE)  ) { // && (gSequencer2_AdvanceStep == gSequenceStepNumber_2)
				gSequencerMode_2 = gPrevSequencerMode_2;
			};

			if (Steps[1][gSequenceStepNumber_2].b.OpModeSTOP) {
				gPrevSequencerMode_2 = gSequencerMode_2;
				gSequencerMode_2 = SEQUENCER_MODE_STOP;				
			};
			
			if ( (Steps[1][gSequenceStepNumber_2].b.OpModeENABLE) )  {
				if( (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 1)) 
				{
					if(gSequencerMode_2 != SEQUENCER_MODE_WAIT_HI_Z)
					{	
						gPrevSequencerMode_2 = gSequencerMode_2;
						gSequencerMode_2 = SEQUENCER_MODE_WAIT_HI_Z;
					}
				}
		};
				
			if ( (Steps[1][gSequenceStepNumber_2].b.OpModeSUSTAIN) )  {
				if( (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 0)) 
				{
					if(gSequencerMode_2 != SEQUENCER_MODE_STAY_HI_Z)
					{	
						gPrevSequencerMode_2 = gSequencerMode_2;
						gSequencerMode_2 = SEQUENCER_MODE_STAY_HI_Z;
						InitStart_2_SignalTimer();
					}
				}
			};
				
			if ( (!(Steps[1][gSequenceStepNumber_2].b.OpModeSTOP)) &&
					(!(Steps[1][gSequenceStepNumber_2].b.OpModeENABLE)) &&
					(!(Steps[1][gSequenceStepNumber_2].b.OpModeSUSTAIN)) ) {
			};

			gStepWidth_2 = 0;

			/* Generate pulse at the end of every step */
			if (gSequencerMode_2 == SEQUENCER_MODE_RUN) {
					gSequenceStepNumber_2 = GetNextStep(1, gSequenceStepNumber_2);
				
				PULSE_LED_II_ALL_ON;
				
				if (Steps[1][gSequenceStepNumber_2].b.OutputPulse1) {
					PULSE_LED_II_1_ON;
				};
				if (Steps[1][gSequenceStepNumber_2].b.OutputPulse2) {
					PULSE_LED_II_2_ON;
				};	
				
				TIM_Cmd(TIM8, ENABLE);
			TIM_SetCounter(TIM8, 0x00);
				
			};
		};
	

	if (gSequencerMode_2 == SEQUENCER_MODE_WAIT) {
		if(gSequenceStepNumber_2 != (unsigned int)(pots_step[1]-1))
		{
			gSequenceStepNumber_2 = (unsigned int)(pots_step[1]-1);
			DAC_SetChannel2Data(DAC_Align_12b_R, GetStepVoltage(1, gSequenceStepNumber_2));
			
			PULSE_LED_II_ALL_ON;

			if (Steps[1][gSequenceStepNumber_2].b.OutputPulse1) {
				PULSE_LED_II_1_ON;
			};
			if (Steps[1][gSequenceStepNumber_2].b.OutputPulse2) {
				PULSE_LED_II_2_ON;
			};	

			TIM_Cmd(TIM8, ENABLE);
			TIM_SetCounter(TIM8, 0x00);
		}
	};	
	
		if (gSequencerMode_2 == SEQUENCER_MODE_WAIT_STROBE) {

			gSequenceStepNumber_2 = (unsigned int)(pots_step[1]-1);
			DAC_SetChannel2Data(DAC_Align_12b_R, GetStepVoltage(1, gSequenceStepNumber_2));

		gSequencerMode_2 = gPrevSequencerMode_2;
	}
	
	
	if ( gDisplayMode == DISPLAY_MODE_VIEW_2 ) {
		DisplayUpdateFlags.b.MainDisplay = 1;
		DisplayUpdateFlags.b.StepsDisplay = 1;
	};	
	
		return;
};


/*
	Init 2 timers to control steps
*/
void mTimersInit(void)
{
	TIM_TimeBaseInitTypeDef myTimer;
	NVIC_InitTypeDef nvicStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseStructInit(&myTimer);
	myTimer.TIM_Prescaler = STEP_TIMER_PRESCALER;
	myTimer.TIM_Period = 1;
	myTimer.TIM_ClockDivision = TIM_CKD_DIV1;
	myTimer.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM4, &myTimer);	
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	TIM_TimeBaseInit(TIM5, &myTimer);
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
	
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	
	
	
	nvicStructure.NVIC_IRQChannel = TIM4_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 0;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
	
	nvicStructure.NVIC_IRQChannel = TIM5_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 0;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
	
	NVIC_SetPriority (TIM4_IRQn, 0);
	NVIC_SetPriority (TIM5_IRQn, 0);
	
	SCB->AIRCR = AIRCR_VECTKEY_MASK | NVIC_PriorityGroup_0;
	
	gStepWidth_1 = 8;
	gStepWidth_2 = 8;
	gFullStepWidth_1 = 8;
	gFullStepWidth_2 = 8;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	
	TIM_TimeBaseStructInit(&myTimer);
	myTimer.TIM_Prescaler = 210;
	myTimer.TIM_Period = 320;
	myTimer.TIM_ClockDivision = TIM_CKD_DIV1;
	myTimer.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM14, &myTimer);	
	
	TIM_Cmd(TIM14, DISABLE);
	TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	
	TIM_TimeBaseStructInit(&myTimer);
	myTimer.TIM_Prescaler = 210;
	myTimer.TIM_Period = 640;
	myTimer.TIM_ClockDivision = TIM_CKD_DIV1;
	myTimer.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM8, &myTimer);	
	
	TIM_Cmd(TIM8, DISABLE);
	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
	
};
//Section 2
//Turn off pulses
//Section 1
void TIM8_UP_TIM13_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
	{
		TIM_Cmd(TIM8, DISABLE);
		PULSE_LED_II_ALL_OFF;
		PULSE_LED_II_1_OFF;
		PULSE_LED_II_2_OFF;
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	}		
}

//Section 2
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM14, TIM_IT_Update) != RESET)
	{
		TIM_Cmd(TIM14, DISABLE);
		PULSE_LED_I_ALL_OFF;
		PULSE_LED_I_1_OFF;
		PULSE_LED_I_2_OFF;
		TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
	}		
}

//Timer Interrupt handler for start switch scan
//Section 1
void TIM3_IRQHandler()
{
	TIM3->SR = (uint16_t) ~TIM_IT_Update;	
	
	if ( (gSequencerMode_1 == SEQUENCER_MODE_WAIT_HI_Z) 
			&& (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == 1) ) {
	}
	else if((gSequencerMode_1 == SEQUENCER_MODE_WAIT_HI_Z) 
			&& (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == 0))
	{
		gSequencerMode_1 = SEQUENCER_MODE_RUN;
		gSequenceStepNumber_1 = GetNextStep(0, gSequenceStepNumber_1);
		TIM3->CR1 &= ~TIM_CR1_CEN;
	}
	
	if((gSequencerMode_1 == SEQUENCER_MODE_STAY_HI_Z) 
			&& (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == 0))
	{

	}
	else if((gSequencerMode_1 == SEQUENCER_MODE_STAY_HI_Z) 
			&& (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == 1))
	{
		gSequencerMode_1 = SEQUENCER_MODE_RUN;
		gSequenceStepNumber_1 = GetNextStep(0, gSequenceStepNumber_1);
		TIM3->CR1 &= ~TIM_CR1_CEN;
	}
};

//Section 2
void TIM7_IRQHandler()
{
	TIM7->SR = (uint16_t) ~TIM_IT_Update;	
		
	if ( (gSequencerMode_2 == SEQUENCER_MODE_WAIT_HI_Z) 
			&& (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 1) ) {
	}
	else if((gSequencerMode_2 == SEQUENCER_MODE_WAIT_HI_Z) 
			&& (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 0))
	{
		gSequencerMode_2 = SEQUENCER_MODE_RUN;
		gSequenceStepNumber_2 = GetNextStep(1, gSequenceStepNumber_2);
		TIM7->CR1 &= ~TIM_CR1_CEN;
	}
	
	if((gSequencerMode_2 == SEQUENCER_MODE_STAY_HI_Z) 
			&& (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 0))
	{
	}
	else if((gSequencerMode_2 == SEQUENCER_MODE_STAY_HI_Z) 
			&& (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 1))
	{
		gSequencerMode_2 = SEQUENCER_MODE_RUN;
		gSequenceStepNumber_2 = GetNextStep(1, gSequenceStepNumber_2);
		TIM7->CR1 &= ~TIM_CR1_CEN;
	}
};

//Clear switch scan
void TIM6_DAC_IRQHandler()
{
	uint8_t i;
	uButtons myButtons;
	static uint8_t clear_counter1 = 0, clear_counter2 = 0;
	//Сбрасывает флаг обработки прерывания
	TIM6->SR = (uint16_t) ~TIM_IT_Update;	

	myButtons.value = GetButton();

	if(clear_counter1 < 30 && clear_counter2 < 30)
	{
		if(!myButtons.b.ClearUp || !myButtons.b.ClearDown) 
		{
			if(!myButtons.b.ClearUp) clear_counter1++;
			else clear_counter1 = 0;
			if(!myButtons.b.ClearDown) clear_counter2++;
			else clear_counter2 = 0;
		}
		else
		{
			clear_counter1 = 0;
			clear_counter2 = 0;
			TIM_SetCounter(TIM6, 0x00);
			TIM6->CR1 &= ~TIM_CR1_CEN;
		}
	}
	else if(clear_counter1 == 30 || clear_counter2 == 30)
	{
		//Clear state after leds blinking
		LED_STEP_SendWord(0x0000);
		delay_ms(500);
		LED_STEP_SendWord(0xFFFF);
		delay_ms(500);
		LED_STEP_SendWord(0x0000);
		delay_ms(500);
		LED_STEP_SendWord(0xFFFF);
		delay_ms(500);
		LED_STEP_SendWord(0x0000);
		delay_ms(500);
		LED_STEP_SendWord(0xFFFF);

		TIM_SetCounter(TIM6, 0x00);
		TIM6->CR1 &= ~TIM_CR1_CEN;
	
		if(clear_counter1 == 30)
		{	
			//Clear section 1 state	
			Steps[0][0].val[3] = 0x00;
			Steps[0][0].val[4] = 0x00;
			Steps[0][0].val[5] = 0x00;
			Steps[0][0].b.TimeRange_30 = 1;
			Steps[0][0].b.FullRange = 1;
			
			for(i=0; i<16; i++)
			{
				Steps[0][i] = Steps[0][0];
				Steps[0][i+16] = Steps[0][0];
			}
			gSequencerMode_1 = SEQUENCER_MODE_STOP;
			gSequenceStepNumber_1 = 0;
		}
		else if(clear_counter2 == 30)
		{
			//Clear section 2 state
			Steps[1][0].val[3] = 0x00;
			Steps[1][0].val[4] = 0x00;
			Steps[1][0].val[5] = 0x00;
			Steps[1][0].b.TimeRange_30 = 1;
			Steps[1][0].b.FullRange = 1;
			for(i=0; i<16; i++)
			{
				Steps[1][i] = Steps[1][0];
				Steps[1][i+16] = Steps[1][0];
			};
			gSequencerMode_2 = SEQUENCER_MODE_STOP;
			gSequenceStepNumber_2 = 0;
		};
		
		clear_counter1 = 0;
		clear_counter2 = 0;
		
		//If not in view mode switch in it
		if (gDisplayMode == DISPLAY_MODE_LOAD_1 || gDisplayMode == DISPLAY_MODE_SAVE_1) gDisplayMode = DISPLAY_MODE_VIEW_1;
		if (gDisplayMode == DISPLAY_MODE_LOAD_2 || gDisplayMode == DISPLAY_MODE_SAVE_2) gDisplayMode = DISPLAY_MODE_VIEW_2; 		
	};
};

/*
Init GPIO for pulses generation
*/
void PulsesInit()
{
	GPIO_InitTypeDef GPIO_Pulses;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	memset(&GPIO_Pulses, 0, sizeof(GPIO_Pulses));
	GPIO_Pulses.GPIO_Pin 		= PULSE_LED_I_ALL|PULSE_LED_I_1|PULSE_LED_I_2;
	GPIO_Pulses.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_Pulses.GPIO_OType	= GPIO_OType_PP;
	GPIO_Pulses.GPIO_PuPd		= GPIO_PuPd_NOPULL;
	GPIO_Pulses.GPIO_Speed	= GPIO_Speed_100MHz;
	
	GPIO_Init(GPIOB, &GPIO_Pulses);
	
	PULSE_LED_I_ALL_OFF;
	PULSE_LED_I_1_OFF;
	PULSE_LED_I_2_OFF;
	
		
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	memset(&GPIO_Pulses, 0, sizeof(GPIO_Pulses));
	GPIO_Pulses.GPIO_Pin 		= PULSE_LED_II_ALL|PULSE_LED_II_1|PULSE_LED_II_2;
	GPIO_Pulses.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_Pulses.GPIO_OType	= GPIO_OType_PP;
	GPIO_Pulses.GPIO_PuPd		= GPIO_PuPd_NOPULL;
	GPIO_Pulses.GPIO_Speed	= GPIO_Speed_100MHz;
	
	GPIO_Init(GPIOA, &GPIO_Pulses);
	
	PULSE_LED_II_ALL_OFF;
	PULSE_LED_II_1_OFF;
	PULSE_LED_II_2_OFF;
};

/*
Init GPIOs for display leds
*/
void DisplayLedsIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= DISPLAY_LED_I|DISPLAY_LED_II;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
};

/*
 Init internal DAC
*/
void InternalDACInit(void)
{
	DAC_InitTypeDef mDacInit;
	GPIO_InitTypeDef mGPIO_InitStructure;
	
	DAC_StructInit(&mDacInit);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//DAC init
	
	mGPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_4|GPIO_Pin_5;
	mGPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_Init(GPIOA, &mGPIO_InitStructure);
		
	/* DAC Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	
	mDacInit.DAC_Trigger 				= DAC_Trigger_None;
	mDacInit.DAC_OutputBuffer 	= DAC_OutputBuffer_Disable;
	mDacInit.DAC_WaveGeneration = DAC_WaveGeneration_None;

	DAC_DeInit();
	
	DAC_Init(DAC_Channel_1, &mDacInit);
	DAC_Init(DAC_Channel_2, &mDacInit);
	
	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_Cmd(DAC_Channel_2, ENABLE);
	
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);
	DAC_SetChannel2Data(DAC_Align_12b_R, 0);
};

/*
	Scan all switches
*/
unsigned char keyb_proc(uButtons * key)
{
	unsigned char StepNum = 0, Section = 0, max_step;
	uStep tmpStep;
	static uint16_t counterL = 0; 
	static uint16_t counterR = 0;
	if(Is_Expander_Present()) max_step = 31;
	else max_step = 15;
	
	
		 
	/* Determine step num for different DisplayModes */
	if (gDisplayMode == DISPLAY_MODE_VIEW_1) {
		StepNum = gSequenceStepNumber_1;
		Section = 0;
	};
	if (gDisplayMode == DISPLAY_MODE_VIEW_2) {
		StepNum = gSequenceStepNumber_2;
		Section = 1;
	};
	if (gDisplayMode == DISPLAY_MODE_EDIT_1) {
		StepNum = gEditModeStepNum;
		Section = 0;
	};
	if (gDisplayMode == DISPLAY_MODE_EDIT_2) {
		StepNum = gEditModeStepNum;
		Section = 1;
	};
	
	if (gDisplayMode == DISPLAY_MODE_SAVE_1) {
		StepNum = gEditModeStepNum;
		Section = 0;
	};
	if (gDisplayMode == DISPLAY_MODE_SAVE_2) {
		StepNum = gEditModeStepNum;
		Section = 1;
	};
	if (gDisplayMode == DISPLAY_MODE_LOAD_1) {
		StepNum = gEditModeStepNum;
		Section = 0;
	};
	if (gDisplayMode == DISPLAY_MODE_LOAD_2) {
		StepNum = gEditModeStepNum;
		Section = 1;
	};
	
	tmpStep = Steps[Section][StepNum];	
				
		/* Middle section */		
		if ( !key->b.Voltage0 ) {
			tmpStep.b.Voltage0 = 1;
			tmpStep.b.Voltage2 = 0;
			tmpStep.b.Voltage4 = 0;
			tmpStep.b.Voltage6 = 0;
			tmpStep.b.Voltage8 = 0;
			tmpStep.b.FullRange = 0;
		};
		
		if ( !key->b.Voltage2 ) {
			tmpStep.b.Voltage0 = 0;
			tmpStep.b.Voltage2 = 1;
			tmpStep.b.Voltage4 = 0;
			tmpStep.b.Voltage6 = 0;
			tmpStep.b.Voltage8 = 0;
			tmpStep.b.FullRange = 0;
		};
		
		if ( !key->b.Voltage4 ) {
			tmpStep.b.Voltage0 = 0;
			tmpStep.b.Voltage2 = 0;
			tmpStep.b.Voltage4 = 1;
			tmpStep.b.Voltage6 = 0;
			tmpStep.b.Voltage8 = 0;
			tmpStep.b.FullRange = 0;
		};
		
		if ( !key->b.Voltage6 ) {
			tmpStep.b.Voltage0 = 0;
			tmpStep.b.Voltage2 = 0;
			tmpStep.b.Voltage4 = 0;
			tmpStep.b.Voltage6 = 1;
			tmpStep.b.Voltage8 = 0;
			tmpStep.b.FullRange = 0;
		};
		
		if ( !key->b.Voltage8 ) {
			tmpStep.b.Voltage0 = 0;
			tmpStep.b.Voltage2 = 0;
			tmpStep.b.Voltage4 = 0;
			tmpStep.b.Voltage6 = 0;
			tmpStep.b.Voltage8 = 1;
			tmpStep.b.FullRange = 0;
		};
		
		if ( !key->b.FullRangeOn ) {
			tmpStep.b.Voltage0 = 0;
			tmpStep.b.Voltage2 = 0;
			tmpStep.b.Voltage4 = 0;
			tmpStep.b.Voltage6 = 0;
			tmpStep.b.Voltage8 = 0;
			tmpStep.b.FullRange = 1;
		};
		
		if ( !key->b.Pulse1On ) {
			tmpStep.b.OutputPulse1 = 1;
		};
		
		if ( !key->b.Pulse1Off ) {
			tmpStep.b.OutputPulse1 = 0;
		};
		
		if ( !key->b.Pulse2On ) {
			tmpStep.b.OutputPulse2 = 1;
		};
		
		if ( !key->b.Pulse2Off ) {
			tmpStep.b.OutputPulse2 = 0;
		};
		
		if ( !key->b.OutputQuantize ) {
			tmpStep.b.Quantize = 1;
		};
		
		if ( !key->b.OutputContinuous ) {
			tmpStep.b.Quantize = 0;
		};
		
		if ( !key->b.IntegrationSloped ) {
			tmpStep.b.Sloped = 1;
		};
		
		if ( !key->b.IntegrationStepped ) {
			tmpStep.b.Sloped = 0;
		};
		
		if ( !key->b.SourceExternal ) {
			tmpStep.b.VoltageSource = 1;
		};
		
		if ( !key->b.SourceInternal ) {
			tmpStep.b.VoltageSource = 0;
		};
		
		if ( !key->b.StopOn ) {
			tmpStep.b.OpModeSTOP = 1;
			tmpStep.b.OpModeENABLE = 0;
			tmpStep.b.OpModeSUSTAIN = 0;
			if ((gDisplayMode == DISPLAY_MODE_VIEW_1 || gDisplayMode == DISPLAY_MODE_EDIT_1) && gSequencerMode_1 == SEQUENCER_MODE_STAY_HI_Z) {
				gSequencerMode_1 = SEQUENCER_MODE_STOP;
			};
			if ((gDisplayMode == DISPLAY_MODE_VIEW_2 || gDisplayMode == DISPLAY_MODE_EDIT_2) && gSequencerMode_2 == SEQUENCER_MODE_STAY_HI_Z) {
				gSequencerMode_2 = SEQUENCER_MODE_STOP;
			};
			if ((gDisplayMode == DISPLAY_MODE_VIEW_1 || gDisplayMode == DISPLAY_MODE_EDIT_1) && gSequencerMode_1 == SEQUENCER_MODE_WAIT_HI_Z) {
				gSequencerMode_1 = SEQUENCER_MODE_STOP;
			};
			if ((gDisplayMode == DISPLAY_MODE_VIEW_2 || gDisplayMode == DISPLAY_MODE_EDIT_2) && gSequencerMode_2 == SEQUENCER_MODE_WAIT_HI_Z) {
				gSequencerMode_2 = SEQUENCER_MODE_STOP;
			};
		};
		
		if ( !key->b.StopOff ) {
			tmpStep.b.OpModeSTOP = 0;
			/* Determine step num for different DisplayModes */
		};
		
		if ( !key->b.SustainOn ) {
			tmpStep.b.OpModeSUSTAIN = 1;
			tmpStep.b.OpModeSTOP = 0;
			tmpStep.b.OpModeENABLE = 0;
				if ((gDisplayMode == DISPLAY_MODE_VIEW_1 || gDisplayMode == DISPLAY_MODE_EDIT_1) && gSequencerMode_1 == SEQUENCER_MODE_WAIT_HI_Z) {
				gSequencerMode_1 = gPrevSequencerMode_1;
			};
			if ((gDisplayMode == DISPLAY_MODE_VIEW_2 || gDisplayMode == DISPLAY_MODE_EDIT_2) && gSequencerMode_2 == SEQUENCER_MODE_WAIT_HI_Z) {
				gSequencerMode_2 = gPrevSequencerMode_2;
			};
		};
		
		if ( !key->b.SustainOff ) {
			tmpStep.b.OpModeSUSTAIN = 0;
			if ((gDisplayMode == DISPLAY_MODE_VIEW_1 || gDisplayMode == DISPLAY_MODE_EDIT_1) && gSequencerMode_1 == SEQUENCER_MODE_STAY_HI_Z) {
				gSequencerMode_1 = gPrevSequencerMode_1;
			};
			if ((gDisplayMode == DISPLAY_MODE_VIEW_2 || gDisplayMode == DISPLAY_MODE_EDIT_2) && gSequencerMode_2 == SEQUENCER_MODE_STAY_HI_Z) {
				gSequencerMode_2 = gPrevSequencerMode_2;
			};
		};
		
		if ( !key->b.EnableOn ) {
			tmpStep.b.OpModeENABLE = 1;
			tmpStep.b.OpModeSTOP = 0;
			tmpStep.b.OpModeSUSTAIN = 0;
						if ((gDisplayMode == DISPLAY_MODE_VIEW_1 || gDisplayMode == DISPLAY_MODE_EDIT_1) && gSequencerMode_1 == SEQUENCER_MODE_STAY_HI_Z) {
				gSequencerMode_1 = gPrevSequencerMode_1;
			};
			if ((gDisplayMode == DISPLAY_MODE_VIEW_2 || gDisplayMode == DISPLAY_MODE_EDIT_2) && gSequencerMode_2 == SEQUENCER_MODE_STAY_HI_Z) {
				gSequencerMode_2 = gPrevSequencerMode_2;
			};
		};
		
		if ( !key->b.EnableOff ) {
			tmpStep.b.OpModeENABLE = 0;
				if ((gDisplayMode == DISPLAY_MODE_VIEW_1 || gDisplayMode == DISPLAY_MODE_EDIT_1) && gSequencerMode_1 == SEQUENCER_MODE_WAIT_HI_Z) {
				gSequencerMode_1 = gPrevSequencerMode_1;
			};
			if ((gDisplayMode == DISPLAY_MODE_VIEW_2 || gDisplayMode == DISPLAY_MODE_EDIT_2) && gSequencerMode_2 == SEQUENCER_MODE_WAIT_HI_Z) {
				gSequencerMode_2 = gPrevSequencerMode_2;
			};
		};
		
		if ( !key->b.FirstOn ) {
			tmpStep.b.CycleFirst = 1;
			tmpStep.b.CycleLast = 0;
		};
		
		if ( !key->b.FirstOff ) {
			tmpStep.b.CycleFirst = 0;
		};
		
		if ( !key->b.LastOn ) {		
			tmpStep.b.CycleLast = 1;
			tmpStep.b.CycleFirst = 0;
		};
		
		if ( !key->b.LastOff ) {
			tmpStep.b.CycleLast = 0;
		};
		
		if ( !key->b.TimeSourceExternal ) {
			tmpStep.b.TimeSource = 1;
		};
		
		if ( !key->b.TimeSourceInternal ) {
			tmpStep.b.TimeSource = 0;
		};
		
		if (!key->b.TimeRange1) {
			tmpStep.b.TimeRange_p03 = 1;
			tmpStep.b.TimeRange_p3 =  0;
			tmpStep.b.TimeRange_3 =   0;
			tmpStep.b.TimeRange_30 =  0;                        
		};

		if (!key->b.TimeRange2) {
			tmpStep.b.TimeRange_p03 = 0;
			tmpStep.b.TimeRange_p3 =  1;
			tmpStep.b.TimeRange_3 =   0;
			tmpStep.b.TimeRange_30 =  0; 
		};

		if (!key->b.TimeRange3) {
			tmpStep.b.TimeRange_p03 = 0;
			tmpStep.b.TimeRange_p3 =  0;
			tmpStep.b.TimeRange_3 =   1;
			tmpStep.b.TimeRange_30 =  0; 
		};

		if (!key->b.TimeRange4) {
			tmpStep.b.TimeRange_p03 = 0;
			tmpStep.b.TimeRange_p3 =  0;
			tmpStep.b.TimeRange_3 =   0;
			tmpStep.b.TimeRange_30 =  1; 
		};
		
		//Go to load mode
		if (!key->b.ClearUp)  {
			//Init timer to detect long press (clear comman)
			InitClear_Timer();
			if (gDisplayMode == DISPLAY_MODE_LOAD_1) {
				
				//if in load mode, restore sequence number gEditModeStepNum
				LoadSequence(gEditModeStepNum);
				gKeysNotValid = 1;
				gDisplayMode = DISPLAY_MODE_VIEW_1;
				
				gPrevSequencerMode_1 = gSequencerMode_1;
				gSequenceStepNumber_1 = 0;
				gSequencerMode_1 = SEQUENCER_MODE_STOP;
				
				goto t1;
			};
			if (gDisplayMode == DISPLAY_MODE_LOAD_2) {
				//if in load mode, restore sequence number gEditModeStepNum
				LoadSequence(gEditModeStepNum);
				gKeysNotValid = 1;
				gDisplayMode = DISPLAY_MODE_VIEW_2;

				gPrevSequencerMode_2 = gSequencerMode_2;
				gSequenceStepNumber_2 = 0;
				gSequencerMode_2 = SEQUENCER_MODE_STOP;
				
				goto t1;
			};
			
			if (gDisplayMode == DISPLAY_MODE_VIEW_1) {
				//if in view mode switch to load mode
				gDisplayMode = DISPLAY_MODE_LOAD_1;
				gEditModeStepNum = 0;
				DisplayUpdateFlags.b.StepsDisplay = 1;
				DisplayUpdateFlags.b.MainDisplay = 1;
				goto t1;
			};
			if (gDisplayMode == DISPLAY_MODE_VIEW_2) {
				//if in view mode switch to load mode
				gDisplayMode = DISPLAY_MODE_LOAD_2;
				gEditModeStepNum = 0;
				DisplayUpdateFlags.b.StepsDisplay = 1;
				DisplayUpdateFlags.b.MainDisplay = 1;
				goto t1;
			};			
		};
		
		t1:
		
		if (!key->b.ClearDown)  {
			InitClear_Timer();
			if (gDisplayMode == DISPLAY_MODE_SAVE_1) {
				//if in save mode - save sequence to memory cell gEditModeStepNum
				SaveSequence(gEditModeStepNum);
				gDisplayMode = DISPLAY_MODE_VIEW_1;
				goto t2;
			};
			if (gDisplayMode == DISPLAY_MODE_SAVE_2) {
				//if in save mode - save sequence to memory cell gEditModeStepNum
				SaveSequence(gEditModeStepNum);
				gDisplayMode = DISPLAY_MODE_VIEW_2;
				goto t2;
			};	
			
			if (gDisplayMode == DISPLAY_MODE_VIEW_1) {
				//if in view mode - switch to save mode
				gDisplayMode = DISPLAY_MODE_SAVE_1;
				gEditModeStepNum = 0;
				DisplayUpdateFlags.b.StepsDisplay = 1;
				DisplayUpdateFlags.b.MainDisplay = 1;
				goto t2;
			};
			if (gDisplayMode == DISPLAY_MODE_VIEW_2) {
				//if in view mode - switch to save mode
				gDisplayMode = DISPLAY_MODE_SAVE_2;
				gEditModeStepNum = 0;
				DisplayUpdateFlags.b.StepsDisplay = 1;
				DisplayUpdateFlags.b.MainDisplay = 1;
				goto t2;
			};			
		};
		
		t2:
		//switch to edit mode		
		if ( !key->b.StepLeft ) {
			if (gDisplayMode == DISPLAY_MODE_VIEW_1) {
				gDisplayMode = DISPLAY_MODE_EDIT_1;
				gEditModeStepNum = 1;
			};
			if (gDisplayMode == DISPLAY_MODE_VIEW_2) {
				gDisplayMode = DISPLAY_MODE_EDIT_2;
				gEditModeStepNum = 1;
			};
			if ( (gDisplayMode == DISPLAY_MODE_EDIT_1) ||
						(gDisplayMode == DISPLAY_MODE_EDIT_2) ) {
				if (gEditModeStepNum > 0) {
					if(counterL == 0) gEditModeStepNum--;
					//if long press switch to repeate selection
					else if(counterL >= 500)
					{
						if(counterL > 600)
						{
							counterL = 500;
							gEditModeStepNum--;
						}
					}
					counterL++;
					DisplayUpdateFlags.b.MainDisplay = 1;
					DisplayUpdateFlags.b.StepsDisplay = 1;
				} else {
					if(counterL == 0) gEditModeStepNum = max_step;
					else if(counterL >= 500)
					{
						if(counterL > 600)
						{
							counterL = 500;
							gEditModeStepNum = max_step;
						}
					}
					counterL++;
					DisplayUpdateFlags.b.MainDisplay = 1;
					DisplayUpdateFlags.b.StepsDisplay = 1;
				};
			};
			
			//if in save or load mode left buttons select memory cell for save/recall
			if ( (gDisplayMode == DISPLAY_MODE_SAVE_1) || (gDisplayMode == DISPLAY_MODE_SAVE_2) ||
				 (gDisplayMode == DISPLAY_MODE_LOAD_1) || (gDisplayMode == DISPLAY_MODE_LOAD_2) ) {
				if (gEditModeStepNum > 0) {
					gEditModeStepNum--;
					DisplayUpdateFlags.b.StepsDisplay = 1;
				} else {
					if(!Is_Expander_Present())
					{	
					gEditModeStepNum = 15;
					if(bank == 1) 
					{
						bank = 2;
						
					}
					else 
					{
						bank = 1;
					}
				}
					else gEditModeStepNum = 31;
					DisplayUpdateFlags.b.StepsDisplay = 1;
				};
			};
		}
		else 
		{
			counterL = 0;
		};
		
		if ( !key->b.StepRight ) {
			if (gDisplayMode == DISPLAY_MODE_VIEW_1) {
				gDisplayMode = DISPLAY_MODE_EDIT_1;
				gEditModeStepNum = max_step;
			};
			if (gDisplayMode == DISPLAY_MODE_VIEW_2) {
				gDisplayMode = DISPLAY_MODE_EDIT_2;
				gEditModeStepNum = max_step;
			};
			if ( (gDisplayMode == DISPLAY_MODE_EDIT_1) ||
						(gDisplayMode == DISPLAY_MODE_EDIT_2) ) {
				if (gEditModeStepNum < max_step) {
					if(counterR == 0) gEditModeStepNum++;
					else if(counterR >= 500)
					{
						if(counterR > 600)
						{
							counterR = 500;
							gEditModeStepNum++;
						}
					}
					counterR++;
					DisplayUpdateFlags.b.MainDisplay = 1;
					DisplayUpdateFlags.b.StepsDisplay = 1;
				} else {
										if(counterR == 0) gEditModeStepNum = 0;
					else if(counterR >= 500)
					{
						if(counterR > 600)
						{
							counterR = 500;
							gEditModeStepNum = 0;
						}
					}
					counterR++;
					DisplayUpdateFlags.b.MainDisplay = 1;
					DisplayUpdateFlags.b.StepsDisplay = 1;
				};
			};			
						
			if ( (gDisplayMode == DISPLAY_MODE_SAVE_1) || (gDisplayMode == DISPLAY_MODE_SAVE_2) ||
				(gDisplayMode == DISPLAY_MODE_LOAD_1) || (gDisplayMode == DISPLAY_MODE_LOAD_2)) {
				if (gEditModeStepNum < max_step) {
					gEditModeStepNum++;
					DisplayUpdateFlags.b.StepsDisplay = 1;
				} else {
					if(!Is_Expander_Present())
					{	
						gEditModeStepNum = 0;
						if(bank == 1) 
						{
							bank = 2;
							
						}
						else 
						{
							bank = 1;
						}
					}
					else 
					{
						gEditModeStepNum = 0;
					}
					DisplayUpdateFlags.b.StepsDisplay = 1;
				};
			};
		}
		else 
		{
			counterR = 0;
		};
		
		key_locked = 1;
		
		//Sections 1/2	
	if (gKeysNotValid == 0) {
		
	if (!key->b.StageAddress1Display) {
			if (gDisplayMode != DISPLAY_MODE_VIEW_1) {			
				gDisplayMode = DISPLAY_MODE_VIEW_1;
			};		
		};
		

		if (!key->b.StageAddress2Display) {
			if (gDisplayMode != DISPLAY_MODE_VIEW_2) {			
				gDisplayMode = DISPLAY_MODE_VIEW_2;
				key_locked = 0;
			};
		};
		
	};
		
		if ( (!key->b.StageAddress1Reset)  ) {
				if(gSequencerMode_1 != SEQUENCER_MODE_WAIT)
				{
				gSequenceStepNumber_1 = 0;
				DisplayUpdateFlags.b.MainDisplay = 1;
				DisplayUpdateFlags.b.StepsDisplay = 1;
				if (gSequencerMode_1 == SEQUENCER_MODE_WAIT_HI_Z || gSequencerMode_1 == SEQUENCER_MODE_STAY_HI_Z) {
				gSequencerMode_1 = gPrevSequencerMode_1;
				}
			};
		};
					

		if ( (!key->b.StageAddress2Reset)  ) {
				if(gSequencerMode_2 != SEQUENCER_MODE_WAIT)
				{						
					gSequenceStepNumber_2 = 0;
					DisplayUpdateFlags.b.MainDisplay = 1;
					DisplayUpdateFlags.b.StepsDisplay = 1;
					
					if (gSequencerMode_2 == SEQUENCER_MODE_WAIT_HI_Z || gSequencerMode_2 == SEQUENCER_MODE_STAY_HI_Z) {
					gSequencerMode_2 = gPrevSequencerMode_2;
				}
		};
		}
				
		if( key->b.Empty5 && strobe_banana_flag1 == 0)
		{
			strobe_banana_flag1 = 1;
				gSequenceStepNumber_1 = (unsigned int) (pots_step[0]-1);
			
			if ( gDisplayMode == DISPLAY_MODE_VIEW_1 ) {
				DisplayUpdateFlags.b.MainDisplay = 1;
				DisplayUpdateFlags.b.StepsDisplay = 1;
			};
			
				PULSE_LED_I_ALL_ON;
				
				if (Steps[0][gSequenceStepNumber_1].b.OutputPulse1) {
					PULSE_LED_I_1_ON;
				};
				if (Steps[0][gSequenceStepNumber_1].b.OutputPulse2) {
					PULSE_LED_I_2_ON;
				};	
				
				TIM_Cmd(TIM14, ENABLE);
				TIM_SetCounter(TIM14, 0x00);
		}
		
		if(!key->b.Empty5) 
		{
			strobe_banana_flag1 = 0;
		}
		if ( (!key->b.StageAddress1PulseSelect) ) {
				gSequenceStepNumber_1 = (unsigned int) (pots_step[0]-1);
					if ( gDisplayMode == DISPLAY_MODE_VIEW_1 ) {
		DisplayUpdateFlags.b.MainDisplay = 1;
		DisplayUpdateFlags.b.StepsDisplay = 1;
	};
			
				PULSE_LED_I_ALL_ON;
				
				if (Steps[0][gSequenceStepNumber_1].b.OutputPulse1) {
					PULSE_LED_I_1_ON;
				};
				if (Steps[0][gSequenceStepNumber_1].b.OutputPulse2) {
					PULSE_LED_I_2_ON;
				};	
				
				TIM_Cmd(TIM14, ENABLE);
				TIM_SetCounter(TIM14, 0x00);
		};
		
						
		if( key->b.Empty2 && strobe_banana_flag2 == 0)
		{
			strobe_banana_flag2 = 1;
				gSequenceStepNumber_2 = (unsigned int) (pots_step[1]-1);
					if ( gDisplayMode == DISPLAY_MODE_VIEW_2 ) {
		DisplayUpdateFlags.b.MainDisplay = 1;
		DisplayUpdateFlags.b.StepsDisplay = 1;
	};
			
				PULSE_LED_II_ALL_ON;
				
				if (Steps[0][gSequenceStepNumber_1].b.OutputPulse1) {
					PULSE_LED_II_1_ON;
				};
				if (Steps[0][gSequenceStepNumber_1].b.OutputPulse2) {
					PULSE_LED_II_2_ON;
				};	
				
				TIM_Cmd(TIM8, ENABLE);
			TIM_SetCounter(TIM8, 0x00);
		}
		
		if(!key->b.Empty2) 
		{
			strobe_banana_flag2 = 0;
		}

		if ( (!key->b.StageAddress2PulseSelect)) {
				gSequenceStepNumber_2 = (unsigned int) (pots_step[1]-1);
					if ( gDisplayMode == DISPLAY_MODE_VIEW_2 ) {
		DisplayUpdateFlags.b.MainDisplay = 1;
		DisplayUpdateFlags.b.StepsDisplay = 1;
	};
			
				PULSE_LED_II_ALL_ON;
				
				if (Steps[1][gSequenceStepNumber_2].b.OutputPulse1) {
					PULSE_LED_II_1_ON;
				};
				if (Steps[1][gSequenceStepNumber_2].b.OutputPulse2) {
					PULSE_LED_II_2_ON;
				};	
				
				TIM_Cmd(TIM8, ENABLE);
			TIM_SetCounter(TIM8, 0x00);
		};		
	
				
	/* Stage address ADVANCE 1 KEY*/
	if (!key->b.StageAddress1ContiniousSelect) {
		if (gSequencerMode_1 != SEQUENCER_MODE_WAIT) {
			gPrevSequencerMode_1 = gSequencerMode_1;
			gSequencerMode_1 = SEQUENCER_MODE_WAIT;		
			DisplayUpdateFlags.b.MainDisplay = 1;
			key_locked = 0;
		};
	} else {
		if (gSequencerMode_1 == SEQUENCER_MODE_WAIT) {
			gSequencerMode_1 = gPrevSequencerMode_1;
			DisplayUpdateFlags.b.MainDisplay = 1;
			DisplayUpdateFlags.b.StepsDisplay = 1;
			key_locked = 0;
		};
	};
	
	if (!key->b.StageAddress2ContiniousSelect) {	
		if (gSequencerMode_2 != SEQUENCER_MODE_WAIT) {
			gPrevSequencerMode_2 = gSequencerMode_2;
			gSequencerMode_2 = SEQUENCER_MODE_WAIT;
			DisplayUpdateFlags.b.MainDisplay = 1;
			key_locked = 0;
		};
	} else {
		if (gSequencerMode_2 == SEQUENCER_MODE_WAIT) {			
			gSequencerMode_2 = gPrevSequencerMode_2;
			DisplayUpdateFlags.b.MainDisplay = 1;
			DisplayUpdateFlags.b.StepsDisplay = 1;			
			key_locked = 0;
		};
	};
	
	if (!key->b.StageAddress1Advance) {
		
		advanced_counter_1++;
		if(advanced_counter_1 == 10)
		{
		if(gSequencerMode_1 != SEQUENCER_MODE_WAIT)
		{
		if(gSequencerMode_1 == SEQUENCER_MODE_ADVANCE) 
		{
			PreviousStep = GetStepVoltage(0, gSequenceStepNumber_1);
			gSequenceStepNumber_1 = GetNextStep(0, gSequenceStepNumber_1);
			gStepWidth_1 = 0;
		}
		else{
			if(gSequencerMode_1 != SEQUENCER_MODE_STAY_HI_Z && gSequencerMode_1 != SEQUENCER_MODE_WAIT_HI_Z )(gPrevSequencerMode_1 = gSequencerMode_1);
			gSequenceStepNumber_1 = GetNextStep(0, gSequenceStepNumber_1);
		}
		gSequencerMode_1 = SEQUENCER_MODE_ADVANCE;
						PULSE_LED_I_ALL_ON;
				
				if (Steps[0][gSequenceStepNumber_1].b.OutputPulse1) {
					PULSE_LED_I_1_ON;
				};
				if (Steps[0][gSequenceStepNumber_1].b.OutputPulse2) {
					PULSE_LED_I_2_ON;
				};	
				
				TIM_Cmd(TIM14, ENABLE);
				TIM_SetCounter(TIM14, 0x00);
	
	}	}
	} else advanced_counter_1 = 0;
	
		if (!key->b.StageAddress2Advance) {
				advanced_counter_2++;
		if(advanced_counter_2 == 10)
		{	
		if(gSequencerMode_2 != SEQUENCER_MODE_WAIT)
				{
		if(gSequencerMode_2 == SEQUENCER_MODE_ADVANCE) 
		{
			PreviousStep_2 = GetStepVoltage(1, gSequenceStepNumber_2);
			gSequenceStepNumber_2 = GetNextStep(1, gSequenceStepNumber_2);
			gStepWidth_2 = 0;
			//gSequencer2_AdvanceStep++;
		}
		else 
		{
			if(gSequencerMode_2 != SEQUENCER_MODE_STAY_HI_Z && gSequencerMode_2 != SEQUENCER_MODE_WAIT_HI_Z )(gPrevSequencerMode_2 = gSequencerMode_2);
			gSequenceStepNumber_2 = GetNextStep(1, gSequenceStepNumber_2);
		}
		gSequencerMode_2 = SEQUENCER_MODE_ADVANCE;
				PULSE_LED_II_ALL_ON;
				
				if (Steps[1][gSequenceStepNumber_2].b.OutputPulse1) {
					PULSE_LED_II_1_ON;
				};
				if (Steps[1][gSequenceStepNumber_2].b.OutputPulse2) {
					PULSE_LED_II_2_ON;
				};	
				
				TIM_Cmd(TIM8, ENABLE);
			TIM_SetCounter(TIM8, 0x00);
				
			}
	}
	}else advanced_counter_2 = 0;
				
	if (gKeysNotValid == 0) {
		Steps[Section][StepNum] = tmpStep;				
		DisplayUpdateFlags.b.MainDisplay = 1;	
	} else {
		gKeysNotValid = 0;
	};
				
	return 1;
}

/*
	Update leds function
*/
void UpdateModeSection(void)
{
	unsigned char StepNum = 0, Section = 0;
	uLeds mLeds;
	uStep *mStep;
	
	mLeds.value[0] = 0xFF;
	mLeds.value[1] = 0xFF;
	mLeds.value[2] = 0xFF;
	mLeds.value[3] = 0xFF;

	if ((gSequencerMode_1 == SEQUENCER_MODE_RUN) ||
			(gSequencerMode_1 == SEQUENCER_MODE_ADVANCE)) {
		mLeds.b.Seq1Run = 0;
	};
	if ( (gSequencerMode_1 == SEQUENCER_MODE_WAIT) ||
			(gSequencerMode_1 == SEQUENCER_MODE_WAIT_HI_Z ) ||
			(gSequencerMode_1 == SEQUENCER_MODE_STAY_HI_Z)  ) {
		mLeds.b.Seq1Wait = 0;
	};
			
	if (gSequencerMode_1 == SEQUENCER_MODE_STOP) {
		mLeds.b.Seq1Stop = 0;
	};
	
	if ((gSequencerMode_2 == SEQUENCER_MODE_RUN) ||
			(gSequencerMode_2 == SEQUENCER_MODE_ADVANCE) ) {
		mLeds.b.Seq2Run = 0;
	};
	if ((gSequencerMode_2 == SEQUENCER_MODE_WAIT) ||
			(gSequencerMode_2 == SEQUENCER_MODE_WAIT_HI_Z ) ||
			(gSequencerMode_2 == SEQUENCER_MODE_STAY_HI_Z) ) {
		mLeds.b.Seq2Wait = 0;
	};
	if (gSequencerMode_2 == SEQUENCER_MODE_STOP) {
		mLeds.b.Seq2Stop = 0;
	};
	
	
	/* Determine step num for different DisplayModes*/
	if ( gDisplayMode == DISPLAY_MODE_VIEW_1 ) {
		StepNum = gSequenceStepNumber_1;
		Section = 0;
	}
	if ( gDisplayMode == DISPLAY_MODE_VIEW_2 ) {
		StepNum = gSequenceStepNumber_2;
		Section = 1;
	};
	if ( gDisplayMode == DISPLAY_MODE_EDIT_1 ) {
		StepNum = gEditModeStepNum;
		Section = 0;
	}
	if ( gDisplayMode == DISPLAY_MODE_EDIT_2 ) {
		StepNum = gEditModeStepNum;
		Section = 1;
	};
	
	mStep = (uStep*) &Steps[Section][StepNum];
	
	mLeds.b.VoltageFull  	= ~mStep->b.FullRange;
  mLeds.b.Voltage0     	= ~mStep->b.Voltage0;
  mLeds.b.Voltage2     	= ~mStep->b.Voltage2;
  mLeds.b.Voltage4     	= ~mStep->b.Voltage4;
  mLeds.b.Voltage6     	= ~mStep->b.Voltage6;
  mLeds.b.Voltage8     	= ~mStep->b.Voltage8;
  mLeds.b.Pulse1       	= ~mStep->b.OutputPulse1;
  mLeds.b.Pulse2       	= ~mStep->b.OutputPulse2;
  mLeds.b.CycleFirst   	= ~mStep->b.CycleFirst;
  mLeds.b.CycleLast    	= ~mStep->b.CycleLast;
  mLeds.b.VoltageSource = ~mStep->b.VoltageSource;
  mLeds.b.Integration   = ~mStep->b.Sloped;
  mLeds.b.Quantization  = ~mStep->b.Quantize;
  mLeds.b.TimeRange0   	= ~mStep->b.TimeRange_p03;
  mLeds.b.TimeRange1   	= ~mStep->b.TimeRange_p3;
  mLeds.b.TimeRange2   	= ~mStep->b.TimeRange_3;
  mLeds.b.TimeRange3   	= ~mStep->b.TimeRange_30;
  mLeds.b.TimeSource   	= ~mStep->b.TimeSource;
  mLeds.b.OPStop       	= ~mStep->b.OpModeSTOP;
  mLeds.b.OPSustain    	= ~mStep->b.OpModeSUSTAIN;
  mLeds.b.OPEnable     	= ~mStep->b.OpModeENABLE;
	
	if ( (gDisplayMode == DISPLAY_MODE_SAVE_1) || (gDisplayMode == DISPLAY_MODE_SAVE_2) ||
		(gDisplayMode == DISPLAY_MODE_LOAD_1) || (gDisplayMode == DISPLAY_MODE_LOAD_2) ) {
		mLeds.value[0] = 0xFF;
		mLeds.value[1] = 0xFF;
		mLeds.value[2] = 0xFF;
		mLeds.value[3] = 0xFF;
		
		
			
		if((gDisplayMode == DISPLAY_MODE_SAVE_1) || (gDisplayMode == DISPLAY_MODE_SAVE_2))
		{
			mLeds.b.Seq2Wait = 1;
			save_counter++;
			if(save_counter < 1500)
			{
				 mLeds.b.Seq1Wait = 0;
			}
			else if(save_counter < 3000)
			{
				 mLeds.b.Seq1Wait = 1;
			}
			else save_counter = 0;
			
			
		}
		else if((gDisplayMode == DISPLAY_MODE_LOAD_1) || (gDisplayMode == DISPLAY_MODE_LOAD_2))
		{
			mLeds.b.Seq1Wait = 1;
			load_counter++;
			if(load_counter < 1500)
			{
				 mLeds.b.Seq2Wait = 0;
			}
			else if(load_counter < 3000)
			{
				 mLeds.b.Seq2Wait = 1;
			}
			else load_counter = 0;
			
			
		}
			
		if(!Is_Expander_Present())
		{	
			if(bank == 1) mLeds.value[0] &= ~(1 << 6);
			else mLeds.value[0] &= ~(1 << 7);
		}
	};
	
	//Send data to leds
	LEDS_modes_SendStruct(&mLeds);
	
	if ( (gDisplayMode == DISPLAY_MODE_VIEW_1) ||
			(gDisplayMode == DISPLAY_MODE_EDIT_1) ) {
		DISPLAY_LED_I_ON;
		DISPLAY_LED_II_OFF;
	};
			
	if ( (gDisplayMode == DISPLAY_MODE_VIEW_2) ||
			(gDisplayMode == DISPLAY_MODE_EDIT_2) ) {
		DISPLAY_LED_II_ON;
		DISPLAY_LED_I_OFF;
	};
};

/*
	Steps section leds update function
*/
void UpdateStepSection(void)
{
	if ( gDisplayMode == DISPLAY_MODE_VIEW_1 ) {
		LED_STEP_LightStep(gSequenceStepNumber_1);
	};
	if ( gDisplayMode == DISPLAY_MODE_VIEW_2 ) {
		LED_STEP_LightStep(gSequenceStepNumber_2);
	};
	if ( ( gDisplayMode == DISPLAY_MODE_EDIT_1 ) ||
		( gDisplayMode == DISPLAY_MODE_EDIT_2 ) ||
		( gDisplayMode == DISPLAY_MODE_SAVE_1 ) ||
		( gDisplayMode == DISPLAY_MODE_SAVE_2 ) ||
		(gDisplayMode == DISPLAY_MODE_LOAD_1) || 
		(gDisplayMode == DISPLAY_MODE_LOAD_2)
		) {
		LED_STEP_LightStep(gEditModeStepNum);
	};
};

void Calibration(void)
{
	unsigned long i;
	uButtons myButtons;
	uLeds mLeds;	
	volatile unsigned long long int key_state;
	
	mLeds.value[0]  	= 0xFF;
	mLeds.value[1]  	= 0xFF;
	mLeds.value[2]  	= 0xFF;
	mLeds.value[3]  	= 0xFF;
	DISPLAY_LED_II_OFF;
	DISPLAY_LED_I_OFF;
	
	while(myButtons.b.StageAddress2Advance)
	{
		//Run/Wait/Stop leds blinking
		i++;
		if(i < 2000)
		{
			mLeds.b.Seq1Run = 1;
			mLeds.b.Seq1Wait = 1;
			mLeds.b.Seq1Stop = 0;
			mLeds.b.Seq2Run = 1;
			mLeds.b.Seq2Wait = 1;
			mLeds.b.Seq2Stop = 0;
		}
		else if(i < 4000)
		{
			mLeds.b.Seq1Run = 0;
			mLeds.b.Seq1Wait = 1;
			mLeds.b.Seq1Stop = 1;
			mLeds.b.Seq2Run = 0;
			mLeds.b.Seq2Wait = 1;
			mLeds.b.Seq2Stop = 1;
		}
		else if(i < 6000)
		{
			mLeds.b.Seq1Run = 1;
			mLeds.b.Seq1Wait = 0;
			mLeds.b.Seq1Stop = 1;
			mLeds.b.Seq2Run = 1;
			mLeds.b.Seq2Wait = 0;
			mLeds.b.Seq2Stop = 1;
		}
		else i = 0;
		
		LEDS_modes_SendStruct(&mLeds);

		myButtons.value = GetButton();
	}
	//Measure external inputs
	for(i = 0; i < 8; i++)
	{
		CalConstants[i] = AddData[i];
		if(CalConstants[i] < 100) CalConstants[i] = 4095;
	}
	ADCPause();
	//Store calibration consstants
	CAT25512_write_block(100*sizeof(Steps), (unsigned char *) CalConstants, sizeof(CalConstants));
	mADC_init();
	while(!myButtons.b.StageAddress2Advance)
	{
				i++;
		if(i < 2000)
		{
			mLeds.b.Seq1Run = 1;
			mLeds.b.Seq1Wait = 1;
			mLeds.b.Seq1Stop = 0;
			mLeds.b.Seq2Run = 1;
			mLeds.b.Seq2Wait = 1;
			mLeds.b.Seq2Stop = 0;
		}
		else if(i < 4000)
		{
			mLeds.b.Seq1Run = 0;
			mLeds.b.Seq1Wait = 1;
			mLeds.b.Seq1Stop = 1;
			mLeds.b.Seq2Run = 0;
			mLeds.b.Seq2Wait = 1;
			mLeds.b.Seq2Stop = 1;
		}
		else if(i < 6000)
		{
			mLeds.b.Seq1Run = 1;
			mLeds.b.Seq1Wait = 0;
			mLeds.b.Seq1Stop = 1;
			mLeds.b.Seq2Run = 1;
			mLeds.b.Seq2Wait = 0;
			mLeds.b.Seq2Stop = 1;
		}
		else i = 0;
		LEDS_modes_SendStruct(&mLeds);
		myButtons.value = GetButton();
	}
}

int main(void)
{
	uButtons myButtons;
	uLeds mLeds;	
	unsigned char _cnt;
	volatile unsigned long long int key_state, prev_key_state;
	unsigned char KeyThreshHoldCnt = 0, max_step;
	uint16_t  next_step_tres = 0, prev_step_tres = 0, temp;
	int i, j;
	long acc;	
	
	/* Reset update states */
	DisplayUpdateFlags.value = 0x00;
	DisplayUpdateFlags.b.MainDisplay 	= 1;
	DisplayUpdateFlags.b.StepsDisplay = 1;
	
	/* Init steps structures */
	Steps[0][0].b.TimeRange_p3 = 1;
	Steps[0][0].b.FullRange = 1;
	Steps[1][0] = Steps[0][0];
	
	for(_cnt=1;_cnt<=15;_cnt++) 
	{
		Steps[0][_cnt] = Steps[0][0];		
		Steps[1][_cnt] = Steps[0][0];
		Steps[0][_cnt+16] = Steps[0][0];		
		Steps[1][_cnt+16] = Steps[0][0];
	};

	PulsesInit();
	DisplayLedsIOInit();
	DipConfig_init();	
	gDipConfig = GetDipConfig(); 
	

	
	/* Out dip switch state to panel */ 
	LED_STEP_init();
	LED_STEP_SendWord(0xFFFF);
	delay_ms(1000);
	LED_STEP_SendWord(0xFFF0|(*((uint8_t*) (&gDipConfig))));
	delay_ms(1000);
	
	//Init external memory
	CAT25512_init();
		
	/* LEDs for mode display */
	LEDS_modes_init();	
	mLeds.value[0] = 0xFF;
	mLeds.value[1] = 0xFF;
	mLeds.value[2] = 0xFF;
	mLeds.value[3] = 0xFF;
	mLeds.b.Seq1Stop = 0;
	mLeds.b.Seq2Stop = 0;
	LEDS_modes_SendStruct(&mLeds);
	
	/* Switches input config */
	init_HC165();
	key_state = GetButton();
	prev_key_state = 0x7fbf67f7fffdff;//key_state;
	
	/* External DAC config */
	MAX5135init();
	
	/* ADC configuration */
	ADC_POTS_selector_init();
	ADC_POTS_selector_Ch(0);
	mADC_init();
	
	mTimersInit();
	mInterruptInit();
	
	InternalDACInit();
	Init_Expander_GPIO();
	
	gSequencerMode_1 = SEQUENCER_MODE_STOP;
	gSequencerMode_2 = SEQUENCER_MODE_STOP;
		
	//Scan initial state	
	key_state == GetButton();
	myButtons.value = key_state;
	if(!myButtons.b.StageAddress1Advance) 
	{
		//if advance switch is pressed start calibration
		Calibration();
	}
	else 
	{
		//if not restore calibration constants from memory
		CAT25512_read_block(100*sizeof(Steps), (unsigned char *) CalConstants, sizeof(CalConstants));
		for(i = 0; i < 8; i++)
		{
			if(CalConstants[i] < 100) CalConstants[i] = 4095;
		}
	}
			
	while(1) {		
		gDipConfig = GetDipConfig();
		//Set offset and divider which depend on dip switch state
		if(gDipConfig.b.V_OUT_1V == 1) 
		{
			offset = (4095.0/10.0);
			divider = 10.0;
		}
		else
		{
			if(gDipConfig.b.V_OUT_1V2 == 1)
			{
				offset = (4095.0/8.333);
				divider = 8.333;
			}
			else
			{
				offset = (4095.0/5.0);
				divider = 5.0;
			}
		}

		/* keys proceed */
		if (KeyThreshHoldCnt == 0) {
			key_state = GetButton();
		};		
		
		if (KeyThreshHoldCnt == 2) {
			if (key_state == GetButton()) {	
				myButtons.value = key_state;			
							if ( 	(gDisplayMode != DISPLAY_MODE_SAVE_1) && (gDisplayMode != DISPLAY_MODE_SAVE_2) &&
						(gDisplayMode != DISPLAY_MODE_LOAD_1) && (gDisplayMode != DISPLAY_MODE_LOAD_2) ) 
			{
				if (key_state != prev_key_state || myButtons.b.StepRight == 0 || myButtons.b.StepLeft == 0) {
						
						keyb_proc(&myButtons);
						prev_key_state = key_state;
				};
			}
			else 
			{
							if (key_state != prev_key_state) {
						
						keyb_proc(&myButtons);
						prev_key_state = key_state;
				};	
			}
			};
		};
		
		KeyThreshHoldCnt++;
		if (KeyThreshHoldCnt > 2) {
			KeyThreshHoldCnt = 0;
		};
		/* ENDOF: keys proceed */
		
		//Update panel state
		if (DisplayUpdateFlags.b.MainDisplay) {
			UpdateModeSection();
			DisplayUpdateFlags.b.MainDisplay = 0;
			if ( 	(gDisplayMode == DISPLAY_MODE_SAVE_1) || (gDisplayMode == DISPLAY_MODE_SAVE_2) ||
						(gDisplayMode == DISPLAY_MODE_LOAD_1) || (gDisplayMode == DISPLAY_MODE_LOAD_2) ) 
			{
				DisplayUpdateFlags.b.MainDisplay = 1;
			}
		};
		if (DisplayUpdateFlags.b.StepsDisplay) {			
			UpdateStepSection();
			DisplayUpdateFlags.b.StepsDisplay = 0;
		};
		
		//delay_us(1000);	
	
	for(j = 0; j < 2; j++)
	{
		//Calculation of step number if external control is on
			if(Is_Expander_Present()) max_step = 31;
			else 
			{
				max_step = 15;
				if(pots_step[j] > 16) pots_step[j] = 1;
			}
		
			next_step_tres = 0;
			prev_step_tres = 0;
			
			previous_step[j] = pots_step[j];
			
			if(pots_step[j] < (max_step+1)) next_step_tres = (pots_step[j])*CalConstants[ADC_STAGEADDRESS_Ch_1+j]/(max_step+1) + 20;
			if(pots_step[j] > 1) prev_step_tres = (pots_step[j]-1)*CalConstants[ADC_STAGEADDRESS_Ch_1+j]/(max_step+1) - 20;
		
			acc = 0;
			for(i =0; i<10; i++)
			{
				if(j == 0) 
				{
					temp = (AddData[ADC_STAGEADDRESS_Ch_1]*4095/CalConstants[ADC_STAGEADDRESS_Ch_1])/1.48;
					if(temp > 4095) temp = 4095; 
					acc += temp;
				}
				else 
				{
					temp = (AddData[ADC_STAGEADDRESS_Ch_2]*4095/CalConstants[ADC_STAGEADDRESS_Ch_2])/1.48;
					if(temp > 4095) temp = 4095; 
					acc += temp;
				}
			}
			if(pots_step[j] < (max_step+1)) {if(acc/10 > next_step_tres) 
			{
				pots_step[j]++;
			}
			}
			if(pots_step[j] > 1) {if(acc/10 < prev_step_tres) 
			{
				pots_step[j]--;
			}
			}
		}


	};
};


void delay_ms(unsigned int ms)
{
	volatile uint32_t nCount;
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq (&RCC_Clocks);

	nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
	for (; nCount!=0; nCount--);
}

void delay_us(unsigned int us)
{
	volatile uint32_t nCount;
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq (&RCC_Clocks);

	nCount=(RCC_Clocks.HCLK_Frequency/10000000)*us;
	for (; nCount!=0; nCount--);
}

void delay_ns(unsigned int ns)
{
	volatile uint32_t nCount;
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq (&RCC_Clocks);

	nCount=(RCC_Clocks.HCLK_Frequency/10000000000)*ns;
	for (; nCount!=0; nCount--);
}

