#include <stm32f4xx.h>
#include "stm32f4xx_syscfg.h"
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_tim.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "MAX5135.h"
#include "HC165.h"
#include "adc_pots_selector.h"
#include "CAT25512.h"
#include "leds_step.h"
#include "leds_modes.h"
#include "data_types.h"
#include "dip_config.h"
#include "expander.h"
#include "version.h"
#include "stdio.h"
#include "program.h"
#include "delays.h"
#include "analog_data.h"
#include "afg.h"
#include "display.h"
#include "controller.h"

volatile uint8_t adc_pot_sel = 0;

#define SEQUENCER_DATA_SIZE (6*2*32)

// Dip switch state
volatile uDipConfig dip_config;

// Clocks
RCC_ClocksTypeDef RCC_Clocks;

unsigned char revision; 

// ADC interrupt handler
void ADC_IRQHandler() {
  uint8_t stage = 0;

  if (adc_pot_sel < 16 && ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == SET) {
    // POT_TYPE_VOLTAGE EOC
    stage = adc_pot_sel;
    WriteVoltageSlider(stage, ADC1->DR);
  }
  else if (adc_pot_sel < 24 && ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == SET) {
    // POT_TYPE_OTHER EOC
    stage = adc_pot_sel - 16;
    WriteOtherCv(stage, ADC2->DR);
  }
  else if (adc_pot_sel < 40 && ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == SET) {
    // POT_TYPE_TIME EOC
    stage = adc_pot_sel - 24;
    WriteTimeSlider(stage, ADC1->DR);
  }
  else if (adc_pot_sel < 56 && ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == SET) {
    // More POT_TYPE_VOLTAGE EOC
    stage = adc_pot_sel - 24;
    WriteVoltageSlider(stage, ADC1->DR);
  } else if (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == SET) {
    // More POT_TYPE_TIME EOC
    stage = adc_pot_sel - 40;
    WriteTimeSlider(stage, ADC1->DR);
  }

  if (Is_Expander_Present()) {
    // Increment the slider, including expander sliders
    adc_pot_sel = ADC_inc_expanded(adc_pot_sel);
  } else {
    // Increments the slider
    adc_pot_sel = ADC_inc(adc_pot_sel);
  }

  // Reading ADCs done
  ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  ADC_ClearFlag(ADC2, ADC_FLAG_EOC);

  delay_us(10);
}


/*
	Setting up acts Timer 2 as source for ADC start conversion
	with 40 ksamples per second.
 */
void mADC_init(void)
{
  GPIO_InitTypeDef GPIO_Init_user;
  ADC_InitTypeDef ADC_InitType;
  TIM_TimeBaseInitTypeDef TimeBaseInit;
  NVIC_InitTypeDef nvicStructure;

  //Timer init
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseStructInit(&TimeBaseInit);
  TimeBaseInit.TIM_Prescaler 			= 1;
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
  GPIO_Init_user.GPIO_Mode 	= GPIO_Mode_AN; //Analog mode
  GPIO_Init(GPIOA, & GPIO_Init_user);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

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

  adc_pot_sel = 0;
  ADC_POTS_selector_Ch(0);

  NVIC_EnableIRQ(ADC_IRQn);
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
  ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
};


void ADCPause(void) {
  NVIC_DisableIRQ(ADC_IRQn);
};

void ADCResume(void) {
  NVIC_EnableIRQ(ADC_IRQn);
};

// External interrupts init for start and stop
void mInterruptInit(void) {
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
  // SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource6);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource7);
  // SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource8);

  // START-STOP LINE INIT Interrupt
  EXTI_DeInit();
  mInt.EXTI_Line = EXTI_Line0|EXTI_Line1|EXTI_Line5|/*EXTI_Line6|*/EXTI_Line7/*|EXTI_Line8*/;
  mInt.EXTI_Mode = EXTI_Mode_Interrupt;
  mInt.EXTI_Trigger = EXTI_Trigger_Rising;
  mInt.EXTI_LineCmd = ENABLE;
  EXTI_Init(&mInt);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority 				= 0x00; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0xF0;
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
  EXTI_ClearITPendingBit(EXTI_Line7);
};

// AFG1 stop interrupt
void EXTI0_IRQHandler() {
  // Handled in the main loop
  // Not used for anything
  EXTI_ClearITPendingBit(EXTI_Line0);	
};

// AFG2 stop interrupt
void EXTI1_IRQHandler() {
  // Handled in the main loop
  // Not used for anything
  EXTI_ClearITPendingBit(EXTI_Line1);
};


// Interrupt handler for strobe signals.
void EXTI9_5_IRQHandler() {
  if (EXTI->PR & (1<<5)) {
    // Strobe signal 1 high
    DoStrobe1();
    EXTI_ClearITPendingBit(EXTI_Line5);
  };

  if (EXTI->PR & (1<<7)) {
    // Strobe 2 signal high
    DoStrobe2();
    EXTI_ClearITPendingBit(EXTI_Line7);
  };
};

/*
	Timer interrupt handler for AFG1 clock.
	Every interrupt of Timer 4 triggers new output voltages and a check if the step has ended.
 */
void TIM4_IRQHandler() {
  AfgTick1();

  // Adjust timer prescaler for time multiplier
  TIM4->PSC = STEP_TIMER_PRESCALER; // (uint16_t) (GetTimeMultiplier1() * STEP_TIMER_PRESCALER);

  // Clear interrupt flag for Timer 4
  TIM4->SR = (uint16_t) ~TIM_IT_Update;
};

/*
  Timer interrupt handler for AFG2 clock.
  Keep in sync with TIM4_IRQHandler().
 */
void TIM5_IRQHandler() {
  AfgTick2();

  // Adjust timer prescaler for time multiplier
  TIM5->PSC = STEP_TIMER_PRESCALER; // (uint16_t) (GetTimeMultiplier2() * STEP_TIMER_PRESCALER);

  // Clear interrupt flag for Timer 5
  TIM5->SR = (uint16_t) ~TIM_IT_Update;
};


#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

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
  myTimer.TIM_Period = 640;// Seq2 pulse duration
  myTimer.TIM_ClockDivision = TIM_CKD_DIV1;
  myTimer.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM8, &myTimer);

  TIM_Cmd(TIM8, DISABLE);
  TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
  NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

};

// Turn off pulses afg 1
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

// Turn off pulses afg 2
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

// Timer Interrupt handler for start scan section 1
// This is only used when timer is started by sustain or enable mode
void TIM3_IRQHandler() {
  TIM3->SR = (uint16_t) ~TIM_IT_Update;

  if (CheckStart1()) TIM3->CR1 &= ~TIM_CR1_CEN;
};

// Timer Interrupt handler for start scan section 1
// This is only used when timer is started by sustain or enable mode
void TIM7_IRQHandler() {
  TIM7->SR = (uint16_t) ~TIM_IT_Update;

  if (CheckStart2()) TIM7->CR1 &= ~TIM_CR1_CEN;
};

// Clear switch scan
void TIM6_DAC_IRQHandler() {
  ControllerCheckClear();
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

  // GPIOs init

  mGPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_4|GPIO_Pin_5;
  mGPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOA, &mGPIO_InitStructure);

  /* DAC Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

  //DAC init
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

void PermutePulses(void)
{
  unsigned int i=0; 
  uButtons myButtons;
  uLeds mLeds;
  mLeds.value[0]  	= 0xFF;
  mLeds.value[1]  	= 0xFF;
  mLeds.value[2]  	= 0xFF;
  mLeds.value[3]  	= 0xFF;
  DISPLAY_LED_I_OFF;
  DISPLAY_LED_II_OFF;

  myButtons.value = GetButton();
  // Flash pulse 1 until one of the switches is activated
  while (myButtons.b.Pulse1On && myButtons.b.Pulse2On) {
    i++;
    if (i<1200) {
      mLeds.b.Pulse1 = 1;
    }
    else if (i<2400) {
      mLeds.b.Pulse1 = 0;
    }
    else {
      i = 0; 
    }
    LEDS_modes_SendStruct(&mLeds);
    myButtons.value = GetButton();
  }
  if (!myButtons.b.Pulse1On) // user is happy with pulse 1 
  {
    swapped_pulses =0;
  }
  else { // user is not happy with pulse 1 so swap
    swapped_pulses = 1;
  }
  // Write selection to memory, reset LEDs and return
  CAT25512_write_block(100*sizeof(steps)+sizeof(cal_constants),&swapped_pulses,1);
  mLeds.b.Pulse1=0;
  LEDS_modes_SendStruct(&mLeds);
  while (!(myButtons.b.Pulse1On && myButtons.b.Pulse2On)) {
    myButtons.value = GetButton(); 
  }
}


void Calibration(void)
{
  unsigned int i=0;
  uButtons myButtons;
  uLeds mLeds;
  myButtons.value = GetButton();

  mLeds.value[0]  	= 0xFF;
  mLeds.value[1]  	= 0xFF;
  mLeds.value[2]  	= 0xFF;
  mLeds.value[3]  	= 0xFF;
  DISPLAY_LED_II_OFF;
  DISPLAY_LED_I_OFF;

  while(myButtons.b.StageAddress2Advance)
  {
    //printf("(myButtons %d \n ",__LINE__);
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

  for(i = 0; i < 8 ; i++)
  {
    cal_constants[i] = add_data[i];
    if(cal_constants[i] < 100) cal_constants[i] = 4095;

  };
  //
  ADCPause();
  //printf("ADCPause %d \n ",__LINE__);
  //Store calibration constants
  CAT25512_write_block(100*sizeof(steps), (unsigned char *) cal_constants, sizeof(cal_constants));
  mADC_init();
  return; //Get outta here properly when calibration is completed. Fixes problem where CalConstant[8] wasn't being calculate properly  SB 4/30/20
  //printf("mADC_init %d \n ",__LINE__);

  while(!myButtons.b.StageAddress2Advance)
  {
    //printf("!myButtons %d \n ",__LINE__);
    i++;
    if(i < 1000)
    {
      mLeds.b.Seq1Run = 1;
      mLeds.b.Seq1Wait = 1;
      mLeds.b.Seq1Stop = 0;
      mLeds.b.Seq2Run = 1;
      mLeds.b.Seq2Wait = 1;
      mLeds.b.Seq2Stop = 0;
    }
    else if(i < 2000)
    {
      mLeds.b.Seq1Run = 0;
      mLeds.b.Seq1Wait = 1;
      mLeds.b.Seq1Stop = 1;
      mLeds.b.Seq2Run = 0;
      mLeds.b.Seq2Wait = 1;
      mLeds.b.Seq2Stop = 1;
    }
    else if(i < 3000)
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

// TODO(maxl0rd): still more init code can be factored out

int main(void)
{
  uButtons myButtons;
  uLeds mLeds;

  /* Reset update states */
  display_update_flags.value = 0x00;
  display_update_flags.b.MainDisplay 	= 1;
  display_update_flags.b.StepsDisplay = 1;

  InitProgram();

  RCC_GetClocksFreq(&RCC_Clocks);

  systickInit(1000);

  PulsesInit();
  DisplayLedsIOInit();

  DipConfig_init();
  dip_config = GetDipConfig();

  // Check for expander and memoize result for Is_Expander_Present() calls
  Init_Expander();

  /* Out dip switch state to panel */
  LED_STEP_init();
  LED_STEP_SendWord(0xFFFF);
  delay_ms(1000);
  LED_STEP_SendWord(0xFFF0|(*((uint8_t*) (&dip_config))));
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

  /* External DAC config */
  MAX5135init();

  /* ADC configuration */
  ADC_POTS_selector_init();
  ADC_POTS_selector_Ch(0);
  mADC_init();

  mTimersInit();
  mInterruptInit();

  InternalDACInit();

  afg1_mode = MODE_STOP;
  afg2_mode = MODE_STOP;

  // Check which version of the MCU we have
  versionInit();
  revision = versionRevised();

  // Scan initial state
  myButtons.value = GetButton();

  if(!myButtons.b.StageAddress1Advance) {
    // If advance 1 switch is pressed, enter calibration loop
    Calibration();
  }
  else if (!myButtons.b.Pulse1On || !myButtons.b.Pulse2On) {
    // Store that we need pulses switched
    PermutePulses();
  } else {
    // If not restore calibration constants from memory
    // Pass in the pointer to the call_constants array from analog_data.c (slightly sketchy)
    CAT25512_read_block(100*sizeof(steps), (unsigned char *) cal_constants, sizeof(cal_constants));
    swapped_pulses = CAT25512_ReadByte(100*sizeof(steps)+sizeof(cal_constants));
    PrecomputeCalibration();
  }

  // Set magic for voltage scaling from dip switch state
  SetVoltageRange(dip_config);

  ControllerMainLoop(); // does not return
};

