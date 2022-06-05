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

volatile uint8_t adc_pot_sel = 0;

#define SEQUENCER_DATA_SIZE (6*2*32)

// Do the pulse LEDs need to be swapped? 
unsigned char swapped_pulses = 0; 

unsigned char edit_mode_step_num = 0;

volatile unsigned char strobe_key = 0;

// Variable used for key lock during the VIEW_MODE key changes steps options
volatile unsigned char key_locked = 0;	
volatile unsigned char keys_not_valid = 0;

//Dip switch state
volatile uDipConfig dip_config;

unsigned char revision; 

// Current patches bank
volatile unsigned char bank = 1;
volatile unsigned char strobe_banana_flag1 = 0, strobe_banana_flag2 = 0;
volatile unsigned int save_counter = 0, load_counter = 0;
volatile unsigned char advanced_counter_1 = 0, advanced_counter_2 = 0;

uint16_t counterL = 0; 
uint16_t counterR = 0;
uint16_t i;
uint16_t tick;

#define KEY_DEBOUNCE_COUNT 3
#define KEY_TIMER 5 // scan switches every 5ms
#define EXTCLOCK_WINDOW 4

uint32_t jackpins = 0;
uint32_t prev_jackpins = 0;
unsigned char start1 = 0;
unsigned char stop1 = 0; 
unsigned char start2 = 0;
unsigned char stop2 = 0;
int swing1 = 0;
int swing2 = 0;

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

// Start Timer 6 for clear switch measurement
void InitClear_Timer() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  TIM6->PSC = 21000;
  TIM6->ARR = 200;
  TIM6->CNT = 0;
  TIM6->DIER = TIM_DIER_UIE;
  TIM6->CR1 |= TIM_CR1_CEN;

  NVIC_EnableIRQ(TIM6_DAC_IRQn);
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
};



// Interrupt handler for strobe signals.
void EXTI9_5_IRQHandler() {
  // Strobe 1
  if (EXTI->PR & (1<<5)) {
    JumpToStep1((unsigned int) (afg1_stage_address));
    EXTI_ClearITPendingBit(EXTI_Line5);
  };

  // Strobe 2
  if (EXTI->PR & (1<<7)) {
    JumpToStep2((unsigned int) (afg2_stage_address));
    EXTI_ClearITPendingBit(EXTI_Line7);
  };
  update_display();
};



/*
	Save current sequence to memory
 */
unsigned char SaveSequence(unsigned char SequenceCell)
{	
  ADCPause();
  if(!Is_Expander_Present())
  {
    CAT25512_write_block(bank*SequenceCell*sizeof(steps), (unsigned char *) steps[0], sizeof(steps[0]));
    CAT25512_write_block(bank*SequenceCell*sizeof(steps)+sizeof(steps[0]), (unsigned char *) steps[1], sizeof(steps[1]));
  }
  else
  {
    CAT25512_write_block((SequenceCell+32)*sizeof(steps), (unsigned char *) steps[0], sizeof(steps[0]));
    CAT25512_write_block((SequenceCell+32)*sizeof(steps)+sizeof(steps[0]), (unsigned char *) steps[1], sizeof(steps[1]));
  }
  mADC_init();
  return 0;
};

//Load a sequence from cell number SequenceCell
void LoadSequence(unsigned char SequenceCell)
{
  unsigned char cnt;

  ADCPause();
  if(!Is_Expander_Present())
  {
    CAT25512_read_block(bank*SequenceCell*sizeof(steps), (unsigned char *) steps[0], sizeof(steps[0]));
    CAT25512_read_block(bank*SequenceCell*sizeof(steps)+sizeof(steps[0]), (unsigned char *) steps[1], sizeof(steps[1]));
  }
  else
  {
    CAT25512_read_block((SequenceCell+32)*sizeof(steps), (unsigned char *) steps[0], sizeof(steps[0]));
    CAT25512_read_block((SequenceCell+32)*sizeof(steps)+sizeof(steps[0]), (unsigned char *) steps[1], sizeof(steps[1]));
  }

  //Block sliders scanning while voltages from slider and preset aren't equal
  if (dip_config.b.SAVE_V_LEVEL == 1) {
    for(cnt=0; cnt<16; cnt++)
    {
      steps[0][cnt].b.WaitVoltageSlider = 1;
      steps[0][cnt].b.WaitTimeSlider = 1;
      steps[1][cnt].b.WaitVoltageSlider = 1;
      steps[1][cnt].b.WaitTimeSlider = 1;
      steps[0][cnt+16].b.WaitVoltageSlider = 1;
      steps[0][cnt+16].b.WaitTimeSlider = 1;
      steps[1][cnt+16].b.WaitVoltageSlider = 1;
      steps[1][cnt+16].b.WaitTimeSlider = 1;
    };
  };

  afg1_mode = MODE_STOP;
  afg2_mode = MODE_STOP;
  if (steps[0][0].b.Swing){
    swing1 = 1;
  }
  if (steps[1][0].b.Swing){
    swing2 = 1;
  }

  mADC_init();

};

/*
	Timer interrupt handler for AFG1 clock.
	Every interrupt of Timer 4 triggers new output voltages and a check if the step has ended.
 */
void TIM4_IRQHandler() {
  AfgTick1();

  // Adjust timer prescaler for time multiplier
  TIM4->PSC = (uint16_t) (GetTimeMultiplier1() * STEP_TIMER_PRESCALER);

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
  TIM5->PSC = (uint16_t) (GetTimeMultiplier2() * STEP_TIMER_PRESCALER);

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

  afg1_step_cnt = 8;
  afg2_step_cnt = 8;
  afg1_step_width = 8;
  afg2_step_width = 8;

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

// Timer Interrupt handler for start scan
// Section 1
// This is only used when timer is started by sustain or enable mode
void TIM3_IRQHandler()
{
  TIM3->SR = (uint16_t) ~TIM_IT_Update;

  // TODO(maxl0rd): definitely refactor this logic into afg.c

  if ( (afg1_mode == MODE_WAIT_HI_Z)
      && (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 0) ) {
  }
  else if((afg1_mode == MODE_WAIT_HI_Z)
      && (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 1))
  {
    afg1_mode = MODE_RUN;
    afg1_step_num = GetNextStep(0, afg1_step_num);
    TIM3->CR1 &= ~TIM_CR1_CEN;
  }

  if((afg1_mode == MODE_STAY_HI_Z)
      && (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 1))
  {


  }
  else if((afg1_mode == MODE_STAY_HI_Z)
      && (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 0))
  {


    afg1_mode = MODE_RUN;
    afg1_step_num = GetNextStep(0, afg1_step_num);
    TIM3->CR1 &= ~TIM_CR1_CEN;
    DoStepOutputPulses1();
  }
};

// Section 2
// This is only used when timer is started by sustain or enable mode

void TIM7_IRQHandler()
{
  TIM7->SR = (uint16_t) ~TIM_IT_Update;

  // TODO(maxl0rd): definitely refactor this logic into afg.c

  if ( (afg2_mode == MODE_WAIT_HI_Z)
      && (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 0) ) {
  }
  else if((afg2_mode == MODE_WAIT_HI_Z)
      && (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 1))
  {
    afg2_mode = MODE_RUN;
    afg2_step_num = GetNextStep(1, afg2_step_num);
    TIM7->CR1 &= ~TIM_CR1_CEN;
  }

  if((afg2_mode == MODE_STAY_HI_Z)
      && (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 1))
  {
  }
  else if((afg2_mode == MODE_STAY_HI_Z)
      && (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 0))
  {
    afg2_mode = MODE_RUN;
    afg2_step_num = GetNextStep(1, afg2_step_num);
    TIM7->CR1 &= ~TIM_CR1_CEN;
    DoStepOutputPulses2();
  }
};

// Clear switch scan
void TIM6_DAC_IRQHandler()
{
  uint8_t i;
  uButtons myButtons;
  static uint8_t clear_counter1 = 0, clear_counter2 = 0;

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

      steps[0][0].val[3] = 0x00;
      steps[0][0].val[4] = 0x00;
      steps[0][0].val[5] = 0x00;
      steps[0][0].b.TimeRange_p3 = 1;
      steps[0][0].b.FullRange = 1;

      for(i=0; i<16; i++)
      {
        steps[0][i] = steps[0][0];
        steps[0][i+16] = steps[0][0];
      }
      afg1_mode = MODE_STOP;
      afg1_step_num = 0;
    }
    else if(clear_counter2 == 30)
    {
      //Clear section 2 state

      steps[1][0].val[3] = 0x00;
      steps[1][0].val[4] = 0x00;
      steps[1][0].val[5] = 0x00;
      steps[1][0].b.TimeRange_p3 = 1;
      steps[1][0].b.FullRange = 1;
      for(i=0; i<16; i++)
      {
        steps[1][i] = steps[1][0];
        steps[1][i+16] = steps[1][0];
      };
      afg2_mode = MODE_STOP;
      afg2_step_num = 0;
    };

    clear_counter1 = 0;
    clear_counter2 = 0;

    //If not in view mode switch in it
    if (display_mode == DISPLAY_MODE_LOAD_1 || display_mode == DISPLAY_MODE_SAVE_1) display_mode = DISPLAY_MODE_VIEW_1;
    if (display_mode == DISPLAY_MODE_LOAD_2 || display_mode == DISPLAY_MODE_SAVE_2) display_mode = DISPLAY_MODE_VIEW_2;
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

// Implement most of the UI logic for switch changes.
// TODO(maxl0rd): figure out a way to break this up.
unsigned char keyb_proc(uButtons * key)
{
  unsigned char StepNum = 0, Section = 0, max_step;
  uStep tmpStep;

  if(Is_Expander_Present()) max_step = 31;
  else max_step = 15;

  /* Determine step num for different DisplayModes */
  if (display_mode == DISPLAY_MODE_VIEW_1) {
    StepNum = afg1_step_num;
    Section = 0;
  };
  if (display_mode == DISPLAY_MODE_VIEW_2) {
    StepNum = afg2_step_num;
    Section = 1;
  };
  if (display_mode == DISPLAY_MODE_EDIT_1) {
    StepNum = edit_mode_step_num;
    Section = 0;
  };
  if (display_mode == DISPLAY_MODE_EDIT_2) {
    StepNum = edit_mode_step_num;
    Section = 1;
  };

  if (display_mode == DISPLAY_MODE_SAVE_1) {
    StepNum = edit_mode_step_num;
    Section = 0;
  };
  if (display_mode == DISPLAY_MODE_SAVE_2) {
    StepNum = edit_mode_step_num;
    Section = 1;
  };
  if (display_mode == DISPLAY_MODE_LOAD_1) {
    StepNum = edit_mode_step_num;
    Section = 0;
  };
  if (display_mode == DISPLAY_MODE_LOAD_2) {
    StepNum = edit_mode_step_num;
    Section = 1;
  };

  tmpStep = steps[Section][StepNum];


  // Programming section
  // TODO(maxl0rd): extract this into a function that takes tmpStep and key

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
    if ((display_mode == DISPLAY_MODE_VIEW_1 || display_mode == DISPLAY_MODE_EDIT_1) && afg1_mode == MODE_STAY_HI_Z) {
      afg1_mode = MODE_STOP;
    };
    if ((display_mode == DISPLAY_MODE_VIEW_2 || display_mode == DISPLAY_MODE_EDIT_2) && afg2_mode == MODE_STAY_HI_Z) {
      afg2_mode = MODE_STOP;
    };
    if ((display_mode == DISPLAY_MODE_VIEW_1 || display_mode == DISPLAY_MODE_EDIT_1) && afg1_mode == MODE_WAIT_HI_Z) {
      afg1_mode = MODE_STOP;
    };
    if ((display_mode == DISPLAY_MODE_VIEW_2 || display_mode == DISPLAY_MODE_EDIT_2) && afg2_mode == MODE_WAIT_HI_Z) {
      afg2_mode = MODE_STOP;
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
    if ((display_mode == DISPLAY_MODE_VIEW_1 || display_mode == DISPLAY_MODE_EDIT_1) && afg1_mode == MODE_WAIT_HI_Z) {
      afg1_mode = afg1_prev_mode;
    };
    if ((display_mode == DISPLAY_MODE_VIEW_2 || display_mode == DISPLAY_MODE_EDIT_2) && afg2_mode == MODE_WAIT_HI_Z) {
      afg2_mode = afg2_prev_mode;
    };
  };

  if ( !key->b.SustainOff ) {
    tmpStep.b.OpModeSUSTAIN = 0;
    if ((display_mode == DISPLAY_MODE_VIEW_1 || display_mode == DISPLAY_MODE_EDIT_1) && afg1_mode == MODE_STAY_HI_Z) {
      afg1_mode = afg1_prev_mode;
    };
    if ((display_mode == DISPLAY_MODE_VIEW_2 || display_mode == DISPLAY_MODE_EDIT_2) && afg2_mode == MODE_STAY_HI_Z) {
      afg2_mode = afg2_prev_mode;
    };
  };

  if ( !key->b.EnableOn ) {
    tmpStep.b.OpModeENABLE = 1;
    tmpStep.b.OpModeSTOP = 0;
    tmpStep.b.OpModeSUSTAIN = 0;
    if ((display_mode == DISPLAY_MODE_VIEW_1 || display_mode == DISPLAY_MODE_EDIT_1) && afg1_mode == MODE_STAY_HI_Z) {
      afg1_mode = afg1_prev_mode;
    };
    if ((display_mode == DISPLAY_MODE_VIEW_2 || display_mode == DISPLAY_MODE_EDIT_2) && afg2_mode == MODE_STAY_HI_Z) {
      afg2_mode = afg2_prev_mode;
    };
  };

  if ( !key->b.EnableOff ) {
    tmpStep.b.OpModeENABLE = 0;
    if ((display_mode == DISPLAY_MODE_VIEW_1 || display_mode == DISPLAY_MODE_EDIT_1) && afg1_mode == MODE_WAIT_HI_Z) {
      afg1_mode = afg1_prev_mode;
    };
    if ((display_mode == DISPLAY_MODE_VIEW_2 || display_mode == DISPLAY_MODE_EDIT_2) && afg2_mode == MODE_WAIT_HI_Z) {
      afg2_mode = afg2_prev_mode;
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


  // TODO(maxl0rd) refactor out clear up processing
  if (!key->b.ClearUp)  {
    //Init timer to detect long press (clear comman)
    InitClear_Timer();
    if (display_mode == DISPLAY_MODE_LOAD_1) {
      //if in load mode, restore sequence number gEditModeStepNum
      LoadSequence(edit_mode_step_num);
      keys_not_valid = 1;
      display_mode = DISPLAY_MODE_VIEW_1;

      afg1_prev_mode = afg1_mode;
      afg1_step_num = 0;
      afg1_mode = MODE_STOP;
    }
    else if (display_mode == DISPLAY_MODE_LOAD_2) {
      //if in load mode, restore sequence number gEditModeStepNum
      LoadSequence(edit_mode_step_num);
      keys_not_valid = 1;
      display_mode = DISPLAY_MODE_VIEW_2;

      afg2_prev_mode = afg2_mode;
      afg2_step_num = 0;
      afg2_mode = MODE_STOP;
    }

    else if (display_mode == DISPLAY_MODE_VIEW_1) {
      //if in view mode switch to load mode
      display_mode = DISPLAY_MODE_LOAD_1;
      edit_mode_step_num = 0;
      display_update_flags.b.StepsDisplay = 1;
      display_update_flags.b.MainDisplay = 1;
    }
    else if (display_mode == DISPLAY_MODE_VIEW_2) {
      //if in view mode switch to load mode
      display_mode = DISPLAY_MODE_LOAD_2;
      edit_mode_step_num = 0;
      display_update_flags.b.StepsDisplay = 1;
      display_update_flags.b.MainDisplay = 1;
    };
  };

  if (!key->b.ClearDown)  {
    InitClear_Timer();
    if (display_mode == DISPLAY_MODE_SAVE_1) {
      //if in save mode - save sequence to memory cell gEditModeStepNum
      SaveSequence(edit_mode_step_num);
      display_mode = DISPLAY_MODE_VIEW_1;
    }
    else if (display_mode == DISPLAY_MODE_SAVE_2) {
      //if in save mode - save sequence to memory cell gEditModeStepNum
      SaveSequence(edit_mode_step_num);
      display_mode = DISPLAY_MODE_VIEW_2;
    }

    else if (display_mode == DISPLAY_MODE_VIEW_1) {
      //if in view mode - switch to save mode
      display_mode = DISPLAY_MODE_SAVE_1;
      edit_mode_step_num = 0;
      display_update_flags.b.StepsDisplay = 1;
      display_update_flags.b.MainDisplay = 1;
    }

    else if (display_mode == DISPLAY_MODE_VIEW_2) {
      //if in view mode - switch to save mode
      display_mode = DISPLAY_MODE_SAVE_2;
      edit_mode_step_num = 0;
      display_update_flags.b.StepsDisplay = 1;
      display_update_flags.b.MainDisplay = 1;

    };
  };


  // TODO: refactor out left/right logic
  //switch to edit mode
  if ( !key->b.StepLeft ) {
    if (display_mode == DISPLAY_MODE_VIEW_1) {
      display_mode = DISPLAY_MODE_EDIT_1;
      edit_mode_step_num = 1;
    };
    if (display_mode == DISPLAY_MODE_VIEW_2) {
      display_mode = DISPLAY_MODE_EDIT_2;
      edit_mode_step_num = 1;
    };
    if ( (display_mode == DISPLAY_MODE_EDIT_1) ||
        (display_mode == DISPLAY_MODE_EDIT_2) ) {
      if (edit_mode_step_num > 0) {
        if(counterL == 0) edit_mode_step_num--;
        //if long press switch to repeate selection
        else if(counterL > 120) {
          counterL = 100;
          edit_mode_step_num--;
        }
        counterL++;
        display_update_flags.b.MainDisplay = 1;
        display_update_flags.b.StepsDisplay = 1;
      } else {
        if(counterL == 0) edit_mode_step_num = max_step;
        else if(counterL > 120)	{
          counterL = 100;
          edit_mode_step_num = max_step;
        }
        counterL++;
        //gEditModeStepNum = max_step;
        display_update_flags.b.MainDisplay = 1;
        display_update_flags.b.StepsDisplay = 1;
      };
    };

    // Split load/save left/right

    //if in save or load mode left buttons select memory cell for save/recall
    if ( (display_mode == DISPLAY_MODE_SAVE_1) || (display_mode == DISPLAY_MODE_SAVE_2) ||
        (display_mode == DISPLAY_MODE_LOAD_1) || (display_mode == DISPLAY_MODE_LOAD_2) ) {
      if (edit_mode_step_num > 0) {
        if(counterL == 0) edit_mode_step_num--;
        else if(counterL > 120) {
          counterL = 100;
          edit_mode_step_num--;
        }
        counterL++;
        display_update_flags.b.StepsDisplay = 1;
      } else {
        if(counterL == 0)
        {
          if(!Is_Expander_Present())
          {
            edit_mode_step_num = 15;
            if(bank == 1)
            {
              bank = 2;
            }
            else
            {
              bank = 1;
            }
          }
          else edit_mode_step_num = 31;
        }
        else if(counterL > 120) {
          counterL = 100;
          if(!Is_Expander_Present())
          {
            edit_mode_step_num = 15;
            if(bank == 1)
            {
              bank = 2;
            }
            else
            {
              bank = 1;
            }
          }
          else edit_mode_step_num = 31;
        }
        counterL++;

        display_update_flags.b.StepsDisplay = 1;
      };
    };
  }
  else
  {
    counterL = 0;
  };

  if ( !key->b.StepRight ) {
    if (display_mode == DISPLAY_MODE_VIEW_1) {
      display_mode = DISPLAY_MODE_EDIT_1;
      edit_mode_step_num = max_step;
    };
    if (display_mode == DISPLAY_MODE_VIEW_2) {
      display_mode = DISPLAY_MODE_EDIT_2;
      edit_mode_step_num = max_step;
    };
    if ( (display_mode == DISPLAY_MODE_EDIT_1) ||
        (display_mode == DISPLAY_MODE_EDIT_2) ) {
      if (edit_mode_step_num < max_step) {
        if(counterR == 0) edit_mode_step_num++;
        else if (counterR > 120)//was 600
        {
          counterR = 100;//was 500
          edit_mode_step_num++;
        }
      } else {
        if(counterR == 0) edit_mode_step_num = 0;
        else if(counterR > 120)
        {
          counterR = 100;
          edit_mode_step_num = 0;
        }
      }
      counterR++;
      display_update_flags.b.MainDisplay = 1;
      display_update_flags.b.StepsDisplay = 1;
    }

    //if in save or load mode right buttons select memory cell for save/recall
    if ( (display_mode == DISPLAY_MODE_SAVE_1) || (display_mode == DISPLAY_MODE_SAVE_2) ||
        (display_mode == DISPLAY_MODE_LOAD_1) || (display_mode == DISPLAY_MODE_LOAD_2)) {
      if (edit_mode_step_num < max_step) {
        if(counterR == 0) edit_mode_step_num++;
        else if(counterR > 120) {
          counterR = 100;
          edit_mode_step_num++;
        }
        counterR++;
        display_update_flags.b.StepsDisplay = 1;
      } else {
        if(counterR == 0)
        {
          if(!Is_Expander_Present())
          {
            edit_mode_step_num = 0;
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
            edit_mode_step_num = 0;
          }
        }
        else if(counterR > 120) {
          counterR = 100;
          if(!Is_Expander_Present())
          {
            edit_mode_step_num = 0;
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
            edit_mode_step_num = 0;
          }
        }
        counterR++;

        display_update_flags.b.StepsDisplay = 1;
      };
    };
  }
  else
  {
    counterR = 0;
  };
  key_locked = 1;

  //Sections 1/2
  if (keys_not_valid == 0) {

    if (!key->b.StageAddress1Display) {
      if (display_mode != DISPLAY_MODE_VIEW_1) {
        display_mode = DISPLAY_MODE_VIEW_1;
      }
    }


    if (!key->b.StageAddress2Display) {
      if (display_mode != DISPLAY_MODE_VIEW_2) {
        display_mode = DISPLAY_MODE_VIEW_2;
        key_locked = 0;
      }
    }
  }

  if (!key->b.StageAddress1Reset) {
    DoReset1();
  }

  if (!key->b.StageAddress2Reset) {
    DoReset2();
  }

  // Refactor out the strobe logic

  if( key->b.Empty5 && strobe_banana_flag1 == 0)
  {
    // Strobe
    // TODO(maxl0rd): call JumpToStep1() here
    swing1 = 0;
    strobe_banana_flag1 = 1;
    afg1_step_num = (unsigned int) (afg1_stage_address);

    if ( display_mode == DISPLAY_MODE_VIEW_1 ) {
      display_update_flags.b.MainDisplay = 1;
      display_update_flags.b.StepsDisplay = 1;
    };

    DoStepOutputPulses1();
  }

  // What key is Empty5 Is it strobe ?!?!?!
  if(!key->b.Empty5)
  {
    strobe_banana_flag1 = 0;
  }
  if ( (!key->b.StageAddress1PulseSelect) ) {
    // TODO: WHAT IS THIS DOING?
    swing1 = 1;
    int cnt1;
    for(cnt1=0; cnt1<31; cnt1++)
    {
      steps[0][cnt1].b.Swing = 1;

    }

    afg1_step_num = (unsigned int) (afg1_stage_address);
    if ( display_mode == DISPLAY_MODE_VIEW_1 ) {
      display_update_flags.b.MainDisplay = 1;
      display_update_flags.b.StepsDisplay = 1;
    };

    DoStepOutputPulses1();
  };


  if( key->b.Empty2 && strobe_banana_flag2 == 0)
  {
    // TODO: Is this ever reached? Doesn't the interrupt fire first?
    // Strobe
    // Call JumpToStep2() here
    strobe_banana_flag2 = 1;


    afg2_step_num = (unsigned int) (afg2_stage_address);
    if ( display_mode == DISPLAY_MODE_VIEW_2 ) {
      display_update_flags.b.MainDisplay = 1;
      display_update_flags.b.StepsDisplay = 1;
    };

    DoStepOutputPulses2();
  }

  if(!key->b.Empty2)
  {
    strobe_banana_flag2 = 0;

  }

  if ( (!key->b.StageAddress2PulseSelect)) {
    // WHAT DOES THIS DO?
    swing2 = 1;

    int cnt1;
    for(cnt1=0; cnt1<31; cnt1++)
    {
      steps[1][cnt1].b.Swing = 1;

    }
    afg2_step_num = (unsigned int) (afg2_stage_address);
    if ( display_mode == DISPLAY_MODE_VIEW_2 ) {
      display_update_flags.b.MainDisplay = 1;
      display_update_flags.b.StepsDisplay = 1;
    };

    DoStepOutputPulses2();
  };


  if (!key->b.StageAddress1ContiniousSelect) {
    EnableContinuousStageAddress1();
    key_locked = 0;
  } else {
    DisableContinuousStageAddress1();
    key_locked = 0;
  }
  if (!key->b.StageAddress2ContiniousSelect) {
    EnableContinuousStageAddress2();
    key_locked = 0;
  } else {
    DisableContinuousStageAddress2();
    key_locked = 0;
  }

  if (!key->b.StageAddress1Advance) {
    DoAdvance1();
  }
  if (!key->b.StageAddress2Advance) {
    DoAdvance2();
  }

  if (keys_not_valid == 0) {
    steps[Section][StepNum] = tmpStep;
    display_update_flags.b.MainDisplay = 1;
  } else {
    keys_not_valid = 0;
  };

  return 1;
}


/*
	Update leds function
	TODO: refactor this whole function into display.c
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

  if ((afg1_mode == MODE_RUN) ||
      (afg1_mode == MODE_ADVANCE)) {
    mLeds.b.Seq1Run = 0;
  };
  if ( (afg1_mode == MODE_WAIT) ||
      (afg1_mode == MODE_WAIT_HI_Z ) ||
      (afg1_mode == MODE_STAY_HI_Z)  ) {
    mLeds.b.Seq1Wait = 0;
  };

  if (afg1_mode == MODE_STOP) {
    mLeds.b.Seq1Stop = 0;
  };

  if ((afg2_mode == MODE_RUN) ||
      (afg2_mode == MODE_ADVANCE) ) {
    mLeds.b.Seq2Run = 0;
  };
  if ((afg2_mode == MODE_WAIT) ||
      (afg2_mode == MODE_WAIT_HI_Z ) ||
      (afg2_mode == MODE_STAY_HI_Z) ) {
    mLeds.b.Seq2Wait = 0;
  };
  if (afg2_mode == MODE_STOP) {
    mLeds.b.Seq2Stop = 0;
  };


  /* Determine step num for different DisplayModes*/
  if ( display_mode == DISPLAY_MODE_VIEW_1 ) {
    StepNum = afg1_step_num;
    Section = 0;
  }
  if ( display_mode == DISPLAY_MODE_VIEW_2 ) {
    StepNum = afg2_step_num;
    Section = 1;
  };
  if ( display_mode == DISPLAY_MODE_EDIT_1 ) {
    StepNum = edit_mode_step_num;
    Section = 0;
  }
  if ( display_mode == DISPLAY_MODE_EDIT_2 ) {
    StepNum = edit_mode_step_num;
    Section = 1;
  };

  mStep = (uStep*) &steps[Section][StepNum];

  mLeds.b.VoltageFull  	= ~mStep->b.FullRange;
  mLeds.b.Voltage0     	= ~mStep->b.Voltage0;
  mLeds.b.Voltage2     	= ~mStep->b.Voltage2;
  mLeds.b.Voltage4     	= ~mStep->b.Voltage4;
  mLeds.b.Voltage6     	= ~mStep->b.Voltage6;
  mLeds.b.Voltage8     	= ~mStep->b.Voltage8;
  if (swapped_pulses) {
    mLeds.b.Pulse1       	= ~mStep->b.OutputPulse2; //hack for Gate1 Gate2 leds
    mLeds.b.Pulse2       	= ~mStep->b.OutputPulse1;
  }
  else {
    mLeds.b.Pulse1       	= ~mStep->b.OutputPulse1;
    mLeds.b.Pulse2       	= ~mStep->b.OutputPulse2;
  }
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

  if ( (display_mode == DISPLAY_MODE_SAVE_1) || (display_mode == DISPLAY_MODE_SAVE_2) ||
      (display_mode == DISPLAY_MODE_LOAD_1) || (display_mode == DISPLAY_MODE_LOAD_2) ) {
    mLeds.value[0] = 0xFF;
    mLeds.value[1] = 0xFF;
    mLeds.value[2] = 0xFF;
    mLeds.value[3] = 0xFF;

    if((display_mode == DISPLAY_MODE_SAVE_1) || (display_mode == DISPLAY_MODE_SAVE_2))
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
    else if((display_mode == DISPLAY_MODE_LOAD_1) || (display_mode == DISPLAY_MODE_LOAD_2))
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

  if ( (display_mode == DISPLAY_MODE_VIEW_1) ||
      (display_mode == DISPLAY_MODE_EDIT_1) ) {
    DISPLAY_LED_I_ON;
    DISPLAY_LED_II_OFF;
  };

  if ( (display_mode == DISPLAY_MODE_VIEW_2) ||
      (display_mode == DISPLAY_MODE_EDIT_2) ) {
    DISPLAY_LED_II_ON;
    DISPLAY_LED_I_OFF;
  };
};

/*
	Steps section leds update function
	TODO(maxl0rd): refactor into display.c
 */
void UpdateStepSection(void)
{
  if ( display_mode == DISPLAY_MODE_VIEW_1 ) {
    LED_STEP_LightStep(afg1_step_num);
  };
  if ( display_mode == DISPLAY_MODE_VIEW_2 ) {
    LED_STEP_LightStep(afg2_step_num);
  };
  if ( ( display_mode == DISPLAY_MODE_EDIT_1 ) ||
      ( display_mode == DISPLAY_MODE_EDIT_2 ) ||
      ( display_mode == DISPLAY_MODE_SAVE_1 ) ||
      ( display_mode == DISPLAY_MODE_SAVE_2 ) ||
      (display_mode == DISPLAY_MODE_LOAD_1) ||
      (display_mode == DISPLAY_MODE_LOAD_2)
  ) {
    LED_STEP_LightStep(edit_mode_step_num);
  };
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
{	//printf("%d \n ",__LINE__);
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

RCC_ClocksTypeDef RCC_Clocks;

int main(void)
{
  uButtons myButtons;
  uLeds mLeds;
  unsigned char _cnt;
  volatile unsigned long long int key_state, prev_key_state, raw_key_state;
  uint32_t key_timestamp=0;
  uint16_t keys_debounce =0;
  unsigned char keys_stable = 0;

  /* Reset update states */
  display_update_flags.value = 0x00;
  display_update_flags.b.MainDisplay 	= 1;
  display_update_flags.b.StepsDisplay = 1;

  /* Init steps structures */
  steps[0][0].b.TimeRange_p3 = 1;
  steps[0][0].b.FullRange = 1;
  steps[0][0].b.Swing = 0;
  steps[1][0] = steps[0][0];


  for(_cnt=1;_cnt<=31;_cnt++)
  {
    steps[0][_cnt] = steps[0][0];
    steps[1][_cnt] = steps[0][0];
  };

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

  afg1_mode = MODE_STOP;
  afg2_mode = MODE_STOP;

  // Check which version of the MCU we have
  versionInit();
  revision = versionRevised();

  // Scan initial state
  key_state = GetButton();
  myButtons.value = key_state;

  if(!myButtons.b.StageAddress1Advance)
  {
    //if advance switch is pressed start calibration
    Calibration();
  }
  else if (!myButtons.b.Pulse1Off && !myButtons.b.Pulse2Off)
    // set up pulse LED permutation
  {
    PermutePulses();
  }
  else
  {
    // If not restore calibration constants from memory
    // Pass in the pointer to the call_constants array from analog_data.c (slightly sketchy)
    CAT25512_read_block(100*sizeof(steps), (unsigned char *) cal_constants, sizeof(cal_constants));
    PrecomputeCalibration();

    swapped_pulses = CAT25512_ReadByte(100*sizeof(steps)+sizeof(cal_constants));
  }

  // Set magic for voltage scaling from dip switch state
  SetVoltageRange(dip_config);

  while (1) {

    /* keys proceed */
    //		if (KeyThreshHoldCnt == 0) {
    if ((uint16_t)(get_millis() - key_timestamp) > KEY_TIMER) { // time to scan the switches
      raw_key_state = GetButton();
      key_timestamp = get_millis();
      if (raw_key_state == key_state) {
        if (--keys_debounce == 0) { // stable now
          keys_stable = 1;
          keys_debounce = KEY_DEBOUNCE_COUNT;
        }
      }
      else {
        keys_debounce = KEY_DEBOUNCE_COUNT;
        key_state = raw_key_state;
        keys_stable = 0;
      }
      if (keys_stable) {
        myButtons.value = key_state;
        if (key_state != prev_key_state || myButtons.b.StepRight == 0 || myButtons.b.StepLeft == 0) {
          tick = 0;
          keyb_proc(&myButtons);
          tick = 1;
          prev_key_state = key_state;
        }
      }
    };

    // Update panel state
    // TODO(maxl0rd): Refactor into display.c
    if (display_update_flags.b.MainDisplay) {
      UpdateModeSection();
      display_update_flags.b.MainDisplay = 0;
      if ( 	(display_mode == DISPLAY_MODE_SAVE_1) || (display_mode == DISPLAY_MODE_SAVE_2) ||
          (display_mode == DISPLAY_MODE_LOAD_1) || (display_mode == DISPLAY_MODE_LOAD_2) )
      {
        display_update_flags.b.MainDisplay = 1;
      }
    };
    if (display_update_flags.b.StepsDisplay) {
      UpdateStepSection();
      display_update_flags.b.StepsDisplay = 0;
    };

    // Compute continuous stage address
    ComputeContinuousStep1();
    ComputeContinuousStep2();

    // Process Stop and Start signals
    prev_jackpins = jackpins;
    jackpins = GPIO_ReadInputData(GPIOB);
    if (!(prev_jackpins & 1) && (jackpins & 1)) stop1 = EXTCLOCK_WINDOW; // stop jack rising edge
    if (!(prev_jackpins & (1<<8)) && (jackpins & (1<<8))) start1 = EXTCLOCK_WINDOW; // start jack rising edge
    if (stop1 && start1) {
      // Both start + stop high triggers an advance, same as the advance switch
      DoAdvance1();
      stop1 = 0;
      start1 = 0;
    }
    else if (stop1) {
      if (--stop1 == 0) {
        // stop1 window timed out
        DoStop1();
      }
    }
    else if (start1) {
      if (--start1 == 0) {
        // start1 window timed out
        DoStart1();
      }
    }

    if (!(prev_jackpins & (1<<1)) && (jackpins & (1<<1))) stop2 = EXTCLOCK_WINDOW; // stop jack rising edge
    if (!(prev_jackpins & (1<<6)) && (jackpins & (1<<6))) start2 = EXTCLOCK_WINDOW; // start jack rising edge
    if (stop2 && start2) {
      // Both start + stop high triggers an advance, same as the advance switch
      DoAdvance2();
      stop2 = 0;
      start2 = 0;
    }
    else if (stop2) {
      if (--stop2 == 0) { // stop1 window timed out
        DoStop2();
      }
    }
    else if (start2) {
      if (--start2 == 0) { // start1 window timed out
        DoStart2();
      }
    }

  }; // end main loop
};

