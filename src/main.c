#include <stm32f4xx.h>
#include "stm32f4xx_syscfg.h"
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_tim.h>
#include <string.h> // memset only

#include "MAX5135.h"
#include "HC165.h"
#include "adc_pots_selector.h"
#include "CAT25512.h"
#include "leds_step.h"
#include "leds_modes.h"
#include "data_types.h"
#include "dip_config.h"
#include "expander.h"
#include "program.h"
#include "delays.h"
#include "analog_data.h"
#include "afg.h"
#include "display.h"
#include "controller.h"
#include "cycle_counter.h"
#include "eprom.h"

// Dip switch state
volatile uDipConfig dip_config;

// Clocks
RCC_ClocksTypeDef RCC_Clocks;

inline FlagStatus ADC_GetFlagStatus_Fast(ADC_TypeDef* ADCx, uint8_t ADC_FLAG) {
  return ((ADCx->SR & ADC_FLAG) != (uint8_t) RESET) ? SET : RESET;
}

// ADC interrupt handler
void ADC_IRQHandler() {
  volatile uint8_t stage = 0;
  __disable_irq();

  uint8_t adc1_eoc = ADC_GetFlagStatus_Fast(ADC1, ADC_FLAG_EOC) == SET;
  uint8_t adc2_eoc = ADC_GetFlagStatus_Fast(ADC2, ADC_FLAG_EOC) == SET;

  if (adc1_eoc) ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  if (adc2_eoc) ADC_ClearFlag(ADC2, ADC_FLAG_EOC);

  if (controller_job_flags.inhibit_adc) {
    __enable_irq();
    return;
  }

  if (controller_job_flags.adc_pot_sel < 16 && adc1_eoc) {
    // POT_TYPE_VOLTAGE EOC
    stage = controller_job_flags.adc_pot_sel;
    WriteVoltageSlider(stage, ADC1->DR);
    controller_job_flags.adc_mux_shift_out = 1;
  }
  else if (controller_job_flags.adc_pot_sel < 24 && adc2_eoc) {
    // POT_TYPE_OTHER EOC
    stage = controller_job_flags.adc_pot_sel - 16;
    WriteOtherCv(stage, ADC2->DR);
    controller_job_flags.adc_mux_shift_out = 1;
  }
  else if (controller_job_flags.adc_pot_sel < 40 && adc1_eoc) {
    // POT_TYPE_TIME EOC
    stage = controller_job_flags.adc_pot_sel - 24;
    WriteTimeSlider(stage, ADC1->DR);
    controller_job_flags.adc_mux_shift_out = 1;
  }
  else if (controller_job_flags.adc_pot_sel < 56 && adc1_eoc) {
    // More POT_TYPE_VOLTAGE EOC
    stage = controller_job_flags.adc_pot_sel - 24;
    WriteVoltageSlider(stage, ADC1->DR);
    controller_job_flags.adc_mux_shift_out = 1;
  } else if (adc1_eoc) {
    // More POT_TYPE_TIME EOC
    stage = controller_job_flags.adc_pot_sel - 40;
    WriteTimeSlider(stage, ADC1->DR);
    controller_job_flags.adc_mux_shift_out = 1;
  }
  __enable_irq();
}


// Init ADCs with timer 2 as source for ADC start conversion
void mADC_init(void)
{
  GPIO_InitTypeDef GPIO_Init_user;
  ADC_InitTypeDef ADC_InitType;
  TIM_TimeBaseInitTypeDef TimeBaseInit;
  NVIC_InitTypeDef nvicStructure;

  //Timer init
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseStructInit(&TimeBaseInit);
  TimeBaseInit.TIM_Prescaler 			= 1000;
  TimeBaseInit.TIM_CounterMode 		= TIM_CounterMode_Up;
  TimeBaseInit.TIM_Period 				= 6;
  TimeBaseInit.TIM_ClockDivision 	= TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2, &TimeBaseInit); 

  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_OC2Ref);
  TIM_CCxCmd(TIM2, TIM_Channel_2, TIM_CCx_Enable);
  TIM_SetCompare2(TIM2, 1);
  TIM2->CCMR1 |= TIM_CCMR1_OC2M;
  TIM_Cmd(TIM2, ENABLE); 

  // ADC Init
  NVIC_SetPriority (ADC_IRQn, 0);

  // ADC GPIO Init
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

  // ADC interrupts init
  nvicStructure.NVIC_IRQChannel = ADC_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 0;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);

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
  // Clear interrupt flag for Timer 4
  TIM4->SR = (uint16_t) ~TIM_IT_Update;
  __disable_irq();

  // Process one time window and return the programmed output levels
  controller_job_flags.afg1_outputs = AfgTick1();

  // Update internal dac (fast)
  DAC_SetChannel1Data(DAC_Align_12b_R, controller_job_flags.afg1_outputs.voltage);

  // Update output pulses
  if (controller_job_flags.afg1_outputs.all_pulses) {
    PULSE_LED_I_ALL_ON;
  } else {
    PULSE_LED_I_ALL_OFF;
  }
  if (controller_job_flags.afg1_outputs.pulse1) {
    PULSE_LED_I_1_ON;
  } else {
    PULSE_LED_I_1_OFF;
  }
  if (controller_job_flags.afg1_outputs.pulse2) {
    PULSE_LED_I_2_ON;
  } else {
    PULSE_LED_I_2_OFF;
  }

  // Set flag to flush time and ref levels to external dac in main loop (slow)
  controller_job_flags.afg1_tick = 1;
  __enable_irq();
};

/*
  Timer interrupt handler for AFG2 clock.
  Keep in sync with TIM4_IRQHandler().
 */
void TIM5_IRQHandler() {
  // Clear interrupt flag for Timer 5
  TIM5->SR = (uint16_t) ~TIM_IT_Update;
  __disable_irq();
  controller_job_flags.afg2_outputs = AfgTick2();

  if (controller_job_flags.afg2_outputs.all_pulses) {
    PULSE_LED_II_ALL_ON;
  } else {
    PULSE_LED_II_ALL_OFF;
  }
  if (controller_job_flags.afg2_outputs.pulse1) {
    PULSE_LED_II_1_ON;
  } else {
    PULSE_LED_II_1_OFF;
  }
  if (controller_job_flags.afg2_outputs.pulse2) {
    PULSE_LED_II_2_ON;
  } else {
    PULSE_LED_II_2_OFF;
  }

  DAC_SetChannel2Data(DAC_Align_12b_R, controller_job_flags.afg2_outputs.voltage);
  controller_job_flags.afg2_tick = 1;
  __enable_irq();
};


#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

// Init Timers 4 and 5 to control afg function generation
void mTimersInit(void) {
  TIM_TimeBaseInitTypeDef myTimer;
  NVIC_InitTypeDef nvicStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  TIM_TimeBaseStructInit(&myTimer);
  myTimer.TIM_Prescaler = AFG_TIMER_PRESCALER;
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
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 1;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);

  nvicStructure.NVIC_IRQChannel = TIM5_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 1;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);

  NVIC_SetPriority (TIM4_IRQn, 1);
  NVIC_SetPriority (TIM5_IRQn, 1);

  SCB->AIRCR = AIRCR_VECTKEY_MASK | NVIC_PriorityGroup_0;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

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

// Timer 8 IRQ. No longer used
void TIM8_UP_TIM13_IRQHandler(void) {
  if(TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET) {
    TIM_Cmd(TIM8, DISABLE);
    TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
  }
}

// Timer 14 IRQ. No longer used.
void TIM8_TRG_COM_TIM14_IRQHandler(void) {
  if(TIM_GetITStatus(TIM14, TIM_IT_Update) != RESET) {
    TIM_Cmd(TIM14, DISABLE);
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

// Init GPIO for pulse outputs
void PulsesGpioInit() {
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

// Init GPIOs for display leds
void DisplayLedsIOInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin 	= DISPLAY_LED_I|DISPLAY_LED_II;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
};

// Init internal DAC
void InternalDACInit(void) {
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


int main(void) {
  uButtons switches;

  // Initialize all the peripherals

  start_cycle_timer();
  InitProgram();
  RCC_GetClocksFreq(&RCC_Clocks);
  systickInit(1000);
  PulsesGpioInit();
  DisplayLedsIOInit();
  DipConfig_init();
  dip_config = GetDipConfig();
  SetVoltageRange(dip_config);
  Init_Expander();
  CAT25512_init();
  EpromInitializeMemoryLayout();
  LEDS_modes_init();
  LED_STEP_init();
  HC165_InitializeGPIO();
  MAX5135_Initialize();
  AdcMuxGpioInitialize();
  AdcMuxResetAllOff();
  mADC_init();
  AdcMuxResetAllOff();
  mTimersInit();
  mInterruptInit();
  InternalDACInit();
  DisplayAllInitialize();

  // Settle down
  delay_ms(50);

  // Scan initial state
  switches.value = HC165_ReadSwitches();

  if (!switches.b.StageAddress1Advance) {
    // If advance 1 switch is pressed, enter calibration loop
    ControllerCalibrationLoop();
  } else {
    ControllerLoadCalibration();
  }

  ControllerMainLoop(); // does not return
};

