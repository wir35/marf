#include "adc_pots_selector.h"

#include <stm32f4xx_gpio.h>
#include <stdlib.h>
#include <string.h> // memset

#include "delays.h"

#define ADC_PS_SH_PIN	GPIO_Pin_13
#define ADC_PS_DS_PIN	GPIO_Pin_14
#define ADC_PS_ST_PIN	GPIO_Pin_15

#define ADC_POTS_SELECTOR_SHIFT_HIGH		GPIOC->BSRRL = ADC_PS_SH_PIN
#define ADC_POTS_SELECTOR_SHIFT_LOW			GPIOC->BSRRH = ADC_PS_SH_PIN

#define ADC_POTS_SELECTOR_STORAGE_HIGH	GPIOC->BSRRL = ADC_PS_ST_PIN
#define ADC_POTS_SELECTOR_STORAGE_LOW		GPIOC->BSRRH = ADC_PS_ST_PIN

#define ADC_POTS_SELECTOR_DATA_HIGH			GPIOC->BSRRL = ADC_PS_DS_PIN
#define ADC_POTS_SELECTOR_DATA_LOW			GPIOC->BSRRH = ADC_PS_DS_PIN

// Data for ADC channels selection
// No longer used, but a useful reference.

#if 0
const uint64_t adc_mux_channel_select_data[72] = {
	0xFFFFFFF0, //Time1 CH
	0xFFFFFFF1, //Time2 CH
	0xFFFFFFF2, //Time3 CH
	0xFFFFFFF3, //Time4 CH
	0xFFFFFFF4, //Time5 CH
	0xFFFFFFF5, //Time6 CH
	0xFFFFFFF6, //Time7 CH
	0xFFFFFFF7, //Time8 CH
	0xFFFFFF0F, //Time9 CH
	0xFFFFFF1F, //Time10 CH
	0xFFFFFF2F, //Time11 CH
	0xFFFFFF3F, //Time12 CH
	0xFFFFFF4F, //Time13 CH
	0xFFFFFF5F, //Time14 CH
	0xFFFFFF6F, //Time15 CH
	0xFFFFFF7F, //Time16 CH
	0xFFFFF0F0, //EXT INPUT A
	0xFFFFF1F1, //EXT INPUT B
	0xFFFFF2F2, //EXT INPUT C
	0xFFFFF3F3, //EXT INPUT D
	0xFFFFF4F4, //TIME MULT 1
	0xFFFFF5F5, //TIME MULT 2
	0xFFFFF6F6, //EXT STAGE 1
	0xFFFFF7F7, //EXT STAGE 2
	// Both are technically correct since the last byte is irrelevant to that mux
	/* 0xFFFFF0FF, //EXT INPUT A */
	/* 0xFFFFF1FF, //EXT INPUT B */
	/* 0xFFFFF2FF, //EXT INPUT C */
	/* 0xFFFFF3FF, //EXT INPUT D */
	/* 0xFFFFF4FF, //TIME MULT 1 */
	/* 0xFFFFF5FF, //TIME MULT 2 */
	/* 0xFFFFF6FF, //EXT STAGE 1 */
	/* 0xFFFFF7FF, //EXT STAGE 2 */
	0xFFF0FFFF, //Volt1 CH
	0xFFF1FFFF, //Volt2 CH
	0xFFF2FFFF, //Volt3 CH
	0xFFF3FFFF, //Volt4 CH
	0xFFF4FFFF, //Volt5 CH
	0xFFF5FFFF, //Volt6 CH
	0xFFF6FFFF, //Volt7 CH
	0xFFF7FFFF, //Volt8 CH
	0xFF0FFFFF, //Volt9 CH
	0xFF1FFFFF, //Volt10 CH
	0xFF2FFFFF, //Volt11 CH
	0xFF3FFFFF, //Volt12 CH
	0xFF4FFFFF, //Volt13 CH
	0xFF5FFFFF, //Volt14 CH
	0xFF6FFFFF, //Volt15 CH
	0xFF7FFFFF, //Volt16 CH
	0xF0FFFFFF,
	0xF1FFFFFF,
	0xF2FFFFFF,
	0xF3FFFFFF,
	0xF4FFFFFF,
	0xF5FFFFFF,
	0xF6FFFFFF,
	0xF7FFFFFF,
	0x0FFFFFFF,
	0x1FFFFFFF,
	0x2FFFFFFF,
	0x3FFFFFFF,
	0x4FFFFFFF,
	0x5FFFFFFF,
	0x6FFFFFFF,
	0x7FFFFFFF,
	0x0FFFFFFFF,
	0x1FFFFFFFF,
	0x2FFFFFFFF,
	0x3FFFFFFFF,
	0x4FFFFFFFF,
	0x5FFFFFFFF,
	0x6FFFFFFFF,
	0x7FFFFFFFF,
	0x0FFFFFFFFF,
	0x1FFFFFFFFF,
	0x2FFFFFFFFF,
	0x3FFFFFFFFF,
	0x4FFFFFFFFF,
	0x5FFFFFFFFF,
	0x6FFFFFFFFF,
	0x7FFFFFFFFF
	
	//0xFFFFFFFF	//ALL CHANNELS OFF
};
#endif


// Send one byte to the shift registers that drive the ADC multiplexers
inline static void adc_mux_send_byte(const uint8_t data) {
  uint8_t dat = data;

  for (uint8_t cnt = 0; cnt < 8; cnt++) {
    if ((dat & 0x80) > 0) {
      ADC_POTS_SELECTOR_DATA_HIGH;
    } else {
      ADC_POTS_SELECTOR_DATA_LOW;
    }
    ADC_POTS_SELECTOR_SHIFT_LOW;
    DELAY_NOPS_120NS();
    ADC_POTS_SELECTOR_SHIFT_HIGH;
    DELAY_NOPS_120NS();
    dat = dat << 1;
  }
  ADC_POTS_SELECTOR_DATA_LOW;
}

// Send one half byte to the shift registers that drive the ADC multiplexers
inline static void adc_mux_send_nibble(const uint8_t data) {
  uint8_t dat = data;

  for(uint8_t cnt = 0; cnt < 4; cnt++) {
    if ((dat & 0x8) > 0) {
      ADC_POTS_SELECTOR_DATA_HIGH;
    } else {
      ADC_POTS_SELECTOR_DATA_LOW;
    }
    ADC_POTS_SELECTOR_SHIFT_LOW;
    DELAY_NOPS_120NS();
    ADC_POTS_SELECTOR_SHIFT_HIGH;
    DELAY_NOPS_120NS();
    dat = dat << 1;
  }
  ADC_POTS_SELECTOR_DATA_LOW;
}

// Send five bytes to the shift registers that drive the ADC multiplexers.
// This updates all five shift registers in an expanded module.
inline static void adc_mux_send_word(const unsigned long long int data) {
  ADC_POTS_SELECTOR_STORAGE_LOW;
  adc_mux_send_byte((uint8_t) ((data&0xFF00000000) >>32));
  adc_mux_send_byte((uint8_t) ((data&0x00FF000000) >>24));
  adc_mux_send_byte((uint8_t) ( data&0x00000000FF));
  adc_mux_send_byte((uint8_t) ((data&0x000000FF00) >>8));
  adc_mux_send_byte((uint8_t) ((data&0x0000FF0000) >>16));
  ADC_POTS_SELECTOR_STORAGE_HIGH;
}

void AdcMuxResetAllOff() {
  adc_mux_send_word(0xFFFFFFFFFF);
}

uint8_t AdcMuxReset() {
  adc_mux_send_word(0xFFFFFFFFF0);
  return 0;
}

// Init GPIOs for ADC channels multiplexers
void AdcMuxGpioInitialize(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitStructure));
	GPIO_InitStructure.GPIO_Pin 	= ADC_PS_SH_PIN|ADC_PS_ST_PIN|ADC_PS_DS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// Select next ADC channel and return its index. For unexpanded modules.
// Muxes are addressed by three shift registers in series as follows:
//   1. time sliders aka pots 24-39
//   2. (first half) external voltages aka pots 16-23
//   2. (unused half a shift register)
//   3. voltage sliders  aka pots 0 - 15

uint8_t AdcMuxAdvance(uint8_t pot) {
  uint8_t next_pot, channel;

  // Here is a fancy hack.
  // The lowest three bits of the pot number (0-7) tell which channels of the mux (A,B,C) are live
  channel = pot & 0x7;

  ADC_POTS_SELECTOR_STORAGE_LOW;

  // Here is an even fancier hack. This addressing scheme requires shifting at most 1 byte every change, rather than 3.
  if (pot >= 8 && pot < 16) {
    // On the final set of time sliders
    channel = (channel + 1) & 0x7;   // increment the channel and wrap
    adc_mux_send_nibble(channel);    //
    next_pot = 24 + channel;         // time slider 1-8 are pots 24 - 31.
  }
  else if (pot >=16 && pot < 24) {
    // We're on the external mux
    adc_mux_send_byte(0xFF);        // Need to shift twice to get to the voltage slider
    next_pot = channel;             // voltage sliders are pots 0-7
  }
  else {
    adc_mux_send_nibble(0xF);       // just shift to next mux
    if (pot >= 32 && pot < 40) {
      // Final time sliders, external voltages are next
      next_pot = pot - 16;
    } else {
      next_pot = pot + 8;           // shift just moves pot count along by 8
    }
  }
  // Activate the shift registers with the new data
  ADC_POTS_SELECTOR_STORAGE_HIGH;
  DELAY_NOPS_120NS();
  return next_pot;
}

// Select next ADC channel and return its index. For expanded modules.
// The logic is the same as above with an additional two more registers.
// Muxes are addressed by five shift registers in series as follows:
//   1. time sliders aka pots 24-39
//   2. (first half) external voltages aka pots 16-23
//   2. (unused half a shift register)
//   3. voltage sliders  aka pots 0 - 15
//   4. expander voltage sliders aka pots 40 - 55
//   5. expander time sliders aka pots 56 - 71

uint8_t AdcMuxAdvanceExpanded(uint8_t pot) {
  uint8_t next_pot, channel;

  channel = pot & 0x7;
  if (pot >= 64 && pot < 72) {
    // Final register
    channel = (channel + 1) & 0x7;
    adc_mux_send_nibble(channel);
    next_pot = 24 + channel;  // time slider 1-8 are pots 24 - 31.
  }
  else if (pot >=16 && pot < 24) {
    // Second (external) register
    adc_mux_send_byte(0xFF);
    next_pot = 0 + channel; // voltage sliders are pots 0-7
  }
  else {
    adc_mux_send_nibble(0xF); // just shift to next mux
    if (pot >= 32 && pot < 40) {
      // final time sliders, external voltages are next
      next_pot = pot - 16;
    } else if (pot >=8 && pot < 16) {
      // from mux 3 to mux 4
      next_pot = pot + 32; 
    } else  {
      next_pot = pot + 8; // otherwise shift 8
    }
  }
  // activate the shift register with the new data
  ADC_POTS_SELECTOR_STORAGE_LOW;
  DELAY_CLOCK_20();  // Slightly longer settling time
  ADC_POTS_SELECTOR_STORAGE_HIGH;
  return next_pot; 
}

// Select one of the adc2 channels explicitly (0-7)
void AdcMuxSelectAdc2(uint8_t pot) {
  ADC_POTS_SELECTOR_STORAGE_LOW;

  adc_mux_send_nibble(0xF); // Unused nibble
  adc_mux_send_nibble(pot); // Select the channel
  adc_mux_send_byte(0xFF);  // Shift

  // Activate the shift registers with the new data
  ADC_POTS_SELECTOR_STORAGE_HIGH;
  DELAY_NOPS_120NS();
}
