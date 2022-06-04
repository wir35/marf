#ifndef __PROGRAM_H
#define __PROGRAM_H

#include <stm32f4xx.h>

#include "data_types.h"
#include "expander.h"
#include "analog_data.h"

// Main steps array data
// This is a global that is accessed throughout every source file (for now) especially main.c
extern volatile uStep steps[2][32];

inline uint8_t get_max_step() {
  return Is_Expander_Present() ? 31 : 15;
}

void WriteVoltageSlider(uint8_t slider_num, uint32_t new_adc_reading);

void WriteTimeSlider(uint8_t slider_num, uint32_t new_adc_reading);

void WriteOtherCv(uint8_t cv_num, uint32_t new_adc_reading);

uint16_t GetStepVoltage(uint8_t section, uint8_t step_num);

uint32_t GetStepWidth(uint8_t section, uint8_t step_num);

#endif
