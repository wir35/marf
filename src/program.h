#ifndef __PROGRAM_H
#define __PROGRAM_H

#include <stdbool.h>
#include <stm32f4xx.h>

#include "data_types.h"
#include "expander.h"
#include "analog_data.h"

// Main structure for step data type
// TODO (maxl0rd) eventually separate VLevel and TLevel out of this
// into its own Sliders struct. These don't belong together in the same model.
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

// Main steps array data
// This is a global that is accessed throughout every source file (for now) especially main.c
extern volatile uStep steps[2][32];

inline uint8_t get_max_step() {
  return Is_Expander_Present() ? 31 : 15;
}

// A 12bit value shifted into range for 32 or 16 step selection
inline uint8_t get_max_step_shift12() {
  return Is_Expander_Present() ? 7 : 8;
}

void WriteVoltageSlider(uint8_t slider_num, uint32_t new_adc_reading);

void WriteTimeSlider(uint8_t slider_num, uint32_t new_adc_reading);

void WriteOtherCv(uint8_t cv_num, uint32_t new_adc_reading);

uint16_t GetStepVoltage(uint8_t section, uint8_t step_num);

uint32_t GetStepWidth(uint8_t section, uint8_t step_num);

uint8_t GetNextStep(uint8_t section, uint8_t step_num);

#endif
