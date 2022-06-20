#ifndef __PROGRAM_H
#define __PROGRAM_H

#include <stdbool.h>
#include <stm32f4xx.h>

#include "data_types.h"
#include "expander.h"
#include "analog_data.h"
#include "HC165.h"

// Main structure for step data type
typedef union
{
  struct {
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
  unsigned char val[3];
} uStep;

typedef struct {
    unsigned int VLevel:12;
    unsigned int TLevel:12;
} StepSliders;

// Main steps and sliders array data
// This is extern visible so that we can inline fast access to it, but
// DO NOT ACCESS IT DIRECTLY from any other source file.
extern volatile uStep steps[32];
extern volatile StepSliders sliders[32];

void InitProgram();

inline uint8_t get_max_step() {
  return Is_Expander_Present() ? 31 : 15;
}

// A 12bit value shifted into range for 32 or 16 step selection
inline uint8_t get_max_step_shift12() {
  return Is_Expander_Present() ? 7 : 8;
}

inline uStep get_step_programming(uint8_t section, uint8_t step_num) {
  step_num += section << 4;
  return steps[step_num];
}

void WriteVoltageSlider(uint8_t slider_num, uint32_t new_adc_reading);

void WriteTimeSlider(uint8_t slider_num, uint32_t new_adc_reading);

void WriteOtherCv(uint8_t cv_num, uint32_t new_adc_reading);

// Return the voltage for step number in section
inline uint16_t GetStepVoltage(uint8_t section, uint8_t step_num) {

  float voltage_level = 0.0; // stay in floating point throughout!
  uint8_t ext_ban_num = 0;
  uint8_t slider_num = step_num;

  step_num += section << 4; // section select

  if (steps[step_num].b.VoltageSource) {
    // Step voltage is set externally
    ext_ban_num = sliders[slider_num].VLevel >> 10;
    voltage_level = read_calibrated_add_data_float(ext_ban_num);
  } else {
    // Step voltage is set by slider
    voltage_level = sliders[slider_num].VLevel;
  };

  // Clamp if smoothing or something has gone awry
  if (voltage_level > 4095.0) {
    voltage_level = 4095.0;
  } else if (voltage_level < 0.0) {
    voltage_level = 0.0;
  }

  if (!steps[step_num].b.FullRange) {
    // Scale voltage for limited range
    voltage_level *= limited_range_multiplier;
    if (steps[step_num].b.Voltage2) {
      voltage_level += octave_offset;
    } else if (steps[step_num].b.Voltage4) {
      voltage_level += octave_offset * 2;
    } else if (steps[step_num].b.Voltage6) {
      voltage_level += octave_offset * 3;
    } else if (steps[step_num].b.Voltage8) {
      voltage_level += octave_offset * 4;
    }
  }

  if (steps[step_num].b.Quantize) {
    // Quantize the output to semitones.
    // Use the precomputed magic values to avoid float divisions
    voltage_level += 0.5 * semitone_offset;
    voltage_level = (float) ((int) (voltage_level * quantizer_magic));
    voltage_level *= semitone_offset;
  }

  return (unsigned int) voltage_level + 0.5;
};

inline uint16_t GetStepTime(uint8_t section, uint8_t step_num) {
  float time_level = 0.0;
  uint8_t ext_ban_num = 0;
  uint8_t slider_num = step_num;

  step_num += section << 4; // section select

  if (steps[step_num].b.TimeSource) {
    // Step time is set externally
    ext_ban_num = (uint8_t) sliders[slider_num].TLevel >> 10;
    time_level = read_calibrated_add_data_uint16(ext_ban_num);
  } else {
    // Step time is set on panel
    time_level = sliders[slider_num].TLevel;
  };
  return time_level;
}
// Timing constants

// AFG updates its state at 32kHz
#define AFG_TICK_FREQUENCY  32000

// Prescaler value for the timers that drive the AFG ticks
#define AFG_TIMER_PRESCALER ( 168000000 / AFG_TICK_FREQUENCY / 4 ) // 4 = APB prescaler

#define STEP_WIDTH_2S (AFG_TICK_FREQUENCY * 2.0)
#define STEP_WIDTH_28S (AFG_TICK_FREQUENCY * 28.0)
#define RECIPROCAL_12BIT (1.0 / 4096.0)

#define PULSE_ACTIVE_STEP_WIDTH (AFG_TICK_FREQUENCY / 1000) // 1 ms

inline uint32_t GetStepWidth(uint8_t section, uint8_t step_num, float time_multiplier) {
  float step_width = 0.0;
  float time_level = 0.0;
  uint8_t ext_ban_num = 0;
  uint8_t slider_num = step_num;

  step_num += section << 4; // section select

  if (steps[step_num].b.TimeSource) {
    // Step time is set externally
    ext_ban_num = (uint8_t) sliders[slider_num].TLevel >> 10;
    time_level = read_calibrated_add_data_float(ext_ban_num);
  } else {
    // Step time is set on panel
    time_level = (float) sliders[slider_num].TLevel;
  };

  // This magic number is 112000/4095
  // This is the step width for the 2-30 range
  step_width = (time_level * RECIPROCAL_12BIT * STEP_WIDTH_28S) + STEP_WIDTH_2S;

  if (steps[step_num].b.TimeRange_p03 == 1) {
    step_width *= 0.001;
  } else if (steps[step_num].b.TimeRange_p3 == 1) {
    step_width *= 0.01;
  } else if (steps[step_num].b.TimeRange_3 == 1) {
    step_width *= 0.1;
  }

  return (uint32_t) (step_width * time_multiplier + 0.5);
};

uint8_t GetNextStep(uint8_t section, uint8_t step_num);

void ApplyProgrammingSwitches(uint8_t section, uint8_t step_num, uButtons *switches);

void ClearProgram(uint8_t section);

void PinSliders();

#endif
