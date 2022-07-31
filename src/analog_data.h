#ifndef _ANALOG_DATA_H
#define _ANALOG_DATA_H

#include <stm32f4xx.h>
#include "dip_config.h"

#define ADC_EXT_VOLTAGE_A     0x00
#define ADC_EXT_VOLTAGE_B     0x01
#define ADC_EXT_VOLTAGE_C     0x02
#define ADC_EXT_VOLTAGE_D     0x03
#define ADC_TIMEMULTIPLY_Ch_1 0x04
#define ADC_TIMEMULTIPLY_Ch_2 0x05
#define ADC_STAGEADDRESS_Ch_1 0x06
#define ADC_STAGEADDRESS_Ch_2 0x07

// Values of pulse inputs
typedef struct {
  uint8_t start;
  uint8_t stop;
  uint8_t strobe;
} PulseInputs;

PulseInputs PULSE_INPUTS_NONE;

inline uint8_t any_pulses_high(PulseInputs in) {
  return in.start || in.stop || in.strobe;
}

// Additional analog data exposed globally for now

extern volatile uint16_t add_data[8];
extern volatile uint16_t cal_constants[8];

// Calibration scalers for external inputs, precomputed in setup
extern float external_cal[8];

// Precomputed magic numbers for voltage scaling
// In the context of 12 bit range / 0.0 - 4095.0

extern float limited_range_multiplier; // octaves per 10v range
extern float octave_offset; // span of 1 octave
extern float semitone_offset; // span of 1 semitone
extern float quantizer_magic; // reciprocal of semitone_offset

// Voltage smoothers are low pass filters that keep intermediate 16 bit state from smoothed 12 bit readings.
// The filtering increases as the readings converge mainly to reduce jitter noise.

// Applies the voltage smoother to the state var passed.
// The new_reading should already be shifted to a 16 bit value.
// The returned value is shifted back down to 12 bit range.
inline uint16_t apply_voltage_smoother(uint16_t new_reading, volatile uint16_t *state) {
  register uint16_t delta;

  if (new_reading > *state) {
    delta = new_reading - *state;
  } else {
    delta = *state - new_reading;
  }
  if (delta < 128) {
    // Apply a lot of filtering when the reading is close
    *state += (new_reading - *state) >> 4;
  } else if (delta < 512) {
    // More filtering
    *state += (new_reading - *state) >> 3;
  } else if (delta < 1024) {
    // Less filtering
    *state += (new_reading - *state) >> 2;
  } else if (delta < 2048) {
    // Less filtering
    *state += (new_reading - *state) >> 1;
  } else {
    // No filtering
    *state = new_reading;
  }
  return *state >> 4;
}

inline uint16_t read_calibrated_add_data_uint16(uint8_t d) {
  return (uint16_t) ((float) add_data[d]) * external_cal[d] + 0.5;
}

inline float read_calibrated_add_data_float(uint8_t d) {
  return ((float) add_data[d]) * external_cal[d];
}

void WriteOtherCv(uint8_t cv_num, uint32_t new_adc_reading);

void WriteOtherCvWithoutSmoothing(uint8_t cv_num, uint32_t new_adc_reading);

void PrecomputeCalibration(void);

void SetVoltageRange(uDipConfig dip_config);

#define EXTI_LINE_START1  EXTI_Line8
#define EXTI_LINE_STOP1   EXTI_Line0
#define EXTI_LINE_STROBE1 EXTI_Line5

#define EXTI_LINE_START2  EXTI_Line6
#define EXTI_LINE_STOP2   EXTI_Line1
#define EXTI_LINE_STROBE2 EXTI_Line7

// Return the interrupt flag status for each pulse
inline PulseInputs get_afg1_pulse_interrupts() {
  PulseInputs pulse_inputs = {};
  pulse_inputs.start  = EXTI_GetFlagStatus(EXTI_LINE_START1)  == SET;
  pulse_inputs.stop   = EXTI_GetFlagStatus(EXTI_LINE_STOP1)   == SET;
  pulse_inputs.strobe = EXTI_GetFlagStatus(EXTI_LINE_STROBE1) == SET;
  return pulse_inputs;
}

// Return the interrupt flag status for each pulse
inline PulseInputs get_afg2_pulse_interrupts() {
  PulseInputs pulse_inputs = {};
  pulse_inputs.start  = EXTI_GetFlagStatus(EXTI_LINE_START2)  == SET;
  pulse_inputs.stop   = EXTI_GetFlagStatus(EXTI_LINE_STOP2)   == SET;
  pulse_inputs.strobe = EXTI_GetFlagStatus(EXTI_LINE_STROBE2) == SET;
  return pulse_inputs;
}

#endif
