#ifndef __PROGRAM_H
#define __PROGRAM_H

#include <stdbool.h>
#include <stm32f4xx.h>

#include "data_types.h"
#include "expander.h"
#include "analog_data.h"
#include "HC165.h"
#include "constants.h"

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
    unsigned int Unused:4;
  } b;
  unsigned char val[3];
} uStep;

// Slider positions
typedef struct {
    uint16_t VLevel;
    uint16_t TLevel;
} StepSliders;

// Book keeping for which sliders are pinned after loading a stored program.
// This is kept little endian (so LSB is slider 0 and MSB is slider 31).
// High bit means the slider reading is higher than the pin.
// Low bit means the slider reading is lower than the pin value.
// When both high and low bits are unset then the slider is un-pinned.
typedef struct {
  uint32_t high;
  uint32_t low;
} PinnedSliders;

// Afg mode and run state for coordination between controller and display
typedef struct {
  // Run mode, strictly one of the defs above
  uint8_t mode;
  // The step section (normally 0 or 1 if shifted to 16-31)
  uint8_t section;
  // The current step number
  uint8_t step_num;
} AfgControllerState;


// Main steps and sliders array data
// This is extern visible so that we can inline fast access to it, but
// DO NOT ACCESS IT DIRECTLY from any other source file.
extern volatile uStep steps[32];

extern volatile StepSliders sliders[32];
extern volatile PinnedSliders voltage_slider_pins;
extern volatile PinnedSliders time_slider_pins;

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
float GetStepVoltage(uint8_t section, uint8_t step_num);

// The time multiplier panel is marked for log scale (0.5, 1, 2, 4) but linear pots are used.

// Scale the time multipliers to more closely match the panel.
// Use a linear interpolation between the points instead of log2.

inline float scale_time_fake_log2(float linear_val) {
  if (linear_val < 1365.0) {
    // 512 - 1024 or 0.5 - 1
    return linear_val * 0.375 + 512.0;
  } else if (linear_val < 2730) {
    // 1024 - 2048 or 1 - 2
    return (linear_val - 1365) * 0.882 + 1024.0;
  } else {
    // 2048 - 4095 or 2 - 4
    return (linear_val - 2730) * 1.5 + 2048.0;
  }
}

// Get step time slider level as simple linear value
inline uint16_t get_time_slider_level(uint8_t slider_num) {
  return sliders[slider_num].TLevel;
}

uint32_t GetStepWidth(uint8_t section, uint8_t step_num, float time_multiplier);

uint8_t GetNextStep(uint8_t section, uint8_t step_num);

void ApplyProgrammingSwitches(uint8_t section, uint8_t step_num, uButtons *switches);

void ClearProgram(uint8_t section);

inline void pin_all_sliders() {
  voltage_slider_pins.high = 0xFFFFFFFF;
  voltage_slider_pins.low = 0xFFFFFFFF;
  time_slider_pins.high = 0xFFFFFFFF;
  time_slider_pins.low = 0xFFFFFFFF;
}

inline void unpin_all_sliders() {
  voltage_slider_pins.high = 0x00000000;
  voltage_slider_pins.low = 0x00000000;
  time_slider_pins.high = 0x00000000;
  time_slider_pins.low = 0x00000000;
}

#endif
