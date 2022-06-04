#include "program.h"

#include <stm32f4xx.h>
#include "analog_data.h"

// Main step programming
volatile uStep steps[2][32];

void WriteVoltageSlider(uint8_t slider_num, uint32_t new_adc_reading) {
  static volatile uint16_t voltage_smoothers[32];

  uint16_t adc_reading = (uint16_t) (new_adc_reading & 0xfff);
  uint16_t smoothed = apply_voltage_smoother(adc_reading << 4, &voltage_smoothers[slider_num]);

  for (uint8_t j = 0; j < 2; j++) {
    if (steps[j][slider_num].b.WaitVoltageSlider) {
      if (smoothed >> 4 == steps[j][slider_num].b.VLevel >> 4) {
        // Unpin the slider
        steps[j][slider_num].b.WaitVoltageSlider = 0;
        steps[j][slider_num].b.VLevel = smoothed;
      }
    } else {
      steps[j][slider_num].b.VLevel = smoothed;
    }
  }
}

void WriteTimeSlider(uint8_t slider_num, uint32_t new_adc_reading) {
  static volatile uint16_t voltage_smoothers[32];

  uint16_t adc_reading = (uint16_t) (new_adc_reading & 0xfff) << 4;
  uint16_t smoothed = apply_voltage_smoother(adc_reading, &voltage_smoothers[slider_num]);

  for (uint8_t j = 0; j < 2; j++) {
    if (steps[j][slider_num].b.WaitTimeSlider) {
      if (smoothed >> 4 == steps[j][slider_num].b.TLevel >> 4) {
        // Unpin the slider
        steps[j][slider_num].b.WaitTimeSlider = 0;
        steps[j][slider_num].b.TLevel = smoothed;
      }
    } else {
      steps[j][slider_num].b.TLevel = smoothed;
    }
  }
}

// Return the voltage for step number in section
uint16_t GetStepVoltage(uint8_t section, uint8_t step_num) {

  float voltage_level = 0.0; // stay in floating point throughout!
  uint8_t ext_ban_num = 0;

  if (steps[section][step_num].b.VoltageSource) {
    // Step voltage is set externally
    ext_ban_num = steps[section][step_num].b.VLevel >> 10;
    voltage_level = read_calibrated_add_data_float(ext_ban_num);
  } else {
    // Step voltage is set by slider
    voltage_level = steps[section][step_num].b.VLevel;
  };

  // Clamp if smoothing or something has gone awry
  if (voltage_level > 4095.0) {
    voltage_level = 4095.0;
  } else if (voltage_level < 0.0) {
    voltage_level = 0.0;
  }

  if (!steps[section][step_num].b.FullRange) {
    // Scale voltage for limited range
    voltage_level *= limited_range_multiplier;
    if (steps[section][step_num].b.Voltage2) {
      voltage_level += octave_offset;
    } else if (steps[section][step_num].b.Voltage4) {
      voltage_level += octave_offset * 2;
    } else if (steps[section][step_num].b.Voltage6) {
      voltage_level += octave_offset * 3;
    } else if (steps[section][step_num].b.Voltage8) {
      voltage_level += octave_offset * 4;
    }
  }

  if (steps[section][step_num].b.Quantize) {
    // Quantize the output to semitones.
    // Use the precomputed magic values to avoid float divisions
    voltage_level += 0.5 * semitone_offset;
    voltage_level = (float) ((int) (voltage_level * quantizer_magic));
    voltage_level *= semitone_offset;
  }

  return (unsigned int) voltage_level + 0.5;
};

// Return the time for step number in section
uint32_t GetStepWidth(uint8_t section, uint8_t step_num) {
  uint32_t ret_val = 0;
  uint32_t time_level = 0;
  uint8_t ext_ban_num = 0;

  if (steps[section][step_num].b.TimeSource) {
    // Step time is set externally
    ext_ban_num = (uint8_t) steps[section][step_num].b.TLevel >> 10;
    time_level = read_calibrated_add_data_uint16(ext_ban_num);
  } else {
    // Step time is set on panel
    time_level = (steps[section][step_num].b.TLevel + 1);
  };

  // TODO(maxl0rd): factor out divisions

  if (steps[section][step_num].b.TimeRange_p03 == 1) {
    ret_val = (unsigned long int) ((((float) time_level * 112)/4095) +8);
  };

  if (steps[section][step_num].b.TimeRange_p3 == 1) {
    ret_val = (unsigned long int) ((((float) time_level * 1120)/4095) +80);
  };

  if (steps[section][step_num].b.TimeRange_3 == 1) {
    ret_val = (unsigned long int) ((((float) time_level * 11200)/4095) +800);
  };

  if (steps[section][step_num].b.TimeRange_30 == 1) {
    ret_val = (unsigned long int) ((((float) time_level * 112000)/4095) +8000);
  };

  return ret_val;
};
