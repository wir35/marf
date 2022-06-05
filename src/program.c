#include "program.h"

#include <stm32f4xx.h>

#include "analog_data.h"
#include "HC165.h"

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

// Return the step width in ticks for step number.

// This is how this math works:
// Each tick is 250 usec.
// The minimum value is 0.002s or 2ms or 8 ticks
// The maximum range is 2s-30s which is 112000 ticks.
uint32_t GetStepWidth(uint8_t section, uint8_t step_num) {
  float time_level = 0.0, step_width = 0.0;
  uint8_t ext_ban_num = 0;

  if (steps[section][step_num].b.TimeSource) {
    // Step time is set externally
    ext_ban_num = (uint8_t) steps[section][step_num].b.TLevel >> 10;
    time_level = read_calibrated_add_data_float(ext_ban_num);
  } else {
    // Step time is set on panel
    time_level = (float) steps[section][step_num].b.TLevel;
  };

  // This magic number is 112000/4095
  // This is the step width for the 2-30 range
  step_width = time_level * 27.35043 + 8000.0;

  if (steps[section][step_num].b.TimeRange_p03 == 1) {
    step_width *= 0.001;
  } else if (steps[section][step_num].b.TimeRange_p3 == 1) {
    step_width *= 0.01;
  } else if (steps[section][step_num].b.TimeRange_3 == 1) {
    step_width *= 0.1;
  }

  return (uint32_t) step_width;
};

// Calculate the number of next step.
// In the event that the end of a loop is reached,
// the closest previous "first" step is next (or 0 if none).
uint8_t GetNextStep(uint8_t section, uint8_t step_num) {
  uint8_t next_step = step_num;

  if (steps[section][step_num].b.CycleLast) {
    // Current step is the end of a loop.
    // Search backwards to the closest previous first step or 0
    while (next_step > 0) {
      if (steps[section][next_step].b.CycleFirst) break;
      next_step--;
    };
  } else {
    // Otherwise just advance 1 and check for wrap around
    next_step++;
    if (next_step > get_max_step()) next_step = 0;
  }
  return next_step;
};

// Apply switch programming directly to the step data
// Low value means that the switch is selected/active
void ApplyProgrammingSwitches(uint8_t section, uint8_t step_num, uButtons* switches) {

  volatile uStep* step = &steps[section][step_num];

  if ( !switches->b.Voltage0 ) {
    step->b.Voltage0 = 1;
    step->b.Voltage2 = 0;
    step->b.Voltage4 = 0;
    step->b.Voltage6 = 0;
    step->b.Voltage8 = 0;
    step->b.FullRange = 0;
  };
  if ( !switches->b.Voltage2 ) {
    step->b.Voltage0 = 0;
    step->b.Voltage2 = 1;
    step->b.Voltage4 = 0;
    step->b.Voltage6 = 0;
    step->b.Voltage8 = 0;
    step->b.FullRange = 0;
  };
  if ( !switches->b.Voltage4 ) {
    step->b.Voltage0 = 0;
    step->b.Voltage2 = 0;
    step->b.Voltage4 = 1;
    step->b.Voltage6 = 0;
    step->b.Voltage8 = 0;
    step->b.FullRange = 0;
  };
  if ( !switches->b.Voltage6 ) {
    step->b.Voltage0 = 0;
    step->b.Voltage2 = 0;
    step->b.Voltage4 = 0;
    step->b.Voltage6 = 1;
    step->b.Voltage8 = 0;
    step->b.FullRange = 0;
  };
  if ( !switches->b.Voltage8 ) {
    step->b.Voltage0 = 0;
    step->b.Voltage2 = 0;
    step->b.Voltage4 = 0;
    step->b.Voltage6 = 0;
    step->b.Voltage8 = 1;
    step->b.FullRange = 0;
  };
  if ( !switches->b.FullRangeOn ) {
    step->b.Voltage0 = 0;
    step->b.Voltage2 = 0;
    step->b.Voltage4 = 0;
    step->b.Voltage6 = 0;
    step->b.Voltage8 = 0;
    step->b.FullRange = 1;
  };
  if ( !switches->b.Pulse1On ) {
    step->b.OutputPulse1 = 1;
  };
  if ( !switches->b.Pulse1Off ) {
    step->b.OutputPulse1 = 0;
  };
  if ( !switches->b.Pulse2On ) {
    step->b.OutputPulse2 = 1;
  };
  if ( !switches->b.Pulse2Off ) {
    step->b.OutputPulse2 = 0;
  };
  if ( !switches->b.OutputQuantize ) {
    step->b.Quantize = 1;
  };
  if ( !switches->b.OutputContinuous ) {
    step->b.Quantize = 0;
  };
  if ( !switches->b.IntegrationSloped ) {
    step->b.Sloped = 1;
  };
  if ( !switches->b.IntegrationStepped ) {
    step->b.Sloped = 0;
  };
  if ( !switches->b.SourceExternal ) {
    step->b.VoltageSource = 1;
  };
  if ( !switches->b.SourceInternal ) {
    step->b.VoltageSource = 0;
  };
  if ( !switches->b.StopOn ) {
    step->b.OpModeSTOP = 1;
    step->b.OpModeENABLE = 0;
    step->b.OpModeSUSTAIN = 0;
  };
  if ( !switches->b.StopOff ) {
    step->b.OpModeSTOP = 0;
  };
  if ( !switches->b.SustainOn ) {
    step->b.OpModeSUSTAIN = 1;
    step->b.OpModeSTOP = 0;
    step->b.OpModeENABLE = 0;
  };
  if ( !switches->b.SustainOff ) {
    step->b.OpModeSUSTAIN = 0;
  };
  if ( !switches->b.EnableOn ) {
    step->b.OpModeENABLE = 1;
    step->b.OpModeSTOP = 0;
    step->b.OpModeSUSTAIN = 0;
  };
  if ( !switches->b.EnableOff ) {
    step->b.OpModeENABLE = 0;
  };
  if ( !switches->b.FirstOn ) {
    step->b.CycleFirst = 1;
    step->b.CycleLast = 0;
  };
  if ( !switches->b.FirstOff ) {
    step->b.CycleFirst = 0;
  };
  if ( !switches->b.LastOn ) {
    step->b.CycleLast = 1;
    step->b.CycleFirst = 0;
  };
  if ( !switches->b.LastOff ) {
    step->b.CycleLast = 0;
  };
  if ( !switches->b.TimeSourceExternal ) {
    step->b.TimeSource = 1;
  };
  if ( !switches->b.TimeSourceInternal ) {
    step->b.TimeSource = 0;
  };
  if (!switches->b.TimeRange1) {
    step->b.TimeRange_p03 = 1;
    step->b.TimeRange_p3 =  0;
    step->b.TimeRange_3 =   0;
    step->b.TimeRange_30 =  0;
  };
  if (!switches->b.TimeRange2) {
    step->b.TimeRange_p03 = 0;
    step->b.TimeRange_p3 =  1;
    step->b.TimeRange_3 =   0;
    step->b.TimeRange_30 =  0;
  };
  if (!switches->b.TimeRange3) {
    step->b.TimeRange_p03 = 0;
    step->b.TimeRange_p3 =  0;
    step->b.TimeRange_3 =   1;
    step->b.TimeRange_30 =  0;
  };
  if (!switches->b.TimeRange4) {
    step->b.TimeRange_p03 = 0;
    step->b.TimeRange_p3 =  0;
    step->b.TimeRange_3 =   0;
    step->b.TimeRange_30 =  1;
  };
}

void ClearProgram(uint8_t section) {
   steps[section][0].val[3] = 0x00;
   steps[section][0].val[4] = 0x00;
   steps[section][0].val[5] = 0x00;
   steps[section][0].b.TimeRange_p3 = 1;
   steps[section][0].b.FullRange = 1;
   for(uint8_t i=0; i < 16; i++) {
     steps[section][i] = steps[0][0];
     steps[section][i+16] = steps[0][0];
   }
}
