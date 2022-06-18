#include "program.h"

#include <stm32f4xx.h>

#include "analog_data.h"
#include "HC165.h"
#include "afg.h"

// Main step programming
volatile uStep steps[32];
volatile StepSliders sliders[32];

void InitProgram() {
  uStep clear_step = {{ 0x00, 0x00, 0x00 }};
  clear_step.b.TimeRange_30 = 1;
  clear_step.b.FullRange = 1;

  StepSliders clear_slider;
  clear_slider.VLevel = 0;
  clear_slider.TLevel = 0;

  for(uint8_t s = 0; s < 32; s++) {
    steps[s] = clear_step;
    sliders[s] = clear_slider;
  };
}

void WriteVoltageSlider(uint8_t slider_num, uint32_t new_adc_reading) {
  static volatile uint16_t voltage_smoothers[32];

  uint16_t adc_reading = (uint16_t) (new_adc_reading & 0xfff);
  uint16_t smoothed = apply_voltage_smoother(adc_reading << 4, &voltage_smoothers[slider_num]);

  if (steps[slider_num].b.WaitVoltageSlider) {
    if (smoothed >> 4 == sliders[slider_num].VLevel >> 4) {
      // Unpin the slider
      steps[slider_num].b.WaitVoltageSlider = 0;
      sliders[slider_num].VLevel = smoothed;
    }
  } else {
    sliders[slider_num].VLevel = smoothed;
  }
}

void WriteTimeSlider(uint8_t slider_num, uint32_t new_adc_reading) {
  static volatile uint16_t voltage_smoothers[32];

  uint16_t adc_reading = (uint16_t) (new_adc_reading & 0xfff) << 4;
  uint16_t smoothed = apply_voltage_smoother(adc_reading, &voltage_smoothers[slider_num]);

  if (steps[slider_num].b.WaitTimeSlider) {
    if (smoothed >> 4 == sliders[slider_num].TLevel >> 4) {
      // Unpin the slider
      steps[slider_num].b.WaitTimeSlider = 0;
      sliders[slider_num].TLevel = smoothed;
    }
  } else {
    sliders[slider_num].TLevel = smoothed;
  }
}

// Calculate the number of next step.
// In the event that the end of a loop is reached,
// the closest previous "first" step is next (or 0 if none).
uint8_t GetNextStep(uint8_t section, uint8_t step_num) {
  uint8_t step_zero = section << 4;
  uint8_t next_step = step_num + step_zero;
  uint8_t max_step = get_max_step() + step_zero;
  step_num += step_zero;

  if (steps[step_num].b.CycleLast) {
    // Current step is the end of a loop.
    // Search backwards to the closest previous first step or 0
    while (next_step > 0) {
      if (steps[next_step].b.CycleFirst) break;
      next_step--;
    };
  } else {
    // Otherwise just advance 1 and check for wrap around
    next_step++;
    if (next_step > max_step) {
      next_step = step_zero;
    }
  }
  next_step -= step_zero;
  return next_step;
};

// Apply switch programming directly to the step data
// Low value means that the switch is selected/active
void ApplyProgrammingSwitches(uint8_t section, uint8_t step_num, uButtons* switches) {
  volatile uStep* step;

  step_num += section << 4; // section select
  step = &steps[step_num];

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

// Set WaitX on all voltage and time sliders
void PinSliders() {
  for(uint8_t cnt = 0; cnt < 32; cnt++) {
    steps[cnt].b.WaitVoltageSlider = 1;
    steps[cnt].b.WaitTimeSlider = 1;
  };
}
