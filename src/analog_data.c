#include "analog_data.h"

#include <stm32f4xx.h>
#include "dip_config.h"

// Additional analog data
volatile uint16_t add_data[8];

// Stored calibration
volatile uint16_t cal_constants[8] = {0xFFF, 0xFFF, 0xFFF, 0xFFF, 0xFFF, 0xFFF, 0xFFF, 0xFFF};

// Calibration scalers for external inputs, precomputed in setup
float external_cal[8] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

// Precomputed magic numbers for voltage scaling
// In the context of 12 bit range / 0.0 - 4095.0

float limited_range_multiplier; // octaves per 10v range
float octave_offset; // span of 1 octave
float semitone_offset; // span of 1 semitone
float quantizer_magic; // reciprocal of semitone_offset

// Set magic numbers from dip switch state
void SetVoltageRange(uDipConfig dip_config) {
  if (dip_config.b.V_OUT_1V) {
    // 1v per octave, who dis?
    octave_offset = 409.5;
    semitone_offset = 34.125;
    quantizer_magic = 0.0293;
    limited_range_multiplier = 0.1;
  } else if (dip_config.b.V_OUT_1V2) {
    // 1.2v per octave, the one true way
    octave_offset = 491.4;
    semitone_offset = 40.95;
    quantizer_magic = 0.02442;
    limited_range_multiplier = 0.12;
  } else {
    // 2v per octave for the OG's
    octave_offset = 819.0;
    semitone_offset = 68.25;
    quantizer_magic = 0.01465;
    limited_range_multiplier = 0.2;
  }
}


void WriteOtherCv(uint8_t cv_num, uint32_t new_adc_reading) {
  static volatile uint16_t voltage_smoothers[8];

  uint16_t adc_reading = (uint16_t) (new_adc_reading & 0xfff) << 4;
  add_data[cv_num] = apply_voltage_smoother(adc_reading, &voltage_smoothers[cv_num]);
}

void PrecomputeCalibration(void) {
  for(uint8_t i = 0; i < 8; i++) {
    if (cal_constants[i] < 100) {
      // Fix anything weirdly low (or disconnected)
      cal_constants[i] = 4095;
    }
    external_cal[i] = 4095.0 / (float) cal_constants[i];
  }
}

