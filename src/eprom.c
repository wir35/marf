#include "eprom.h"

#include <stdint.h>

#include "program.h"
#include "analog_data.h"
#include "display.h"

// 64kb available in eprom memory
EpromMemory eprom_memory = {};

void EpromInitializeMemoryLayout() {
  SavedProgram savedProgram;
  volatile uint16_t start = 0, size = 0; // in bytes

  // Saved programs at the head
  size = sizeof(savedProgram);
  for (uint8_t p = 0; p < 16; p++) {
    eprom_memory.programs[p].start = start;
    eprom_memory.programs[p].size = size;
    start += size;
  }

  // Analog calibration data at the tail
  start = 0xFFFF;
  size = sizeof(cal_constants);
  start -= size;
  eprom_memory.analog_cal_data.start = start;
  eprom_memory.analog_cal_data.size = size;


  // Switch pulse leds
  size = sizeof(swapped_pulses);
  start -= size;
  eprom_memory.pulse_leds_switched.start = start;
  eprom_memory.pulse_leds_switched.size = size;
}
