#ifndef __EPROM_H
#define __EPROM_H

#include <stdint.h>

#include "CAT25512.h"
#include "program.h"

// Typedef for one section of eprom memory layout
typedef struct {
  uint16_t start;
  uint16_t size;
} MemoryRange;

// Typedef for full eprom memory layout
typedef struct {
  // 16 saved programs
  MemoryRange programs[16];

  // Whether the pulse leds are switched
  MemoryRange pulse_leds_switched;

  // Analog calibration data
  MemoryRange analog_cal_data;
} EpromMemory;

typedef struct {
  uStep steps[32];
  StepSliders sliders[32];
} SavedProgram;

extern EpromMemory eprom_memory;

void EpromInitializeMemoryLayout();

#endif
