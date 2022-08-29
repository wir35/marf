#ifndef __AFG_H
#define __AFG_H

#include "data_types.h"
#include "program.h"
#include "display.h"
#include "analog_data.h"

#include <stm32f4xx.h>

// Defs for referencing which function generator when calling all afg functions

#define AFG1 0
#define AFG2 1

// AFG modes

#define MODE_STOP 0       // Stopped
#define MODE_RUN  1       // Started and running
#define MODE_WAIT 2       // Continuous strobe mode. Better name?
#define MODE_WAIT_HI_Z 3  // Running, but holding on enable step
#define MODE_STAY_HI_Z 4  // Running, but holding on sustain step

// Afg state
typedef struct {
  // The current step voltage level
  float step_level;
  // The sampled and held voltage level of the previous step
  float prev_step_level;
  // Number of ticks into the current step
  uint32_t step_cnt;
  // The length in ticks of the current step
  uint32_t step_width;
  // Run mode, strictly one of the defs above
  uint8_t mode;
  // Previous mode, before going into MODE_WAIT (continuous stage address)
  uint8_t prev_mode;
  // The step value derived from the stage address cv
  uint8_t stage_address;
  // The step section (normally 0 or 1 if shifted to 16-31)
  uint8_t section;
  // The current step number
  uint8_t step_num;
} AfgState;

// Values of programmed output voltages
typedef struct {
  float voltage;
  uint16_t time;
  uint16_t ref;
  // Boolean flags
  uint8_t all_pulses:1;
  uint8_t pulse1:1;
  uint8_t pulse2:1;
  uint8_t sloped:1;
} ProgrammedOutputs;

typedef struct {
  float level;
  float target_level;
  float increment;
} SlopingOutput;

// Initial zero both afg states
void AfgAllInitialize();

// Get a snapshot of the afg state for coordination between controller and display
AfgControllerState AfgGetControllerState(uint8_t afg_num);

// Move the section programming offset to steps 0-15 or 16-31
void AfgSetSection(uint8_t afg_num, uint8_t section);

// Clock mode control from start, stop, strobe interrupts
void AfgProcessModeChanges(uint8_t afg_num, PulseInputs pulses);

// Check timer after handling start signal
uint8_t AfgCheckStart(uint8_t afg_num, uint8_t start_signal);

// Immediately jump to the given step
void AfgJumpToStep(uint8_t afg_num, uint8_t step);

// Hard stop and reset
void AfgHardStop(uint8_t afg_num);

// Reset to first step
void AfgReset(uint8_t afg_num);

// Process one time window
ProgrammedOutputs AfgTick(uint8_t afg_num, PulseInputs pulses, uint8_t ticks);

// Go in and out of continuous state address mode
void EnableContinuousStageAddress(uint8_t afg_num);
void DisableContinuousStageAddress(uint8_t afg_num);

// Calculate the length of the step, based on time settings and time multiplier
void AfgRecalculateStepWidths();

// Get the number of ticks remaining in the current step.
// A return value of 0xFFFFFFFF indicates that the step is ended.
uint32_t AfgGetStepTicksRemaining(uint8_t afg_num);

#endif
