#ifndef __AFG_H
#define __AFG_H

#include "data_types.h"
#include "program.h"
#include "display.h"
#include "analog_data.h"

#include <stm32f4xx.h>

// Current step number
extern volatile uint8_t afg1_step_num, afg2_step_num;

// Length of the current step in timer "ticks"
extern volatile uint32_t afg1_step_width, afg2_step_width;

// Step counters
extern volatile uint32_t afg1_step_cnt, afg2_step_cnt;

// AFG modes

#define MODE_STOP 0       // Stopped
#define MODE_RUN  1       // Started and running
#define MODE_WAIT 2       // Continuous strobe mode. Better name?
#define MODE_WAIT_HI_Z 3  // Running, but holding on enable step
#define MODE_STAY_HI_Z 4  // Running, but holding on sustain step

// Values of programmed output voltages
typedef struct {
  uint16_t voltage;
  uint16_t time;
  uint16_t ref;
  uint8_t all_pulses:1;
  uint8_t pulse1:1;
  uint8_t pulse2:1;
} ProgrammedOutputs;

// Sequencer modes
extern volatile unsigned char afg1_mode;
extern volatile unsigned char afg2_mode;
extern volatile unsigned char afg1_prev_mode;
extern volatile unsigned char afg2_prev_mode;
extern volatile unsigned char afg1_advance;
extern volatile unsigned char afg2_advance;

// The voltage level of the current step
extern volatile unsigned int afg1_step_level;
extern volatile unsigned int afg2_step_level;

// The voltage level of the previous step
extern volatile unsigned int afg1_prev_step_level;
extern volatile unsigned int afg2_prev_step_level;

// The stage address selected step
extern volatile uint8_t afg1_stage_address;
extern volatile uint8_t afg2_stage_address;

// The offset into step programming for each afg [0-15][16-31] 0 or 1
extern volatile uint8_t afg1_section;
extern volatile uint8_t afg2_section;

// Get the step num, taking into account section offset
inline uint8_t get_afg1_step_num() {
  return afg1_step_num + (afg1_section << 4);
}

// Get the step num, taking into account section offset
inline uint8_t get_afg2_step_num() {
  return afg2_step_num + (afg2_section << 4);
}

void AfgAllInitialize();

// Jump steps

void JumpToStep1(unsigned int step);
void JumpToStep2(unsigned int step);

// Clock mode control from start, stop, strobe interrupts
void ProcessPulseInputs1();
void ProcessModeChanges1();

void ProcessPulseInputs2();
void ProcessModeChanges2();

// Check timer after handling start signal
uint8_t CheckStart1();
uint8_t CheckStart2();

inline void HardStop1() {
  afg1_mode = MODE_STOP;
  afg1_step_num = 0;
  afg1_step_cnt = 0xFFFFFFFF;
}

inline void HardStop2() {
  afg2_mode = MODE_STOP;
  afg2_step_num = 0;
  afg2_step_cnt = 0xFFFFFFFF;
}

inline void DoReset1() {
  if (afg1_mode != MODE_WAIT) {
    JumpToStep1(0);
  }
}

inline void DoReset2() {
  if (afg2_mode != MODE_WAIT) {
    JumpToStep2(0);
  }
}

// Process one time window

ProgrammedOutputs AfgTick1();
ProgrammedOutputs AfgTick2();

// Compute continuous step stage selection

inline void ComputeContinuousStep1() {
  afg1_stage_address = read_calibrated_add_data_uint16(ADC_STAGEADDRESS_Ch_1) >> get_max_step_shift12();
  if (afg1_stage_address > get_max_step()) afg1_stage_address = get_max_step();
}

inline void ComputeContinuousStep2() {
  afg2_stage_address = read_calibrated_add_data_uint16(ADC_STAGEADDRESS_Ch_2) >> get_max_step_shift12();
  if (afg2_stage_address > get_max_step()) afg2_stage_address = get_max_step();
}

inline void EnableContinuousStageAddress1() {
  if (afg1_mode != MODE_WAIT) {
    afg1_prev_mode = afg1_mode;
    afg1_mode = MODE_WAIT;
    update_main_display();
  }
}

inline void DisableContinuousStageAddress1() {
  if (afg1_mode == MODE_WAIT) {
    afg1_mode = afg1_prev_mode;
    update_display();
  }
}

inline void EnableContinuousStageAddress2() {
  if (afg2_mode != MODE_WAIT) {
    afg2_prev_mode = afg2_mode;
    afg2_mode = MODE_WAIT;
    update_main_display();
  }
}

inline void DisableContinuousStageAddress2() {
  if (afg2_mode == MODE_WAIT) {
    afg2_mode = afg2_prev_mode;
    update_display();
  }
}

float GetTimeMultiplier1();

float GetTimeMultiplier2();

#endif
