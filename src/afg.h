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

// Sequencer modes
#define MODE_RUN  0
#define MODE_WAIT 1
#define MODE_STOP 2
#define MODE_WAIT_STROBE  3
#define MODE_WAIT_HI_Z    4
#define MODE_STAY_HI_Z    5
#define MODE_ADVANCE      6

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

// Modes for start condition
#define START_MODE_ZERO       0
#define START_MODE_WAIT_HI_Z  1
#define START_MODE_HI_Z       2

// Current mode for start condition
extern volatile unsigned char afg1_start_mode;
extern volatile unsigned char afg2_start_mode;

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

// Clock mode control
void ProcessStopStart(uint16_t gpiob_data);

// Handle stop signal
void DoStop1();
void DoStop2();

// Handle start signal
void DoStart1();
void DoStart2();

// Check timer after handling start signal
uint8_t CheckStart1();
uint8_t CheckStart2();

// Handle strobe signal
void DoStrobe1();
void DoStrobe2();

inline void HardStop1() {
  afg1_mode = MODE_STOP;
  afg1_step_num = 0;
}

inline void HardStop2() {
  afg2_mode = MODE_STOP;
  afg2_step_num = 0;
}

inline void DoAdvance1() {
  if (afg1_mode != MODE_WAIT) {
    if (afg1_mode == MODE_STOP) afg1_mode = MODE_ADVANCE;
    JumpToStep1(GetNextStep(afg1_section, afg1_step_num));
  }
}

inline void DoAdvance2() {
  if (afg2_mode != MODE_WAIT) {
    if (afg2_mode == MODE_STOP) afg2_mode = MODE_ADVANCE;
    JumpToStep2(GetNextStep(afg2_section, afg2_step_num));
  }
}

inline void DoReset1() {
  if (afg1_mode != MODE_WAIT) {
    if (afg1_mode == MODE_STOP) afg1_mode = MODE_ADVANCE;
    JumpToStep1(0);
  }
}

inline void DoReset2() {
  if (afg2_mode != MODE_WAIT) {
    if (afg2_mode == MODE_STOP) afg2_mode = MODE_ADVANCE;
    JumpToStep2(0);
  }
}

// Process one time window

ProgrammedOutputs AfgTick1();
ProgrammedOutputs AfgTick2();

// Compute continuous step stage selection

inline void ComputeContinuousStep1() {
  afg1_stage_address = read_calibrated_add_data_uint16(ADC_STAGEADDRESS_Ch_1) >> get_max_step_shift12();
}

inline void ComputeContinuousStep2() {
  afg2_stage_address = read_calibrated_add_data_uint16(ADC_STAGEADDRESS_Ch_2) >> get_max_step_shift12();
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
