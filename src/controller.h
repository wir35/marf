#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <stm32f4xx.h>

#include "HC165.h"
#include "afg.h"

#define CONTROLLER_MODAL_NONE 0
#define CONTROLLER_MODAL_LOAD 1
#define CONTROLLER_MODAL_SAVE 2

// Controller Job Flags
// This is state that is passed back and forth between the controller,
// its main loop, and the few IRQs that need to "do stuff."
typedef struct  {
  // The current adc mux pot selection
  uint8_t adc_pot_sel;

  // Flag that the controller main loop needs to shift the mux
  uint8_t adc_mux_shift_out;

  // Flag that the controller is shifting the mux and adc reads are invalid
  uint8_t inhibit_adc;

  // Flag that the afg1 tick irq has refreshed the output data
  uint8_t afg1_tick;
  ProgrammedOutputs afg1_outputs;

  // Flag that the afg2 tick irq has refreshed the output data
  uint8_t afg2_tick;
  ProgrammedOutputs afg2_outputs;

  // Flags that we are going into a modal loop
  uint8_t modal_loop;
} ControllerJobFlags;

extern volatile ControllerJobFlags controller_job_flags;

// The section and step number in editing mode
extern volatile uint8_t edit_mode_step_num;
extern volatile uint8_t edit_mode_section;


#define KEY_DEBOUNCE_COUNT 5  // 5 passes

#define LONG_COUNTER_TICKS  30;
#define SHORT_COUNTER_TICKS 10;
#define SCROLL_WAIT_COUNTER 120;

void ControllerMainLoop();

void ControllerApplyProgrammingSwitches(uButtons * key);

void ControllerProcessStageAddressSwitches(uButtons * key);

void ControllerProcessNavigationSwitches(uButtons* key);

void ControllerCheckClear();

void ControllerCalibrationLoop();

void ControllerLoadCalibration();

void ControllerLoadProgramLoop();

void ControllerSaveProgramLoop();

#endif
