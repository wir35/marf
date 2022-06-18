#ifndef __DISPLAY_H
#define __DISPLAY_H

#include <stm32f4xx.h>

// Union with flags which allows to update different parts of panel
typedef union {
  struct {
    unsigned char MainDisplay:1;
    unsigned char StepsDisplay:1;
    unsigned char OT1:1;
    unsigned char OT2:1;
    unsigned char OT3:1;
    unsigned char OT4:1;
    unsigned char OT5:1;
    unsigned char OT6:1;
  } b;
  unsigned char value;
} uDisplayUpdateFlag;

// Flags which can be set by anything to update the led displays
extern volatile uDisplayUpdateFlag display_update_flags;

// Display modes
#define DISPLAY_MODE_VIEW_1       0
#define DISPLAY_MODE_VIEW_2       1
#define DISPLAY_MODE_EDIT_1       2
#define DISPLAY_MODE_EDIT_2       3
#define DISPLAY_MODE_SAVE_1       4
#define DISPLAY_MODE_SAVE_2       5
#define DISPLAY_MODE_LOAD_1       6
#define DISPLAY_MODE_LOAD_2       7

// Current display mode
extern volatile uint8_t display_mode;

// Do the pulse LEDs need to be swapped?
extern uint8_t swapped_pulses;

inline void update_display() {
  display_update_flags.b.MainDisplay  = 1;
  display_update_flags.b.StepsDisplay = 1;
}

inline void update_main_display() {
  display_update_flags.b.MainDisplay  = 1;
}

inline void update_steps_display() {
  display_update_flags.b.StepsDisplay = 1;
}

// Update display leds in programming section
void UpdateModeSectionLeds();

// Update step leds
void UpdateStepSectionLeds();

// Flush updates out to led shift registers
void FlushLedUpdates();

// Flash leds for clear
void RunClearAnimation();

#endif
