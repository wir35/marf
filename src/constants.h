#ifndef __CONSTANTS_H
#define __CONSTANTS_H

// Timing constants

// AFG updates its state at 32kHz
#define AFG_TICK_FREQUENCY  32000

// Prescaler value for the timers that drive the AFG ticks
#define AFG_TIMER_PRESCALER ( 168000000 / AFG_TICK_FREQUENCY / 4 ) // 4 = APB prescaler

#define STEP_WIDTH_2S (AFG_TICK_FREQUENCY * 2.0)
#define STEP_WIDTH_28S (AFG_TICK_FREQUENCY * 28.0)
#define RECIPROCAL_12BIT (1.0 / 4096.0)

#define PULSE_ACTIVE_STEP_WIDTH (AFG_TICK_FREQUENCY / 1000) // 32 ticks or 1 ms

// Process up to 32 ticks at a time
#define TICKS_WINDOW 32

#endif
