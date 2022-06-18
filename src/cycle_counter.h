
#ifndef __CYCLE_COUNTER_H
#define __CYCLE_COUNTER_H

// Gets the current processor cycle count register

#define CLOCK_SOURCE_GET_TIMER() (*((volatile uint32_t*)0xE0001004))

inline void start_cycle_timer() {
    // Enable CYCCNT register
    *((volatile uint32_t*)0xE0001000) = 0x40000001;
  }

#endif
