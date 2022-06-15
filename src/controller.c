#include "controller.h"

#include <stm32f4xx.h>
#include "display.h"
#include "afg.h"
#include "leds_step.h"
#include "expander.h"

// Step selected for editing (0-31)
volatile uint8_t edit_mode_step_num = 0;
volatile uint8_t edit_mode_section = 0;

// Variable used for key lock during the VIEW_MODE key changes steps options
volatile uint8_t key_locked = 0;
volatile uint8_t keys_not_valid = 0;

// Current patches bank
volatile uint8_t bank = 1;

// Start Timer 6 for clear switch measurement
void InitClear_Timer() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  TIM6->PSC = 21000;
  TIM6->ARR = 200;
  TIM6->CNT = 0;
  TIM6->DIER = TIM_DIER_UIE;
  TIM6->CR1 |= TIM_CR1_CEN;

  NVIC_EnableIRQ(TIM6_DAC_IRQn);
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
};


#define LONG_COUNTER_TICKS 60;
#define SHORT_COUNTER_TICKS 3;

void ControllerProcessSwitches(uButtons* key) {
  // Down counters which track the number of ticks of this method
  // before left and right switches should be processed again.
  // This enables some debouncing but also long press and hold to scroll.
  static uint16_t left_counter = SHORT_COUNTER_TICKS;
  static uint16_t right_counter = SHORT_COUNTER_TICKS;

  if (!key->b.ClearUp || !key->b.ClearDown)  {
    // Wait for long press on clear switch
    InitClear_Timer();
  }

  // Display/edit mode changes
  if (!key->b.StageAddress1Display && display_mode != DISPLAY_MODE_VIEW_1) {
    display_mode = DISPLAY_MODE_VIEW_1;
  }
  if (!key->b.StageAddress2Display && display_mode != DISPLAY_MODE_VIEW_2) {
    display_mode = DISPLAY_MODE_VIEW_2;
  }
  if (!key->b.StepLeft || !key->b.StepRight) {
    if (display_mode == DISPLAY_MODE_VIEW_1) {
      display_mode = DISPLAY_MODE_EDIT_1;
      edit_mode_section = afg1_section;
      right_counter = LONG_COUNTER_TICKS;
      left_counter = LONG_COUNTER_TICKS;
    }
    else if (display_mode == DISPLAY_MODE_VIEW_2) {
      display_mode = DISPLAY_MODE_EDIT_2;
      edit_mode_section = afg2_section;
      right_counter = LONG_COUNTER_TICKS;
      left_counter = LONG_COUNTER_TICKS;
    }
  }

  // Section shift for each afg in 16 slider mode
  if (!Is_Expander_Present()) {
    if (!key->b.StepLeft && !key->b.StageAddress1Display) {
      afg1_section = 0;
      display_mode = DISPLAY_MODE_VIEW_1;
    } else if (!key->b.StepRight && !key->b.StageAddress1Display) {
      afg1_section = 1;
      display_mode = DISPLAY_MODE_VIEW_1;
    } else if (!key->b.StepLeft && !key->b.StageAddress2Display) {
      afg2_section = 0;
      display_mode = DISPLAY_MODE_VIEW_2;
    } else if (!key->b.StepRight && !key->b.StageAddress2Display) {
      afg2_section = 1;
      display_mode = DISPLAY_MODE_VIEW_2;
    }
  }

  // Decrement and reset counters
  if (!key->b.StepLeft) {
    if (left_counter) left_counter--;
  } else {
    left_counter = SHORT_COUNTER_TICKS;
  }
  if (!key->b.StepRight) {
    if (right_counter) right_counter--;
  } else {
    right_counter = SHORT_COUNTER_TICKS;
  }

  // Left counter expired, do step left
  if (!left_counter) {
    if (edit_mode_step_num == 0) {
      // Wrap around to max step
      edit_mode_step_num = get_max_step();
    } else {
      // Decrement edit step
      edit_mode_step_num -= 1;
    }
    // Long count when held down
    if (!key->b.StepLeft) left_counter = LONG_COUNTER_TICKS else left_counter = SHORT_COUNTER_TICKS;
    update_display();
  }

  // Right counter expired, do step right
  if (!right_counter) {
    if (edit_mode_step_num == get_max_step()) {
      // Wrap around to 0
      edit_mode_step_num = 0;
    } else {
      // Increment edit step
      edit_mode_step_num += 1;
    }
    // Long count when held down
    if (!key->b.StepRight) right_counter = LONG_COUNTER_TICKS else right_counter = SHORT_COUNTER_TICKS;
    update_display();
  }
}

void ControllerCheckClear() {
  uButtons myButtons;
  static uint8_t clear_counter1 = 0, clear_counter2 = 0;

  TIM6->SR = (uint16_t) ~TIM_IT_Update;

  myButtons.value = GetButton();

  if (clear_counter1 < 30 && clear_counter2 < 30) {
    if (!myButtons.b.ClearUp || !myButtons.b.ClearDown) {
      if (!myButtons.b.ClearUp) clear_counter1++;
      else clear_counter1 = 0;
      if (!myButtons.b.ClearDown) clear_counter2++;
      else clear_counter2 = 0;
    } else {
      clear_counter1 = 0;
      clear_counter2 = 0;
      TIM_SetCounter(TIM6, 0x00);
      TIM6->CR1 &= ~TIM_CR1_CEN;
    }
  }
  else if (clear_counter1 == 30 || clear_counter2 == 30) {
    // Signal by flashing step leds
    RunClearAnimation();

    TIM_SetCounter(TIM6, 0x00);
    TIM6->CR1 &= ~TIM_CR1_CEN;

    if (clear_counter1 == 30) {
      InitProgram();
      HardStop1();
      HardStop2();
    }
    else if (clear_counter2 == 30) {
      InitProgram();
      HardStop1();
      HardStop2();
    };

    clear_counter1 = 0;
    clear_counter2 = 0;
  };
}
