#include "HC165.h"

unsigned char keyb_proc(uButtons * key);
void mADC_init(void);
void ADCPause(void);

void delay_ms(unsigned int ms);
void delay_us(unsigned int us);
void delay_ns(unsigned int ns);

uint32_t GetStepWidth(unsigned char _Section, unsigned char _StepNum);

uint16_t GetStepVoltage(unsigned char _Section, unsigned char _StepNum);

uint8_t GetNextStep(unsigned char _Section, unsigned char _StepNum);

static inline void DoStepOutputPulses1();
static inline void DoStepOutputPulses2();
