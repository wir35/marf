#include "HC165.h"

unsigned char keyb_proc(uButtons * key);
void mADC_init(void);
void ADCPause(void);

void delay_ms(unsigned int ms);
void delay_us(unsigned int us);
void delay_ns(unsigned int ns);

unsigned long int GetStepWidth(unsigned char _Section, unsigned char _StepNum);

unsigned int GetStepVoltage(unsigned char _Section, unsigned char _StepNum);

unsigned char GetNextStep(unsigned char _Section, unsigned char _StepNum);

void DoStepOutputPulses1();
void DoStepOutputPulses2();
