#ifndef LEDS_STEP_H_
#define LEDS_STEP_H_

void LED_STEP_init(void);
void LED_STEP_SendByte(unsigned char data);
void LED_STEP_SendWord(unsigned long int data);
void LED_STEP_LightStep(unsigned int StepNum);

#endif /* LEDS_STEP_H_ */
