#ifndef ADC_POTS_SELECTOR_H_
#define ADC_POTS_SELECTOR_H_

void ADC_POTS_selector_init(void);
void ADC_POTS_selector_SendByte(unsigned char data);
void ADC_POTS_selector_SendHalfByte(unsigned char data);
void ADC_POTS_selector_SendDWord(unsigned long long int data);
void ADC_POTS_selector_Ch(unsigned char Ch);
unsigned char ADC_inc(unsigned char pot); 
extern void delay_us(unsigned int us);
#endif /* ADC_POTS_SELECTOR_H_ */
