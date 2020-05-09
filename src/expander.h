#include <stm32f4xx.h>

#define EXPANDER_PIN GPIO_Pin_14
#define EXPANDER_GPIO GPIOB

uint8_t Is_Expander_Present(void);
void Init_Expander_GPIO(void);

