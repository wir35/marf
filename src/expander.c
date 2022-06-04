#include "expander.h"

#include <stm32f4xx.h>
#include "dip_config.h"

uint8_t has_expander = 0;

void Init_Expander(void) {
  uDipConfig DipConfig = GetDipConfig();
  if (DipConfig.b.EXPANDER_ON) has_expander = 1;
}
