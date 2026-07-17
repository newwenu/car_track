#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"
#include "config.h"

void adc_init(void);
u16  adc_get_channel(u8 ch);

#endif
