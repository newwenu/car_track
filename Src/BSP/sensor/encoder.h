#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"
#include "config.h"

void encoder_init(void);
void encoder_left_inc(void);
void encoder_right_inc(void);
int  encoder_read(void);

#endif
