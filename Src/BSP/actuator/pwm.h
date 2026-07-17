#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"
#include "config.h"

void pwm_motor_init(u16 arr, u16 psc);
void pwm_motor_set(u16 left, u16 right);
void pwm_buzzer_init(u16 arr, u16 psc);
void pwm_buzzer_set(u16 val);

#endif
