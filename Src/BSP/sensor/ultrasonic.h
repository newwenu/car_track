#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f10x.h"
#include "config.h"

void  ultrasonic_init(void);
void  ultrasonic_start(void);
float ultrasonic_get_distance(void);
float ultrasonic_measure_blocking(void);

#endif
