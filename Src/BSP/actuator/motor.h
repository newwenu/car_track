#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "sys.h"
#include "config.h"

#define MOTOR_L_IN1  PBout(BSP_MOTOR_IN1_BIT)
#define MOTOR_L_IN2  PBout(BSP_MOTOR_IN2_BIT)
#define MOTOR_R_IN3  PBout(BSP_MOTOR_IN3_BIT)
#define MOTOR_R_IN4  PBout(BSP_MOTOR_IN4_BIT)

void motor_init(void);
void motor_run(int left_speed, int right_speed);
void motor_stop(void);

#endif
