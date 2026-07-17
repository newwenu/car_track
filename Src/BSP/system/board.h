#ifndef __BOARD_H
#define __BOARD_H

#include "sys.h"
#include "config.h"
#include "../display/soft_i2c.h"
#include "../sensor/encoder.h"
#include "../sensor/adc.h"
#include "../actuator/pwm.h"
#include "../actuator/motor.h"
#include "../sensor/trace.h"
#include "../sensor/ultrasonic.h"
#include "../display/oled_spi.h"
#include "../input/key.h"
#include "../actuator/led.h"
#include "../actuator/buzzer.h"

void all_init(void);

#endif
