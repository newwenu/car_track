#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"
#include "config.h"
#include "board.h"

#define LED_STAT   PAout(BSP_LED_STAT_BIT)
#define LED_ALARM  PBout(BSP_LED_ALARM_BIT)

void led_init(void);
void led_stat_on(void);
void led_stat_off(void);
void led_stat_toggle(void);
void led_alarm_on(void);
void led_alarm_off(void);
void led_alarm_toggle(void);

#endif
