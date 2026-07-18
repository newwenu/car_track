#ifndef __BUZZER_H
#define __BUZZER_H

#include "stm32f10x.h"
#include "config.h"

void buzzer_on(u16 freq);
void buzzer_off(void);
void buzzer_beep(u16 freq, u16 duration_ms);

/* 非阻塞蜂鸣器驱动，需每 10ms 调用一次 */
void buzzer_update(void);

#endif
