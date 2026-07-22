#ifndef __BUZZER_H
#define __BUZZER_H

#include "stm32f10x.h"
#include "config.h"

void buzzer_on(u16 freq);
void buzzer_off(void);
void buzzer_beep(u16 freq, u16 duration_ms);

/* 非阻塞蜂鸣器驱动，需每 10ms 调用一次 */
void buzzer_update(void);

/* ===================== 调试验证接口 ===================== */

/* PB1 硬件连通性验证：把 PB1 切为 GPIO 推挽输出并 1Hz 翻转 6 周期。
 * 若 PB1 焊盘/走线正常，示波器应看到干净 1Hz 方波；若仍是杂波，则硬件开路。 */
void buzzer_verify_hardware(void);

/* PB1 PWM 输出验证：输出 1kHz、50% 占空比方波 2s。
 * 用于确认 TIM3_CH4 配置正确且能驱动到 PB1。 */
void buzzer_verify_pwm(void);

/* 打印 PB1 与 TIM3 相关寄存器，用于确认时钟、GPIO 模式、PWM 配置 */
void buzzer_dump_regs(void);

#endif
