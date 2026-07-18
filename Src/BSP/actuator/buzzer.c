#include "buzzer.h"
#include "pwm.h"

static u16 buzzer_initialized = 0;
static u16 buzzer_current_freq = 0;
static u16 buzzer_beep_ticks = 0;

/* 按指定频率输出 PWM 音，freq 变化时自动重配定时器 */
static void buzzer_output(u16 freq)
{
    if (freq == 0)
    {
        pwm_buzzer_set(0);
        return;
    }

    if (!buzzer_initialized || freq != buzzer_current_freq)
    {
        pwm_buzzer_init(72000000 / freq - 1, 0);
        buzzer_initialized = 1;
        buzzer_current_freq = freq;
    }
    pwm_buzzer_set(36000000 / freq);
}

void buzzer_on(u16 freq)
{
    /* 持续鸣响优先级高于单次提示音，取消未完成的 beep */
    buzzer_beep_ticks = 0;
    buzzer_output(freq);
}

void buzzer_off(void)
{
    buzzer_beep_ticks = 0;
    buzzer_current_freq = 0;
    pwm_buzzer_set(0);
}

/* 非阻塞提示音，duration_ms 按 10ms 节拍取整 */
void buzzer_beep(u16 freq, u16 duration_ms)
{
    buzzer_beep_ticks = (duration_ms + 5) / 10;
    if (buzzer_beep_ticks == 0)
    {
        buzzer_beep_ticks = 1;
    }
    buzzer_output(freq);
}

/* 每 10ms 调用一次，驱动单次提示音倒计时 */
void buzzer_update(void)
{
    if (buzzer_beep_ticks > 0)
    {
        buzzer_beep_ticks--;
        if (buzzer_beep_ticks == 0)
        {
            pwm_buzzer_set(0);
        }
    }
}
