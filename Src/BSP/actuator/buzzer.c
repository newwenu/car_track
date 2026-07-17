#include "buzzer.h"
#include "pwm.h"
#include "delay.h"

static u16 buzzer_initialized = 0;

void buzzer_on(u16 freq)
{
    if (!buzzer_initialized)
    {
        pwm_buzzer_init(72000000 / freq - 1, 0);
        buzzer_initialized = 1;
    }
    pwm_buzzer_set(36000000 / freq);
}

void buzzer_off(void)
{
    pwm_buzzer_set(0);
}

void buzzer_beep(u16 freq, u16 duration_ms)
{
    buzzer_on(freq);
    delay_ms(duration_ms);
    buzzer_off();
}
