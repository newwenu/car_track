#include "buzzer.h"
#include "pwm.h"
#include "delay.h"
#include <stdio.h>

static u16 buzzer_initialized = 0;
static u16 buzzer_current_freq = 0;
static u16 buzzer_beep_ticks = 0;

/* 按指定频率输出 PWM 音，freq 变化时自动重配定时器 */
static void buzzer_output(u16 freq)
{
    u32 arr_plus_1;
    u32 pulse;

    if (freq == 0)
    {
        pwm_buzzer_set(0);
        return;
    }

    if (!buzzer_initialized || freq != buzzer_current_freq)
    {
        /* PSC=71 将 TIM3 时钟降到 1MHz，避免 1kHz 附近 ARR 溢出 u16。
         * 旧算法 72000000/freq-1 在 1000Hz 时得到 71999，截断成 6303，
         * 而 CCR4=36000 大于 ARR，导致 PWM 恒为高电平，蜂鸣器无声。 */
        arr_plus_1 = 1000000UL / freq;
        pwm_buzzer_init((u16)(arr_plus_1 - 1), 71);
        buzzer_initialized = 1;
        buzzer_current_freq = freq;
    }

    pulse = 1000000UL / freq / 4;   /* 25% 占空比，降低音量 */
    pwm_buzzer_set((u16)pulse);
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

/* ===================== 调试验证实现 ===================== */

/* PB1 硬件连通性验证：把 PB1 切为 GPIO 推挽输出并 1Hz 翻转 6 周期。
 * 若 PB1 焊盘/走线正常，示波器应看到干净 1Hz 方波；若仍是杂波，则硬件开路。 */
void buzzer_verify_hardware(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    u8 i;

    /* 先关闭 TIM3，释放 PB1 控制权 */
    TIM_Cmd(BSP_BUZZER_TIM, DISABLE);

    /* 将 PB1 配置为普通推挽输出 */
    RCC_APB2PeriphClockCmd(BSP_BUZZER_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = BSP_BUZZER_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BSP_BUZZER_PORT, &GPIO_InitStructure);

    /* 1Hz 方波，持续 6 秒 */
    for (i = 0; i < 6; i++)
    {
        GPIO_SetBits(BSP_BUZZER_PORT, BSP_BUZZER_PIN);
        delay_ms(500);
        GPIO_ResetBits(BSP_BUZZER_PORT, BSP_BUZZER_PIN);
        delay_ms(500);
    }

    /* 恢复为 PWM 复用输出 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(BSP_BUZZER_PORT, &GPIO_InitStructure);
    buzzer_initialized = 0;
    buzzer_current_freq = 0;
    pwm_buzzer_init(7199, 0);
    pwm_buzzer_set(0);
}

/* PB1 PWM 输出验证：输出 1kHz、50% 占空比方波 2s。
 * 用于确认 TIM3_CH4 配置正确且能驱动到 PB1。 */
void buzzer_verify_pwm(void)
{
    /* 强制重新初始化，避免之前的配置干扰 */
    buzzer_initialized = 0;
    buzzer_current_freq = 0;

    /* PSC=71 降频到 1MHz，ARR=999 即 1kHz，避免 u16 截断警告 */
    pwm_buzzer_init(999, 71);
    pwm_buzzer_set(500);        /* 50% 占空比 */
    delay_ms(2000);
    pwm_buzzer_set(0);
}

/* 打印 PB1 与 TIM3 相关寄存器，用于确认时钟、GPIO 模式、PWM 配置 */
void buzzer_dump_regs(void)
{
    printf("=== buzzer regs ===\r\n");
    printf("GPIOB CRL=0x%08lX IDR=0x%08lX ODR=0x%08lX\r\n",
           BSP_BUZZER_PORT->CRL,
           BSP_BUZZER_PORT->IDR,
           BSP_BUZZER_PORT->ODR);
    printf("TIM3 CR1=0x%04X CCMR2=0x%04X CCER=0x%04X\r\n",
           TIM3->CR1, TIM3->CCMR2, TIM3->CCER);
    printf("TIM3 CCR4=0x%04X ARR=0x%04X PSC=0x%04X\r\n",
           TIM3->CCR4, TIM3->ARR, TIM3->PSC);
    printf("RCC APB1ENR=0x%08lX APB2ENR=0x%08lX\r\n",
           RCC->APB1ENR, RCC->APB2ENR);
}
