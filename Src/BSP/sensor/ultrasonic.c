#include "ultrasonic.h"
#include "delay.h"
#include <stddef.h>

/* Timing constants (microseconds) */
#define US_TX_US       500    /* 40kHz TX burst duration */
#define US_BLIND_US    250    /* blind zone (~4cm), skip transducer ringing */
#define US_TO_US       2000   /* timeout (~34cm range), adjust for longer range */

/* EMA filter: new_val = alpha * raw + (1-alpha) * old, alpha=0.3 gives smooth output */
#define EMA_ALPHA      0.3f   /* 0.1~0.5: smaller=smoother but slower response */
#define EMA_ALPHA_FAST 0.8f   /* fast response when obstacle appears (distance decreases) */
#define EMA_ALPHA_SLOW 0.5f   /* faster recovery when obstacle disappears (distance increases) */

/* Safety distance when timeout (no echo): keep in hysteresis zone to avoid premature clear */
#define US_SAFE_DIST_CM   50.0f  /* between AVOID_DIST(20) and AVOID_CLEAR(35) */

/* Obstacle avoidance threshold - must match obstacle_guard.c */
#define AVOID_DIST_CM     20.0f   /* trigger emergency brake < this distance */

/* Internal state */
static volatile u16 us_echo_us  = 0;
static volatile u8  us_ok       = 0;
static volatile u8  us_listen   = 0;

static float us_ema_dist = 0.0f;     /* EMA filtered distance */
static u32   us_timeout_cnt = 0;     /* consecutive timeout counter */

/* Weak callback for emergency brake */
#ifndef __weak
    #ifdef __GNUC__
        #define __weak  __attribute__((weak))
    #else
        #define __weak
    #endif
#endif

__weak void ultrasonic_distance_ready_callback(float distance)
{
    (void)distance;
}

void ultrasonic_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* TX: TIM1_CH1, PA8, 40kHz PWM */
    RCC_APB2PeriphClockCmd(BSP_US_TX_CLK | BSP_US_TX_TIM_CLK | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_US_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BSP_US_TX_PORT, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 1800 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(BSP_US_TX_TIM, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 900;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(BSP_US_TX_TIM, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(BSP_US_TX_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(BSP_US_TX_TIM, ENABLE);
    TIM_CtrlPWMOutputs(BSP_US_TX_TIM, ENABLE);
    TIM_Cmd(BSP_US_TX_TIM, DISABLE);

    /* RX: TIM4_CH1, PB6, input capture @ 1us resolution */
    RCC_APB2PeriphClockCmd(BSP_US_ECHO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(BSP_US_ECHO_TIM_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_US_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(BSP_US_ECHO_PORT, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 5000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(BSP_US_ECHO_TIM, &TIM_TimeBaseStructure);

    TIM_ICInitStructure.TIM_Channel = BSP_US_ECHO_CH;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0F;
    TIM_ICInit(BSP_US_ECHO_TIM, &TIM_ICInitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(BSP_US_ECHO_TIM, TIM_IT_CC1 | TIM_IT_Update, ENABLE);
    TIM_Cmd(BSP_US_ECHO_TIM, ENABLE);
}

static void ultrasonic_trig(void)
{
    us_listen = 0;
    us_ok = 0;
    us_echo_us = 0;

    TIM_SetCounter(BSP_US_TX_TIM, 0);
    TIM_CtrlPWMOutputs(BSP_US_TX_TIM, ENABLE);
    TIM_Cmd(BSP_US_TX_TIM, ENABLE);
    delay_us(US_TX_US);
    TIM_Cmd(BSP_US_TX_TIM, DISABLE);
    GPIO_ResetBits(BSP_US_TX_PORT, BSP_US_TX_PIN);

    delay_us(US_BLIND_US);

    TIM_SetCounter(BSP_US_ECHO_TIM, 0);
    TIM_ClearITPendingBit(BSP_US_ECHO_TIM, TIM_IT_CC1);
    us_listen = 1;
}

void ultrasonic_start(void)
{
    ultrasonic_trig();
}

float ultrasonic_get_distance(void)
{
    static u32 s_no_update_cnt = 0;
    static float s_last_valid = 0.0f;

    if (us_ok)
    {
        float raw;

        if (us_echo_us == 0xFFFF)
        {
            raw = 999.0f;
        }
        else
        {
            raw = (US_BLIND_US + us_echo_us) / 58.0f;
        }

        if (raw >= 900.0f)
        {
            us_timeout_cnt++;

            /* 分段式超时处理策略（解决999被覆盖导致避障无法解除的问题） */
            if (us_timeout_cnt <= 3)  // 阶段1：极短记忆窗口(30ms)，防止紧贴抖动
            {
                if (us_ema_dist > 0.0f && us_ema_dist < (AVOID_DIST_CM - 5.0f))  // 仅<15cm保持
                {
                    raw = us_ema_dist;  // 真正危险距离才保持旧值
                }
                else
                {
                    raw = US_SAFE_DIST_CM;  // 其他情况立即给安全值(50cm)
                }
            }
            else if (us_timeout_cnt <= 8)  // 阶段2：快速退出区(30~80ms)
            {
                float exit_ratio = (float)(us_timeout_cnt - 3) / 5.0f;  // 0→1过渡
                if (exit_ratio > 1.0f) exit_ratio = 1.0f;

                float target = US_SAFE_DIST_CM;  // 目标：50cm
                raw = us_ema_dist + (target - us_ema_dist) * exit_ratio;
                // 快速从旧值过渡到安全距离（而非缓慢漂移）
            }
            else  // 阶段3：明确报告无障碍(>80ms)
            {
                raw = 100.0f;  // 直接报告100cm，明确表示"远方无目标"
            }
        }
        else
        {
            us_timeout_cnt = 0;
            
            /* 正常测量值：进行合理性检查 */
            if (raw > 200.0f || raw < 0.1f)
            {
                raw = s_last_valid;  /* 异常值用上次有效值替代 */
            }
        }

        if (us_ema_dist == 0.0f)
        {
            us_ema_dist = raw;
        }
        else
        {
            float alpha;

            if (raw < us_ema_dist - 5.0f)
            {
                alpha = EMA_ALPHA_FAST;
            }
            else if (raw > us_ema_dist + 5.0f)
            {
                alpha = EMA_ALPHA_SLOW;
            }
            else
            {
                alpha = EMA_ALPHA;
            }

            us_ema_dist = alpha * raw + (1.0f - alpha) * us_ema_dist;
        }

        s_last_valid = us_ema_dist;

        if (raw < AVOID_DIST_CM || (us_ema_dist > 0.0f && us_ema_dist < AVOID_DIST_CM))
        {
            float callback_dist = (raw < us_ema_dist) ? raw : us_ema_dist;
            ultrasonic_distance_ready_callback(callback_dist);
        }

        us_ok = 0;
        s_no_update_cnt = 0;
        ultrasonic_trig();
    }
    else
    {
        s_no_update_cnt++;
        if (s_no_update_cnt > 10)
        {
            us_ema_dist = 999.0f;
            s_no_update_cnt = 0;
            ultrasonic_trig();
        }
    }

    return us_ema_dist;
}

float ultrasonic_measure_blocking(void)
{
    ultrasonic_trig();

    while (!us_ok)
    {
        if (TIM_GetCounter(BSP_US_ECHO_TIM) >= US_TO_US)
        {
            us_listen = 0;
            return 0.0f;
        }
    }

    us_listen = 0;
    return (US_BLIND_US + us_echo_us) / 58.0f;
}

void ultrasonic_get_raw(u16 *echo_us, u8 *ok, u8 *listening)
{
    __disable_irq();
    if (echo_us) *echo_us = us_echo_us;
    if (ok) *ok = us_ok;
    if (listening) *listening = us_listen;
    __enable_irq();
}

void ultrasonic_set_pull(u8 pull_up)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    static u8 s_pull_up = 0;

    s_pull_up = pull_up ? 1 : 0;
    GPIO_InitStructure.GPIO_Pin = BSP_US_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = s_pull_up ? GPIO_Mode_IPU : GPIO_Mode_IPD;
    GPIO_Init(BSP_US_ECHO_PORT, &GPIO_InitStructure);
}

u8 ultrasonic_get_pull(void)
{
    return 0;
}

u32 ultrasonic_get_timeout_cnt(void)
{
    return us_timeout_cnt;
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(BSP_US_ECHO_TIM, TIM_IT_CC1) == SET)
    {
        if (us_listen)
        {
            u16 capture = TIM_GetCapture1(BSP_US_ECHO_TIM);

            if (capture >= 20 && capture <= US_TO_US)
            {
                us_echo_us = capture;
                us_ok = 1;
                us_listen = 0;
            }
            else if (capture > US_TO_US)
            {
                us_echo_us = 0xFFFF;
                us_ok = 1;
                us_listen = 0;
            }
        }

        TIM_ClearITPendingBit(BSP_US_ECHO_TIM, TIM_IT_CC1);
    }

    if (TIM_GetITStatus(BSP_US_ECHO_TIM, TIM_IT_Update) == SET)
    {
        if (us_listen)
        {
            us_echo_us = 0xFFFF;
            us_ok = 1;
            us_listen = 0;
        }

        TIM_ClearITPendingBit(BSP_US_ECHO_TIM, TIM_IT_Update);
    }
}
