#include "ultrasonic.h"
#include "delay.h"

/* Timing constants (microseconds) */
#define US_TX_US     500   /* 40kHz TX burst duration */
#define US_BLIND_US  250   /* blind zone (~4cm), skip transducer ringing */
#define US_TO_US     3500  /* timeout (~65cm range) */

/* Internal state */
static volatile u16 us_echo_us  = 0;   /* echo arrival time (us from blind end) */
static volatile u8  us_ok       = 0;   /* capture complete flag */
static volatile u8  us_listen   = 0;   /* 0=blind/ignore, 1=listening */

/* [修复] 兼容 Keil MDK / GCC 的弱符号定义 */
#ifndef __weak
    #ifdef __GNUC__
        #define __weak  __attribute__((weak))
    #else
        #define __weak
    #endif
#endif

/* [修复] 默认空实现；应用层可重定义以快速响应测距结果（用于紧急刹车）*/
__weak void ultrasonic_distance_ready_callback(float distance)
{
    (void)distance);
}

void ultrasonic_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /*--- TX: TIM1_CH1, PA8, 40kHz PWM ---*/
    RCC_APB2PeriphClockCmd(BSP_US_TX_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BSP_US_TX_TIM_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_US_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BSP_US_TX_PORT, &GPIO_InitStructure);

    TIM_InternalClockConfig(BSP_US_TX_TIM);
    TIM_TimeBaseStructure.TIM_Period = 1800 - 1;         /* 72M/1800=40kHz */
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(BSP_US_TX_TIM, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 900;                 /* 50% duty cycle */
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(BSP_US_TX_TIM, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(BSP_US_TX_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(BSP_US_TX_TIM, ENABLE);
    TIM_CtrlPWMOutputs(BSP_US_TX_TIM, ENABLE);

    TIM_Cmd(BSP_US_TX_TIM, DISABLE);   /* TX off initially */

    /*--- RX: TIM4_CH1, PB6, input capture @ 1us resolution ---*/
    RCC_APB2PeriphClockCmd(BSP_US_ECHO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(BSP_US_ECHO_TIM_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_US_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(BSP_US_ECHO_PORT, &GPIO_InitStructure);

    TIM_InternalClockConfig(BSP_US_ECHO_TIM);
    TIM_TimeBaseStructure.TIM_Period = 5000 - 1;         /* 5ms = ~86cm max */
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;        /* 1us tick */
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

    TIM_ITConfig(BSP_US_ECHO_TIM, TIM_IT_CC1, ENABLE);
    TIM_Cmd(BSP_US_ECHO_TIM, ENABLE);
}

/*
 * Transmit a 40kHz burst, wait through blind zone, then arm echo capture.
 * Called by ultrasonic_start() and auto-retriggered after each measurement.
 */
static void ultrasonic_trig(void)
{
    us_listen = 0;
    us_ok = 0;
    us_echo_us = 0;

    /* 1. TX burst: 500us of 40kHz pulses */
    TIM_SetCounter(BSP_US_TX_TIM, 0);
    TIM_Cmd(BSP_US_TX_TIM, ENABLE);
    delay_us(US_TX_US);
    TIM_Cmd(BSP_US_TX_TIM, DISABLE);

    /* 2. Blind zone: skip transducer ringing & electrical crosstalk */
    delay_us(US_BLIND_US);

    /* 3. Arm capture: reset echo timer, clear pending, start listening */
    TIM_SetCounter(BSP_US_ECHO_TIM, 0);
    TIM_ClearITPendingBit(BSP_US_ECHO_TIM, TIM_IT_CC1);
    us_listen = 1;
}

static float us_last_distance = 0.0f;

/* Non-blocking: trigger the next measurement cycle */
void ultrasonic_start(void)
{
    ultrasonic_trig();
}

/*
 * Poll for the latest completed measurement.
 * Returns last valid distance (cm), or 0.0 if no measurement yet.
 * Auto-retriggers after each successful capture.
 * [修复] 恢复回调调用 + 超时保护机制
 */
float ultrasonic_get_distance(void)
{
    static u32 s_no_update_cnt = 0;  /* [修复] 连续无更新计数器 */

    if (us_ok)
    {
        /* Total flight time = blind zone + captured timer (us)
         * Distance (cm) = flight_time_us / 58
         * (sound travels 1cm in ~29us, round-trip = 58us/cm) */
        us_last_distance = (US_BLIND_US + us_echo_us) / 58.0f;

        /* [修复] 通知应用层测距完成（用于紧急刹车响应）*/
        ultrasonic_distance_ready_callback(us_last_distance);

        us_ok = 0;
        s_no_update_cnt = 0;  /* [修复] 重置超时计数 */
        ultrasonic_trig();     /* auto-start next measurement */
    }
    else
    {
        /* [修复] 超时保护：防止丢波后一直返回旧数据导致误判 */
        s_no_update_cnt++;
        if (s_no_update_cnt > 300)  /* 300次 * 10ms(调用周期) ≈ 3s无更新 */
        {
            us_last_distance = 0.0f;  /* 标记无效距离 */
            s_no_update_cnt = 0;
            ultrasonic_trig();         /* 强制重新触发测量 */
        }
    }
    return us_last_distance;
}

/*
 * Blocking convenience function — matches user's Test_Distance() pattern.
 * Blocks up to ~3.5ms for a single synchronous measurement.
 * Returns distance in cm, or 0.0 on timeout.
 */
float ultrasonic_measure_blocking(void)
{
    u32 timeout;

    ultrasonic_trig();

    timeout = US_TO_US;
    while (!us_ok && --timeout) { /* spin */ }
    us_listen = 0;

    if (us_ok)
        return (US_BLIND_US + us_echo_us) / 58.0f;
    else
        return 0.0f;
}

/*
 * TIM4 capture interrupt — fires on first rising edge after blind zone.
 * Capture time = echo arrival time from end of blind zone.
 */
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(BSP_US_ECHO_TIM, TIM_IT_CC1) == SET)
    {
        if (us_listen)
        {
            us_echo_us = TIM_GetCapture1(BSP_US_ECHO_TIM);
            us_ok = 1;
        }
        TIM_ClearITPendingBit(BSP_US_ECHO_TIM, TIM_IT_CC1);
    }
}