#include "ultrasonic.h"
#include "delay.h"

#define US_TX_PULSE_COUNT   8

/* 输入捕获定时器分频后 tick = 0.1ms，声波往返 1 tick 对应 1.7cm */
#define US_CM_PER_TICK      1.7f

/* 无回波超时：30ms = 300 个 tick，防止丢波后测距卡死 */
#define US_TIMEOUT_TICKS    300

/* 兼容 Keil __weak 与 GCC 语法检查 */
#ifndef __weak
    #ifdef __GNUC__
        #define __weak  __attribute__((weak))
    #else
        #define __weak
    #endif
#endif

static volatile u8  us_capture_ok = 0;
static volatile u16 us_capture_val = 0;
static volatile u8  us_echo_edge = 0;
static volatile float us_last_distance = 0.0f;

void ultrasonic_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(BSP_US_TX_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BSP_US_TX_TIM_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_US_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BSP_US_TX_PORT, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 1799;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(BSP_US_TX_TIM, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 900;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(BSP_US_TX_TIM, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(BSP_US_TX_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(BSP_US_TX_TIM, ENABLE);

    /* TIM1 是高级定时器，必须使能主输出，否则 PA8 没有 PWM 输出 */
    TIM_CtrlPWMOutputs(BSP_US_TX_TIM, ENABLE);

    RCC_APB2PeriphClockCmd(BSP_US_ECHO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(BSP_US_ECHO_TIM_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_US_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(BSP_US_ECHO_PORT, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 0xffff;
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
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
    TIM_SetCounter(BSP_US_TX_TIM, 0);
    TIM_Cmd(BSP_US_TX_TIM, ENABLE);
    delay_us(200);
    TIM_Cmd(BSP_US_TX_TIM, DISABLE);
}

void ultrasonic_start(void)
{
    us_capture_ok = 0;
    us_echo_edge = 0;
    us_last_distance = 0.0f;
    ultrasonic_trig();
}

float ultrasonic_get_distance(void)
{
    if (us_capture_ok)
    {
        us_capture_ok = 0;
        ultrasonic_trig();
    }
    else if (TIM_GetCounter(BSP_US_ECHO_TIM) >= US_TIMEOUT_TICKS)
    {
        /* 超时未收到回波：标记无效、复位捕获状态并重新触发，
         * 避免丢波后测距卡死，同时防止计数器持续增长导致 10ms 内反复重触发。 */
        us_last_distance = 0.0f;
        us_echo_edge = 0;
        TIM_SetCounter(BSP_US_ECHO_TIM, 0);
        ultrasonic_trig();
    }
    return us_last_distance;
}

/* 默认空实现；应用层可重定义以快速响应测距结果 */
__weak void ultrasonic_distance_ready_callback(float distance)
{
    (void)distance;
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(BSP_US_ECHO_TIM, TIM_IT_CC1) && us_echo_edge == 0)
    {
        TIM_OC1PolarityConfig(BSP_US_ECHO_TIM, TIM_ICPolarity_Falling);
        us_echo_edge = 1;
        TIM_SetCounter(BSP_US_ECHO_TIM, 0);
    }
    else if (TIM_GetITStatus(BSP_US_ECHO_TIM, TIM_IT_CC1) && us_echo_edge == 1)
    {
        us_capture_val = TIM_GetCapture1(BSP_US_ECHO_TIM);
        us_last_distance = us_capture_val * US_CM_PER_TICK;
        us_capture_ok = 1;

        /* 测距完成的瞬间通知应用层，便于实现 <10ms 的紧急刹车 */
        ultrasonic_distance_ready_callback(us_last_distance);

        TIM_OC1PolarityConfig(BSP_US_ECHO_TIM, TIM_ICPolarity_Rising);
        us_echo_edge = 0;
        TIM_SetCounter(BSP_US_ECHO_TIM, 0);
    }
    TIM_ClearITPendingBit(BSP_US_ECHO_TIM, TIM_IT_CC1 | TIM_IT_Update);
}
