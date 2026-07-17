#include "ultrasonic.h"
#include "delay.h"

#define US_TX_PULSE_COUNT   8

static u8  us_capture_ok = 0;
static u16 us_capture_val = 0;
static u8  us_echo_edge = 0;

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

static float us_last_distance = 0.0f;

void ultrasonic_start(void)
{
    us_capture_ok = 0;
    us_echo_edge = 0;
    ultrasonic_trig();
}

float ultrasonic_get_distance(void)
{
    if (us_capture_ok)
    {
        us_last_distance = us_capture_val * 0.017f;
        us_capture_ok = 0;
        ultrasonic_trig();
    }
    return us_last_distance;
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
        us_capture_ok = 1;
        TIM_OC1PolarityConfig(BSP_US_ECHO_TIM, TIM_ICPolarity_Rising);
        us_echo_edge = 0;
        TIM_SetCounter(BSP_US_ECHO_TIM, 0);
    }
    TIM_ClearITPendingBit(BSP_US_ECHO_TIM, TIM_IT_CC1 | TIM_IT_Update);
}
