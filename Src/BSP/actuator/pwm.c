#include "pwm.h"

void pwm_motor_init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(BSP_MOTOR_PWM_TIM_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BSP_MOTOR_PWM_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_MOTOR_PWM_L_PIN | BSP_MOTOR_PWM_R_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BSP_MOTOR_PWM_PORT, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(BSP_MOTOR_PWM_TIM, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(BSP_MOTOR_PWM_TIM, &TIM_OCInitStructure);
    TIM_OC2Init(BSP_MOTOR_PWM_TIM, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(BSP_MOTOR_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(BSP_MOTOR_PWM_TIM, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(BSP_MOTOR_PWM_TIM, ENABLE);
    TIM_Cmd(BSP_MOTOR_PWM_TIM, ENABLE);

    /* TIM2 PWM 频率为 10kHz（100us），借 Update 中断软件 10 分频得到 1ms 系统 tick。
     * 不影响 CH1/CH2 的电机 PWM 输出。 */
    TIM_ClearFlag(BSP_MOTOR_PWM_TIM, TIM_FLAG_Update);
    TIM_ITConfig(BSP_MOTOR_PWM_TIM, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void pwm_motor_set(u16 left, u16 right)
{
    BSP_MOTOR_PWM_TIM->CCR1 = left;
    BSP_MOTOR_PWM_TIM->CCR2 = right;
}

void pwm_buzzer_init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(BSP_BUZZER_TIM_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BSP_BUZZER_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_BUZZER_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BSP_BUZZER_PORT, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(BSP_BUZZER_TIM, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(BSP_BUZZER_TIM, &TIM_OCInitStructure);

    TIM_OC4PreloadConfig(BSP_BUZZER_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(BSP_BUZZER_TIM, ENABLE);
    TIM_Cmd(BSP_BUZZER_TIM, ENABLE);
}

void pwm_buzzer_set(u16 val)
{
    TIM_SetCompare4(BSP_BUZZER_TIM, val);
}
