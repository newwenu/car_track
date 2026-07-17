#include "motor.h"
#include "pwm.h"

void motor_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(BSP_MOTOR_DIR_CLK_ALL, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = BSP_MOTOR_IN1_PIN | BSP_MOTOR_IN2_PIN;
    GPIO_Init(BSP_MOTOR_IN1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BSP_MOTOR_IN3_PIN | BSP_MOTOR_IN4_PIN;
    GPIO_Init(BSP_MOTOR_IN3_PORT, &GPIO_InitStructure);

    motor_stop();
}

void motor_run(int left_speed, int right_speed)
{
    u16 left_pwm, right_pwm;

    /* 限制速度在有效 PWM 范围内，防止 CCR 超过 ARR */
    if (left_speed > BSP_MOTOR_PWM_PERIOD)  left_speed = BSP_MOTOR_PWM_PERIOD;
    if (left_speed < -BSP_MOTOR_PWM_PERIOD) left_speed = -BSP_MOTOR_PWM_PERIOD;
    if (right_speed > BSP_MOTOR_PWM_PERIOD)  right_speed = BSP_MOTOR_PWM_PERIOD;
    if (right_speed < -BSP_MOTOR_PWM_PERIOD) right_speed = -BSP_MOTOR_PWM_PERIOD;

    if (left_speed > 0)
    {
        MOTOR_L_IN1 = 1;
        MOTOR_L_IN2 = 0;
        left_pwm = left_speed;
    }
    else if (left_speed < 0)
    {
        MOTOR_L_IN1 = 0;
        MOTOR_L_IN2 = 1;
        left_pwm = -left_speed;
    }
    else
    {
        MOTOR_L_IN1 = 0;
        MOTOR_L_IN2 = 0;
        left_pwm = 0;
    }

    if (right_speed > 0)
    {
        MOTOR_R_IN3 = 1;
        MOTOR_R_IN4 = 0;
        right_pwm = right_speed;
    }
    else if (right_speed < 0)
    {
        MOTOR_R_IN3 = 0;
        MOTOR_R_IN4 = 1;
        right_pwm = -right_speed;
    }
    else
    {
        MOTOR_R_IN3 = 0;
        MOTOR_R_IN4 = 0;
        right_pwm = 0;
    }

    pwm_motor_set(left_pwm, right_pwm);
}

void motor_stop(void)
{
    MOTOR_L_IN1 = 0;
    MOTOR_L_IN2 = 0;
    MOTOR_R_IN3 = 0;
    MOTOR_R_IN4 = 0;
    pwm_motor_set(0, 0);
}
