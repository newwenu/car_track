#include "motion_control.h"
#include "../../BSP/actuator/motor.h"
#include "../../BSP/actuator/pwm.h"
#include "../../BSP/system/config.h"
#include "../app.h"
#include <stddef.h>

/* 速度斜坡步进：保持与原来 50ms/5% 相同的总斜坡时间约 1s。
 * 现在 motion_update() 每 APP_MOTOR_PERIOD_MS 调用一次，故步进按比例缩小。
 * 按电机参数小车加速度能力富余，此处取保守斜坡以避免起步打滑。
 * 若现场起步响应慢，可适当增大步进。 */
#define MOTION_RAMP_STEP    ((APP_MOTOR_PERIOD_MS * 5) / 50)

static int s_target_left = 0;
static int s_target_right = 0;
static int s_current_left = 0;
static int s_current_right = 0;
static motion_state_t s_state = MOTION_STATE_STOP;

static int motion_clamp(int val, int min, int max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/* 百分比转 PWM 比较值，-100~100 映射到 -MOTOR_PWM_PERIOD~+MOTOR_PWM_PERIOD */
static int motion_pct_to_pwm(int pct)
{
    pct = motion_clamp(pct, -100, 100);
    return (int)((long)BSP_MOTOR_PWM_PERIOD * pct / 100);
}

/* 速度斜坡：current 平滑逼近 target */
static void motion_ramp(void)
{
    if (s_state == MOTION_STATE_BRAKE)
    {
        s_current_left = 0;
        s_current_right = 0;
        return;
    }

    if (s_current_left < s_target_left)
    {
        s_current_left += MOTION_RAMP_STEP;
        if (s_current_left > s_target_left) s_current_left = s_target_left;
    }
    else if (s_current_left > s_target_left)
    {
        s_current_left -= MOTION_RAMP_STEP;
        if (s_current_left < s_target_left) s_current_left = s_target_left;
    }

    if (s_current_right < s_target_right)
    {
        s_current_right += MOTION_RAMP_STEP;
        if (s_current_right > s_target_right) s_current_right = s_target_right;
    }
    else if (s_current_right > s_target_right)
    {
        s_current_right -= MOTION_RAMP_STEP;
        if (s_current_right < s_target_right) s_current_right = s_target_right;
    }

    if (s_current_left == 0 && s_current_right == 0 &&
        s_target_left == 0 && s_target_right == 0)
    {
        s_state = MOTION_STATE_STOP;
    }
    else
    {
        s_state = MOTION_STATE_RUN;
    }
}

void motion_init(void)
{
    s_target_left = 0;
    s_target_right = 0;
    s_current_left = 0;
    s_current_right = 0;
    s_state = MOTION_STATE_STOP;
    motor_stop();
}

void motion_update(void)
{
    motion_ramp();

    if (s_state == MOTION_STATE_BRAKE)
    {
        /* 能耗制动：H 桥两输入同高，PWM 全高，电机两端短接 */
        MOTOR_L_IN1 = 1;
        MOTOR_L_IN2 = 1;
        MOTOR_R_IN3 = 1;
        MOTOR_R_IN4 = 1;
        pwm_motor_set(BSP_MOTOR_PWM_PERIOD, BSP_MOTOR_PWM_PERIOD);
        return;
    }

    if (s_state == MOTION_STATE_STOP)
    {
        motor_stop();
        return;
    }

    motor_run(motion_pct_to_pwm(s_current_left), motion_pct_to_pwm(s_current_right));
}

void motion_set_wheels(int left_pct, int right_pct)
{
    /* 仅更新目标速度，不自动解除刹车；刹车释放由调用方显式控制 */
    s_target_left = motion_clamp(left_pct, -100, 100);
    s_target_right = motion_clamp(right_pct, -100, 100);
}

void motion_run_dir(motion_dir_t dir, int speed_pct)
{
    speed_pct = motion_clamp(speed_pct, 0, 100);

    switch (dir)
    {
        case MOTION_DIR_FORWARD:
            motion_set_wheels(speed_pct, speed_pct);
            break;
        case MOTION_DIR_BACKWARD:
            motion_set_wheels(-speed_pct, -speed_pct);
            break;
        case MOTION_DIR_LEFT:
            motion_set_wheels(-speed_pct, speed_pct);
            break;
        case MOTION_DIR_RIGHT:
            motion_set_wheels(speed_pct, -speed_pct);
            break;
        default:
            motion_set_wheels(0, 0);
            break;
    }
}

void motion_set_speed(int speed_pct)
{
    speed_pct = motion_clamp(speed_pct, -100, 100);
    motion_set_wheels(speed_pct, speed_pct);
}

void motion_stop(void)
{
    s_target_left = 0;
    s_target_right = 0;
    if (s_state == MOTION_STATE_BRAKE)
    {
        s_state = MOTION_STATE_STOP;
    }
}

void motion_brake(void)
{
    s_state = MOTION_STATE_BRAKE;
    s_target_left = 0;
    s_target_right = 0;
    s_current_left = 0;
    s_current_right = 0;
}

motion_state_t motion_get_state(void)
{
    return s_state;
}

void motion_get_current(int *left_pct, int *right_pct)
{
    if (left_pct != NULL)  *left_pct = s_current_left;
    if (right_pct != NULL) *right_pct = s_current_right;
}
