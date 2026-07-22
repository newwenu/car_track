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
#define MOTION_RAMP_STEP            ((APP_MOTOR_PERIOD_MS * 5) / 50)

/* 电机不对称补偿系数：消除左右电机/机械差异导致的直线跑偏。
 * 标定方法：关闭循迹，给左右轮相同 PWM 直行 3 米，测量横向偏移。
 * 向右侧偏 -> 左轮偏快 -> 减小 MOTOR_LEFT_GAIN。
 * 默认 1.0/1.0 表示不补偿，有效范围通常在 0.90~1.10 之间。
 * 使用变量而非宏，便于 DEBUG 页面标定时读取/调整。 */
static float s_motor_left_gain = 1.00f;
static float s_motor_right_gain = 1.00f;

/* 起步助力：从静止目标 0 切换到非零目标时，先给一个较高占空比
 * 克服静摩擦/启动死区，持续一段时间后恢复斜坡控制。
 * KICK_PCT 按现场电机启动阈值调整（当前电源 7.9V，50% 不转、80% 才转，
 * 说明启动阈值在 60%~70% 附近，此处取 75% 留有余量）。 */
#define MOTION_STARTUP_KICK_PCT     75
#define MOTION_STARTUP_KICK_TICKS   (150 / APP_MOTOR_PERIOD_MS)

static int s_target_left = 0;
static int s_target_right = 0;
static int s_current_left = 0;
static int s_current_right = 0;
static motion_state_t s_state = MOTION_STATE_STOP;
static u16 s_startup_tick = 0;      /* 起步助力剩余 tick */

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

    /* 起步助力：从静止刚下发非零目标时，给一个较高占空比打破静摩擦 */
    if (s_startup_tick > 0)
    {
        if (s_target_left > 0)
        {
            s_current_left = (s_target_left > MOTION_STARTUP_KICK_PCT)
                             ? s_target_left : MOTION_STARTUP_KICK_PCT;
        }
        else if (s_target_left < 0)
        {
            s_current_left = (s_target_left < -MOTION_STARTUP_KICK_PCT)
                             ? s_target_left : -MOTION_STARTUP_KICK_PCT;
        }
        else
        {
            s_current_left = 0;
        }

        if (s_target_right > 0)
        {
            s_current_right = (s_target_right > MOTION_STARTUP_KICK_PCT)
                              ? s_target_right : MOTION_STARTUP_KICK_PCT;
        }
        else if (s_target_right < 0)
        {
            s_current_right = (s_target_right < -MOTION_STARTUP_KICK_PCT)
                              ? s_target_right : -MOTION_STARTUP_KICK_PCT;
        }
        else
        {
            s_current_right = 0;
        }

        s_startup_tick--;
        s_state = MOTION_STATE_RUN;
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
    s_startup_tick = 0;
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
    int new_left = motion_clamp(left_pct, -100, 100);
    int new_right = motion_clamp(right_pct, -100, 100);

    /* 应用电机不对称补偿系数：降低较快一侧的输出比提升较慢一侧更安全。
     * 注意：在起步助力判断之前应用，确保助力逻辑基于补偿后的目标。 */
    new_left = (int)(new_left * s_motor_left_gain);
    new_right = (int)(new_right * s_motor_right_gain);
    new_left = motion_clamp(new_left, -100, 100);
    new_right = motion_clamp(new_right, -100, 100);

    /* 从静止目标 0 切换到非零目标时触发一次起步助力，克服静摩擦/启动死区。
     * 仅在非刹车状态下触发；刹车释放由调用方显式控制。 */
    if (s_target_left == 0 && s_target_right == 0 &&
        (new_left != 0 || new_right != 0) &&
        s_state != MOTION_STATE_BRAKE)
    {
        s_startup_tick = MOTION_STARTUP_KICK_TICKS;
    }

    s_target_left = new_left;
    s_target_right = new_right;
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
    s_startup_tick = 0;
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
    s_startup_tick = 0;
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

void motion_get_target(int *left_pct, int *right_pct)
{
    if (left_pct != NULL)  *left_pct = s_target_left;
    if (right_pct != NULL) *right_pct = s_target_right;
}

/* 获取/设置电机补偿增益，供 DEBUG 页面标定使用 */
void motion_get_gains(float *left_gain, float *right_gain)
{
    if (left_gain != NULL)  *left_gain = s_motor_left_gain;
    if (right_gain != NULL) *right_gain = s_motor_right_gain;
}

void motion_set_gains(float left_gain, float right_gain)
{
    if (left_gain < 0.5f) left_gain = 0.5f;
    if (left_gain > 1.5f) left_gain = 1.5f;
    if (right_gain < 0.5f) right_gain = 0.5f;
    if (right_gain > 1.5f) right_gain = 1.5f;
    s_motor_left_gain = left_gain;
    s_motor_right_gain = right_gain;
}
