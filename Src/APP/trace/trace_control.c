#include "trace_control.h"
#include "../../BSP/sensor/trace.h"
#include "../../BSP/system/config.h"
#include <stddef.h>

/* ===================== 可调参数 ===================== */
#define TRACE_BLACK_THR         2000    /* ADC 大于此值认为在黑线上 */

/* 基础速度百分比。
 * 按电机参数表：5V 空载线速度约 39m/min = 65cm/s；带载后实际最大约 40~55cm/s。
 * 55% PWM 对应空载约 36cm/s，带载约 22~30cm/s，适合普通循迹。
 * 30% PWM 对应空载约 20cm/s，带载约 12~17cm/s，适合过弯。
 * 若赛道直道较长且小车稳定，可适度提高 BASE_SPEED_NORMAL。 */
#define BASE_SPEED_NORMAL       55      /* 直道基础速度百分比 */
#define BASE_SPEED_CURVE        40      /* 弯道基础速度百分比 */
#define CURVE_ERROR_THR         1.0f    /* |error| 超过此值认为在弯道 */

#define LINE_KP                 18      /* 循迹 P 增益 */
#define LINE_KD                 5       /* 循迹 D 增益；trace 周期从 50ms 改为 20ms，D 按 0.4 比例下调 */

/* 丢线保护参数：trace 周期 20ms */
#define LOST_THRESHOLD          3       /* 连续 3 次 (60ms) 全未检测到黑线认为丢线 */
#define LOST_RECOVERY_TICKS     50      /* 丢线后寻线超时：50 * 20ms = 1s */
#define LOST_SPEED_PCT          30      /* 寻线基础速度 */
#define LOST_STEER_PCT          40      /* 寻线转向量 */

/* 传感器权重：AO1最右=+2，AO5最左=-2 */
static const float s_weights[BSP_TRACE_CH_COUNT] = {
    +2.0f, +1.0f, 0.0f, -1.0f, -2.0f
};

/* ===================== 静态变量 ===================== */
static u16  s_vals[BSP_TRACE_CH_COUNT];
static float s_error = 0.0f;
static float s_last_error = 0.0f;
static int  s_left_pct = 0;
static int  s_right_pct = 0;
static u8   s_lost_cnt = 0;            /* 连续丢线计数 */
static u8   s_lost_recovery_cnt = 0;   /* 寻线耗时计数 */
static u8   s_is_lost = 0;             /* 丢线标志 */

/* ===================== 内部辅助函数 ===================== */

/* 计算加权平均误差 */
static float trace_calc_error(void)
{
    float sum = 0.0f;
    float cnt = 0.0f;
    u8 i;

    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        if (s_vals[i] > TRACE_BLACK_THR)
        {
            sum += s_weights[i];
            cnt += 1.0f;
        }
    }

    if (cnt > 0.0f)
    {
        /* 重新检测到黑线，清零丢线状态 */
        s_lost_cnt = 0;
        s_lost_recovery_cnt = 0;
        s_is_lost = 0;
        return sum / cnt;
    }

    /* 全部未检测到黑线：递增丢线计数 */
    if (s_lost_cnt < LOST_THRESHOLD)
    {
        s_lost_cnt++;
    }

    if (s_lost_cnt >= LOST_THRESHOLD)
    {
        s_is_lost = 1;
    }

    /* 短暂丢线时沿用上一次误差，用于惯性寻线 */
    return s_last_error;
}

/* ===================== 接口实现 ===================== */

void trace_control_init(void)
{
    u8 i;
    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        s_vals[i] = 0;
    }
    s_error = 0.0f;
    s_last_error = 0.0f;
    s_left_pct = 0;
    s_right_pct = 0;
    s_lost_cnt = 0;
    s_lost_recovery_cnt = 0;
    s_is_lost = 0;
}

void trace_control_update(void)
{
    float error;
    float diff;
    int base_speed;

    trace_read(s_vals);

    error = trace_calc_error();
    s_error = error;

    /* 丢线处理：低速按历史误差方向寻线，超时后停车 */
    if (s_is_lost)
    {
        s_lost_recovery_cnt++;
        if (s_lost_recovery_cnt >= LOST_RECOVERY_TICKS)
        {
            /* 寻线超时：输出 0，让上层决定停车 */
            s_left_pct = 0;
            s_right_pct = 0;
            return;
        }

        base_speed = LOST_SPEED_PCT;
        /* 按 last_error 方向转弯寻线：last_error > 0 黑线偏右，应右转 */
        diff = (s_last_error >= 0.0f) ? LOST_STEER_PCT : -LOST_STEER_PCT;
    }
    else
    {
        /* 弯道减速 */
        if (error > CURVE_ERROR_THR || error < -CURVE_ERROR_THR)
        {
            base_speed = BASE_SPEED_CURVE;
        }
        else
        {
            base_speed = BASE_SPEED_NORMAL;
        }

        /* PD 输出 */
        diff = LINE_KP * error + LINE_KD * (error - s_last_error);
        s_last_error = error;
    }

    s_left_pct  = base_speed + (int)diff;
    s_right_pct = base_speed - (int)diff;
}

float trace_control_get_error(void)
{
    return s_error;
}

u8 trace_control_is_all_black(void)
{
    u8 i;
    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        if (s_vals[i] <= TRACE_BLACK_THR)
        {
            return 0;
        }
    }
    return 1;
}

void trace_control_get_wheel_targets(int *left_pct, int *right_pct)
{
    if (left_pct != NULL)
    {
        *left_pct = s_left_pct;
    }
    if (right_pct != NULL)
    {
        *right_pct = s_right_pct;
    }
}

/* 判断是否处于丢线状态（连续多次未检测到黑线） */
u8 trace_control_is_lost(void)
{
    return s_is_lost;
}

/* 判断丢线后寻线是否已超时 */
u8 trace_control_is_lost_timeout(void)
{
    return s_is_lost && (s_lost_recovery_cnt >= LOST_RECOVERY_TICKS);
}

float trace_control_get_last_error(void)
{
    return s_last_error;
}

void trace_control_get_lost_info(u8 *lost_cnt, u8 *recovery_cnt)
{
    if (lost_cnt != NULL)
    {
        *lost_cnt = s_lost_cnt;
    }
    if (recovery_cnt != NULL)
    {
        *recovery_cnt = s_lost_recovery_cnt;
    }
}
