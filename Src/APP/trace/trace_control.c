#include "trace_control.h"
#include "../../BSP/sensor/trace.h"
#include "../../BSP/system/config.h"
#include <stddef.h>

/* ===================== 可调参数 ===================== */
/* 施密特触发器阈值：抑制黑白边界抖动。
 * 上升阈值（白→黑）与下降阈值（黑→白）之间形成死区。
 * 死区宽度建议为 ADC 量程的 5%~10%（约 200~400）。 */
#define TRACE_TH_HIGH_DEFAULT   850     /* ADC 高于此值判定为黑线 */
#define TRACE_TH_LOW_DEFAULT    600     /* ADC 低于此值判定为白底 */

/* 基础速度百分比。
 * 按电机参数表：5V 空载线速度约 39m/min = 65cm/s；带载后实际最大约 40~55cm/s。
 * 55% PWM 对应空载约 36cm/s，带载约 22~30cm/s，适合普通循迹。
 * 30% PWM 对应空载约 20cm/s，带载约 12~17cm/s，适合过弯。
 * 若赛道直道较长且小车稳定，可适度提高 BASE_SPEED_NORMAL。 */
#define BASE_SPEED_NORMAL       40      /* 直道基础速度百分比：调低便于观察调试 */
#define BASE_SPEED_CURVE        30      /* 弯道基础速度百分比：调低便于观察调试 */
#define CURVE_ERROR_THR         0.9f    /* |error| 超过此值认为在弯道；4路最大误差为1.5 */

/* PD 增益：赛道弯道半径约 0.6m，转弯响应偏慢，适度提高 P 增益；
 * D 增益保留一定阻尼，防止超调。 */
#define LINE_KP                 20      /* 循迹 P 增益 */
#define LINE_KD                 6       /* 循迹 D 增益 */
#define LINE_MAX_DIFF           25      /* 最大转向量限制，避免转弯角度过大 */

/* 丢线保护参数：trace 周期 20ms */
#define LOST_THRESHOLD          3       /* 连续 3 次 (60ms) 全未检测到黑线认为丢线 */
#define LOST_RECOVERY_TICKS     150     /* 丢线后寻线超时：150 * 20ms = 3s */
#define LOST_PHASE_TICKS        25      /* 每 0.5s 切换一次寻线方向，避免单向一直旋转 */
#define LOST_SPEED_PCT          30      /* 寻线基础速度：保证内轮不为 0 */
#define LOST_STEER_PCT          10      /* 寻线转向量：减小丢线时转弯角度，双轮差速寻线 */
#define STRAIGHT_COAST_THR      0.4f    /* |error| 小于此值认为之前在直道 */
#define STRAIGHT_COAST_TICKS    10      /* 直道短暂丢线后继续直行的周期数 (10 * 20ms = 200ms) */

/* 传感器权重：4路等距布局，AO1最右=+1.5，AO4最左=-1.5 */
static const float s_weights[BSP_TRACE_CH_COUNT] = {
    +1.5f, +0.5f, -0.5f, -1.5f
};

/* ===================== 静态变量 ===================== */
static u16  s_vals[BSP_TRACE_CH_COUNT];
static u8   s_black_state[BSP_TRACE_CH_COUNT];  /* 施密特触发器输出：1=黑线，0=白底 */
static float s_error = 0.0f;
static float s_last_error = 0.0f;
static int  s_left_pct = 0;
static int  s_right_pct = 0;
static u8   s_lost_cnt = 0;            /* 连续丢线计数 */
static u8   s_lost_recovery_cnt = 0;   /* 寻线耗时计数 */
static u8   s_is_lost = 0;             /* 丢线标志 */
static u8   s_straight_coast_cnt = 0;  /* 直道丢线后继续直行计数 */

/* ===================== 内部辅助函数 ===================== */

/* 施密特触发器：根据当前 ADC 值和上次状态更新黑线判定状态。 */
static void trace_update_black_state(void)
{
    u8 i;

    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        if (s_black_state[i])
        {
            /* 当前为黑线：需低于下降阈值才切回白底 */
            if (s_vals[i] < TRACE_TH_LOW_DEFAULT)
            {
                s_black_state[i] = 0;
            }
        }
        else
        {
            /* 当前为白底：需高于上升阈值才切到黑线 */
            if (s_vals[i] > TRACE_TH_HIGH_DEFAULT)
            {
                s_black_state[i] = 1;
            }
        }
    }
}

/* 计算加权平均误差 */
static float trace_calc_error(void)
{
    float sum = 0.0f;
    float cnt = 0.0f;
    u8 i;

    trace_update_black_state();

    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        if (s_black_state[i])
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
        s_straight_coast_cnt = 0;
        s_is_lost = 0;

        /* 中间两个通道（AO2、AO3）任意一个检测到黑线且两侧都没有时，
         * 认为小车居中在线上，按直行处理，抑制小幅抖动。 */
        if ((s_black_state[0] == 0) && (s_black_state[3] == 0) &&
            ((s_black_state[1] == 1) || (s_black_state[2] == 1)))
        {
            return 0.0f;
        }

        return sum / cnt;
    }

    /* 全部未检测到黑线：若之前处于直道（误差较小），允许短暂继续直行，
     * 不立即计为丢线，提高对黑线断续或反光间隙的容忍度。 */
    if ((s_last_error > -STRAIGHT_COAST_THR) &&
        (s_last_error < STRAIGHT_COAST_THR) &&
        (s_straight_coast_cnt < STRAIGHT_COAST_TICKS))
    {
        s_straight_coast_cnt++;
        return 0.0f;
    }

    /* 非直道丢线或直道滑行超时，递增丢线计数 */
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
        s_black_state[i] = 0;
    }
    s_error = 0.0f;
    s_last_error = 0.0f;
    s_left_pct = 0;
    s_right_pct = 0;
    s_lost_cnt = 0;
    s_lost_recovery_cnt = 0;
    s_straight_coast_cnt = 0;
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

    /* 丢线处理：低速按历史误差方向寻线，期间一旦重新检测到黑线会自动退出丢线状态并恢复循迹 */
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
        /* 按 last_error 方向转弯寻线：last_error > 0 黑线偏右，应右转。
         * 每隔 LOST_PHASE_TICKS 切换一次方向，避免单向一直旋转超过 270°。 */
        if (((s_lost_recovery_cnt / LOST_PHASE_TICKS) % 2) == 0)
        {
            diff = (s_last_error >= 0.0f) ? LOST_STEER_PCT : -LOST_STEER_PCT;
        }
        else
        {
            diff = (s_last_error >= 0.0f) ? -LOST_STEER_PCT : LOST_STEER_PCT;
        }
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

        /* 限制最大转向量，防止 0.6m 半径弯道下转弯角度过大 */
        if (diff > LINE_MAX_DIFF)
        {
            diff = LINE_MAX_DIFF;
        }
        else if (diff < -LINE_MAX_DIFF)
        {
            diff = -LINE_MAX_DIFF;
        }
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
        if (!s_black_state[i])
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
