#include "trace_control.h"
#include "../../BSP/sensor/trace.h"
#include "../../BSP/system/config.h"
#include <stddef.h>

/* ===================== 可调参数 ===================== */
/* 施密特触发器阈值：抑制黑白边界抖动。
 * 上升阈值（白→黑）与下降阈值（黑→白）之间形成死区。
 * 默认阈值在学习完成前使用；学习完成后会替换为每路自动标定的阈值。
 * 死区宽度建议为 ADC 量程的 5%~10%（约 200~400）。 */
#define TRACE_TH_HIGH_DEFAULT   850     /* ADC 高于此值判定为黑线 */
#define TRACE_TH_LOW_DEFAULT    600     /* ADC 低于此值判定为白底 */

/* 阈值自学习参数 */
#define TRACE_CALIB_SAMPLES     100     /* 采样次数：100 * 20ms = 2s */
#define TRACE_CALIB_MIN_RANGE   600     /* 要求黑白差距至少 600，确保阈值可靠 */

/* 基础速度百分比。
 * 按电机参数表：5V 空载线速度约 39m/min = 65cm/s；带载后实际最大约 40~55cm/s。
 * 55% PWM 对应空载约 36cm/s，带载约 22~30cm/s，适合普通循迹。
 * 30% PWM 对应空载约 20cm/s，带载约 12~17cm/s，适合过弯。
 * 若赛道直道较长且小车稳定，可适度提高 BASE_SPEED_NORMAL。 */
#define BASE_SPEED_NORMAL       55      /* 直道基础速度百分比 */
#define BASE_SPEED_CURVE        40      /* 弯道基础速度百分比 */
#define CURVE_ERROR_THR         0.9f    /* |error| 超过此值认为在弯道；4路最大误差为1.5 */

/* PD 增益：4路误差范围由 5 路的 ±2 缩小为 ±1.5，KP 按范围比例上调。
 * 起始建议：KP=24，KD=7；最终必须根据现场标定。 */
#define LINE_KP                 24      /* 循迹 P 增益 */
#define LINE_KD                 7       /* 循迹 D 增益 */

/* 丢线保护参数：trace 周期 20ms */
#define LOST_THRESHOLD          3       /* 连续 3 次 (60ms) 全未检测到黑线认为丢线 */
#define LOST_RECOVERY_TICKS     50      /* 丢线后寻线超时：50 * 20ms = 1s */
#define LOST_SPEED_PCT          30      /* 寻线基础速度 */
#define LOST_STEER_PCT          40      /* 寻线转向量 */

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

/* 阈值自学习数据 */
static u16  s_calib_min[BSP_TRACE_CH_COUNT];    /* 学习到的白底最小值 */
static u16  s_calib_max[BSP_TRACE_CH_COUNT];    /* 学习到的黑线最大值 */
static u16  s_calib_high[BSP_TRACE_CH_COUNT];   /* 学习后的上升阈值 */
static u16  s_calib_low[BSP_TRACE_CH_COUNT];    /* 学习后的下降阈值 */
static u16  s_calib_cnt = 0;                    /* 已采样次数 */
static u8   s_calib_state = 0;                  /* 0=未学习, 1=学习中, 2=学习成功, 3=学习失败 */

/* ===================== 内部辅助函数 ===================== */

/* 施密特触发器：根据当前 ADC 值和上次状态更新黑线判定状态。
 * 学习成功后使用每路独立阈值，否则使用默认阈值。 */
static void trace_update_black_state(void)
{
    u8 i;
    u16 th_high, th_low;
    u8 use_calib = (s_calib_state == 2);

    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        if (use_calib)
        {
            th_high = s_calib_high[i];
            th_low  = s_calib_low[i];
        }
        else
        {
            th_high = TRACE_TH_HIGH_DEFAULT;
            th_low  = TRACE_TH_LOW_DEFAULT;
        }

        if (s_black_state[i])
        {
            /* 当前为黑线：需低于下降阈值才切回白底 */
            if (s_vals[i] < th_low)
            {
                s_black_state[i] = 0;
            }
        }
        else
        {
            /* 当前为白底：需高于上升阈值才切到黑线 */
            if (s_vals[i] > th_high)
            {
                s_black_state[i] = 1;
            }
        }
    }
}

/* 阈值自学习：持续采样，记录每路最小/最大 ADC */
static void trace_calib_update(void)
{
    u8 i;
    u16 range;

    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        if (s_vals[i] < s_calib_min[i])
        {
            s_calib_min[i] = s_vals[i];
        }
        if (s_vals[i] > s_calib_max[i])
        {
            s_calib_max[i] = s_vals[i];
        }
    }

    s_calib_cnt++;
    if (s_calib_cnt >= TRACE_CALIB_SAMPLES)
    {
        /* 采样完成，计算每路阈值并检查黑白差距 */
        s_calib_state = 2;
        for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
        {
            range = s_calib_max[i] - s_calib_min[i];
            if (range < TRACE_CALIB_MIN_RANGE)
            {
                s_calib_state = 3;      /* 失败 */
                break;
            }
            /* 阈值放在黑白区间约 70%/30% 处，死区约占 40% */
            s_calib_high[i] = s_calib_min[i] + (u16)(range * 0.7f);
            s_calib_low[i]  = s_calib_min[i] + (u16)(range * 0.3f);
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
        s_black_state[i] = 0;
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

    /* 阈值自学习中：只更新学习数据，不输出电机控制 */
    if (s_calib_state == 1)
    {
        trace_calib_update();
        s_left_pct = 0;
        s_right_pct = 0;
        return;
    }

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

/* ===================== 阈值自学习接口 ===================== */

/* 启动阈值自学习：上电后把传感器依次在黑线和白底上移动约 2 秒 */
void trace_calib_start(void)
{
    u8 i;
    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        s_calib_min[i] = 4095;
        s_calib_max[i] = 0;
        s_calib_high[i] = TRACE_TH_HIGH_DEFAULT;
        s_calib_low[i]  = TRACE_TH_LOW_DEFAULT;
    }
    s_calib_cnt = 0;
    s_calib_state = 1;
}

/* 返回自学习状态：0=未学习, 1=学习中, 2=成功, 3=失败 */
u8 trace_calib_get_state(void)
{
    return s_calib_state;
}

/* 获取学习结果：0=未成功，1=成功 */
u8 trace_calib_is_ok(void)
{
    return (s_calib_state == 2) ? 1 : 0;
}

/* 获取指定通道学习到的最小/最大 ADC 值 */
void trace_calib_get_range(u8 idx, u16 *min_val, u16 *max_val)
{
    if (idx >= BSP_TRACE_CH_COUNT)
    {
        return;
    }
    if (min_val != NULL)
    {
        *min_val = s_calib_min[idx];
    }
    if (max_val != NULL)
    {
        *max_val = s_calib_max[idx];
    }
}

/* 获取指定通道学习后的阈值 */
void trace_calib_get_thresholds(u8 idx, u16 *th_high, u16 *th_low)
{
    if (idx >= BSP_TRACE_CH_COUNT)
    {
        return;
    }
    if (th_high != NULL)
    {
        *th_high = (s_calib_state == 2) ? s_calib_high[idx] : TRACE_TH_HIGH_DEFAULT;
    }
    if (th_low != NULL)
    {
        *th_low  = (s_calib_state == 2) ? s_calib_low[idx]  : TRACE_TH_LOW_DEFAULT;
    }
}
