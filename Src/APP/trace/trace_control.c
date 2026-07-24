#include "trace_control.h"
#include "../../BSP/sensor/trace.h"
#include "../../BSP/system/config.h"
#include <stddef.h>

/* ===================== 可调参数 ===================== */
/* 施密特触发器阈值：抑制黑白边界抖动。
 * 上升阈值（白→黑）与下降阈值（黑→白）之间形成死区。
 * 死区宽度建议为 ADC 量程的 5%~10%（约 200~400）。 */
#define TRACE_TH_HIGH_DEFAULT   750    /* ADC 高于此值判定为黑线 */
#define TRACE_TH_LOW_DEFAULT    600     /* ADC 低于此值判定为白底 */

/* 基础速度百分比。
 * 按电机参数表：5V 空载线速度约 39m/min = 65cm/s；带载后实际最大约 40~55cm/s。
 * 55% PWM 对应空载约 36cm/s，带载约 22~30cm/s，适合普通循迹。
 * 30% PWM 对应空载约 20cm/s，带载约 12~17cm/s，适合过弯。
 * 若赛道直道较长且小车稳定，可适度提高 BASE_SPEED_NORMAL。 */
#define BASE_SPEED_NORMAL       65      /* 直道基础速度百分比：调低便于观察调试 */
#define BASE_SPEED_CURVE        43      /* 弯道基础速度百分比：调低便于观察调试 */
// #define CURVE_ERROR_THR         0.95f    /* |error| 超过此值认为在弯道；4路最大误差为1.5 */

/* ===================== 分段PD控制参数 ===================== */
/* 分段P增益：解决"转不过来"和"转过头"的矛盾。
 * 小误差时高灵敏度快速响应，大误差时降低增益防止超调。
 * 配合弯道出口制动机制，有效防止圆弧切线位置出线。 */
#define LINE_KP_LOW             28      /* 小误差区(|e|<0.5) P增益：提高直道灵敏度 */
#define LINEKP_MID              20      /* 中误差区(0.5~1.0) P增益：标准响应 */
#define LINE_KP_HIGH            14      /* 大误差区(|e|>1.0) P增益：防止急弯超调 */
#define LINE_KD                 8       /* 循迹 D 增益（原6→8，增强阻尼防超调） */
#define LINE_MAX_DIFF           35      /* 最大转向量限制 */

/* ===================== 平滑速度控制参数 ===================== */
/* 渐进式速度切换：消除阶跃突变，实现平滑过渡。
 * 使用一阶低通滤波，每周期向目标速度靠近一定比例。
 * RATE越大过渡越快，越小越平滑。推荐值0.05~0.15。 */
#define SPEED_TRANSITION_RATE   0.1f    /* 速度过渡速率：每周期接近目标的10% */
#define CURVE_ERROR_THR         0.95f   /* |error| 超过此值认为在弯道 */

/* ===================== 弯道出口制动参数 ===================== */
/* 圆弧切线位置防出线机制：
 * 当检测到error从大值快速减小时（正在离开弯道），
 * 提前施加额外制动转向量，抵消车辆惯性。
 * 类似汽车进弯前预刹车，但这里是"出弯后预纠偏"。 */
#define EXIT_BRAKE_ZONE         0.6f    /* 制动触发区：|error|低于此值且正在减小 */
#define EXIT_BRAKE_COEF         4.0f    /* 制动强度系数：乘以error减小速率 */

/* 丢线保护参数：trace 周期 20ms */
#define LOST_THRESHOLD          3       /* 连续 3 次 (60ms) 全未检测到黑线认为丢线 */
#define LOST_RECOVERY_TICKS     150     /* 丢线后寻线超时：150 * 20ms = 3s */
#define LOST_PHASE_TICKS        25      /* 每 0.5s 切换一次寻线方向，避免单向一直旋转 */
#define LOST_SPEED_PCT          30      /* 寻线基础速度：保证内轮不为 0 */
#define LOST_STEER_PCT          10      /* 寻线转向量：减小丢线时转弯角度，双轮差速寻线 */
#define STRAIGHT_COAST_THR      0.4f    /* |error| 小于此值认为之前在直道 */
#define STRAIGHT_COAST_TICKS    10      /* 直道短暂丢线后继续直行的周期数 (10 * 20ms = 200ms) */
#define ALL_BLACK_COAST_TICKS   5       /* 全黑时维持之前状态，5 * 20ms = 100ms */

/* 左右轮平衡补偿：针对硬件个体差异导致的双轮出力不对称。
 * 标准值 100 = 无补偿；若左轮偏弱，增大 MOTOR_LEFT_BALANCE；
 * 若右轮偏弱，增大 MOTOR_RIGHT_BALANCE。范围建议 80~120。 */
#define MOTOR_LEFT_BALANCE      105     /* 左轮补偿百分比 */
#define MOTOR_RIGHT_BALANCE     100     /* 右轮补偿百分比 */

/* 传感器权重：4路等距布局，AO1最右=+1.5，AO4最左=-1.5 */
static const float s_weights[BSP_TRACE_CH_COUNT] = {
    +1.5f, +0.5f, -0.5f, -1.5f
};

/* ===================== 静态变量 ===================== */
static u16  s_vals[BSP_TRACE_CH_COUNT];
static u8   s_black_state[BSP_TRACE_CH_COUNT];  /* 施密特触发器输出：1=黑线，0=白底 */
static float s_error = 0.0f;
static float s_last_error = 0.0f;
static float s_last_last_error = 0.0f;  /* 上上次误差，用于二阶导数（加速度） */
static int  s_left_pct = 0;
static int  s_right_pct = 0;
static u8   s_lost_cnt = 0;            /* 连续丢线计数 */
static u8   s_lost_recovery_cnt = 0;   /* 寻线耗时计数 */
static u8   s_is_lost = 0;             /* 丢线标志 */
static u8   s_straight_coast_cnt = 0;  /* 直道丢线后继续直行计数 */
static u8   s_all_black_coast_cnt = 0; /* 全黑时维持之前状态计数 */
static float s_current_speed = 65.0f;  /* 当前实际速度（平滑后），初始化为NORMAL */

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

        /* 全部检测到黑线：短暂维持之前运动状态，
         * 避免十字路口或宽线处突然直行导致偏离。 */
        if (cnt == 4.0f)
        {
            if (s_all_black_coast_cnt < ALL_BLACK_COAST_TICKS)
            {
                s_all_black_coast_cnt++;
                return s_last_error;
            }
        }
        else
        {
            s_all_black_coast_cnt = 0;
        }

        /* 中间两个通道（AO2、AO3）任意一个检测到黑线且两侧都没有时，
         * 认为小车居中在线上，按直行处理，抑制小幅抖动。 */
        if ((s_black_state[0] == 0) && (s_black_state[3] == 0) &&
            ((s_black_state[1] == 1) || (s_black_state[2] == 1)))
        {
            return 0.0f;
        }

        return sum / cnt;
    }

    /* 全部未检测到黑线：清除全黑计数，若之前处于直道（误差较小），允许短暂继续直行，
     * 不立即计为丢线，提高对黑线断续或反光间隙的容忍度。 */
    s_all_black_coast_cnt = 0;
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

/* 分段P增益选择：根据误差大小动态调整P增益
 * 解决"转不过来"(小误差响应不足)和"转过头"(大误差超调)的矛盾 */
static float trace_get_adaptive_kp(float error)
{
    float abs_err = (error >= 0.0f) ? error : -error;

    if (abs_err < 0.5f)
    {
        /* 小误差区：高灵敏度，快速纠偏 */
        return LINE_KP_LOW;
    }
    else if (abs_err < 1.0f)
    {
        /* 中误差区：标准增益 */
        return LINEKP_MID;
    }
    else
    {
        /* 大误差区：低增益，防止急弯超调 */
        return LINE_KP_HIGH;
    }
}

/* 弯道出口制动计算：检测error是否正在快速减小（离开弯道）
 * 如果是，施加额外反向转向量抵消惯性 */
static float trace_calc_exit_brake(float error, float last_error, float last_last_error)
{
    float abs_err = (error >= 0.0f) ? error : -error;
    float error_vel = error - last_error;      /* 一阶导数（速度） */
    float error_acc = error_vel - (last_error - last_last_error); /* 二阶导数（加速度） */

    /* 只在以下条件同时满足时触发制动：
     * 1. 当前误差已进入中等范围（接近切线位置）
     * 2. error正在减小（向零靠近）
     * 3. 减小速度较快（明显在回正）
     * 4. 上次误差较大（确实是从弯道出来） */
    float abs_last_err = (last_error >= 0.0f) ? last_error : -last_error;

    if ((abs_err < EXIT_BRAKE_ZONE) &&
        (abs_err > 0.1f) &&              /* 不能太接近0，否则过度敏感 */
        (error * last_error > 0.0f) &&   /* 同号，都在同一侧 */
        (error_vel * error < 0.0f) &&    /* error与变化率异号：正在减小 */
        (abs_last_err > EXIT_BRAKE_ZONE)) /* 确实从大误差回来 */
    {
        /* 制动量 = 系数 × 减小速率
         * error_vel为负值（减小），乘以系数后得到正值或负值的制动量
         * 方向与error相反，帮助提前回正 */
        float brake = EXIT_BRAKE_COEF * error_vel;

        /* 限制最大制动力，防止过度矫正 */
        const float max_brake = 8.0f;
        if (brake > max_brake) brake = max_brake;
        if (brake < -max_brake) brake = -max_brake;

        return brake;
    }

    return 0.0f;
}

/* 平滑速度过渡：使用一阶低通滤波实现渐进式速度切换
 * 消除阶跃突变，让电机加速度连续平滑 */
static int trace_smooth_speed(int target_speed)
{
    /* 低通滤波公式：current = current + rate × (target - current)
     * 每周期向目标靠近 RATE 的比例 */
    s_current_speed += SPEED_TRANSITION_RATE * ((float)target_speed - s_current_speed);

    /* 四舍五入到整数 */
    return (int)(s_current_speed + 0.5f);
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
    s_last_last_error = 0.0f;
    s_left_pct = 0;
    s_right_pct = 0;
    s_lost_cnt = 0;
    s_lost_recovery_cnt = 0;
    s_straight_coast_cnt = 0;
    s_is_lost = 0;
    s_current_speed = (float)BASE_SPEED_NORMAL;  /* 初始化为直道速度 */
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
        /* 确定目标速度（尚未应用，先用于计算） */
        int target_base_speed;
        if (error > CURVE_ERROR_THR || error < -CURVE_ERROR_THR)
        {
            target_base_speed = BASE_SPEED_CURVE;
        }
        else
        {
            target_base_speed = BASE_SPEED_NORMAL;
        }

        /* 平滑速度过渡：消除阶跃突变 */
        base_speed = trace_smooth_speed(target_base_speed);

        /* 分段P控制：根据误差大小选择不同增益 */
        float adaptive_kp = trace_get_adaptive_kp(error);

        /* PD输出（使用自适应KP） */
        diff = adaptive_kp * error + LINE_KD * (error - s_last_error);

        /* 弯道出口制动：防止圆弧切线位置出线 */
        float exit_brake = trace_calc_exit_brake(error, s_last_error, s_last_last_error);
        diff += exit_brake;

        /* 更新误差历史（用于下一次的D项和制动计算） */
        s_last_last_error = s_last_error;
        s_last_error = error;

        /* 限制最大转向量 */
        if (diff > LINE_MAX_DIFF)
        {
            diff = LINE_MAX_DIFF;
        }
        else if (diff < -LINE_MAX_DIFF)
        {
            diff = -LINE_MAX_DIFF;
        }
    }

    s_left_pct  = (int)((long)(base_speed + (int)diff) * MOTOR_LEFT_BALANCE / 100);
    s_right_pct = (int)((long)(base_speed - (int)diff) * MOTOR_RIGHT_BALANCE / 100);
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
