#include "mic.h"
#include "../../APP/app.h"

/* ===================== 可调参数 ===================== */
/* 消抖阈值：连续 MIC_DEBOUNCE_CNT 次 (10ms/次) 高电平认为有效声控信号。
 * 值越大抗干扰越强，但响应越慢。建议 2~5。 */
#define MIC_DEBOUNCE_CNT        3       /* 3 * 10ms = 30ms 消抖 */

/* 连续拍手识别时间窗口：两次有效信号间隔不超过此值视为连续拍手。
 * 设为 0 禁用连续拍手检测，每次有效信号都触发。 */
#define MIC_DOUBLE_TAP_MS       800     /* 800ms 内两次拍手 */

/* ===================== 内部状态 ===================== */
typedef enum {
    MIC_ST_IDLE,            /* 空闲，等待第一次有效信号 */
    MIC_ST_DEBOUNCING,      /* 检测到信号，消抖中 */
    MIC_ST_CONFIRMED,       /* 已确认一次有效信号 */
    MIC_ST_WAIT_SECOND      /* 等待第二次拍手（如果启用双击模式） */
} mic_state_t;

static mic_state_t s_state = MIC_ST_IDLE;
static u8           s_debounce_cnt = 0;
static u16          s_last_trigger_tick = 0;   /* 上次触发的系统 tick */
static u8           s_triggered = 0;            /* 本次扫描是否触发（供外部读取） */
static u8           s_enabled = 0;             /* 声控功能使能标志，默认关闭 */

/* ===================== 接口实现 ===================== */

void mic_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(BSP_MIC_CLK, ENABLE);

    /* PB7 配置为下拉输入。
     * LM393 比较器：无声音时输出高（开漏上拉），有声音时输出低（拉地）。
     * 若实际极性相反，改为 GPIO_Mode_IPU 并将 mic_scan() 中 if(raw) 改为 if(!raw)。 */
    GPIO_InitStructure.GPIO_Pin  = BSP_MIC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(BSP_MIC_PORT, &GPIO_InitStructure);

    s_state = MIC_ST_IDLE;
    s_debounce_cnt = 0;
    s_last_trigger_tick = 0;
    s_triggered = 0;
}

u8 mic_get_raw(void)
{
    return MIC_DET;
}

/* 状态机扫描函数，由 app_update() 每 10ms 调用一次 */
u8 mic_scan(void)
{
    u8 raw = MIC_DET;
    u32 now;
    u16 elapsed;

    s_triggered = 0;

    switch (s_state)
    {
        case MIC_ST_IDLE:
            if (!raw)  /* 低电平有效：有声音时 LM393 输出拉低 */
            {
                s_state = MIC_ST_DEBOUNCING;
                s_debounce_cnt = 1;
            }
            break;

        case MIC_ST_DEBOUNCING:
            if (!raw)  /* 低电平持续，继续消抖计数 */
            {
                s_debounce_cnt++;
                if (s_debounce_cnt >= MIC_DEBOUNCE_CNT)
                {
                    /* 消抖完成，确认第一次有效信号 */
#if MIC_DOUBLE_TAP_MS > 0
                    s_state = MIC_ST_WAIT_SECOND;
#else
                    /* 单击模式：立即触发 */
                    s_state = MIC_ST_IDLE;
                    s_triggered = 1;
#endif
                    s_last_trigger_tick = app_get_tick();
                }
            }
            else
            {
                /* 信号消失，回退到空闲 */
                s_state = MIC_ST_IDLE;
                s_debounce_cnt = 0;
            }
            break;

        case MIC_ST_WAIT_SECOND:
            /* 计算距上次触发的时长 */
            now = app_get_tick();
            if (now >= s_last_trigger_tick)
            {
                elapsed = (u16)(now - s_last_trigger_tick);
            }
            else
            {
                /* tick 溢出处理（32 位溢出约 49.7 天，几乎不会发生） */
                elapsed = 0xFFFF - (u16)(s_last_trigger_tick - now);
            }

            if (elapsed > MIC_DOUBLE_TAP_MS)
            {
                /* 超时未收到第二次拍手，回到空闲 */
                s_state = MIC_ST_IDLE;
                s_debounce_cnt = 0;
            }
            else if (!raw && s_debounce_cnt == 0)
            {
                /* 在时间窗口内检测到新的信号（低电平），开始第二次消抖 */
                s_state = MIC_ST_DEBOUNCING;
                s_debounce_cnt = 1;
            }
            else if (raw)
            {
                /* 等待期间无信号（高电平），保持等待 */
                s_debounce_cnt = 0;
            }
            break;

        default:
            s_state = MIC_ST_IDLE;
            break;
    }

    return s_triggered;
}

void mic_set_enabled(u8 enabled)
{
    s_enabled = enabled ? 1 : 0;
    if (!s_enabled)
    {
        /* 禁用时重置状态机，避免残留状态 */
        s_state = MIC_ST_IDLE;
        s_debounce_cnt = 0;
        s_triggered = 0;
    }
}

u8 mic_is_enabled(void)
{
    return s_enabled;
}
