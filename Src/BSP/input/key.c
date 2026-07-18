#include "key.h"

/* 非阻塞消抖状态：每路按键独立。
 * key_ext_scan() 由 fsm_update() 每 20ms 调用一次，阈值 2 对应 40ms 消抖。
 * key_start_scan() 保留，UI 当前使用中断方式，通常不调用。 */
#define KEY_DEBOUNCE_THRESHOLD  2

typedef struct {
    u8 released;    /* 1: 已释放，允许检测下一次按下 */
    u8 cnt;         /* 连续按下采样计数 */
} key_state_t;

static key_state_t s_start_key = {1, 0};
static key_state_t s_ext_key   = {1, 0};

void key_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(BSP_KEY_START_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BSP_KEY_EXT_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_KEY_START_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(BSP_KEY_START_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BSP_KEY_EXT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(BSP_KEY_EXT_PORT, &GPIO_InitStructure);
}

static u8 key_scan_state(u8 pin_state, key_state_t *ks)
{
    u8 result = 0;

    if (pin_state == 0)  /* 按下：上拉输入，低电平有效 */
    {
        if (ks->released)
        {
            ks->cnt++;
            if (ks->cnt >= KEY_DEBOUNCE_THRESHOLD)
            {
                ks->cnt = 0;
                ks->released = 0;
                result = 1;
            }
        }
    }
    else
    {
        ks->released = 1;
        ks->cnt = 0;
    }

    return result;
}

u8 key_start_scan(void)
{
    return key_scan_state(KEY_START, &s_start_key);
}

u8 key_ext_scan(void)
{
    return key_scan_state(KEY_EXT, &s_ext_key);
}

/* 配置 KEY_START（PA5）为下降沿外部中断，供 APP 层模式键使用 */
void key_start_irq_init(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);

    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  /* 上拉输入，按下为低 */
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
