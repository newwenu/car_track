#include "key.h"
#include "delay.h"

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

static u8 key_scan(u8 pin_state)
{
    static u8 flag = 1;
    if (flag == 1 && pin_state == 0)
    {
        delay_ms(10);
        if (pin_state == 0)
        {
            flag = 0;
            return 1;
        }
    }
    else if (pin_state == 1)
    {
        flag = 1;
    }
    return 0;
}

u8 key_start_scan(void)
{
    return key_scan(KEY_START);
}

u8 key_ext_scan(void)
{
    return key_scan(KEY_EXT);
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
