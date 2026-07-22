#include "encoder.h"
#include "key.h"
#include <stddef.h>

static volatile s32 enc_left_pulse = 0;
static volatile s32 enc_right_pulse = 0;
static volatile s32 enc_left_total = 0;   /* 上电以来累计，不清零 */
static volatile s32 enc_right_total = 0;  /* 上电以来累计，不清零 */

void encoder_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(BSP_ENC_CLK_ALL, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_ENC_L_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(BSP_ENC_L_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BSP_ENC_R_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(BSP_ENC_R_PORT, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);

    EXTI_InitStructure.EXTI_Line = BSP_ENC_L_EXTI | BSP_ENC_R_EXTI;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* 注意：EXTI9_5_IRQn 与按键模式键共享，最终优先级由 key_start_irq_init 统一设置 */
    NVIC_InitStructure.NVIC_IRQChannel = BSP_ENC_L_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void encoder_left_inc(void)
{
    enc_left_pulse++;
    enc_left_total++;
}

void encoder_right_inc(void)
{
    enc_right_pulse++;
    enc_right_total++;
}

int encoder_read(void)
{
    int val;

    /* 读数和清零需原子执行，防止中断期间丢脉冲 */
    __disable_irq();
    val = (int)(enc_left_pulse + enc_right_pulse);
    enc_left_pulse = 0;
    enc_right_pulse = 0;
    __enable_irq();

    return val;
}

void encoder_get_counts(s32 *left, s32 *right)
{
    __disable_irq();
    if (left != NULL)
    {
        *left = enc_left_total;
    }
    if (right != NULL)
    {
        *right = enc_right_total;
    }
    __enable_irq();
}

void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(BSP_ENC_L_EXTI) != RESET)
    {
        encoder_left_inc();
        EXTI_ClearITPendingBit(BSP_ENC_L_EXTI);
    }
    if (EXTI_GetITStatus(BSP_ENC_R_EXTI) != RESET)
    {
        encoder_right_inc();
        EXTI_ClearITPendingBit(BSP_ENC_R_EXTI);
    }
    if (EXTI_GetITStatus(EXTI_Line5) != RESET)
    {
        key_start_irq_handler();
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}
