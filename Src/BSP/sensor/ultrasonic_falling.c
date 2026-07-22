#include "ultrasonic_falling.h"
#include "delay.h"
#include <stddef.h>

/* Timing constants (microseconds) */
#define US_TX_US     500   /* 40kHz TX burst duration */
#define US_BLIND_US  250   /* blind zone (~4cm), skip transducer ringing */
#define US_TO_US     3500  /* timeout (~65cm range) */

/* Internal state */
static volatile u16 us_echo_us  = 0;   /* echo arrival time (us from blind end) */
static volatile u8  us_ok       = 0;   /* capture complete flag */
static volatile u8  us_listen   = 0;   /* 0=blind/ignore, 1=listening */

/* [兼容] Keil MDK / GCC 的弱符号定义 */
#ifndef __weak
    #ifdef __GNUC__
        #define __weak  __attribute__((weak))
    #else
        #define __weak
    #endif
#endif

/* 默认空实现；应用层可重定义以快速响应测距结果（用于紧急刹车）*/
__weak void ultrasonic_falling_distance_ready_callback(float distance)
{
    (void)distance;
}

/*
 * 初始化超声波模块 - 下降沿捕获版本
 *
 * 硬件配置说明：
 * PA8 (TX): TIM1_CH1 复用推挽输出，产生40kHz PWM载波
 * PB6 (RX/ECHO): TIM4_CH1 浮空输入 + 输入捕获(下降沿触发)
 *
 * 关键修改点（对比原版）：
 * 1. GPIO_Mode_IPD → GPIO_Mode_IN_FLOATING
 *    原因：避免内部下拉与外部上拉(R23)冲突，让模块完全控制电平
 *
 * 2. TIM_ICPolarity_Rising → TIM_ICPolarity_Falling
 *    原因：LM393开漏输出在收到回波时从HIGH→LOW，
 *         需要捕获这个下降沿来测量飞行时间
 */
void ultrasonic_falling_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /*--- TX: TIM1_CH1, PA8, 40kHz PWM ---*/
    RCC_APB2PeriphClockCmd(BSP_US_TX_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BSP_US_TX_TIM_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BSP_US_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BSP_US_TX_PORT, &GPIO_InitStructure);

    TIM_InternalClockConfig(BSP_US_TX_TIM);
    TIM_TimeBaseStructure.TIM_Period = 1800 - 1;         /* 72M/1800=40kHz */
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(BSP_US_TX_TIM, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 900;                 /* 50% duty cycle */
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(BSP_US_TX_TIM, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(BSP_US_TX_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(BSP_US_TX_TIM, ENABLE);
    TIM_CtrlPWMOutputs(BSP_US_TX_TIM, ENABLE);

    TIM_Cmd(BSP_US_TX_TIM, DISABLE);   /* TX off initially */

    /*--- RX: TIM4_CH1, PB6, input capture @ 1us resolution ---*/
    RCC_APB2PeriphClockCmd(BSP_US_ECHO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(BSP_US_ECHO_TIM_CLK, ENABLE);

    /*
     * [关键修改] GPIO模式改为浮空输入
     *
     * 原版代码: GPIO_Mode_IPD (内部下拉 ~40KΩ)
     * 本版代码: GPIO_Mode_IN_FLOATING (浮空)
     *
     * 原因分析：
     * 硬件电路中LM393开漏输出接有R23=10KΩ上拉到3.3V
     * 如果同时启用内部下拉(~40KΩ)，会形成分压：
     *   V_PB6 = 3.3V × 40K / (10K + 40K) = 2.64V
     * 虽然仍可识别为HIGH，但：
     *   - 浪费静态电流 (66μA)
     *   - 降低噪声容限
     *   - 不符合标准设计规范
     *
     * 使用浮空输入让外部上拉完全控制电平，更加可靠
     */
    GPIO_InitStructure.GPIO_Pin = BSP_US_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  /* [修改] 浮空输入 */
    GPIO_Init(BSP_US_ECHO_PORT, &GPIO_InitStructure);

    TIM_InternalClockConfig(BSP_US_ECHO_TIM);
    TIM_TimeBaseStructure.TIM_Period = 5000 - 1;         /* 5ms = ~86cm max */
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;        /* 1us tick */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(BSP_US_ECHO_TIM, &TIM_TimeBaseStructure);

    /*
     * [关键修改] 捕获极性改为下降沿
     *
     * 原版代码: TIM_ICPolarity_Rising (上升沿)
     * 本版代码: TIM_ICPolarity_Falling (下降沿)
     *
     * 时序原理（基于LM393开漏输出）：
     *
     * 时间轴:
     * t=0      t=500us   t=750us      t=T_echo
     *  |        |         |            |
     * TRIG: ___|‾‾‾‾|_____________________
     *
     * ECHO: _________|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|_________
     *              ↑                           ↑
     *           发射开始                     收到回波
     *           (变HIGH)                    (变LOW!)
     *                                       ↓
     *                              这是我们需要捕获的时刻！
     *
     * 工作流程：
     * 1. ultrasonic_trig() 发射后等待盲区(250us)
     * 2. 盲区结束时复位TIM4计数器 CNT=0
     * 3. 设置 us_listen=1 开始监听
     * 4. 当Echo从HIGH→LOW时，TIM4硬件自动捕获CNT值到CCR1
     * 5. 中断服务程序读取CCR1 = 飞行时间
     * 6. 距离 = (盲区时间 + 飞行时间) / 58
     */
    TIM_ICInitStructure.TIM_Channel = BSP_US_ECHO_CH;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;  /* [修改] 下降沿 */
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0F;
    TIM_ICInit(BSP_US_ECHO_TIM, &TIM_ICInitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(BSP_US_ECHO_TIM, TIM_IT_CC1, ENABLE);
    TIM_Cmd(BSP_US_ECHO_TIM, ENABLE);
}

/*
 * 发射40kHz超声波脉冲并启动接收
 *
 * 时序流程：
 * 1. 发送500us的40kHz PWM burst (约20个周期)
 * 2. 等待250us盲区 (跳过探头余振)
 * 3. 复位定时器，清除中断标志，开始监听Echo
 *
 * 注意：此函数不阻塞，立即返回。实际的回波捕获在中断中完成。
 */
static void ultrasonic_trig(void)
{
    us_listen = 0;
    us_ok = 0;
    us_echo_us = 0;

    /* 1. TX burst: 500us of 40kHz pulses */
    TIM_SetCounter(BSP_US_TX_TIM, 0);
    TIM_Cmd(BSP_US_TX_TIM, ENABLE);
    delay_us(US_TX_US);
    TIM_Cmd(BSP_US_TX_TIM, DISABLE);

    /* 2. Blind zone: skip transducer ringing & electrical crosstalk */
    delay_us(US_BLIND_US);

    /* 3. Arm capture: reset echo timer, clear pending, start listening */
    TIM_SetCounter(BSP_US_ECHO_TIM, 0);
    TIM_ClearITPendingBit(BSP_US_ECHO_TIM, TIM_IT_CC1);
    us_listen = 1;
}

static float us_last_distance = 0.0f;

/* Non-blocking: trigger the next measurement cycle */
void ultrasonic_falling_start(void)
{
    ultrasonic_trig();
}

/*
 * 非阻塞方式获取最新测距结果
 *
 * 返回值：
 * - 成功: 距离值 (cm), 范围约 4~86cm
 * - 无效: 0.0 (超时或未完成测量)
 *
 * 工作机制：
 * - 每次调用检查是否有新的有效测量数据
 * - 如果有，计算距离并自动启动下一次测量
 * - 如果无，返回上次的有效值或0.0(超时保护)
 *
 * 超时保护：
 * - 连续300次调用(约3秒)无更新则强制重新触发
 * - 防止丢波后一直返回旧数据导致误判
 */
float ultrasonic_falling_get_distance(void)
{
    static u32 s_no_update_cnt = 0;  /* 连续无更新计数器 */

    if (us_ok)
    {
        /* Total flight time = blind zone + captured timer (us)
         * Distance (cm) = flight_time_us / 58
         * (sound travels 1cm in ~29us, round-trip = 58us/cm) */
        us_last_distance = (US_BLIND_US + us_echo_us) / 58.0f;

        /* 通知应用层测距完成（用于紧急刹车响应）*/
        ultrasonic_falling_distance_ready_callback(us_last_distance);

        us_ok = 0;
        s_no_update_cnt = 0;  /* 重置超时计数 */
        ultrasonic_trig();     /* auto-start next measurement */
    }
    else
    {
        /* 超时保护：防止丢波后一直返回旧数据导致误判 */
        s_no_update_cnt++;
        if (s_no_update_cnt > 300)  /* 300次 * 10ms(调用周期) ≈ 3s无更新 */
        {
            us_last_distance = 0.0f;  /* 标记无效距离 */
            s_no_update_cnt = 0;
            ultrasonic_trig();         /* 强制重新触发测量 */
        }
    }
    return us_last_distance;
}

/*
 * 阻塞式同步测量 - 便捷接口
 *
 * 功能：触发一次测量并等待完成，最多等待约3.5ms
 *
 * 适用场景：
 * - 初始化测试
 * - 单次精确测量需求
 * - 调试诊断
 *
 * 注意：此函数会阻塞CPU，不适合在实时控制循环中频繁调用
 *
 * 返回值：
 * - 成功: 距离 (cm)
 * - 超时: 0.0
 */
float ultrasonic_falling_measure_blocking(void)
{
    u32 timeout;

    ultrasonic_trig();

    timeout = US_TO_US;
    while (!us_ok && --timeout) { /* spin */ }
    us_listen = 0;

    if (us_ok)
        return (US_BLIND_US + us_echo_us) / 58.0f;
    else
        return 0.0f;
}

/*
 * 调试接口：获取原始测量数据
 *
 * 参数：
 * echo_us - [输出] 捕获到的回波时间 (μs), 从盲区结束算起
 * ok      - [输出] 是否有有效的未读取测量结果
 * listening - [输出] 当前是否正在监听回波
 *
 * 用途：
 * - 诊断驱动工作状态
 * - 分析信号质量
 * - 性能调优
 */
void ultrasonic_falling_get_raw(u16 *echo_us, u8 *ok, u8 *listening)
{
    __disable_irq();
    if (echo_us != NULL)
    {
        *echo_us = us_echo_us;
    }
    if (ok != NULL)
    {
        *ok = us_ok;
    }
    if (listening != NULL)
    {
        *listening = us_listen;
    }
    __enable_irq();
}

/*
 * TIM4 输入捕获中断服务程序
 *
 * 触发条件：PB6(Echo)引脚检测到**下降沿**
 *
 * 工作原理：
 * 1. 检查CC1中断标志位
 * 2. 如果当前处于监听状态(us_listen==1):
 *    a. 读取捕获寄存器CCR1的值 → 这是Echo从高变低的时刻
 *    b. 该值等于: 盲区结束到收到回波的时间差 (μs)
 *    c. 设置完成标志us_ok=1
 * 3. 清除中断标志位，准备下一次捕获
 *
 * 重要提示：
 * - 此中断与编码器共用 EXTI9_5_IRQn
 * - 在encoder.c中的EXTI9_5_IRQHandler也会处理按键中断
 * - 确保优先级配置正确避免冲突
 */
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(BSP_US_ECHO_TIM, TIM_IT_CC1) == SET)
    {
        if (us_listen)
        {
            /*
             * [关键] 读取捕获值
             *
             * 此时CCR1中保存的是：
             * 从盲区结束后到Echo下降沿之间的计数值
             * 计数精度: 1μs (72MHz / 72分频)
             *
             * 示例：
             * 如果障碍物距离50cm
             * 往返时间 ≈ 2940μs
             * 盲区时间 = 250μs
             * 则CCR1 ≈ 2940 - 250 = 2690
             * 最终距离 = (250 + 2690) / 58 ≈ 50.0cm ✓
             */
            us_echo_us = TIM_GetCapture1(BSP_US_ECHO_TIM);
            us_ok = 1;
        }
        TIM_ClearITPendingBit(BSP_US_ECHO_TIM, TIM_IT_CC1);
    }
}
