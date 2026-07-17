#include "trace.h"

static void adc_init_single(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(BSP_TRACE_ADC_CLK, ENABLE);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(BSP_TRACE_ADC, &ADC_InitStructure);

    ADC_Cmd(BSP_TRACE_ADC, ENABLE);

    ADC_ResetCalibration(BSP_TRACE_ADC);
    while (ADC_GetResetCalibrationStatus(BSP_TRACE_ADC));
    ADC_StartCalibration(BSP_TRACE_ADC);
    while (ADC_GetCalibrationStatus(BSP_TRACE_ADC));
}

static u16 adc_get_ch(u8 ch)
{
    ADC_RegularChannelConfig(BSP_TRACE_ADC, ch, 1, ADC_SampleTime_239Cycles5);
    ADC_SoftwareStartConvCmd(BSP_TRACE_ADC, ENABLE);
    while (!ADC_GetFlagStatus(BSP_TRACE_ADC, ADC_FLAG_EOC));
    return ADC_GetConversionValue(BSP_TRACE_ADC);
}

static const u8 trace_ch[BSP_TRACE_CH_COUNT] = {
    BSP_TRACE_AO1_CH,
    BSP_TRACE_AO2_CH,
    BSP_TRACE_AO3_CH,
    BSP_TRACE_AO4_CH,
    BSP_TRACE_AO5_CH,
};

void trace_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(BSP_TRACE_CLK_ALL, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin = BSP_TRACE_AO1_PIN;
    GPIO_Init(BSP_TRACE_AO1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BSP_TRACE_AO2_PIN;
    GPIO_Init(BSP_TRACE_AO2_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BSP_TRACE_AO3_PIN;
    GPIO_Init(BSP_TRACE_AO3_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BSP_TRACE_AO4_PIN;
    GPIO_Init(BSP_TRACE_AO4_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BSP_TRACE_AO5_PIN;
    GPIO_Init(BSP_TRACE_AO5_PORT, &GPIO_InitStructure);

    adc_init_single();
}

void trace_read(u16 *buf)
{
    u8 i;
    for (i = 0; i < BSP_TRACE_CH_COUNT; i++)
    {
        buf[i] = adc_get_ch(trace_ch[i]);
    }
}
