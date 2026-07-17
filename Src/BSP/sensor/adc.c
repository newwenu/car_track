#include "adc.h"

void adc_init(void)
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

u16 adc_get_channel(u8 ch)
{
    ADC_RegularChannelConfig(BSP_TRACE_ADC, ch, 1, ADC_SampleTime_239Cycles5);
    ADC_SoftwareStartConvCmd(BSP_TRACE_ADC, ENABLE);
    while (!ADC_GetFlagStatus(BSP_TRACE_ADC, ADC_FLAG_EOC));
    return ADC_GetConversionValue(BSP_TRACE_ADC);
}
