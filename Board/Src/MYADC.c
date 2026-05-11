#include "MYADC.h"

/**
 * @brief  单次读取ADC值
 */
uint16_t ADC_Read(ADC_HandleTypeDef *hadc)
{
    uint16_t adcValue = 0;
    
    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 100) == HAL_OK)
    {
        adcValue = HAL_ADC_GetValue(hadc);
    }
    HAL_ADC_Stop(hadc);
    
    return adcValue;
}

/**
 * @brief  多次采样取平均值（滤波）
 */
uint16_t ADC_ReadAverage(ADC_HandleTypeDef *hadc, uint8_t samples)
{
    uint32_t sum = 0;
    
    if (samples == 0) samples = 1;
    
    for (uint8_t i = 0; i < samples; i++)
    {
        sum += ADC_Read(hadc);
    }
    
    return (uint16_t)(sum / samples);
}

/**
 * @brief  读取ADC并转换为电压值 (mV)
 */
uint16_t ADC_ReadVoltage_mV(ADC_HandleTypeDef *hadc)
{
    uint16_t adcValue = ADC_Read(hadc);
    return ADC_ConvertToVoltage_mV(adcValue);
}

/**
 * @brief  多次采样并转换为电压值 (mV)
 */
uint16_t ADC_ReadVoltageAverage_mV(ADC_HandleTypeDef *hadc, uint8_t samples)
{
    uint16_t adcValue = ADC_ReadAverage(hadc, samples);
    return ADC_ConvertToVoltage_mV(adcValue);
}

/**
 * @brief  ADC原始值转换为电压 (mV)
 */
uint16_t ADC_ConvertToVoltage_mV(uint16_t adcValue)
{
    return (uint16_t)((uint32_t)adcValue * ADC_VREF_MV / ADC_RESOLUTION);
}

/**
 * @brief  ADC原始值转换为电压 (V)
 */
float ADC_ConvertToVoltage_V(uint16_t adcValue)
{
    return (float)adcValue * ADC_VREF_MV / ADC_RESOLUTION / 1000.0f;
}

/**
 * @brief  读取指定通道的ADC值
 */
uint16_t ADC_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t samplingTime)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = samplingTime;
    
    HAL_ADC_ConfigChannel(hadc, &sConfig);
    
    return ADC_Read(hadc);
}

float ADC_FOC_Voltage_Get(uint16_t adcValue)
{
    return (float)adcValue * 3.3f / 4095.0f * 6.1f;
}