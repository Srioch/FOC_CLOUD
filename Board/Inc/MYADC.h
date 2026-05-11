#ifndef __MYADC_H__
#define __MYADC_H__
#include "main.h"

/* 参考电压 (mV) */
#define ADC_VREF_MV         3300

/* ADC分辨率 (12位 = 4096) */
#define ADC_RESOLUTION      4096

/**
 * @brief  单次读取ADC值
 * @param  hadc: ADC句柄指针
 * @retval ADC原始值 (0-4095)
 */
uint16_t ADC_Read(ADC_HandleTypeDef *hadc);

/**
 * @brief  多次采样取平均值（滤波）
 * @param  hadc: ADC句柄指针
 * @param  samples: 采样次数
 * @retval ADC平均值 (0-4095)
 */
uint16_t ADC_ReadAverage(ADC_HandleTypeDef *hadc, uint8_t samples);

/**
 * @brief  读取ADC并转换为电压值
 * @param  hadc: ADC句柄指针
 * @retval 电压值 (mV)
 */
uint16_t ADC_ReadVoltage_mV(ADC_HandleTypeDef *hadc);

/**
 * @brief  多次采样并转换为电压值（滤波）
 * @param  hadc: ADC句柄指针
 * @param  samples: 采样次数
 * @retval 电压值 (mV)
 */
uint16_t ADC_ReadVoltageAverage_mV(ADC_HandleTypeDef *hadc, uint8_t samples);

/**
 * @brief  ADC原始值转换为电压 (mV)
 * @param  adcValue: ADC原始值
 * @retval 电压值 (mV)
 */
uint16_t ADC_ConvertToVoltage_mV(uint16_t adcValue);

/**
 * @brief  ADC原始值转换为电压 (浮点，单位V)
 * @param  adcValue: ADC原始值
 * @retval 电压值 (V)
 */
float ADC_ConvertToVoltage_V(uint16_t adcValue);

/**
 * @brief  读取指定通道的ADC值
 * @param  hadc: ADC句柄指针
 * @param  channel: ADC通道 (ADC_CHANNEL_0 ~ ADC_CHANNEL_x)
 * @param  samplingTime: 采样时间
 * @retval ADC原始值 (0-4095)
 */
uint16_t ADC_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t samplingTime);

float ADC_FOC_Voltage_Get(uint16_t adcValue);

#endif