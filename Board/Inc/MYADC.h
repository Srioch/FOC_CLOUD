#ifndef __MYADC_H__
#define __MYADC_H__

#include "main.h"

#define ADC_VREF_MV 3300U
#define ADC_RESOLUTION 4096U
#define ADC_FILTER_MAX_WINDOW 16U
#define ADC_FILTER_DEFAULT_ALPHA 0.10f
#define ADC_FILTER_DEFAULT_WINDOW 6U
#define ADC_FILTER_DEFAULT_CALIB_SAMPLES 128U

typedef struct
{
    uint16_t raw;
    uint16_t trimmed_raw;
    float filtered_raw;
    float filtered_voltage_v;
    float corrected_voltage_v;
} ADC_FilteredSample_t;

typedef struct
{
    ADC_HandleTypeDef *hadc;
    uint32_t channel;
    uint32_t sampling_time;
    uint8_t window_size;
    float alpha;
    uint16_t offset_raw;
    float filtered_raw;
    uint8_t initialized;
} ADC_ChannelFilter_t;

uint16_t ADC_Read(ADC_HandleTypeDef *hadc);
uint16_t ADC_ReadAverage(ADC_HandleTypeDef *hadc, uint8_t samples);
uint16_t ADC_ReadVoltage_mV(ADC_HandleTypeDef *hadc);
uint16_t ADC_ReadVoltageAverage_mV(ADC_HandleTypeDef *hadc, uint8_t samples);
uint16_t ADC_ConvertToVoltage_mV(uint16_t adcValue);
float ADC_ConvertToVoltage_V(uint16_t adcValue);
uint16_t ADC_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t samplingTime);
float ADC_FOC_Voltage_Get(uint16_t adcValue);

void ADC_ChannelFilter_Init(ADC_ChannelFilter_t *filter,
                            ADC_HandleTypeDef *hadc,
                            uint32_t channel,
                            uint32_t samplingTime,
                            uint8_t windowSize,
                            float alpha);
HAL_StatusTypeDef ADC_ChannelFilter_Calibrate(ADC_ChannelFilter_t *filter, uint16_t sampleCount);
HAL_StatusTypeDef ADC_ChannelFilter_Read(ADC_ChannelFilter_t *filter, ADC_FilteredSample_t *sample);
void ADC_ChannelFilter_Process(ADC_ChannelFilter_t *filter,
                               uint16_t raw,
                               uint16_t trimmedRaw,
                               ADC_FilteredSample_t *sample);
uint16_t ADC_FilterComputeTrimmedAverage(const uint16_t *samples, uint8_t count);

#endif
