#ifndef __MYADC_H__
#define __MYADC_H__

#include "main.h"

#define ADC_VREF_MV 3300U
#define ADC_RESOLUTION 4096U
#define ADC_FILTER_MAX_WINDOW 16U
#define ADC_FILTER_DEFAULT_ALPHA 0.10f
#define ADC_FILTER_DEFAULT_WINDOW 6U
#define ADC_FILTER_DEFAULT_CALIB_SAMPLES 128U
#define ADC_FILTER_DEFAULT_OFFSET_ALPHA 0.002f
#define ADC_FILTER_DEFAULT_TRACK_BAND_RAW 32U
#define ADC_FILTER_DEFAULT_DRIFT_LIMIT_RAW 120U
#define ADC_FILTER_DEFAULT_SATURATION_MARGIN_RAW 8U
#define ADC_PHASE_VOLTAGE_DEFAULT_OFFSET_V 1.67f
#define ADC_PHASE_VOLTAGE_DEFAULT_SCALE 10.0f

typedef struct
{
    uint16_t raw;
    uint16_t trimmed_raw;
    float filtered_raw;
    float filtered_voltage_v;
    float offset_voltage_v;
    float corrected_voltage_v;
    uint8_t valid;
} ADC_FilteredSample_t;

typedef struct
{
    ADC_HandleTypeDef *hadc;
    uint32_t channel;
    uint32_t sampling_time;
    uint8_t window_size;
    float alpha;
    uint16_t offset_raw;
    float offset_raw_f;
    float filtered_raw;
    float offset_alpha;
    float output_gain;
    uint16_t track_band_raw;
    uint16_t drift_limit_raw;
    uint16_t saturation_margin_raw;
    uint8_t initialized;
    uint8_t track_enable;
    uint8_t valid;
} ADC_ChannelFilter_t;

typedef struct
{
    float ua_v;
    float ub_v;
    float uc_v;
    float ua_offset_v;
    float ub_offset_v;
    uint8_t valid;
} PhaseVoltageSample_t;

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
void ADC_ChannelFilter_SetTracking(ADC_ChannelFilter_t *filter,
                                   float offsetAlpha,
                                   uint16_t trackBandRaw,
                                   uint16_t driftLimitRaw,
                                   uint16_t saturationMarginRaw);
void ADC_ChannelFilter_EnableTracking(ADC_ChannelFilter_t *filter, uint8_t enable);
void ADC_ChannelFilter_SetOutputGain(ADC_ChannelFilter_t *filter, float outputGain);
void ADC_ChannelFilter_SeedOffsetVoltage(ADC_ChannelFilter_t *filter, float offsetVoltageV);
HAL_StatusTypeDef ADC_ChannelFilter_Calibrate(ADC_ChannelFilter_t *filter, uint16_t sampleCount);
HAL_StatusTypeDef ADC_ChannelFilter_Read(ADC_ChannelFilter_t *filter, ADC_FilteredSample_t *sample);
void ADC_ChannelFilter_Process(ADC_ChannelFilter_t *filter,
                               uint16_t raw,
                               uint16_t trimmedRaw,
                               ADC_FilteredSample_t *sample);
uint16_t ADC_FilterComputeTrimmedAverage(const uint16_t *samples, uint8_t count);
HAL_StatusTypeDef PhaseVoltageSampler_Update(ADC_ChannelFilter_t *uaFilter,
                                             ADC_FilteredSample_t *uaSample,
                                             ADC_ChannelFilter_t *ubFilter,
                                             ADC_FilteredSample_t *ubSample,
                                             PhaseVoltageSample_t *phaseSample);

#endif
