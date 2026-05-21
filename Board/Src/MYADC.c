#include "MYADC.h"

static uint16_t adc_read_channel_raw(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t samplingTime)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = samplingTime;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        return 0U;
    }

    return ADC_Read(hadc);
}

static uint8_t adc_clamp_window_size(uint8_t windowSize)
{
    if (windowSize < 3U)
    {
        return 3U;
    }

    if (windowSize > ADC_FILTER_MAX_WINDOW)
    {
        return ADC_FILTER_MAX_WINDOW;
    }

    return windowSize;
}

static float adc_clamp_alpha(float alpha)
{
    if (alpha <= 0.0f)
    {
        return ADC_FILTER_DEFAULT_ALPHA;
    }

    if (alpha > 1.0f)
    {
        return 1.0f;
    }

    return alpha;
}

uint16_t ADC_Read(ADC_HandleTypeDef *hadc)
{
    uint16_t adcValue = 0U;

    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 100U) == HAL_OK)
    {
        adcValue = (uint16_t)HAL_ADC_GetValue(hadc);
    }
    HAL_ADC_Stop(hadc);

    return adcValue;
}

uint16_t ADC_ReadAverage(ADC_HandleTypeDef *hadc, uint8_t samples)
{
    uint32_t sum = 0U;
    uint8_t i;

    if (samples == 0U)
    {
        samples = 1U;
    }

    for (i = 0U; i < samples; i++)
    {
        sum += ADC_Read(hadc);
    }

    return (uint16_t)(sum / samples);
}

uint16_t ADC_ReadVoltage_mV(ADC_HandleTypeDef *hadc)
{
    uint16_t adcValue = ADC_Read(hadc);
    return ADC_ConvertToVoltage_mV(adcValue);
}

uint16_t ADC_ReadVoltageAverage_mV(ADC_HandleTypeDef *hadc, uint8_t samples)
{
    uint16_t adcValue = ADC_ReadAverage(hadc, samples);
    return ADC_ConvertToVoltage_mV(adcValue);
}

uint16_t ADC_ConvertToVoltage_mV(uint16_t adcValue)
{
    return (uint16_t)(((uint32_t)adcValue * ADC_VREF_MV) / ADC_RESOLUTION);
}

float ADC_ConvertToVoltage_V(uint16_t adcValue)
{
    return ((float)adcValue * (float)ADC_VREF_MV) / ((float)ADC_RESOLUTION * 1000.0f);
}

uint16_t ADC_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t samplingTime)
{
    return adc_read_channel_raw(hadc, channel, samplingTime);
}

float ADC_FOC_Voltage_Get(uint16_t adcValue)
{
    return ((float)adcValue * 3.3f / 4095.0f) * 6.1f;
}

void ADC_ChannelFilter_Init(ADC_ChannelFilter_t *filter,
                            ADC_HandleTypeDef *hadc,
                            uint32_t channel,
                            uint32_t samplingTime,
                            uint8_t windowSize,
                            float alpha)
{
    if (filter == NULL)
    {
        return;
    }

    filter->hadc = hadc;
    filter->channel = channel;
    filter->sampling_time = samplingTime;
    filter->window_size = adc_clamp_window_size(windowSize);
    filter->alpha = adc_clamp_alpha(alpha);
    filter->offset_raw = 0U;
    filter->filtered_raw = 0.0f;
    filter->initialized = 0U;
}

HAL_StatusTypeDef ADC_ChannelFilter_Calibrate(ADC_ChannelFilter_t *filter, uint16_t sampleCount)
{
    uint32_t sum = 0U;
    uint16_t i;

    if ((filter == NULL) || (filter->hadc == NULL))
    {
        return HAL_ERROR;
    }

    if (sampleCount == 0U)
    {
        sampleCount = ADC_FILTER_DEFAULT_CALIB_SAMPLES;
    }

    for (i = 0U; i < sampleCount; i++)
    {
        sum += adc_read_channel_raw(filter->hadc, filter->channel, filter->sampling_time);
    }

    filter->offset_raw = (uint16_t)(sum / sampleCount);
    filter->filtered_raw = (float)filter->offset_raw;
    filter->initialized = 1U;

    return HAL_OK;
}

HAL_StatusTypeDef ADC_ChannelFilter_Read(ADC_ChannelFilter_t *filter, ADC_FilteredSample_t *sample)
{
    uint16_t window[ADC_FILTER_MAX_WINDOW];
    uint8_t i;
    uint16_t trimmed;
    uint16_t raw;

    if ((filter == NULL) || (sample == NULL) || (filter->hadc == NULL))
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < filter->window_size; i++)
    {
        window[i] = adc_read_channel_raw(filter->hadc, filter->channel, filter->sampling_time);
    }

    raw = window[filter->window_size - 1U];
    trimmed = ADC_FilterComputeTrimmedAverage(window, filter->window_size);
    ADC_ChannelFilter_Process(filter, raw, trimmed, sample);

    return HAL_OK;
}

void ADC_ChannelFilter_Process(ADC_ChannelFilter_t *filter,
                               uint16_t raw,
                               uint16_t trimmedRaw,
                               ADC_FilteredSample_t *sample)
{
    float filteredRaw;
    float filteredVoltage;
    float correctedVoltage;

    if ((filter == NULL) || (sample == NULL))
    {
        return;
    }

    if (filter->initialized == 0U)
    {
        filter->filtered_raw = (float)trimmedRaw;
        filter->initialized = 1U;
    }
    else
    {
        filter->filtered_raw += filter->alpha * ((float)trimmedRaw - filter->filtered_raw);
    }

    filteredRaw = filter->filtered_raw;
    filteredVoltage = ADC_ConvertToVoltage_V((uint16_t)(filteredRaw + 0.5f));

    if (filteredRaw >= (float)filter->offset_raw)
    {
        correctedVoltage = ADC_ConvertToVoltage_V((uint16_t)(filteredRaw - (float)filter->offset_raw + 0.5f));
    }
    else
    {
        correctedVoltage = -ADC_ConvertToVoltage_V((uint16_t)(((float)filter->offset_raw - filteredRaw) + 0.5f));
    }

    sample->raw = raw;
    sample->trimmed_raw = trimmedRaw;
    sample->filtered_raw = filteredRaw;
    sample->filtered_voltage_v = filteredVoltage;
    sample->corrected_voltage_v = correctedVoltage;
}

uint16_t ADC_FilterComputeTrimmedAverage(const uint16_t *samples, uint8_t count)
{
    uint32_t sum = 0U;
    uint16_t minValue;
    uint16_t maxValue;
    uint8_t i;

    if ((samples == NULL) || (count == 0U))
    {
        return 0U;
    }

    minValue = samples[0];
    maxValue = samples[0];

    for (i = 0U; i < count; i++)
    {
        uint16_t value = samples[i];
        sum += value;

        if (value < minValue)
        {
            minValue = value;
        }

        if (value > maxValue)
        {
            maxValue = value;
        }
    }

    if (count >= 3U)
    {
        sum -= minValue;
        sum -= maxValue;
        return (uint16_t)(sum / (uint32_t)(count - 2U));
    }

    return (uint16_t)(sum / count);
}
