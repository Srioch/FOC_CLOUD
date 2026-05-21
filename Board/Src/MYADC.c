#include "MYADC.h"
#include "math.h"

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

static float adc_clamp_offset_alpha(float alpha)
{
    if (alpha <= 0.0f)
    {
        return ADC_FILTER_DEFAULT_OFFSET_ALPHA;
    }

    if (alpha > 1.0f)
    {
        return 1.0f;
    }

    return alpha;
}

static uint16_t adc_clamp_raw_margin(uint16_t raw)
{
    if (raw >= ADC_RESOLUTION)
    {
        return (uint16_t)(ADC_RESOLUTION - 1U);
    }

    return raw;
}

static float adc_voltage_to_raw(float voltageV)
{
    float raw = (voltageV * 1000.0f * (float)ADC_RESOLUTION) / (float)ADC_VREF_MV;

    if (raw < 0.0f)
    {
        return 0.0f;
    }

    if (raw > (float)(ADC_RESOLUTION - 1U))
    {
        return (float)(ADC_RESOLUTION - 1U);
    }

    return raw;
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
    filter->offset_raw_f = 0.0f;
    filter->filtered_raw = 0.0f;
    filter->offset_alpha = ADC_FILTER_DEFAULT_OFFSET_ALPHA;
    filter->output_gain = 1.0f;
    filter->track_band_raw = ADC_FILTER_DEFAULT_TRACK_BAND_RAW;
    filter->drift_limit_raw = ADC_FILTER_DEFAULT_DRIFT_LIMIT_RAW;
    filter->saturation_margin_raw = ADC_FILTER_DEFAULT_SATURATION_MARGIN_RAW;
    filter->initialized = 0U;
    filter->track_enable = 1U;
    filter->valid = 0U;
}

void ADC_ChannelFilter_SetTracking(ADC_ChannelFilter_t *filter,
                                   float offsetAlpha,
                                   uint16_t trackBandRaw,
                                   uint16_t driftLimitRaw,
                                   uint16_t saturationMarginRaw)
{
    if (filter == NULL)
    {
        return;
    }

    filter->offset_alpha = adc_clamp_offset_alpha(offsetAlpha);
    filter->track_band_raw = adc_clamp_raw_margin(trackBandRaw);
    filter->drift_limit_raw = adc_clamp_raw_margin(driftLimitRaw);
    filter->saturation_margin_raw = adc_clamp_raw_margin(saturationMarginRaw);
}

void ADC_ChannelFilter_SetOutputGain(ADC_ChannelFilter_t *filter, float outputGain)
{
    if (filter == NULL)
    {
        return;
    }

    if (outputGain <= 0.0f)
    {
        filter->output_gain = 1.0f;
        return;
    }

    filter->output_gain = outputGain;
}

void ADC_ChannelFilter_SeedOffsetVoltage(ADC_ChannelFilter_t *filter, float offsetVoltageV)
{
    float offsetRaw;

    if (filter == NULL)
    {
        return;
    }

    offsetRaw = adc_voltage_to_raw(offsetVoltageV);
    filter->offset_raw_f = offsetRaw;
    filter->offset_raw = (uint16_t)(offsetRaw + 0.5f);

    if (filter->initialized == 0U)
    {
        filter->filtered_raw = offsetRaw;
    }
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
    filter->offset_raw_f = (float)filter->offset_raw;
    filter->filtered_raw = (float)filter->offset_raw;
    filter->initialized = 1U;
    filter->valid = 1U;

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
    float offsetVoltage;
    float correctedVoltage;
    float offsetError;
    float previousOffsetRaw;
    uint8_t valid = 1U;

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
    offsetError = filteredRaw - filter->offset_raw_f;
    previousOffsetRaw = filter->offset_raw_f;

    if ((raw <= filter->saturation_margin_raw) ||
        (raw >= (uint16_t)((ADC_RESOLUTION - 1U) - filter->saturation_margin_raw)))
    {
        valid = 0U;
    }

    if ((filter->track_enable != 0U) &&
        (fabsf(offsetError) <= (float)filter->track_band_raw))
    {
        filter->offset_raw_f += filter->offset_alpha * offsetError;
    }

    if (fabsf(filter->offset_raw_f - previousOffsetRaw) > (float)filter->drift_limit_raw)
    {
        valid = 0U;
    }

    filter->offset_raw = (uint16_t)(filter->offset_raw_f + 0.5f);
    offsetVoltage = ADC_ConvertToVoltage_V((uint16_t)(filter->offset_raw_f + 0.5f));
    correctedVoltage = (filter->offset_raw_f - filteredRaw) *
                       (((float)ADC_VREF_MV / (float)ADC_RESOLUTION) / 1000.0f) *
                       filter->output_gain;

    filter->valid = valid;

    sample->raw = raw;
    sample->trimmed_raw = trimmedRaw;
    sample->filtered_raw = filteredRaw;
    sample->filtered_voltage_v = filteredVoltage;
    sample->offset_voltage_v = offsetVoltage;
    sample->corrected_voltage_v = correctedVoltage;
    sample->valid = valid;
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

HAL_StatusTypeDef PhaseVoltageSampler_Update(ADC_ChannelFilter_t *uaFilter,
                                             ADC_FilteredSample_t *uaSample,
                                             ADC_ChannelFilter_t *ubFilter,
                                             ADC_FilteredSample_t *ubSample,
                                             PhaseVoltageSample_t *phaseSample)
{
    if ((uaFilter == NULL) || (uaSample == NULL) ||
        (ubFilter == NULL) || (ubSample == NULL) ||
        (phaseSample == NULL))
    {
        return HAL_ERROR;
    }

    if (ADC_ChannelFilter_Read(uaFilter, uaSample) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (ADC_ChannelFilter_Read(ubFilter, ubSample) != HAL_OK)
    {
        return HAL_ERROR;
    }

    phaseSample->ua_v = uaSample->corrected_voltage_v;
    phaseSample->ub_v = ubSample->corrected_voltage_v;
    phaseSample->uc_v = -(phaseSample->ua_v + phaseSample->ub_v);
    phaseSample->ua_offset_v = uaSample->offset_voltage_v;
    phaseSample->ub_offset_v = ubSample->offset_voltage_v;
    phaseSample->valid = (uint8_t)((uaSample->valid != 0U) && (ubSample->valid != 0U));

    return HAL_OK;
}
