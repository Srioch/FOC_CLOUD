#include "MYADC.h"
#include "math.h"

int main(void)
{
    ADC_ChannelFilter_t channel = {0};
    ADC_FilteredSample_t sample = {0};
    ADC_ChannelFilter_t channel_b = {0};
    ADC_FilteredSample_t sample_b = {0};
    PhaseVoltageSample_t phase = {0};
    uint16_t window[6] = {1000U, 1002U, 998U, 4000U, 1001U, 999U};

    ADC_ChannelFilter_Init(&channel, 0, ADC_CHANNEL_4, ADC_SAMPLETIME_144CYCLES, 6U, 0.1f);
    ADC_ChannelFilter_SetTracking(&channel, 0.002f, 32U, 120U, 8U);
    ADC_ChannelFilter_SetOutputGain(&channel, 10.0f);
    ADC_ChannelFilter_SeedOffsetVoltage(&channel, 1.67f);
    channel.initialized = 1U;
    ADC_ChannelFilter_Process(&channel, window[0], channel.offset_raw, &sample);

    ADC_ChannelFilter_Init(&channel_b, 0, ADC_CHANNEL_5, ADC_SAMPLETIME_144CYCLES, 6U, 0.1f);
    ADC_ChannelFilter_SetTracking(&channel_b, 0.002f, 32U, 120U, 8U);
    ADC_ChannelFilter_SetOutputGain(&channel_b, 10.0f);
    ADC_ChannelFilter_SeedOffsetVoltage(&channel_b, 1.67f);
    channel_b.initialized = 1U;
    ADC_ChannelFilter_Process(&channel_b, window[1], channel_b.offset_raw, &sample_b);

    phase.ua_v = sample.corrected_voltage_v;
    phase.ub_v = sample_b.corrected_voltage_v;
    phase.uc_v = -(phase.ua_v + phase.ub_v);
    phase.ua_offset_v = sample.offset_voltage_v;
    phase.ub_offset_v = sample_b.offset_voltage_v;
    phase.valid = (uint8_t)((sample.valid != 0U) && (sample_b.valid != 0U));

    return (sample.filtered_raw >= 0.0f) &&
           (phase.ua_offset_v >= 0.0f) &&
           (sample.corrected_voltage_v < 10.0f) &&
           (fabsf(phase.ua_v + phase.ub_v + phase.uc_v) < 1.0e-6f) ? 0 : 1;
}
