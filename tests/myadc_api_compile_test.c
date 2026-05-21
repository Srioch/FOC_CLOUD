#include "MYADC.h"

int main(void)
{
    ADC_ChannelFilter_t channel = {0};
    ADC_FilteredSample_t sample = {0};
    uint16_t window[6] = {1000U, 1002U, 998U, 4000U, 1001U, 999U};

    ADC_ChannelFilter_Init(&channel, 0, ADC_CHANNEL_4, ADC_SAMPLETIME_144CYCLES, 6U, 0.1f);
    channel.offset_raw = ADC_FilterComputeTrimmedAverage(window, 6U);
    ADC_ChannelFilter_Process(&channel, window[0], channel.offset_raw, &sample);

    return (sample.filtered_raw >= 0.0f) ? 0 : 1;
}
