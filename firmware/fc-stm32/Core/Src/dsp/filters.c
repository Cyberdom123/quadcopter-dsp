#include <dsp/filters.h>

void EMA_filter(EMA_filter_t* ema, float* output, float* input) {
    output[1] = output[0]; // shift old output samples;
    output[0] = ema->alpha * input[0] + (1 - ema->alpha) * output[1];
}