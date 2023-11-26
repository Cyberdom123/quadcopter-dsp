#if !defined(_FILTERS_H_)
#define _FILTERS_H_

#include <inttypes.h>

typedef struct EMA_filter_t {
    float alpha;
    uint8_t buffer_size;
} EMA_filter_t;

void EMA_filter(EMA_filter_t* ema, float* output, float* input);

#endif