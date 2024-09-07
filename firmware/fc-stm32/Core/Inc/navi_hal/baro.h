/**
 * @file baro.h
 * @author Dominik Michalczyk
 * 
 * BARO module platform independent interface.
 */

#if !defined(BARO_H_)
#define BARO_H_

#include <stdbool.h>
#include <stdint.h>

typedef void (*BARO_conversion_callback_t)(const float* pressure, const float* temperature);

void BARO_init(BARO_conversion_callback_t baro_conversion_callback);
void BARO_deinit();
void BARO_proc();

#endif // BARO_H
