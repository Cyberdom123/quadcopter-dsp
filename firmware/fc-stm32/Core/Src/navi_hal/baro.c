/**
 * @file baro.c
 * @author Dominik Michalczyk
 * 
 * BARO module platform specific implementation. 
 */

#include <assert.h>

#include "navi_hal/baro.h"
#include "drivers/bmp280.h"
#include "main.h"

typedef struct BARO_DEVICE
{
    BMP280_STRUCT bmp280;
    bool initialized;
};

static struct BARO_DEVICE baro;

void BARO_init(BARO_conversion_callback_t* baro_conversion_callback) {
    assert(baro_conversion_callback != NULL);

    baro.bmp280.hi2c = &hi2c1;
    BMP280_CONFIG bmp280_config = bmp280_get_default_config();

    bmp280_init(&baro.bmp280, bmp280_config);
    baro.initialized = true;
}

void BARO_deinit() {
    baro.initialized = false;
}