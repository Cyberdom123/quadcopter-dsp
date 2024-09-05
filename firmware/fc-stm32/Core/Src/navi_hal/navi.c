#include "navi_hal/radio.h"
#include "navi_hal/imu.h"
#include "navi_hal/baro.h"
#include "navi_hal/external_memory.h"

void radio_packet_received_callback(const uint8_t* packet, uint8_t packet_length) {

}

void imu_conversion_complete_callback(const float* acc, const float* gyro) {

}

void baro_conversion_callback(const float* pressure, const float* temperature) {

}

void NAVI_init()
{
    // Initialize modules
    RADIO_init(radio_packet_received_callback);
    IMU_init(imu_conversion_complete_callback);
    BARO_init(baro_conversion_callback);
}

void NAVI_deinit()
{
    // Deinitialize modules
    RADIO_deinit();
    IMU_deinit();
    BARO_deinit();
}

void NAVI_proc() 
{
    // TODO: process 
}