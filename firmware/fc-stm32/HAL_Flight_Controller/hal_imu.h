/**
 * @file imu.h
 * @author Dominik Michalczyk
 * 
 * IMU module platform independent interface.
 */

#ifndef IMU_H_
#define IMU_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <stdbool.h>

#define HAL_IMU_INTERFACE_I2C

typedef void (*HAL_IMU_conversion_complete_callback_t)(const float* acc, const float* gyro);

void HAL_IMU_init(HAL_IMU_conversion_complete_callback_t imu_readout_callback);
void HAL_IMU_deinit();
void HAL_IMU_proc();
void HAL_IMU_calibrate();
void HAL_IMU_start_conversion();
void HAL_IMU_request_readout();

#ifdef __cplusplus
}
#endif

#endif // IMU_H_
