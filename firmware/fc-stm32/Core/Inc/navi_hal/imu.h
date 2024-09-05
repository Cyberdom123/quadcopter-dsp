/**
 * @file imu.h
 * @author Dominik Michalczyk
 * 
 * IMU module platform independent interface.
 */

#if !defined(IMU_H_)
#define IMU_H_

#include <stdbool.h>

typedef void (*IMU_conversion_complete_callback_t)(const float* acc, const float* gyro);

void IMU_init(IMU_conversion_complete_callback_t* imu_readout_callback);
void IMU_start_conversion();
void IMU_deinit();
void IMU_calibrate();

#endif // IMU_H_
