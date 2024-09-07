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

void IMU_init(IMU_conversion_complete_callback_t imu_readout_callback);
void IMU_deinit();
void IMU_proc();
void IMU_calibrate();
void IMU_start_conversion();
void IMU_request_readout();

#endif // IMU_H_
