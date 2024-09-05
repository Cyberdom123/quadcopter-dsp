/**
 * @file imu.c
 * @author Dominik Michalczyk
 *
 * IMU module platform specific implementation.
 */

#include <assert.h>

#include "navi_hal/imu.h"
#include "mpu6050.h"
#include "main.h"

enum
{
    IMU_GYRO_CALIBRATION_SAMPLES = 1000,
    IMU_ACC_CALIBRATION_SAMPLES = 1000
};

typedef struct IMU_DEVICE
{
    MPU6050_STRUCT mpu;
    bool initialized;
    bool calibrated;
};

static struct IMU_DEVICE imu;

void IMU_init(IMU_conversion_complete_callback_t *imu_readout_callback)
{
    assert(imu_readout_callback != NULL);
    
    imu.mpu.hi2c = &hi2c1;
    MPU6050_config mpu_config = MPU_get_default_cfg();
    imu.initialized = MPU6050_init(&imu.mpu, &mpu_config);
}

void IMU_deinit()
{
}

void IMU_start_conversion()
{
    if (!imu.initialized)
    {
        return;
    }
    MPU_clear_int(&imu.mpu);
}

void IMU_calibrate()
{
    if (!imu.initialized)
    {
        return;
    }
    MPU_measure_gyro_offset(&imu.mpu, IMU_GYRO_CALIBRATION_SAMPLES);
    MPU_measure_acc_offset(&imu.mpu, IMU_ACC_CALIBRATION_SAMPLES);
}
