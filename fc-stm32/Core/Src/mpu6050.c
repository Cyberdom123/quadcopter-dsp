#include "mpu6050.h"

HAL_StatusTypeDef mpu6050_read_byte(MPU6050_STRUCT *mpu, uint8_t addr, uint8_t *data)
{
    return HAL_I2C_Mem_Read(mpu->hi2c, MPU6050_ADDR<<1, addr, 1, data, 1, HAL_MAX_DELAY);
}
