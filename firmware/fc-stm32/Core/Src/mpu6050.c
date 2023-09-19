#include "mpu6050.h"

#define HAL_error_check(status) if(status != HAL_OK) { return status; }

HAL_StatusTypeDef mpu6050_read_byte(MPU6050_STRUCT *mpu, uint8_t addr, uint8_t *data)
{
    return HAL_I2C_Mem_Read(mpu->hi2c, MPU6050_ADDR<<1, addr, 1, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU_init(MPU6050_STRUCT *mpu) {
    // TODO: make this prettier
    // TODO: make a structure for MPU6050 config for easier configuration
    uint8_t i2c_buffer[2];

    int status = HAL_OK;

    *i2c_buffer = 7; // FIXME: Magic value
    status = HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDR << 1, 
                            SMPLRT_DIV_REG, 1,
                            i2c_buffer, 1,
                            HAL_MAX_DELAY);
    HAL_error_check(status);

    *i2c_buffer = 1; // FIXME: Magic value
    status = HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDR << 1, 
                            PWR_MGMT_1_REG, 1,
                            i2c_buffer, 1,
                            HAL_MAX_DELAY);
    HAL_error_check(status);

    *i2c_buffer = 0; // FIXME: Magic value
    status = HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDR << 1, 
                            CONFIG_REG, 1,
                            i2c_buffer, 1,
                            HAL_MAX_DELAY);
    HAL_error_check(status);

    *i2c_buffer = 24; // FIXME: Magic value
    status = HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDR << 1, 
                            GYRO_CONFIG_REG, 1,
                            i2c_buffer, 1,
                            HAL_MAX_DELAY);
    HAL_error_check(status);

    *i2c_buffer = AFS_SEL << 3;
    status = HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDR << 1, 
                            ACC_CONFIG_REG, 1,
                            i2c_buffer, 1,
                            HAL_MAX_DELAY);
    HAL_error_check(status);

    *i2c_buffer = FS_SEL << 3;
    status = HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDR << 1, 
                            GYRO_CONFIG_REG, 1,
                            i2c_buffer, 1,
                            HAL_MAX_DELAY);
    HAL_error_check(status);

    *i2c_buffer = 1; // FIXME: Magic value
    status = HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDR << 1, 
                            INT_ENABLE_REG, 1,
                            i2c_buffer, 1,
                            HAL_MAX_DELAY);
    HAL_error_check(status);

    // Enable I2C pass-through mode
    *i2c_buffer = (1 << I2C_BYPASS_EN); // FIXME: Magic value
    status = HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDR << 1, 
                            INT_PIN_CFG_REG, 1,
                            i2c_buffer, 1,
                            HAL_MAX_DELAY);
    HAL_error_check(status);

    *i2c_buffer = 0;
    status = HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDR << 1, 
                            USER_CTRL_REG, 1,
                            i2c_buffer, 1,
                            HAL_MAX_DELAY);
    HAL_error_check(status);

    return HAL_OK;
}

HAL_StatusTypeDef MPU_read_acc(MPU6050_STRUCT *mpu, int16_t output[]) {
    HAL_StatusTypeDef status;
    uint8_t i2c_buffer[6];

    status = HAL_I2C_Mem_Read(mpu->hi2c, MPU6050_ADDR << 1,
                            ACC_REG_START, 1,
                            i2c_buffer, 6,
                            HAL_MAX_DELAY);
    HAL_error_check(status);

    for(uint8_t i = 0; i < 3; i++) {
        output[i] = (i2c_buffer[2 * i] << 8) | i2c_buffer[2 * i + 1];
    }

    return HAL_OK;
}

HAL_StatusTypeDef MPU_read_gyro(MPU6050_STRUCT *mpu, int16_t output[]) {
    HAL_StatusTypeDef status;
    uint8_t i2c_buffer[6];

    status = HAL_I2C_Mem_Read(mpu->hi2c, MPU6050_ADDR << 1,
                            GYRO_REG_START, 1,
                            i2c_buffer, 6,
                            HAL_MAX_DELAY);
    HAL_error_check(status);

    for(uint8_t i = 0; i < 3; i++) {
        output[i] = (i2c_buffer[2 * i] << 8) | i2c_buffer[2 * i + 1];
    }

    return HAL_OK;
}