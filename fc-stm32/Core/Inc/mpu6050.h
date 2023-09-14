#if !defined(mpu6050)
#define mpu6050

#include "i2c.h"
#include "gpio.h"
#include "tim.h"

typedef struct
{
    I2C_HandleTypeDef *hi2c;
} MPU6050_STRUCT;


#define MPU6050_ADDR 0x68

HAL_StatusTypeDef mpu6050_read_byte(MPU6050_STRUCT *mpu, uint8_t addr, uint8_t *data);

#endif // mpu6050
