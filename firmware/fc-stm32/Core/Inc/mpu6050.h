#if !defined(mpu6050)
#define mpu6050

#include "i2c.h"
#include "gpio.h"
#include "tim.h"

typedef struct
{
    I2C_HandleTypeDef *hi2c;
} MPU6050_STRUCT;

// TODO: make a structure for MPU6050 config for easier configuration

#define AD0 0
#define MPU6050_ADDR	(0b1101000 | (AD0))

/////////////////////////////////////
/////////// REGISTER MAP ////////////
/////////////////////////////////////

// Self test registers
#define SELF_TEST_X 0x0D
#define SELF_TEST_Y 0x0E
#define SELF_TEST_Z 0x0F
#define SELF_TEST_A 0x10

// Sample rate divider
#define SMPLRT_DIV  0x19

#define CONFIG_REG  0x1A

/**
  * @brief
  * Address of gyroscope config register.
  * Bit 7: XG_ST
  * Bit 6: YG_ST
  * Bit 5: ZG_ST
  * Bits 4-3: FS_SEL[1:0]
*/
#define GYRO_CONFIG_REG 0x1B
/**
  * @brief
  * Address of accelerometer config register.
  * Bit 7: XA_ST
  * Bit 6: YA_ST
  * Bit 5: ZA_ST
  * Bits 4-3: AFS_SEL[1:0]
*/
#define ACC_CONFIG_REG 0x1C
#define FIFO_EN        0x23

#define I2C_MST_CTRL   0x24

#define I2C_SLV0_ADDR  0x25
#define I2C_SLV0_REG   0x26
#define I2C_SLV0_CTRL  0x27

#define I2C_SLV1_ADDR  0x28
#define I2C_SLV1_REG   0x29
#define I2C_SLV1_CTRL  0x2a

#define I2C_SLV2_ADDR  0x2b
#define I2C_SLV2_REG   0x2c
#define I2C_SLV2_CTRL  0x2d

#define I2C_SLV3_ADDR  0x2e
#define I2C_SLV3_REG   0x2f
#define I2C_SLV3_CTRL  0x30

#define I2C_SLV4_ADDR  0x31
#define I2C_SLV4_REG   0x32
#define I2C_SLV4_DO    0x33
#define I2C_SLV4_CTRL  0x34
#define I2C_SLV4_DI    0x35

#define I2C_MST_STATUS 0x36
#define INT_PIN_CFG    0x37
#define INT_ENABLE     0x38
#define INT_STATUS     0x3A
/**
  * @brief
  * Address of the first register with accelerometer data.
  * The values are layed out in the order of X, Y, Z, with
  * 16 bits per axix in Big Endian (MSB first).
*/
#define ACC_REG_START 0x3B
#define ACCEL_XOUT_H  0x3B
#define ACCEL_XOUT_L  0x3C
#define ACCEL_YOUT_H  0x3D
#define ACCEL_YOUT_L  0x3E
#define ACCEL_ZOUT_H  0x3F
#define ACCEL_ZOUT_L  0x40

#define TEMP_OUT_H    0x41
#define TEMP_OUT_L    0x42

/**
  * @brief
  * Address of the first register with gyroscope data.
  * The values are layed out in the order of X, Y, Z, with
  * 16 bits per axix in Big Endian (MSB first).
*/
#define GYRO_REG_START 0x43
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4a
#define EXT_SENS_DATA_02 0x4b
#define EXT_SENS_DATA_03 0x4c
#define EXT_SENS_DATA_04 0x4d
#define EXT_SENS_DATA_05 0x4e
#define EXT_SENS_DATA_06 0x4f
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5a
#define EXT_SENS_DATA_18 0x5b
#define EXT_SENS_DATA_19 0x5c
#define EXT_SENS_DATA_20 0x5d
#define EXT_SENS_DATA_21 0x5e
#define EXT_SENS_DATA_22 0x5f
#define EXT_SENS_DATA_23 0x60

#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66

#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define USER_CTRL          0x6A

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

#define FIFO_COUNT 0x72
#define FIFO_R_W   0x74
#define WHO_AM_I   0x75


#define I2C_BYPASS_EN 1
#define I2C_MST_EN 5


// Gyroscope range
#define FS_SEL 0b11
// Accelerometer range
#define AFS_SEL 1

/**
  * @brief sensitivity scale factor in LSB per g
*/
#define ACC_LSB_PER_G (16384.0f / pow(2, AFS_SEL))
#define ACC_SCALE_FACTOR (pow(2, AFS_SEL) / 16384.0)
#define GYRO_SCALE_FACTOR (pow(2, FS_SEL) / 131.0)


HAL_StatusTypeDef mpu6050_read_byte(MPU6050_STRUCT *mpu, uint8_t addr, uint8_t *data);
HAL_StatusTypeDef MPU_init(MPU6050_STRUCT *mpu);
// todo: read function are to return scaled floating point output
HAL_StatusTypeDef MPU_read_acc(MPU6050_STRUCT *mpu, int16_t output[]);
HAL_StatusTypeDef MPU_read_gyro(MPU6050_STRUCT *mpu, int16_t output[]);
// todo: add functions to change the sensor resolution
// todo: add a function for sampling rate


#endif // mpu6050
