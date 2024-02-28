#include <drivers/bmp280.h>

void bmp280_init_default_params(bmp280_dev *dev) {
	dev->conf.mode = BMP280_MODE_NORMAL;
	dev->conf.filter = BMP280_FILTER_OFF;
	dev->conf.oversampling_pressure = BMP280_STANDARD;
	dev->conf.oversampling_temperature = BMP280_STANDARD;
	dev->conf.standby_time = BMP280_STANDBY_250;
}

// esp_err_t i2c_bmp280_write(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t data){
//     int ret;

//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     //Queue command for I2C master    return ret;

//     i2c_master_start(cmd); 
//     i2c_master_write_byte(cmd, BMP280_I2C_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     //Send all queued commands
//     ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);

//     return ret;
// }

HAL_StatusTypeDef i2c_bmp280_write(bmp280_dev *dev, uint8_t reg_addr, uint8_t data){
    return HAL_I2C_Mem_Write(dev->hi2c, BMP280_I2C_ADDR << 1, reg_addr, 1, &data,
                             1, HAL_MAX_DELAY);
}

// esp_err_t i2c_bmp280_write(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t data){
//     int ret;

//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     //Queue command for I2C master
//     i2c_master_start(cmd); 
//     i2c_master_write_byte(cmd, BMP280_I2C_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     //Send all queued commands
//     ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);

//     return ret;
// }

// HAL_StatusTypeDef i2c_bmp280_read(bmp280_dev *dev, uint8_t reg_addr, uint8_t data){
// 	return HAL_I2C_Mem_Read(dev->hi2c, BMP280_I2C_ADDR << 1, reg_addr, 1, &data,
// 							1, HAL_MAX_DELAY);
// }
