/**********************************************************************
* - Description:		ESP_MESH_BASE_NODE - header file for source file i2c_driver.c
* - File:				i2c_driver.h
* - Compiler:			xtensa-esp32
* - Debugger:			USB2USART
* - Author:				Maciej Kowalski
* - Target:				ESP32
* - Created:			2021-04-01
* - Last changed:		2021-07-01
* - Projects used in development:
*   - https://github.com/mohamed-elsabagh/esp32-mpu6050 - ESP32-MPU6050 - Author: Mohamed El-Sabagh;
*	- https://github.com/BoschSensortec/BME280_driver - BOSH BME280 API v3.5.0.
**********************************************************************/
#ifndef _I2C_DRIVER_H
#define _I2C_DRIVER_H
/*******************************************************
 *                Include Header Files
 *******************************************************/
#include <stdint.h>
#include "driver/i2c.h"
/*******************************************************
 *                Functions Declarations
 *******************************************************/
esp_err_t vI2CInit();
esp_err_t sI2cMasterReadSlave(uint8_t* data_rd, size_t size, uint8_t slave_address);
esp_err_t sI2cMasterWriteSlave(uint8_t* data_wr, size_t size, uint8_t slave_address);
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
void user_delay_msek(uint32_t msek);
int8_t user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
void bme280_i2c_init(void);
#endif
