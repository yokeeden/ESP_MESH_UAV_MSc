/**********************************************************************
* - Description:		ESP_MESH_SENSING_NODE - source file for BME280 configuration and initialization
* - File:				bme280config.c
* - Compiler:			xtensa-esp32
* - Debugger:			USB2USART
* - Author:				Maciej Kowalski
* - Target:				ESP32
* - Created:			2021-04-01
* - Last changed:		2021-07-02
* - Projects used in development:
*   - https://github.com/mohamed-elsabagh/esp32-mpu6050 - ESP32-MPU6050 - Author: Mohamed El-Sabagh;
*	- https://github.com/BoschSensortec/BME280_driver - BOSH BME280 API v3.5.0.
**********************************************************************/

/*******************************************************
 *                Include Header Files
 *******************************************************/
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <stdio.h>
#include "sdkconfig.h"
#include "bme280.h"

/*******************************************************
 *                       Macros
 *******************************************************/
#define SDA_PIN 18
#define SCL_PIN 19

#define TAG_BME280 "BME280"
#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

/*******************************************************
 *          Constants and Variables Definitions
 *******************************************************/
struct bme280_dev dev;
uint8_t dev_addr = BME280_I2C_ADDR_PRIM;

/*******************************************************
 *                Functions Declarations
 *******************************************************/
esp_err_t i2c_master_init(void);
int8_t user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
void user_delay_msek(uint32_t msek);
void bme280_i2c_init(void);


/*******************************************************
 *                Functions Definitions
 *******************************************************/

/**
  * @brief  I2C master initialization
  * @param  None
  * @retval error value
  */
esp_err_t i2c_master_init(void)
{
	int i2c_master_port = I2C_NUM_0;
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = SDA_PIN;
	conf.scl_io_num = SCL_PIN;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 1000000;
	i2c_param_config(i2c_master_port, &conf);
	return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

/**
  * @brief  I2C master write function for BME280
  * @param  None
  * @retval error value
  */
int8_t user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  int8_t rslt = 0;
  uint8_t dev_addr = *(uint8_t*)intf_ptr;
  /*initialize command link */
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  /*start bit*/
  i2c_master_start(cmd);
  /*write one byte - device addres */
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
  /*write one byte - register addres */
  i2c_master_write_byte(cmd, reg_addr, true);
  /*write buffer to I2C */
  i2c_master_write(cmd, reg_data, len, true);
  /*stop bit */
  i2c_master_stop(cmd);
  /*execute commands*/
  rslt = i2c_master_cmd_begin(I2C_NUM_0, cmd, 200/portTICK_PERIOD_MS);

  if (rslt != ESP_OK) {
      return rslt;
  }
  /* free command link*/
  i2c_cmd_link_delete(cmd);
  return rslt;
}

/**
  * @brief  I2C master read function for BME280
  * @param  None
  * @retval error value
  */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  int8_t rslt = 0;

  uint8_t dev_addr = *(uint8_t*)intf_ptr;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  /*start bit */
  i2c_master_start(cmd);
  /*write one byte - device addres */
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
  /*write one byte - register addres */
  i2c_master_write_byte(cmd, reg_addr, true);
  /*start bit */
  i2c_master_start(cmd);
  /*write one byte - read command*/
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
  /*read bytes*/
  if (len > 1)
  	 {
         i2c_master_read(cmd, reg_data, len - 1, I2C_MASTER_ACK);
     }
  i2c_master_read_byte(cmd, reg_data + len - 1, I2C_MASTER_NACK);
  /*stop bit*/
  i2c_master_stop(cmd);
  /*execute commands*/
  rslt = i2c_master_cmd_begin(I2C_NUM_0, cmd, 200/portTICK_PERIOD_MS);
  /*delete command link*/
  i2c_cmd_link_delete(cmd);
  if (rslt != ESP_OK) {
      return rslt;
  }

  return rslt;
}

/**
  * @brief  I2C master delay function for BME280
  * @param  None
  * @retval None
  */
void user_delay_msek(uint32_t msek)
{
	vTaskDelay(msek/portTICK_PERIOD_MS);
}

/**
  * @brief  BME280 initialization function
  * @param  None
  * @retval None
  */
void bme280_i2c_init(void)
{
	int8_t rslt = BME280_OK;

	dev.intf_ptr = &dev_addr;
	dev.intf = BME280_I2C_INTF;
	dev.delay_us = (void*)user_delay_msek;
	dev.write = (void*)user_i2c_write;
	dev.read = (void*)user_i2c_read;
	printf("calling bme280_init\r\n");
	rslt = bme280_init(&dev);
    printf("bme280 init result %s\r\n",  rslt ? "ERROR" : "OK");
}




