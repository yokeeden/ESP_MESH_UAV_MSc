/**********************************************************************
* - Description:		ESP_MESH_BASE_NODE - source file defining I2C fucnctions for initialization, read and write for sensors
* - File:				i2c_driver.c
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
/*******************************************************
 *                Include Header Files
 *******************************************************/
#include "driver/i2c.h"
#include "bme280.h"

/*******************************************************
 *                       Macros
 *******************************************************/
#define I2C_MASTER_NUM 					I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_SCL_IO    			19    		/*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    			18    		/*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ    			1000000     	/*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   	0   		/*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   	0   		/*!< I2C master do not need buffer */
#define WRITE_BIT  						I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   						I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   					0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  					0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    						0x0         /*!< I2C ack value */
#define NACK_VAL   						0x1         /*!< I2C nack value */

/*******************************************************
 *          Constants and Variables Definitions
 *******************************************************/
/*Structure containing BME280 sensor parameters */
struct bme280_dev dev;
/*Variable for storing BME280 I2C address - 0x76 */
uint8_t dev_addr = BME280_I2C_ADDR_PRIM;

/*******************************************************
 *                Functions Declarations
 *******************************************************/
esp_err_t vI2CInit();
esp_err_t sI2cMasterReadSlave(uint8_t* data_rd, size_t size, uint8_t slave_address);
esp_err_t sI2cMasterWriteSlave(uint8_t* data_wr, size_t size, uint8_t slave_address);
int8_t user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
void user_delay_msek(uint32_t msek);
void bme280_i2c_init(void);

/*******************************************************
 *                Functions Definitions
 *******************************************************/

/**
  * @brief  Initializes peripherals used by the I2C driver.
  * @param  None
  * @retval None
  */
esp_err_t vI2CInit()
{
	int i2c_master_port = I2C_MASTER_NUM;
	    i2c_config_t conf;
	    conf.mode = I2C_MODE_MASTER;
	    conf.sda_io_num = I2C_MASTER_SDA_IO;
	    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	    conf.scl_io_num = I2C_MASTER_SCL_IO;
	    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	    i2c_param_config(i2c_master_port, &conf);
	    return i2c_driver_install(i2c_master_port, conf.mode,
	                       I2C_MASTER_RX_BUF_DISABLE,
	                       I2C_MASTER_TX_BUF_DISABLE, 0);
}


/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
esp_err_t sI2cMasterReadSlave(uint8_t* data_rd, size_t size, uint8_t slave_address)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slave_address << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
esp_err_t sI2cMasterWriteSlave(uint8_t* data_wr, size_t size, uint8_t slave_address)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slave_address << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
/**
  * @brief  I2C master write function for BME280 (for BOSH API) mapped on ESP32 structure
  */
int8_t user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  int8_t rslt = 0;
  uint8_t dev_addr = *(uint8_t*)intf_ptr;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_write(cmd, reg_data, len, true);
  i2c_master_stop(cmd);

  rslt = i2c_master_cmd_begin(I2C_NUM_0, cmd, 200/portTICK_PERIOD_MS);
  //  printf("in user_i2c_read, i2c_master_cmd_begin returns %d\r\n", espRc);
  if (rslt != ESP_OK) {
      return rslt;
  }
  i2c_cmd_link_delete(cmd);
  return rslt;
}
/**
  * @brief  I2C master read function for BME280 (for BOSH API) mapped on ESP32 structure
  */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  int8_t rslt = 0;

  uint8_t dev_addr = *(uint8_t*)intf_ptr;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  /*wybór rejestru*/
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_start(cmd);
  /*start czytania*/
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
  if (len > 1)
  	 {
         i2c_master_read(cmd, reg_data, len - 1, I2C_MASTER_ACK);
     }
     i2c_master_read_byte(cmd, reg_data + len - 1, I2C_MASTER_NACK);
     i2c_master_stop(cmd);

  rslt = i2c_master_cmd_begin(I2C_NUM_0, cmd, 200/portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (rslt != ESP_OK) {
      return rslt;
  }

  return rslt;
}
/**
  * @brief  I2C master delay function for BME280 (for BOSH API) mapped on ESP32 structure
  * @param Delay in ms
  * @retval None
  */
void user_delay_msek(uint32_t msek)
{
	vTaskDelay(msek/portTICK_PERIOD_MS);
}
/**
  * @brief  Initializes BME280
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
    printf("bme280 init result %d\r\n", rslt);
}
