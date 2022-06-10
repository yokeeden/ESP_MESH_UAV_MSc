/**********************************************************************
* - Description:		ESP_MESH_BASE_NODE - header file for source fule i2c_application.c
* - File:				i2c_application.h
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
#ifndef _I2C_APPLICATION_H
#define _I2C_APPLICATION_H
/*******************************************************
 *                Include Header Files
 *******************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
/*******************************************************
 *          Constants and Variables Definitions
 *******************************************************/
/* Structure to manipulate buffer sent or received over I2C */
typedef struct I2C_Structure
{
	uint8_t* pBuffer;
	uint16_t sDataLength;
	uint8_t slaveAddress;
} I2C_Status;
/*******************************************************
 *      EXTERN Constants and Variables
 *******************************************************/
/* Declare a variable of type xQueueHandle. This variable is used to store the queue
	that is accessed by any task wants to write to I2C. */
extern QueueHandle_t xQueueI2CWriteBuffer;
extern QueueHandle_t xQueueI2CReadBuffer;

/*Those semaphores are used only for applications which would like to block waiting for end of
 transmission or reception */
extern SemaphoreHandle_t xBinarySemaphoreI2CAppEndOfWrite;
extern SemaphoreHandle_t xBinarySemaphoreI2CAppEndOfRead;

/*******************************************************
 *                Functions Declarations
 *******************************************************/
void vI2CWrite( void *pvParameters );
void vI2CRead( void *pvParameters );

#endif
