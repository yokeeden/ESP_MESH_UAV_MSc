/**********************************************************************
* - Description:		ESP_MESH_BASE_NODE - source file with code defining I2C read and write tasks for MPU6050
* - File:				i2c_application.c
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

#include "i2c_application.h"
#include "i2c_driver.h"

/*******************************************************
 *                       Macros
 *******************************************************/
#define QUEUE_LENGTH            1
#define WRITE_ITEM_SIZE         sizeof( I2C_Status )
#define READ_ITEM_SIZE          sizeof( I2C_Status )
#define GUARD_CYCLE		10

/*******************************************************
 *          Constants and Variables Definitions
 *******************************************************/
/* Declare a variable of type xQueueHandle. This is used to store the queue
	that is accessed by any task wants to write to I2C. */
QueueHandle_t xQueueI2CWriteBuffer;
QueueHandle_t xQueueI2CReadBuffer;

/*Those semaphores are used only for applications which would like to block waiting for end of
 transmission or reception */
SemaphoreHandle_t xBinarySemaphoreI2CAppEndOfWrite;
SemaphoreHandle_t xBinarySemaphoreI2CAppEndOfRead;

/* The array to use as the queue's storage area.  This must be at least
uxQueueLength * uxItemSize bytes. */
uint8_t ucQueueWriteStorageArea[ QUEUE_LENGTH * WRITE_ITEM_SIZE ];
uint8_t ucQueueReadStorageArea[ QUEUE_LENGTH * READ_ITEM_SIZE ];

/* The variable used to hold the queue's data structure. */
static StaticQueue_t xStaticWriteQueue;
static StaticQueue_t xStaticReadQueue;
/* Definition of semaphores variables*/
StaticSemaphore_t xSemaphoreWriteBuffer;
StaticSemaphore_t xSemaphoreReadBuffer;

/**
  * @brief  Write to I2C.
  * @param  None
  * @retval None
  */
void vI2CWrite( void *pvParameters )
{
	/* Declare the variable that will hold the values received from the queue. */
	I2C_Status lReceivedValue;
	portBASE_TYPE xStatus;

	/* The queue is created xStaticReadQueue to hold a maximum of 1 structure entry. */
	xQueueI2CWriteBuffer =
		xQueueCreateStatic( QUEUE_LENGTH, WRITE_ITEM_SIZE, ucQueueWriteStorageArea, &xStaticWriteQueue );

	/* Create a binary semaphore without using any dynamic memory
	allocation.  The semaphore's data structures will be saved into
	the xSemaphoreWriteBuffer variable. */
	xBinarySemaphoreI2CAppEndOfWrite = xSemaphoreCreateBinaryStatic( &xSemaphoreWriteBuffer );
	/* Assert that the handle is not null*/
	configASSERT( xBinarySemaphoreI2CAppEndOfWrite );

	/* Rest of the task code goes here. */

	/* Check the semaphore was created successfully. */
	if( xBinarySemaphoreI2CAppEndOfWrite != NULL )
	{
		/* As per most tasks, this task is implemented within an infinite loop.
			Take the semaphore once to start with so the semaphore is empty before the
			infinite loop is entered.  The semaphore was created before the scheduler
			was started so before this task ran for the first time.*/
		xSemaphoreTake( xBinarySemaphoreI2CAppEndOfWrite, 0 );
	}

	for(;;)
	{
		/* The first parameter is the queue from which data is to be received..

		The second parameter is the buffer into which the received data will be
		placed.  In this case the buffer is simply the address of a variable that
		has the required size to hold the received data.

		the last parameter is the block time - the maximum amount of time that the
		task should remain in the Blocked state to wait for data to be available should
		the queue already be empty. */
		/* Receive queue containing definition of structure which is used to store data read from I2C*/
		xStatus = xQueueReceive( xQueueI2CWriteBuffer, (void *)&lReceivedValue, portMAX_DELAY );
		if( xStatus == pdPASS )
		{
			/* Write on I2C */
			sI2cMasterWriteSlave(lReceivedValue.pBuffer, lReceivedValue.sDataLength,
					lReceivedValue.slaveAddress);

			vTaskDelay( GUARD_CYCLE / portTICK_RATE_MS );

			/* 'Give' the semaphore to unblock the task. */
			xSemaphoreGive( xBinarySemaphoreI2CAppEndOfWrite );
		}
		else
		{
			/* We did not receive anything from the queue even after waiting for blocking time defined.*/
		}
	}

	vTaskDelete(NULL);
}

/**
  * @brief  Read from I2C.
  * @param  None
  * @retval None
  */
void vI2CRead( void *pvParameters )
{
	/* Declare the variable that will hold the values received from the queue. */
	I2C_Status lReceivedValue;
	portBASE_TYPE xStatus;

	/* Create static queue for received structure */
	xQueueI2CReadBuffer =
		xQueueCreateStatic( QUEUE_LENGTH, READ_ITEM_SIZE, ucQueueReadStorageArea, &xStaticReadQueue );

	/* Create a binary semaphore without using any dynamic memory
	allocation.  The semaphore's data structures will be saved into
	the xSemaphoreReadBuffer variable. */
	xBinarySemaphoreI2CAppEndOfRead = xSemaphoreCreateBinaryStatic( &xSemaphoreReadBuffer );

	/* Assert that the handle is not null */
	configASSERT( xBinarySemaphoreI2CAppEndOfRead );

	/* Rest of the task code goes here. */

	/* Check the semaphore was created successfully. */
	if( xBinarySemaphoreI2CAppEndOfRead != NULL )
	{
		/* As per most tasks, this task is implemented within an infinite loop.
			Take the semaphore once to start with so the semaphore is empty before the
			infinite loop is entered.  The semaphore was created before the scheduler
			was started so before this task ran for the first time.*/
		xSemaphoreTake( xBinarySemaphoreI2CAppEndOfRead, 0 );
	}

	for(;;)
	{
		/* The first parameter is the queue from which data is to be received..

		The second parameter is the buffer into which the received data will be
		placed.  In this case the buffer is simply the address of a variable that
		has the required size to hold the received data.

		the last parameter is the block time ? the maximum amount of time that the
		task should remain in the Blocked state to wait for data to be available should
		the queue already be empty. */
		xStatus = xQueueReceive( xQueueI2CReadBuffer, (void *)&lReceivedValue, portMAX_DELAY );
		if( xStatus == pdPASS )
		{
			/* Read from I2C */
			sI2cMasterReadSlave(lReceivedValue.pBuffer, lReceivedValue.sDataLength,\
					lReceivedValue.slaveAddress);

			vTaskDelay( GUARD_CYCLE / portTICK_RATE_MS );

			/* 'Give' the semaphore to unblock the task. */
			xSemaphoreGive( xBinarySemaphoreI2CAppEndOfRead );
		}
		else
		{
			/* We did not receive anything from the queue even after waiting for blocking time defined.*/
		}
	}

	vTaskDelete(NULL);
}
