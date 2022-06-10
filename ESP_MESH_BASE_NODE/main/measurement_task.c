                /**********************************************************************
* - Description:		ESP_MESH_BASE_NODE - source file defining measurement task and MPU6050, TMP36  functions for initialization and readout
* - File:				measurement_task.c
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
 *                To Do and Notes
 *******************************************************/

/*******************************************************
 *                Include Header Files
 *******************************************************/
#include <string.h>
#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <freertos/queue.h>
#include "driver/i2c.h"
#include <esp_log.h>
#include "bme280.h"
#include "driver/adc.h" //header for ADC driver
#include "esp_adc_cal.h" //header for ADC calibration
#include <math.h>
#include <stdio.h>
#include "i2c_application.h"
#include "i2c_driver.h"

/*******************************************************
 *                       Macros
 *******************************************************/

#define TAG_BME280 "BME280"
#define TASK_ID "[MEASUREMENT_TASK]"
#define DEFAULT_VREF    1100        //Default reference value for ADC calibration
#define NO_OF_SAMPLES   64          //Number of samples for multisampling operation
#define OFFSET_VOLTAGE 500
#define OUTPUT_VOLTAGE_SCALING 10.0

#define REFRESH_RATE        0.005
#define MPU6050_SLAVE_ADDR          0x68 //MPU6050 I2C addres
/*MPU6050 accelerometer, gyroscope and temperature sensor data registers and control registers */
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_ACCEL_XOUT_L        0x3C
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_TEMP_OUT_L          0x42
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_CONFIG              0x1A

#define MPU6050_SUCESS					 	       0
#define MPU6050_FAILURE						       1
#define ACCELEROMETER_READING_FREQUENCY_MS	     REFRESH_RATE * 1000	// 5ms second
/*Macros defining measurement scales and sensitivities */
#define _2G_SCALE               0x00
#define _4G_SCALE               0x08
#define _8G_SCALE               0x10
#define _16G_SCALE              0x18

#define _250_DEGREE             0x00
#define _500_DEGREE             0x08
#define _1000_DEGREE            0x10
#define _2000_DEGREE            0x18

#define LSB_Sensitivity_2G      16384.0
#define LSB_Sensitivity_4G      8192.0
#define LSB_Sensitivity_8G      4096.0
#define LSB_Sensitivity_16G     2048.0

#define LSB_Sensitivity_250     131.0
#define LSB_Sensitivity_500     65.5
#define LSB_Sensitivity_1000    32.8
#define LSB_Sensitivity_2000    16.4
/*Macro for synchronization bit xEventWaitBits after measurement_task sensors initialization */
#define INITIALIZATIONS_DONE BIT0
/*structure definition for passing data between tasks */
typedef struct {
    double pressure;
    double temperature;
    double humidity;
    float analog_temperature;
} passed_data_structure;
/*******************************************************
 *          Constants and Variables Definitions
 *******************************************************/
/*MPU6050 constants and variables for storing parameters */
double a_x = 0.0;
double a_y = 0.0;
double a_z = 0.0;

int16_t accelration_x;
int16_t accelration_y;
int16_t accelration_z;

double g_x = 0.0;
double g_y = 0.0;
double g_z = 0.0;

int16_t gyro_x;
int16_t gyro_y;
int16_t gyro_z;

double pitch = 0.0;
double roll = 0.0;

double temperature = 0.0;

double senistivity_a_x = 0.0;
double senistivity_a_y = 0.0;
double senistivity_a_z = 0.0;
/*Structure for passing data to I2C read and write tasks */
static I2C_Status i2cStatus;
static uint8_t mpu6050Buffer[32];
/*Variables for ADC configuration  */
static esp_adc_cal_characteristics_t *adc_chars; //Structure storing characteristics of an ADC
static const adc_channel_t channel = ADC_CHANNEL_0;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
/*Structure containing BME280 measurement data */
struct bme280_data comp_data;

/*******************************************************
 *      EXTERN Constants and Variables
 *******************************************************/

extern struct bme280_dev dev; // from i2c_driver.c
extern uint8_t dev_addr; // from i2c_driver.c
extern EventGroupHandle_t s_mesh_event_group; //from mesh_main.c
extern QueueHandle_t xQueueMeasurementTaskToMeshTXTask; //from mesh_main.c
extern bool sensor_flag; //from mesh_main.c
/*******************************************************
 *                Functions Declarations
 *******************************************************/

uint8_t vMPU6050Init();
uint8_t uMPU6050ReadAccelerometer();
uint8_t uMPU6050ReadGyroscope();
uint8_t uMPU6050ReadTemperature();
void vMPU6050CalculateAngles();
static void print_char_val_type(esp_adc_cal_value_t val_type);
static void check_efuse(void);

/*******************************************************
 *                Functions Definitions
 *******************************************************/

/**
  * @brief  measurement task - data measurement for bme280(i2c), tmp36(analog), mpu6050(i2c)
  * @param  None
  * @retval None
  */
void measurement_task( void *pvParameters )
{
	int8_t rslt;
	uint8_t settings_sel;
	uint32_t adc_reading = 0;

	/*I2C master initialization*/
	ESP_ERROR_CHECK(vI2CInit());

	/*BME280 initialization */
	bme280_i2c_init();

	/*BME280 settings selection */
	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_4X;
	dev.settings.osr_t = BME280_OVERSAMPLING_4X;
	dev.settings.filter = BME280_FILTER_COEFF_OFF;
	dev.settings.standby_time = BME280_STANDBY_TIME_125_MS;
	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL | BME280_STANDBY_SEL;

	/*BME280 set seleceted settings and sensor mode of operation*/
	rslt = bme280_set_sensor_settings(settings_sel, &dev);
	printf("bme280 settings config result %d\r\n", rslt);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
	printf("bme280 set sensor data result %d\r\n", rslt);

	/*MPU6050 initialization */
	if (vMPU6050Init() != MPU6050_SUCESS) {
        vTaskDelete(NULL);
        return;
    }


	/*Check if Two Point or Vref are burned into eFuse - analog TMP36 readout*/
	check_efuse();

	/*Configure ADC*/
	if (unit == ADC_UNIT_1) {
	    adc1_config_width(width);
	    adc1_config_channel_atten(channel, atten);
	 }

	 //Characterize ADC
	 adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	 esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);

	 /*print used calibration method */
	 print_char_val_type(val_type);

	 /*Set bits after end of initializations */
	 xEventGroupSetBits(s_mesh_event_group, INITIALIZATIONS_DONE);

	 /*measurement task infinite loop*/
    for(;;)
	{
    	/*get BME280 measured data - pressure, humidity and temperature */
    	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
    	/*delay 75 ms */
    	dev.delay_us(75, &dev_addr);

    	/*BME280 measured data printing */
    	if (rslt == BME280_OK)
    		{
    			ESP_LOGI(TASK_ID,"[1/3] BME280 data: Pressure: %.2f Pa, Temperature: %.2f C, Humidity: %.2f %%",comp_data.pressure, comp_data.temperature, comp_data.humidity);
    		}
    	else
    		{
    			ESP_LOGE(TAG_BME280, "measure error. code: %d", rslt);
    		}

    	/*MPU6050 data readout */
    	if (uMPU6050ReadAccelerometer() != MPU6050_SUCESS) {
            continue;
        }

        if (uMPU6050ReadGyroscope() != MPU6050_SUCESS) {
            continue;
        }

        if (uMPU6050ReadTemperature() != MPU6050_SUCESS) {
            continue;
        }

        /*MPU6050 calculate angles from measured raw values */
        vMPU6050CalculateAngles();

        /*MPU6050 log results */
        ESP_LOGI(TASK_ID,"[2/3] MPU6050 data: Acc: (%.3f, %.3f, %.3f),Gyro: (%.3f, %.3f, %.3f), Pitch: %.3f, Roll: %.3f, Temperature: %.2f C",
        		a_x, a_y, a_z, g_x, g_y, g_z, pitch, roll, temperature);

        /*ADC data readout - TMP36 */
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
               adc_reading += adc1_get_raw((adc1_channel_t)channel);
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV - calibration and noise reduction
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        /*TMP36 printf results*/
        float analog_temperature=((voltage-OFFSET_VOLTAGE)/OUTPUT_VOLTAGE_SCALING);
        ESP_LOGI(TASK_ID,"[3/3] TMP36 data: Raw: %d mV, Voltage: %d mV, Calibrated temerature:  %0.1f C\n",adc_reading, voltage, analog_temperature);
        /*Enable communication with tasks tx_task - measurement_task if node is sensing node*/
		if (sensor_flag == true){
			portBASE_TYPE xStatus;
			passed_data_structure send_data_structure_to_txtask;

			send_data_structure_to_txtask.pressure=comp_data.pressure;
			send_data_structure_to_txtask.temperature=comp_data.temperature;
			send_data_structure_to_txtask.humidity=comp_data.humidity;
			send_data_structure_to_txtask.analog_temperature=analog_temperature;
			// wait max 10 s to release the queue then write pdFail to x status
			xStatus = xQueueSend( xQueueMeasurementTaskToMeshTXTask, (void *)&send_data_structure_to_txtask, 10*1000 / portTICK_RATE_MS );
			if (xStatus == pdFAIL){
				ESP_LOGW(TASK_ID, "Cannot send queue data structure");
				//vTaskDelay(2*1000/ portTICK_RATE_MS);
			}
		}
        vTaskDelay( 200 / portTICK_RATE_MS );
    } /*end of inifinite loop */
    vTaskDelete(NULL);
}

/**
  * @brief  Configure the MPU 6050.
  * @param  None
  * @retval Success or Fail
  */
uint8_t vMPU6050Init()
{
	/*define status indicator */
	portBASE_TYPE xStatus;

	// Change status from sleep to running
	mpu6050Buffer[0] = MPU6050_PWR_MGMT_1;
	mpu6050Buffer[1] = 0x00;
	// Write data to i2c
	i2cStatus.pBuffer = mpu6050Buffer;
	i2cStatus.sDataLength = 2;
	i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

	/*Send data structure to queue xQueueI2CWriteBuffer */
	xStatus = xQueueSend( xQueueI2CWriteBuffer, (void *)&i2cStatus, portMAX_DELAY );

	// Wait for end of writing process and take semaphore
	xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfWrite, portMAX_DELAY );

	if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

	// Configure accelerometer
	mpu6050Buffer[0] = MPU6050_ACCEL_CONFIG;
	mpu6050Buffer[1] = _2G_SCALE;        // +/-2g
	// Write data to i2c
	i2cStatus.pBuffer = mpu6050Buffer;
	i2cStatus.sDataLength = 2;
	i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

	xStatus = xQueueSend( xQueueI2CWriteBuffer, (void *)&i2cStatus, portMAX_DELAY );

	// Wait for end of writing process
	xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfWrite, portMAX_DELAY );

	if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

	// Configure gyroscope
	mpu6050Buffer[0] = MPU6050_GYRO_CONFIG;
	mpu6050Buffer[1] = _2000_DEGREE;        // +/-2000degree
	// Write data to i2c
	i2cStatus.pBuffer = mpu6050Buffer;
	i2cStatus.sDataLength = 2;
	i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

	xStatus = xQueueSend( xQueueI2CWriteBuffer, (void *)&i2cStatus, portMAX_DELAY );

	// Wait for end of writing process
	xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfWrite, portMAX_DELAY );

	if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

	return MPU6050_SUCESS;
}

/**
  * @brief  Read accelerometer.
  * @param  None
  * @retval Success or Fail
  */
uint8_t uMPU6050ReadAccelerometer()
{
	portBASE_TYPE xStatus;
	// Read accelerometer register
    mpu6050Buffer[0] = MPU6050_ACCEL_XOUT_H;
    // Write data to structure send by queue - write i2C
    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 1;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CWriteBuffer, (void *)&i2cStatus, portMAX_DELAY );

    // Wait for end of writing process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfWrite, portMAX_DELAY );

    if (xStatus == pdFAIL)
    {
    	return MPU6050_FAILURE;
    }
    //Read data from i2c
    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 6;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CReadBuffer, (void *)&i2cStatus, portMAX_DELAY );

    // Wait for end of reading process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfRead, portMAX_DELAY );

    if (xStatus == pdFAIL)
    {
    	return MPU6050_FAILURE;
    }

    accelration_x =
        ((uint16_t)(mpu6050Buffer[0] << 8)) |
        ((uint16_t)(mpu6050Buffer[1]));

    accelration_y =
        ((uint16_t)(mpu6050Buffer[2] << 8)) |
        ((uint16_t)(mpu6050Buffer[3]));

    accelration_z =
        ((uint16_t)(mpu6050Buffer[4] << 8)) |
        ((uint16_t)(mpu6050Buffer[5]));

    a_x = -accelration_x / LSB_Sensitivity_2G;
    a_y = -accelration_y / LSB_Sensitivity_2G;
    a_z = -accelration_z / LSB_Sensitivity_2G;

    return MPU6050_SUCESS;
}

/**
  * @brief  Read gyroscope.
  * @param  None
  * @retval Success or Fail
  */
uint8_t uMPU6050ReadGyroscope()
{

	portBASE_TYPE xStatus;
    // Read accelerometer
    mpu6050Buffer[0] = MPU6050_GYRO_XOUT_H;
    // Write data to i2c
    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 1;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CWriteBuffer, (void *)&i2cStatus, portMAX_DELAY );

    // Wait for end of writing process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfWrite, portMAX_DELAY );

    if (xStatus == pdFAIL)
    {
    	return MPU6050_FAILURE;
    }

    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 6;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CReadBuffer, (void *)&i2cStatus, portMAX_DELAY );

    // Wait for end of reading process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfRead, portMAX_DELAY );

    if (xStatus == pdFAIL)
    {
    	return MPU6050_FAILURE;
    }

    gyro_x =
        ((uint16_t)(mpu6050Buffer[0] << 8)) |
        ((uint16_t)(mpu6050Buffer[1]));

    gyro_y =
        ((uint16_t)(mpu6050Buffer[2] << 8)) |
        ((uint16_t)(mpu6050Buffer[3]));

    gyro_z =
        ((uint16_t)(mpu6050Buffer[4] << 8)) |
        ((uint16_t)(mpu6050Buffer[5]));

    g_x = gyro_x / LSB_Sensitivity_2000;
    g_y = gyro_y / LSB_Sensitivity_2000;
    g_z = gyro_z / LSB_Sensitivity_2000;

    return MPU6050_SUCESS;
}

/**
  * @brief  Read temperature.
  * @param  None
  * @retval Success or Fail
  */
uint8_t uMPU6050ReadTemperature()
{
	portBASE_TYPE xStatus;
	// Read accelerometer
    mpu6050Buffer[0] = MPU6050_TEMP_OUT_H;
    // Write data to i2c
    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 1;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CWriteBuffer, (void *)&i2cStatus, portMAX_DELAY );

    // Wait for end of writing process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfWrite, portMAX_DELAY );

    if (xStatus == pdFAIL)
    {
    	return MPU6050_FAILURE;
    }

    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 2;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CReadBuffer, (void *)&i2cStatus, portMAX_DELAY );

    // Wait for end of reading process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfRead, portMAX_DELAY );

    if (xStatus == pdFAIL)
    {
    	return MPU6050_FAILURE;
    }

    int16_t raw_temp =
        ((uint16_t)(mpu6050Buffer[0] << 8)) |
        ((uint16_t)(mpu6050Buffer[1]));

    temperature = (raw_temp / 340.0) + 36.53;

    return MPU6050_SUCESS;
}

/**
  * @brief  Calculate rolling and pitch angles.
  * @param  None
  * @retval None
  */
void vMPU6050CalculateAngles()
{
    pitch = atan(a_x / a_z)*57.2958;        // convert the radian to degrees
    roll = atan(a_y / a_z)*57.2958;         // convert the radian to degrees
}

/**
  * @brief  Check avalibility of analog calibration with data stored in efuse
  * @param  None
  * @retval None
  */
static void check_efuse(void)
{
    //Check if TP - Two Point - is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

/**
  * @brief  Print method used for ADC calibration
  * @param  None
  * @retval None
  */
static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}
