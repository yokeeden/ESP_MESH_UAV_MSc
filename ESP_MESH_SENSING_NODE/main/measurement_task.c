/**********************************************************************
* - Description:		ESP_MESH_SENSING_NODE - source file for measurement task
* - File:				measurement_task.c
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
#include "driver/adc.h"
#include "esp_adc_cal.h"

/*******************************************************
 *                       Macros
 *******************************************************/
#define TAG_BME280 "BME280"
#define TASK_ID "[MEASUREMENT_TASK]"
#define DEFAULT_VREF    1100        //Default reference value for ADC calibration
#define NO_OF_SAMPLES   64          //Number of samples for multisampling operation
#define OFFSET_VOLTAGE 500
#define OUTPUT_VOLTAGE_SCALING 10.0
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
/*Variables for ADC configuration */
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
extern struct bme280_dev dev; // from bme280config.c
extern uint8_t dev_addr; // from bme280config.c
extern EventGroupHandle_t s_mesh_event_group; //from mesh_main.c
extern QueueHandle_t xQueueMeasurementTaskToMeshTXTask; //from mesh_main.c
extern bool sensor_flag; //from mesh_main.c
/*******************************************************
 *                Functions Declarations
 *******************************************************/
extern esp_err_t i2c_master_init(void); //from bme280config.c
extern void bme280_i2c_init(void); //from bme280config.c
static void print_char_val_type(esp_adc_cal_value_t val_type);
static void check_efuse(void);


/*******************************************************
 *                Functions Definitions
 *******************************************************/

/**
  * @brief  measurement task - data measurement for bme280(i2c) and tmp36(analog)
  * @param  None
  * @retval None
  */
void measurement_task (void *pvParameters)
{
	int8_t rslt;
	uint8_t settings_sel;
	uint32_t adc_reading = 0;

	/*I2C master initialization */
	ESP_ERROR_CHECK(i2c_master_init());

	/*BME280 initialization */
	bme280_i2c_init();

	/*BME280 settings selection */
	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_4X;
	dev.settings.osr_t = BME280_OVERSAMPLING_4X;
	dev.settings.filter = BME280_FILTER_COEFF_OFF;
	dev.settings.standby_time = BME280_STANDBY_TIME_125_MS;
	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL | BME280_STANDBY_SEL;

	/*BME280 set selected settings and sensor mode of operation*/
	rslt = bme280_set_sensor_settings(settings_sel, &dev);
	printf("bme280 settings config result %s\r\n", rslt ? "ERROR" : "OK");
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
	printf("bme280 set sensor data result %s\r\n",  rslt ? "ERROR" : "OK");

	/*Check if Two Point or Vref are burned into eFuse - analog TMP36 readout*/
    check_efuse();

    //Configure ADC
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

    /*measurement task inifinite loop*/
    while(1)
	{

    	/*get BME280 measured data - pressure, humidity and temperature */
    	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
    	/*delay 75 ms */
    	dev.delay_us(75, &dev_addr);
		if (rslt == BME280_OK)
			{
				ESP_LOGI(TASK_ID,"[1/2] BME280 data: Pressure: %.2f Pa, Temperature: %.2f C, Humidity: %.2f %%",comp_data.pressure, comp_data.temperature, comp_data.humidity);
			}
		else
			{
				ESP_LOGE(TAG_BME280, "measure error. code: %d", rslt);
			}

		/*ADC data readout - TMP36 */
		//Multisampling
		for (int i = 0; i < NO_OF_SAMPLES; i++)
		{
			 if (unit == ADC_UNIT_1) {
				 adc_reading += adc1_get_raw((adc1_channel_t)channel);
			 }
		}
		adc_reading /= NO_OF_SAMPLES;
		//Convert adc_reading to voltage in mV - calibration and noise reduction
		uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
		/*TMP36 printf results*/
		float analog_temperature=((voltage-OFFSET_VOLTAGE)/OUTPUT_VOLTAGE_SCALING);
		ESP_LOGI(TASK_ID,"[2/2] TMP36 data: Raw: %d mV, Voltage: %d mV, Calibrated temerature:  %0.1f C\n",adc_reading, voltage, analog_temperature);
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
		vTaskDelay(200/ portTICK_RATE_MS);
	} /*end of inifinite loop */

    vTaskDelete(NULL);
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
