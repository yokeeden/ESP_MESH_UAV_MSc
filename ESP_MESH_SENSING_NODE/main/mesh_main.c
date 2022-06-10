/**********************************************************************
* - Description:		ESP_MESH_SENSING_NODE - main source file
* - File:				mesh_main.c
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
#include <string.h>
#include <stdio.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "bme280.h"

/*******************************************************
 *                Macros
 *******************************************************/
/* macros for defined stack sizes */
#define STACK_SIZE_2048 2048
#define STACK_SIZE_3072 3072
#define STACK_SIZE_4096 4096
/*macros for defined RX and TX buffer sizes used in RX and TX tasks */
#define RX_SIZE          (1500)
#define TX_SIZE          (1460)

#define INITIALIZATIONS_DONE BIT0


#define QUEUE_LENGTH            1
#define WRITE_ITEM_SIZE         sizeof( passed_data_structure )
#define READ_ITEM_SIZE          sizeof( passed_data_structure )

/*structure for passed data in queues*/
typedef struct {
    double pressure;
    double temperature;
    double humidity;
    float analog_temperature;
} passed_data_structure;

/*******************************************************
 *          Constants and Variables Definitions
 *******************************************************/
static const char *MESH_TAG = "measurement_system_mesh";
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};//declaration of ESP-MESH network ID
static uint8_t tx_buf[TX_SIZE] = { 0, }; //declaration of TX buffer
static uint8_t rx_buf[RX_SIZE] = { 0, }; //declaration of RX buffer
static bool is_running; //task running flag
static bool is_mesh_connected = false; //flag indicating parent node connection to root node
static mesh_addr_t mesh_parent_addr; //declaration of union containing parent node MAC address
static mesh_addr_t mesh_root_addr; //declaration of union containing root node MAC address
static int mesh_layer = -1; //declaration of variable indicating mesh layer number
static esp_netif_t *netif_sta = NULL; //declaration of pointer to instance of STATION interface

EventGroupHandle_t s_mesh_event_group; //definition of variable used to handle events groups
QueueHandle_t xQueueMeasurementTaskToMeshTXTask; //definition of variable used to handle queues

bool sensor_flag = true; //declaration of flag indicating sensor node role

/*******************************************************
 *                Function Declarations
 *******************************************************/

extern void measurement_task (void *pvParameters); //from measurement_task.c

/*******************************************************
 *                Function Definitions
 *******************************************************/

/**
  * @brief  MESH TX task
  * @param  None
  * @retval None
  */
void esp_mesh_p2p_tx_main(void *arg)
{
      /* status */
      esp_err_t status; //definition of error handling variable

      /* routing table */
      mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
      int route_table_size;
      int i;
      /*configuration of structure containing TX data that is passed to esp_mesh_send function  */
      mesh_data_t tx_data;
      tx_data.data = tx_buf;
      tx_data.size = TX_SIZE;
      tx_data.proto = MESH_PROTO_BIN;
      tx_data.tos = MESH_TOS_P2P;
      is_running = true;

      /*Wait for the end of initializations from measurement_task*/
      xEventGroupWaitBits(s_mesh_event_group,
              INITIALIZATIONS_DONE,
              pdFALSE,
              pdFALSE,
              portMAX_DELAY);
      /*delete event group*/
      vEventGroupDelete(s_mesh_event_group);

      /*task infinite loop */
      while (is_running) {
    	  if(is_mesh_connected == true){
    		  /* if a node is a root node it will print its children*/
    		  if (esp_mesh_is_root()){
    			  sensor_flag = false;
    			  /* get routing table */
    			  status = esp_mesh_get_routing_table((mesh_addr_t*)&route_table, CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);
    			  /*error handling */
    			  if (status != ESP_OK) {
    				  ESP_LOGW(MESH_TAG, "[TX TASK] cannot read routing table");
    				  vTaskDelay(10 * 1000 / portTICK_RATE_MS);
    				  continue;
    			  }
    			  /*printing children list */
    			  for ( i = 1; i < route_table_size; i++) {
    				  ESP_LOGI(MESH_TAG, "[TX TASK] I am root node. My child [%d/%d] is "MACSTR"\n",i,route_table_size-1,
    						  MAC2STR(route_table[i].addr));
    			  }
    			  vTaskDelay(10*1000 / portTICK_RATE_MS);
    		  }
    		  /*if node is not a root node then receive data from measurement_task and send it wirelessly to root node */
    		  else if(!esp_mesh_is_root()) {

    			  sensor_flag = true;

    			  /*define status indicator */
    			  portBASE_TYPE xStatus;
    			  passed_data_structure received_structure_from_measurement;
    			  /* wait to receive a queue data */
    			  xStatus = xQueueReceive( xQueueMeasurementTaskToMeshTXTask, (void *)&received_structure_from_measurement, portMAX_DELAY );
    			  if( xStatus == pdPASS ){
    				  /*copy received data to TX buffer */
    				  memcpy(tx_buf,(uint8_t*)&received_structure_from_measurement,sizeof(received_structure_from_measurement));
    				  /*send data wirelessly */
    				  status = esp_mesh_send(NULL, &tx_data, 0, NULL, 1);
    				  if (status == ESP_OK){
    					  ESP_LOGI(MESH_TAG, "[TX TASK] Sending measurement data message to root node: "
    							  "("MACSTR")\n", MAC2STR(mesh_root_addr.addr));
    				  }
    				  /*error handling */
    				  else if (status != ESP_OK) {
    					  ESP_LOGW(MESH_TAG, "[TX TASK] cannot send message to root node "MACSTR" : heap:%d [err:0x%x, proto:%d, tos:%d]",
                                 esp_get_free_heap_size(), MAC2STR(mesh_root_addr.addr), status, tx_data.proto, tx_data.tos);
    					  continue;
    				  }
    			  }
    			  /*error handling */
    			  else if ( xStatus == pdFAIL)
    			  {
    				  ESP_LOGW(MESH_TAG,"Queue error receiving from measurement_task");
    			  }
    		  }
    	  }
    	  /*error handling */
    	  else {
    		  ESP_LOGW(MESH_TAG, "Parent disconnected - mesh is not connected");
    		  vTaskDelay(10*1000 / portTICK_RATE_MS);
    	  }
      }/* while */
      vTaskDelete(NULL);
    }

/**
  * @brief  MESH RX task
  * @param  None
  * @retval None
  */
void esp_mesh_p2p_rx_main(void *arg)
{
	  esp_err_t status;
	  int flag = 0;

	  /*configuration of structure containing RX data that is passed from esp_mesh_recv function  */
	  mesh_addr_t from;
	  mesh_data_t rx_data;
	  rx_data.data = rx_buf;
	  rx_data.size = RX_SIZE;
	  is_running = true;

	  /*Wait for the end of initializations from measurement_task*/
	  xEventGroupWaitBits(s_mesh_event_group,
              INITIALIZATIONS_DONE,
              pdFALSE,
              pdFALSE,
              portMAX_DELAY);


	  while (is_running) {
		  /*if a node is root*/
		  if (esp_mesh_is_root()){
			  /*receive data from other nodes and pass it to RX buffer */
			  status = esp_mesh_recv(&from, &rx_data, portMAX_DELAY,&flag, NULL, 1);
			  /*error handling */
			  if (status != ESP_OK || !rx_data.size) {
				  ESP_LOGW(MESH_TAG, "[RX TASK] message receive error: heap:%d [err:0x%x, size:%d]",
						  esp_get_free_heap_size(), status, rx_data.size);
				  continue;
			  }

			  //parse received structure to pointer type structure
			  passed_data_structure *in = (passed_data_structure*) rx_data.data;
			  /*print received data */
			  ESP_LOGI(MESH_TAG, "[RX TASK] Received data: Pressure: %.2f Pa, Temperature: %.2f C, "
					  "Humidity: %.2f, Analog Temperature: %.1f  from node "
					  "("MACSTR")\n",in->pressure,in->temperature,in->humidity,in->analog_temperature, MAC2STR(from.addr));

			  vTaskDelay(100 / portTICK_RATE_MS);
		  }
		  /*if node is sensor node then it do not receive data  */
	  	  else if (!esp_mesh_is_root()) {
	  		  ESP_LOGI(MESH_TAG,"[RX TASK] I am sensor node, so i do not recive any messages. "
	  				  "I am a child to node: "MACSTR"\n",MAC2STR(mesh_parent_addr.addr));
	  		  vTaskDelay(10*1000 / portTICK_RATE_MS);
	  	  }
	  }
	  vTaskDelete(NULL);
	}

/**
  * @brief  MESH tasks starting task, starts other tasks after successful mesh connection
  * @param  None
  * @retval None
  */
esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 6, NULL);
        xTaskCreate(measurement_task,"measurement_task", 3072, NULL, 7, NULL);
    }
    return ESP_OK;
}

/**
  * @brief  Handler function for MESH events
  * @param  None
  * @retval None
  */
void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint16_t last_layer = 0;

    switch (event_id) {
    case MESH_EVENT_STARTED: {
        esp_mesh_get_id(&id);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_MESH_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOPPED>");
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child_connected = (mesh_event_child_connected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"",
                 child_connected->aid,
                 MAC2STR(child_connected->mac));
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
                 child_disconnected->aid,
                 MAC2STR(child_disconnected->mac));
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 no_parent->scan_times);
    }
    /* TODO handler for the failure */
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        memcpy(&mesh_parent_addr.addr, connected->connected.bssid, 6);
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent:"MACSTR"%s, ID:"MACSTR", duty:%d",
                 last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr), connected->duty);
        last_layer = mesh_layer;
        is_mesh_connected = true;
        if (esp_mesh_is_root()) {
            esp_netif_dhcpc_start(netif_sta); //if a node is root node then start DHCP client (only if enabled in interface object)
        }
        esp_mesh_comm_p2p_start(); //function call for tasks creation
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
                 disconnected->reason);
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
    }
    break;
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr = (mesh_event_root_address_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_ADDRESS>root address:"MACSTR"",
                 MAC2STR(root_addr->addr));
        memcpy(&mesh_root_addr.addr,root_addr->addr,sizeof(root_addr->addr));
    }
    break;
    case MESH_EVENT_VOTE_STARTED: {
        mesh_event_vote_started_t *vote_started = (mesh_event_vote_started_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:"MACSTR"",
                 vote_started->attempts,
                 vote_started->reason,
                 MAC2STR(vote_started->rc_addr.addr));
    }
    break;
    case MESH_EVENT_VOTE_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_VOTE_STOPPED>");
        break;
    }
    case MESH_EVENT_ROOT_SWITCH_REQ: {
        mesh_event_root_switch_req_t *switch_req = (mesh_event_root_switch_req_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:"MACSTR"",
                 switch_req->reason,
                 MAC2STR( switch_req->rc_addr.addr));
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_ACK: {
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&mesh_parent_addr);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:"MACSTR"", mesh_layer, MAC2STR(mesh_parent_addr.addr));
    }
    break;
    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *toDs_state = (mesh_event_toDS_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_TODS_REACHABLE>state:%d", *toDs_state);
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_FIXED>%s",
                 root_fixed->is_fixed ? "fixed" : "not fixed");
    }
    break;
    case MESH_EVENT_ROOT_ASKED_YIELD: {
        mesh_event_root_conflict_t *root_conflict = (mesh_event_root_conflict_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_ASKED_YIELD>"MACSTR", rssi:%d, capacity:%d",
                 MAC2STR(root_conflict->addr),
                 root_conflict->rssi,
                 root_conflict->capacity);
    }
    break;
    case MESH_EVENT_CHANNEL_SWITCH: {
        mesh_event_channel_switch_t *channel_switch = (mesh_event_channel_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHANNEL_SWITCH>new channel:%d", channel_switch->channel);
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_SCAN_DONE>number:%d",
                 scan_done->number);
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NETWORK_STATE>is_rootless:%d",
                 network_state->is_rootless);
    }
    break;
    case MESH_EVENT_STOP_RECONNECTION: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOP_RECONNECTION>");
    }
    break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *find_network = (mesh_event_find_network_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_FIND_NETWORK>new channel:%d, router BSSID:"MACSTR"",
                 find_network->channel, MAC2STR(find_network->router_bssid));
    }
    break;
    case MESH_EVENT_ROUTER_SWITCH: {
        mesh_event_router_switch_t *router_switch = (mesh_event_router_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel:%d, "MACSTR"",
                 router_switch->ssid, router_switch->channel, MAC2STR(router_switch->bssid));
    }
    break;
    case MESH_EVENT_PS_PARENT_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_PS_PARENT_DUTY>duty:%d", ps_duty->duty);
    }
    break;
    case MESH_EVENT_PS_CHILD_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_PS_CHILD_DUTY>cidx:%d, "MACSTR", duty:%d", ps_duty->child_connected.aid-1,
                MAC2STR(ps_duty->child_connected.mac), ps_duty->duty);
    }
    break;
    default:
        ESP_LOGI(MESH_TAG, "unknown id:%d", event_id);
        break;
    }
}

/**
  * @brief  Handler function for GOT IP event
  * @param  None
  * @retval None
  */
void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(MESH_TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));

}

void app_main(void)
{
	/*init default NVS partition*/
	ESP_ERROR_CHECK(nvs_flash_init());

	/*  tcp/ip stack initialization*/
    ESP_ERROR_CHECK(esp_netif_init());

    /*  event loop initialization*/
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /*  create network interfaces for mesh (only station instance saved for further manipulation, soft AP instance ignored */
    /*Creates default STA and AP network interfaces for esp-mesh
     * Both netifs are almost identical to the default station and softAP, but with DHCP client and server disabled.
     * Please note that the DHCP client is typically enabled only if the device is promoted to a root node.
     * Returns created interfaces which could be ignored setting parameters to NULL
     * if an application code does not need to save the interface instances for further processing
     * netifs - network interfaces!*/
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));

    /*  wifi default initialization */
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    /* register IP events handler */
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    /*start wifi according to current configuration */
    ESP_ERROR_CHECK(esp_wifi_start());

    /*  mesh initialization */
    ESP_ERROR_CHECK(esp_mesh_init());
    /* register mesh events handler */
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    /*  set mesh topology */
    ESP_ERROR_CHECK(esp_mesh_set_topology(CONFIG_MESH_TOPOLOGY)); //to configure default TREE topology open sdkconfig example configuration

    /*  set mesh max layer according to the topology */
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));     /*    range 1 25 if MESH_TOPO_TREE
        range 1 1000 if MESH_TOPO_CHAIN
        default 6 */
    /*set vote percentage that a node must obtain to become root node */
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    /*set queue length for packets in RX buffer*/
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));

    /*if project configuration in sdkconfig is set to enable power saving mode of esp-mesh network */
#ifdef CONFIG_MESH_ENABLE_PS
    /* Enable mesh power saving function */
    ESP_ERROR_CHECK(esp_mesh_enable_ps());
    /* set associate expire time (time by end of witch node that is not sending any data will be discarded)
         * better to increase the associate expired time, if a small duty cycle is set. */
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(60));
    /* set announce time
         * better to increase the announce interval to avoid too much management traffic, if a small duty cycle is set. */
    ESP_ERROR_CHECK(esp_mesh_set_announce_interval(600, 3300));
#else
    /* Disable mesh power saving function */
    ESP_ERROR_CHECK(esp_mesh_disable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));
#endif

    /*mesh network parameters configuration*/
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    /*set mesh ID */
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);
    /* set communication channel with AP */
    cfg.channel = CONFIG_MESH_CHANNEL;
    /*set SSID*/
    cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
    memcpy((uint8_t *) &cfg.router.ssid, CONFIG_MESH_ROUTER_SSID, cfg.router.ssid_len);
    /*set password */
    memcpy((uint8_t *) &cfg.router.password, CONFIG_MESH_ROUTER_PASSWD, strlen(CONFIG_MESH_ROUTER_PASSWD));
    /* mesh softAP */
    /*set authentication mode*/
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
    /*set maximum connections number*/
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    /*set password*/
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,strlen(CONFIG_MESH_AP_PASSWD));
    /*set configuration*/
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));

    /* start mesh */
    ESP_ERROR_CHECK(esp_mesh_start());
    /*if power saving mode is enabled */
   #ifdef CONFIG_MESH_ENABLE_PS
       /* set the device active duty cycle. (default:12, MESH_PS_DEVICE_DUTY_REQUEST) */
       ESP_ERROR_CHECK(esp_mesh_set_active_duty_cycle(CONFIG_MESH_PS_DEV_DUTY, CONFIG_MESH_PS_DEV_DUTY_TYPE));
       /* set the network active duty cycle. (default:12, -1, MESH_PS_NETWORK_DUTY_APPLIED_ENTIRE) */
       ESP_ERROR_CHECK(esp_mesh_set_network_duty_cycle(CONFIG_MESH_PS_NWK_DUTY, CONFIG_MESH_PS_NWK_DUTY_DURATION, CONFIG_MESH_PS_NWK_DUTY_RULE));
   #endif
    /*print mesh successful connection message, free heap size, root state, topology and power saving mode */
    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%d, %s<%d>%s, ps:%d\n",  esp_get_minimum_free_heap_size(),
                esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed",
                esp_mesh_get_topology(), esp_mesh_get_topology() ? "(chain)":"(tree)", esp_mesh_is_ps_enabled());
    /*create EventGroup*/
    s_mesh_event_group = xEventGroupCreate();
    /*create queue for communication tx_task with measurement_task*/
    xQueueMeasurementTaskToMeshTXTask = xQueueCreate(QUEUE_LENGTH,READ_ITEM_SIZE);
    /*LED blink */
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true)
    {
       gpio_set_level(GPIO_NUM_2, level);
       level = !level;
       vTaskDelay(300 / portTICK_PERIOD_MS);
    }

}
