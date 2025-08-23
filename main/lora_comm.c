#include "lora_comm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "esp_log.h"
#include "ctype.h"
#include "mqtt_client.h"
#include "pump_control.h"


static SemaphoreHandle_t LORA_RX_TX_mutex;

static const char *TAG = "LORA";
uint8_t lora_transmit_buf[256];
uint8_t lora_receive_buf[256];
int INT_result;

extern  int my_esp_mqtt_client_publish(esp_mqtt_client_handle_t client, char* subtopic,const char* message,int par1, int par2, int par3);
extern esp_mqtt_client_handle_t mqtt_client;
extern bool Process_EVENT_DATA(char* ltopic, char* ldata, bool MQTT);
//typedef enum {P_OVER_CURRENT,P_FLOW_PROT,P_DISABLED, P_SUSPENDED,P_DELAY,P_OFF,P_RESUMED,P_ON} T_pump_states;
extern T_pump_states get_pump_id_state(int id);
extern char MQTT_BLE_answer[2048];




void init_lora()
{   ESP_LOGI("LORA","Start init lora");
	    LORA_RX_TX_mutex = xSemaphoreCreateMutex();
		int sendcount=0;
		int err=lora_init();
		ESP_LOGI("LORA","Init: %d",err);
		//lora_initialized();

		lora_set_frequency(433775000);
		lora_set_spreading_factor(12);
		lora_set_tx_power(17);
		lora_set_bandwidth(125000);
		lora_set_coding_rate(8);
		//lora_enable_crc();

        //lora_dump_registers();
		xTaskCreate(&task_rx, "task_rx", 4096, NULL, 5, NULL);

      if(false)
	  {
        lora_dump_registers();

 		ESP_LOGI("LORA","Start sending packets");
        for (sendcount=0;sendcount<1000;sendcount++)
		{
		sprintf((char*)lora_receive_buf,"TEst%d",sendcount);
		/*lora_receive_buf[0]='T';
		lora_receive_buf[1]='E';
		lora_receive_buf[2]='S';
		lora_receive_buf[3]='T';
		lora_receive_buf[4]=0;*/
        
		xSemaphoreTake(LORA_RX_TX_mutex, portMAX_DELAY);
         lora_send_packet(lora_receive_buf,sizeof(lora_receive_buf)-1);
		xSemaphoreGive(LORA_RX_TX_mutex);

		ESP_LOGI("LORA","package%d sent",sendcount);
		vTaskDelay(2*1000 / portTICK_PERIOD_MS);
		}
	 }
	}	

extern void to_upper(const char *str, char *out_str);

void task_rx(void *p)
{
   int x;
   for(;;) {
	  xSemaphoreTake(LORA_RX_TX_mutex, portMAX_DELAY);

      lora_receive();    // put into receive mode
      while(lora_received()) {
		 int port;
		 int value;
         x = lora_receive_packet(lora_receive_buf, sizeof(lora_receive_buf)-1);
         lora_receive_buf[x] = 0;
         printf("Received: %s\n", lora_receive_buf);
		 ESP_LOGI(TAG, "Received: %s\n", lora_receive_buf);
		 //if(sscanf((char*)lora_receive_buf,"DIO%d:%d",&port,&value)==2) writeDO(port,(value==1)?true:false);
		 {  
			int intval;
			char ltopic[256];
			char ldata[256];
			unsigned long tick;
			memset(ltopic,0,sizeof(ltopic));
			memset(ldata,0,sizeof(ldata));
    		if (sscanf((char*)lora_receive_buf,"IRRMOSI_%lu_%[^=]=%[^\n]",&tick,ltopic,ldata)==3)
			{
			 to_upper(ltopic,ltopic);
			 Process_EVENT_DATA(ltopic,ltopic,false);
			 sprintf((char*)lora_transmit_buf,"IRRMISO_%.240s DONE",MQTT_BLE_answer); 
			 lora_send_packet(lora_transmit_buf,strlen((char*)lora_transmit_buf)); 
			}
			if (sscanf((char*)lora_receive_buf,"IRRMISO_%lu_%[^=]=%[^\n]",&tick,ltopic,ldata)==3)
			{
				 my_esp_mqtt_client_publish(mqtt_client, "SLAVE/ACK", (char*)lora_receive_buf, 0, 0, 0);   //Qos=1; retain=1
			}
			else if (sscanf((char*)lora_receive_buf,"IRRMGETI_%lu_%s",&tick,ldata)==2)
			{
			 if (strcmp(ldata,"get_pump_id_state")==0)
			 {
			  sprintf((char*)lora_transmit_buf,"IRRSRETI_%lu_%d",tick,get_pump_id_state(0)); 
			  lora_send_packet(lora_transmit_buf,strlen((char*)lora_transmit_buf)); 
			 }
			 else if (strcmp(ldata,"get_pump_id_struct")==0)
			 {
			  int sizeT_pump=sizeof(T_pump);
			  sprintf((char*)lora_transmit_buf,"IRRSRETB_%lu_",tick);
			  int header_length=strlen((char*)lora_transmit_buf);
			  memcpy(lora_transmit_buf+header_length,&pump[0],sizeT_pump);
			  lora_send_packet(lora_transmit_buf,header_length+sizeT_pump); 
			 }
			}
			else if (sscanf((char*)lora_receive_buf,"IRRSRETI_%lu_%d",&tick,&intval)==2)
			{
			 INT_result=intval; 
			}
			 else if (sscanf((char*)lora_receive_buf,"IRRSRETB_%lu_",&tick)==1)
			{
				memcpy(&pump[1],lora_receive_buf+x-sizeof(T_pump),sizeof(T_pump));	 
			}

		

		 }
		 lora_receive();
      }
     xSemaphoreGive(LORA_RX_TX_mutex);

      vTaskDelay(1);
   }
}

 

void my_lora_send_packet(uint8_t *buf, int size)
{
	xSemaphoreTake(LORA_RX_TX_mutex, portMAX_DELAY);
	 lora_send_packet(buf, size);
 	xSemaphoreGive(LORA_RX_TX_mutex);
}


int my_lora_packet_rssi()
{
	int retval;
	xSemaphoreTake(LORA_RX_TX_mutex, portMAX_DELAY);
	 retval=lora_packet_rssi();
 	xSemaphoreGive(LORA_RX_TX_mutex);
	return (retval);
}