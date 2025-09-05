#include "lora_comm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "esp_log.h"
#include "ctype.h"
#include "mqtt_client.h"
#include "pump_control.h"
#include "pump_switching.h"


static SemaphoreHandle_t LORA_RX_TX_mutex;
QueueHandle_t lora_ans_evt_queue;

static const char *TAG = "LORA";
uint8_t lora_transmit_buf[256];
uint8_t lora_receive_buf[256];
bool lora_comm_initialized=false;
//int INT_result;

extern  int my_esp_mqtt_client_publish(esp_mqtt_client_handle_t client, char* subtopic,const char* message,int par1, int par2, int par3);
extern esp_mqtt_client_handle_t mqtt_client;
extern bool Process_EVENT_DATA(char* ltopic, char* ldata, bool MQTT);
//typedef enum {P_OVER_CURRENT,P_FLOW_PROT,P_DISABLED, P_SUSPENDED,P_DELAY,P_OFF,P_RESUMED,P_ON} T_pump_states;
extern T_pump_states get_pump_id_state(int id);
extern int measure_flowrate_on_local_pump(int running_pump_ID);
extern char MQTT_BLE_answer[2048];
char *lora_cmd_str[lget_pump_id_struct+1]={"state","rate","PCNT","struct"};





int init_lora()
{   ESP_LOGI("LORA","Start init lora");
	    LORA_RX_TX_mutex = xSemaphoreCreateMutex();
		lora_ans_evt_queue = xQueueCreate(10, sizeof(uint32_t));
		int sendcount=0;
		int err=lora_init();
		ESP_LOGI("LORA","Init: %d",err);
		if (err!=1) return (err);
		//lora_comm_initialized();

		lora_set_frequency(433775000);
		lora_set_spreading_factor(10);
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
	lora_comm_initialized=true;
	return (1);
	}	

extern void to_upper(const char *str, char *out_str);

void task_rx(void *p)
{
   int x;
   for(;;) {
	  xSemaphoreTake(LORA_RX_TX_mutex, portMAX_DELAY);

      lora_receive();    // put into receive mode
      while(lora_received()) {
         x = lora_receive_packet(lora_receive_buf, sizeof(lora_receive_buf)-1);
         lora_receive_buf[x] = 0;
         printf("Received: %s\n", lora_receive_buf);
		 ESP_LOGI(TAG, "Received: %s\n", lora_receive_buf);
		 //if(sscanf((char*)lora_receive_buf,"DIO%d:%d",&port,&value)==2) writeDO(port,(value==1)?true:false);
		 {  
			int intval;
			char ltopic[256];
			char ldata[256];
			int length=0;
			unsigned long tick;
			memset(ltopic,0,sizeof(ltopic));
			memset(ldata,0,sizeof(ldata));
    		if (sscanf((char*)lora_receive_buf,"IRRMOSI_%lu_%d:%[^=]=%[^\n]",&tick,&length,ltopic,ldata)>=3)
			{
			 to_upper(ltopic,ltopic);
			 Process_EVENT_DATA(ltopic,ldata,false);
			 sprintf((char*)lora_transmit_buf,"IRRMISO_%lu_%d:%.220s",tick,strlen(MQTT_BLE_answer),MQTT_BLE_answer); 
			 lora_send_packet(lora_transmit_buf,strlen((char*)lora_transmit_buf)); 
			}
			if (sscanf((char*)lora_receive_buf,"IRRMISO_%lu_%d:%s",&tick,&length,ltopic)==3)
			{
				 my_esp_mqtt_client_publish(mqtt_client, "SLAVE/ACK", (char*)lora_receive_buf, 0, 0, 0);   //Qos=1; retain=1
			}
			else if (sscanf((char*)lora_receive_buf,"IRRMCMD_%lu_%d:%s",&tick,&length,ldata)==3)
			{
			 if (sscanf(ldata,"switch_pump_id_to_state:%d",&intval)==1)
			 {
				switch_pump_id_to_state(0,(T_pump_states)intval);
			 }	
			}

			else if (sscanf((char*)lora_receive_buf,"IRRMGETI_%lu_%d:%s",&tick,&length,ldata)==3)
			{
			 if (strcmp(ldata,lora_cmd_str[lget_pump_id_state])==0)
			 {
			  char result[16];
			  sprintf(result,"%d",get_pump_id_state(0));
			  sprintf((char*)lora_transmit_buf,"IRRSRETI_%lu_%d:%s",tick,strlen(result),result); 
			  lora_send_packet(lora_transmit_buf,strlen((char*)lora_transmit_buf)); 
			 }
			 else if (strcmp(ldata,lora_cmd_str[lget_flow_rate])==0)
			 {
			  char result[16];
			  sprintf(result,"%d",measure_flowrate_on_local_pump(0));	
			  sprintf((char*)lora_transmit_buf,"IRRSRETI_%lu_%d:%s",tick,strlen(result),result); 
			  lora_send_packet(lora_transmit_buf,strlen((char*)lora_transmit_buf)); 
			 }
			 else if (strcmp(ldata,lora_cmd_str[lget_PCNT])==0)
			 {
			  int CNT;
			  xSemaphoreTakeRecursive(pump_array_mutex, portMAX_DELAY);
			   get_CNT_from_flowmeter(pump[0].pcnt_unit,&CNT);
			  xSemaphoreGiveRecursive(pump_array_mutex);
			  char result[16];
			  sprintf(result,"%d",CNT);
	     	  sprintf((char*)lora_transmit_buf,"IRRSRETI_%lu_%d:%s",tick,strlen(result),result); 
			  lora_send_packet(lora_transmit_buf,strlen((char*)lora_transmit_buf)); 
			 }		 
			 else if (sscanf((char*)lora_receive_buf,"IRRMGETB_%lu_%d:%s",&tick,&length,ldata)==3)
			{
			 if (strcmp(ldata,lora_cmd_str[lget_pump_id_struct])==0)
			 {
			  int sizeT_pump=sizeof(T_pump);
			  sprintf((char*)lora_transmit_buf,"IRRSRETB_%lu_%d_",tick,sizeT_pump);
			  int header_length=strlen((char*)lora_transmit_buf);
			  xSemaphoreTakeRecursive(pump_array_mutex, portMAX_DELAY);
	 			  memcpy(lora_transmit_buf+header_length,&pump[0],sizeT_pump);
			  xSemaphoreGiveRecursive(pump_array_mutex);
			  lora_send_packet(lora_transmit_buf,header_length+sizeT_pump); 
			 }
			}
			}
			else if (sscanf((char*)lora_receive_buf,"IRRSRETI_%lu_%d:%d",&tick,&length,&intval)==3)
			{
			 xQueueSend(lora_ans_evt_queue, &intval, NULL);
			}
			else if (sscanf((char*)lora_receive_buf,"IRRSRETB_%lu_%d",&tick,&length)==2)
			{
    			xSemaphoreTakeRecursive(pump_array_mutex, portMAX_DELAY);
				    char *ptr0,*ptr1,*ptr2;
					ptr0=&lora_receive_buf;
					int payload_length=0;
					if ((ptr1=strchr((char*)lora_receive_buf,'_'))!=NULL)
					{
					 if ((ptr2=strchr(ptr1+1,'_'))!=NULL)	
					 {
						payload_length=x-(ptr2-ptr0+1);
						memcpy(&pump[2],ptr2+1,payload_length);
					 }
					}


				xSemaphoreGiveRecursive(pump_array_mutex);
				xQueueSend(lora_ans_evt_queue, &payload_length, NULL);
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
	if (!lora_comm_initialized) return;
	xSemaphoreTake(LORA_RX_TX_mutex, portMAX_DELAY);
	 lora_send_packet(buf, size);
 	xSemaphoreGive(LORA_RX_TX_mutex);
}


int my_lora_packet_rssi()
{
	int retval;
	if (!lora_comm_initialized) return (-200);
	xSemaphoreTake(LORA_RX_TX_mutex, portMAX_DELAY);
	 retval=lora_packet_rssi();
 	xSemaphoreGive(LORA_RX_TX_mutex);
	return (retval);
}


float my_lora_packet_snr()
{
	float retval;
	if (!lora_comm_initialized) return (0);
	xSemaphoreTake(LORA_RX_TX_mutex, portMAX_DELAY);
	 retval=lora_packet_snr();
 	xSemaphoreGive(LORA_RX_TX_mutex);
	return (retval);
}