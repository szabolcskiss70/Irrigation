#include "pump_switching.h"
#include "pump_control.h"
#include "lora_comm.h"
#include <string.h>
#include "freertos/event_groups.h"


QueueHandle_t pump_request_queue;
SemaphoreHandle_t pump_array_mutex;

T_active_pump_suspended active_pump_suspended=NO; 
int running_pump_ID=-1;
int pump_num=0;
char* dual_mode_fla_str[modeLAST-ONLY_SLAVE_RELAY]={"ONLY_SLAVE_RELAY","CURR_PROTECTED","VIA_LORA_FUNC","CLONING"};
int dual_mode_flags=0;
bool Cloned_buffer_valid=false;
int other_pump(int id) {return((id==0)?1:0);}

bool is_low_prio_pump_running()
{
 if (pump_num<2) return false;
 if (running_pump_ID==-1) return false;
 return (getPUMP_prio(running_pump_ID)?false:true);
}

int sendcommandtoslave(char* msg)
{
 if (lora_comm_initialized)
	{	
   uint8_t lora_transmit_buf[256];
 	 sprintf((char*)lora_transmit_buf,"IRRMCMD_%lu_%s",xTaskGetTickCount(),msg); 
	 my_lora_send_packet(lora_transmit_buf,strlen((char*)lora_transmit_buf)); 
   // (xQueueReceive(lora_ans_evt_queue, &retval, 5*1000/portTICK_PERIOD_MS )==pdPASS)    return (retval); //portMAX_DELAY
   //else return (-1);
   return 1;
	}
  else return (-1);
}

void gen_switch_pump_id_to_state(int id, T_pump_states new_state)
{
  if (!get_remotePump(id)) switch_pump_id_to_state(id,new_state);
  else
  {
   if (get_autoSwitchON(id)) switch_pump_id_to_state(id,new_state); //switch local relay to power up
   {
    //send command via LoRa
    char cmd[256];
    sprintf(cmd,"switch_pump_id_to_state:%d",(int)new_state);
    sendcommandtoslave(cmd);
   }
  }
}




void process_pump_request(T_pump_switching_request pump_switching_request)
{
 int pump2switch; 
 switch (pump_switching_request.assigned_pump)
 {            
   case BOTH:
                if (running_pump_ID>-1) pump2switch=running_pump_ID;	 
                else
                {
                  int higher_prio_pump=0;
                  if ((pump_num==2) && (getPUMP_prio(1))) higher_prio_pump=1;
                  pump2switch=higher_prio_pump;
                  if ((get_pump_id_state(higher_prio_pump)==P_DISABLED) || (get_pump_id_state(higher_prio_pump)==P_SUSPENDED))
                  pump2switch=(higher_prio_pump==1)?0:1;
                 }
                 break;
   default:     pump2switch=pump_switching_request.assigned_pump;
                break;              
 }

 gen_switch_pump_id_to_state(pump2switch,pump_switching_request.state?P_ON:P_OFF);
}


time_t now_pump()
{
 time_t now;
 time(&now);
 return(now);
}

static void pump_switching_task(void* pvParameters)
{
 while (true)
 {
  T_pump_switching_request  request;
  if (xQueueReceive(pump_request_queue, &request, 0 )==pdPASS)  process_pump_request(request);
     
  if (active_pump_suspended!=NO) 
  {
    if (isPUMP_available(other_pump(active_pump_suspended))) 
    {
    gen_switch_pump_id_to_state(other_pump(active_pump_suspended),P_ON); //switch to an other pump
    active_pump_suspended=NO;
    }
  }
  /*if (is_low_prio_pump_running())
  {
    if (getPUMP_switchbackifavailable(other_pump(running_pump_ID)) && (now_pump()-pump[running_pump_ID].last_pump_on_time)>get_restart_delay(other_pump(running_pump_ID)))
      if (isPUMP_available(other_pump(running_pump_ID))) 
      {
        int other_pump_ID=other_pump(running_pump_ID);
        gen_switch_pump_id_to_state(running_pump_ID,P_OFF);
        gen_switch_pump_id_to_state(other_pump_ID,P_ON);
      }
  }*/
  vTaskDelay(1*1000 / portTICK_PERIOD_MS);
 }
}

int getINTvaluefromslave(char* msg)
{
 if (lora_comm_initialized)
	{	
   uint8_t lora_transmit_buf[256];
   int retval; 
 	 sprintf((char*)lora_transmit_buf,"IRRMGETI_%lu_%s",xTaskGetTickCount(),msg); 
	 my_lora_send_packet(lora_transmit_buf,strlen((char*)lora_transmit_buf)); 
   if (xQueueReceive(lora_ans_evt_queue, &retval, 5*1000/portTICK_PERIOD_MS )==pdPASS)    return (retval); //portMAX_DELAY
   else return (0);
	}
  else return (0);
}




int getpumpbufferfromslave()
{
 uint8_t lora_transmit_buf[256];
	if (lora_comm_initialized)
	{	
   int payloadlength=0; 
 	 sprintf((char*)lora_transmit_buf,"IRRMGETB_%lu_%s",xTaskGetTickCount(),"get_pump_id_struct"); 
	 my_lora_send_packet(lora_transmit_buf,strlen((char*)lora_transmit_buf)); 
   if (xQueueReceive(lora_ans_evt_queue, &payloadlength, 5*1000/portTICK_PERIOD_MS )==pdPASS) return (payloadlength); //portMAX_DELAY
   else return (-1);
	}
  else return -1;
}


T_pump_states get_pump_id_state_array(int id)
{
  T_pump_states retval;
  xSemaphoreTakeRecursive(pump_array_mutex, portMAX_DELAY);  
   retval=pump[id].status;
  xSemaphoreGiveRecursive(pump_array_mutex);
  return (retval);
}

static void pump_cloning_task(void* pvParameters)
{
 while (true)
 {
  //ESP_LOGI("DEBUG_TASK","getpumpbufferfromslave");
  int received_bytes=getpumpbufferfromslave();
  if (received_bytes!=sizeof(T_pump)) Cloned_buffer_valid=false;
  else Cloned_buffer_valid=true;
 }
 vTaskDelay(10*1000 / portTICK_PERIOD_MS);
}


void set_pump_default_params(int id,char* ldata)
{
	char Master_Slave;
	if(sscanf(ldata,"%c",&Master_Slave)==1) 
	{ 
    if (Master_Slave=='S')
	{
		enable_pump(id,true);
		setPUMP_prio(id,false);
		setPUMP_switchbackifavailable(id,false);
		set_restart_delay(id,15);
		set_flow_rate_protection_limit_dl_per_min(id,50);
		set_T_trip(id,260);
		set_T_reset(id,20);
		set_autoSwitchON(id,true);
	}
    else if (Master_Slave=='M')
	{
		enable_pump(id,true);
		setPUMP_prio(id,false);
		setPUMP_switchbackifavailable(id,false);
		set_restart_delay(id,15);
		set_flow_rate_protection_limit_dl_per_min(id,5);
		set_T_trip(id,150);
		set_T_reset(id,20);
	}
	
 }
}

void init_pump_switching(int ldual_mode_flags)
{
  pump_request_queue = xQueueCreate(10, sizeof(T_pump_switching_request));
  xTaskCreate(&pump_switching_task, "pump_switching_task", 4096, NULL, 5, NULL);
  if (ldual_mode_flags & (1<<CLONING)) xTaskCreate(&pump_cloning_task, "pump_cloning_task", 4096, NULL, 5, NULL);
  dual_mode_flags=ldual_mode_flags;
}

void force_gen_switch_pump_id_to_state(int id, T_pump_states new_state) {gen_switch_pump_id_to_state(id,  new_state);}

void init_single_pump(int id, int GPIO_PUMP, int GPIO_PROT,int GPIO_CNT,bool prio, bool switchbackifresumed,int ACS71020_address, bool pump_current_prot )
{
  init_pump( id,  GPIO_PUMP,  GPIO_PROT, GPIO_CNT, prio,  switchbackifresumed, ACS71020_address,pump_current_prot);  
  pump_num++; 
}

void get_LEVEL_string(char* result_string)
{
  *result_string=0;
  for (int id=0;id<pump_num;id++)	
  {
   get_LEVEL_string_for_id( id,result_string);
  }
}
void GetVolumeString(char *result_string)
{
  *result_string=0;
  for (int id=0;id<pump_num;id++)	
  {
    GetVolumeStringfor_pump(id,result_string);
  }
}

bool isPUMP_disabled_or_suspended()
{
  for (int id=0;id<pump_num;id++)	
  {
   if ((get_pump_id_state(id)!=P_DISABLED) && (get_pump_id_state(id)!=P_SUSPENDED)) return false;
  }
  return true;
}

/**
 * @brief check pump protection imputs and change pump states, needs to be called periodicaly
 * 
 * check status of max 2 pumps,
 * SUSPEND pump if protection needed, 
 * automatically activate 2nd pump if available
 * does not retsart the pump if resumed, it needs to be switched ON from higher level together with valves
 * 
 *
 * @return  higher state of available pumps
 */
T_pump_states check_pump_protection()
{
  T_pump_states retval;
  xSemaphoreTakeRecursive(pump_array_mutex, portMAX_DELAY);  
   retval=(pump_num==2)?(pump[0].status>pump[1].status)?pump[0].status:pump[1].status:pump[0].status;
  xSemaphoreGiveRecursive(pump_array_mutex);
  return (retval);
}

void clear_volumes_at_midnight()
{
 static time_t prev_now=0; 
 time_t now;
 time(&now);
 now-=1658700000;
 now%=86400; 
 
 if (now<prev_now)
 {
  for (int id=0;id<pump_num;id++)	
  {
   xSemaphoreTakeRecursive(pump_array_mutex, portMAX_DELAY);  
		pump[id].daily_pump_flowmeter_counts=0;
		pcnt_unit_clear_count(pump[id].pcnt_unit);
    pump[id].prev_daily_pump_flowmeter_counts_flowmeter=0;
    pump[id].prev_daily_pump_flowmeter_counts=0;
   xSemaphoreGiveRecursive(pump_array_mutex);

  }
 }
 prev_now=now;
}



int measure_flowrate()
{
  clear_volumes_at_midnight();
  if(!get_remotePump(running_pump_ID)) return(measure_flowrate_on_local_pump(0));
  else if (dual_mode_flags && (1<<CLONING)) 
 { // value cloned from slave to pump_array[2]
    if (!Cloned_buffer_valid) return 0;
    return (0); //TODO: develop flow rate from cloning 
 }
 else if (dual_mode_flags && (1<<VIA_LORA_FUNC)) return (getINTvaluefromslave("get_flow_rate"));
 else return 0;
}



