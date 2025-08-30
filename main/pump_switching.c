#include "pump_switching.h"
#include "pump_control.h"
#include "lora_comm.h"
#include <string.h>

T_active_pump_suspended active_pump_suspended=NO; 
int running_pump_ID=-1;
int pump_num=0;

int other_pump(int id) {return((id==0)?1:0);}

bool is_low_prio_pump_running()
{
 if (pump_num<2) return false;
 if (running_pump_ID==-1) return false;
 return (pump[running_pump_ID].prio?false:true);
}


void switch_pump(bool on_state, T_pump_list assigned_pump)
{
 int pump2switch; 
 switch (assigned_pump)
 {            
   case BOTH:
                if (running_pump_ID>-1) pump2switch=running_pump_ID;	 
                else
                {
                  int higher_prio_pump=0;
                  if ((pump_num==2) && (pump[1].prio)) higher_prio_pump=1;
                  pump2switch=higher_prio_pump;
                  if ((pump[higher_prio_pump].status==P_DISABLED) || (pump[higher_prio_pump].status==P_SUSPENDED))
                  pump2switch=(higher_prio_pump==1)?0:1;
                 }
                 break;
   default:     pump2switch=assigned_pump;
                break;              
 }

 switch_pump_id_to_state(pump2switch,on_state?P_ON:P_OFF);
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
  if (active_pump_suspended!=NO) 
  {
    if (isPUMP_available(other_pump(active_pump_suspended))) 
    {
    switch_pump_id_to_state(other_pump(active_pump_suspended),P_ON); //switch to an other pump
    active_pump_suspended=NO;
    }
  }
  if (is_low_prio_pump_running())
  {
    if ((pump[other_pump(running_pump_ID)].switchbackifavailable) && (now_pump()-pump[running_pump_ID].last_pump_on_time)>pump[other_pump(running_pump_ID)].pump_restart_delay)
      if (isPUMP_available(other_pump(running_pump_ID))) 
      {
        int other_pump_ID=other_pump(running_pump_ID);
        switch_pump_id_to_state(running_pump_ID,P_OFF);
        switch_pump_id_to_state(other_pump_ID,P_ON);
      }
  }
  vTaskDelay(1*1000 / portTICK_PERIOD_MS);
 }
}

int getvaluefromslave(char* msg)
{
 uint8_t lora_transmit_buf[256];
	//if (run_mode & (1<<USE_LORA))
	{	
 	 sprintf((char*)lora_transmit_buf,"IRRMGETI_%lu_%s",xTaskGetTickCount(),msg); 
	 my_lora_send_packet(lora_transmit_buf,strlen((char*)lora_transmit_buf)); 
   //wait4notify
   return (INT_result);
	}
}

void getpumpbufferfromslave()
{
 uint8_t lora_transmit_buf[256];
	//if (run_mode & (1<<USE_LORA))
	{	
 	 sprintf((char*)lora_transmit_buf,"IRRMGETB_%lu_%s",xTaskGetTickCount(),"get_pump_id_struct"); 
	 my_lora_send_packet(lora_transmit_buf,strlen((char*)lora_transmit_buf)); 
   //wait4notify
	}
  vTaskDelay(5*1000 / portTICK_PERIOD_MS);
}


T_pump_states get_pump_id_state_array(int id)
{
  return (pump[id].status);
}







static void pump_cloning_task(void* pvParameters)
{
 while (true)
 {
  //ESP_LOGI("DEBUG_TASK","getpumpbufferfromslave");
  getpumpbufferfromslave();
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

void init_pump_switching()
{
  xTaskCreate(&pump_switching_task, "pump_switching_task", 4096, NULL, 5, NULL);
  xTaskCreate(&pump_cloning_task, "pump_cloning_task", 4096, NULL, 5, NULL);
}

void force_switch_pump_id_to_state(int id, T_pump_states new_state) {switch_pump_id_to_state(id,  new_state);}

void init_single_pump(int id, int GPIO_PUMP, int GPIO_PROT,int GPIO_CNT,bool prio, bool switchbackifresumed,int ACS71020_address )
{
  init_pump( id,  GPIO_PUMP,  GPIO_PROT, GPIO_CNT, prio,  switchbackifresumed, ACS71020_address);  
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
 return ((pump_num==2)?(pump[0].status>pump[1].status)?pump[0].status:pump[1].status:pump[0].status);
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
		pump[id].daily_pump_flowmeter_counts=0;
		pcnt_unit_clear_count(pump[id].pcnt_unit);
    pump[id].prev_daily_pump_flowmeter_counts_flowmeter=0;
    pump[id].prev_daily_pump_flowmeter_counts=0;
  }
 }
 prev_now=now;
}



int measure_flowrate()
{
  clear_volumes_at_midnight();
  if(running_pump_ID==0)  return( measure_flowrate_on_local_pump(0));
  else if(running_pump_ID==1) return (getvaluefromslave("get_flow_rate"));
  else return 0;
}