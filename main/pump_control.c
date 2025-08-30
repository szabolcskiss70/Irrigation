#include "pump_control.h"
#include <string.h>
#include "esp_timer.h"
#include "DIO.h"
#include "ACS71020.h"
#include "pump_current_protection.h"
#include "pump_switching.h"
#include "pump_params.h"

extern SemaphoreHandle_t I2C_mutex;

extern void Write_Msg_toDisplay(int line, char *Msg);
extern float water_level;
extern bool ACS71020_initialized;



void Chek_pump_current_and_flow_rate_task(void *pvParameters);
static void level_switch_monitoring_task(void* pvParameters);
extern bool motor_protect_func(float I,float T_trip,float T_reset,int looptime_ms,bool pumpstatus, float* T_max);


T_pump pump[3];
T_pump_status_changes pump_status_changes[3];

char* PUMP_status_str[]={"P_UNKNOWN","PROT_T_TRIP","FLOW_PROT","UNDERVOLTAGE","PROT_T_RESET","DISABLED","SUSPENDED","DELAY","OFF","RESUMED","ON"};

static const char *TAG = "PUMP";

T_pump_states get_pump_id_state(int id)
{
 if (!get_remotePump(id)) return (pump[id].status);
 return((T_pump_states) getINTvaluefromslave("get_pump_id_state"));
}

/**
 * @brief Switch pump relay ON/OFF for pump id
 *
 * protects switching ON in case pump status is not allows to activate pump.
 * pump on time is updated and flow meter cnt value is saved 
 *
 * @return  void
 */
void switch_pump_ch_relay(int id,bool on_state)
{
 if ((get_pump_id_state(id)==P_SUSPENDED) || (get_pump_id_state(id)==P_DISABLED)|| (get_pump_id_state(id)==P_DELAY)) on_state=false; 
 
 if (on_state) 
 {
  if (running_pump_ID!=id)
  {//not running yet
   if (get_remotePump(id)) 
   {
    int CNT=getINTvaluefromslave("get_PCNT");
    if (CNT>-1) pump[id].cnt_at_pump_start=CNT;
   }
   else get_CNT_from_flowmeter(pump[id].pcnt_unit,&pump[id].cnt_at_pump_start);
	 pump[id].last_pump_on_time=now_pump();
  }
 } 
	running_pump_ID=(on_state==true)?id:-1;
  writeDO(pump[id].GPIO_PUMP, (on_state==true)?1:0);	
  pump[id].pump_running=on_state;
 
}



/**
 * @brief Switch pump id to new state
 *
 * save timestamps of changes
 * call switch_pump_ch_relay to switching IO
 *
 * @return  void
 */
void switch_pump_id_to_state(int id, T_pump_states new_state)
{
 pump[id].status=new_state;
 pump_status_changes[id].status_change_time[new_state]=now_pump();
 for (int state=PROT_T_TRIP;state<=P_UNDERVOLTAGE;state++) if(state==new_state) {pump[id].suspend_reason|=(1<<state);break;}
 
 switch (new_state)
 {
  case P_ON:              pump[id].T_max=0;
                          pump[id].I_max=0;
                          pump[id].Flow_CNT_at_err=1000;
                          pump[id].suspend_reason=0;
                          pump[id].just_turned_on=true;
                          switch_pump_ch_relay(id,true);	 
                          break;
  default:                switch_pump_ch_relay(id,false);	           
                          break;                
 }
}

void init_pump(int id, int GPIO_PUMP, int GPIO_PROT,int GPIO_CNT,bool prio, bool switchbackifavailable,int ACS71020_address)
{
  char pump_prot_task_name[32];
	pump[id].ID=id;
  pump[id].remote_pump=false;
  pump[id].just_turned_on=false;
  pump[id].pump_running=false;
	pump[id].GPIO_PUMP=GPIO_PUMP;
	pump[id].GPIO_PROT=GPIO_PROT;
	pump[id].GPIO_CNT=GPIO_CNT;
  pump[id].ACS71020_address=ACS71020_address;
  pump[id].max_current=5.0; 
  pump[id].T_trip=150.0; 
  pump[id].T_reset=80.0; 
  pump[id].T_max=0.0; 
  pump[id].I_max=0.0; 
  pump[id].last_T=0.0; 
  pump[id].sink_time=0;
  pump[id].fill_time=0;	
	pump[id].last_pump_on_time=0;
  pump[id].protection_level_off=0;
  pump[id].protection_level_on=0;
	pump[id].pump_restart_delay=10;
  pump[id].prio=prio;
	pump[id].switchbackifavailable=switchbackifavailable;
  pump[id].daily_pump_flowmeter_counts=0;
  pump[id].prev_daily_pump_flowmeter_counts=0;
  pump[id].prev_daily_pump_flowmeter_counts_flowmeter=0;
  pump[id].flow_rate_protection_limit_dl_per_min=5;
  pump[id].Auto_switch_on_if_powered=false;
  pump[id].suspend_reason=0;
	if (GPIO_CNT!=-1) install_pcnt(id, &pump[id].pcnt_unit,pump[id].GPIO_CNT);
  if (GPIO_PUMP!=-1) set_DIO_direction(GPIO_PUMP,GPIO_MODE_OUTPUT);
  if (GPIO_PROT!=-1)
  {
   set_DIO_interrupt(GPIO_PROT,GPIO_MODE_INPUT,GPIO_INTR_ANYEDGE);
   sprintf(pump_prot_task_name,"pump%d_level_task",id);
   xTaskCreate(&level_switch_monitoring_task, "pump_prot_task_name", 4096, &pump[id], 10, NULL);
  }
  
  if (GPIO_PUMP!=-1)
  {
   sprintf(pump_prot_task_name,"pump%d_current_task",id);
   xTaskCreate(&Chek_pump_current_and_flow_rate_task, pump_prot_task_name, 4096, &pump[id], 10, &pump[id].CurrentMonitoringTaskHAndle);
   switch_pump_id_to_state(id,P_OFF);
  }
}


void enable_pump(int ch,bool enable)
{
  if (enable) switch_pump_id_to_state(ch,P_OFF); 
  else switch_pump_id_to_state(ch,P_DISABLED);
}


/**
 * @brief Switch pump higher level
 * 
 * act as one pump but handles max 2 pumps as group
 * Selects higher prio pump if possible
 * use 2nd pump if installed and higher prio suspended or disabled
 * 
 *
 * @return  void
 */


float convertCNT2Liter(int delta_volume_cnt)
{
  return(1.0*delta_volume_cnt/YF_DN32_PULSE_PER_LITER);
}


void GetPumpStatusString(int id, char* message, int buf_size)
{
  char susp_reason_str[64];
  susp_reason_str[0]=0;
  for (int state=PROT_T_TRIP;state<=P_UNDERVOLTAGE;state++) if(pump[id].suspend_reason & (1<<state)) {strcat(susp_reason_str," ");strcat(susp_reason_str,PUMP_status_str[state]);} 
 if (pump[id].GPIO_CNT!=-1) pcnt_unit_get_count(pump[id].pcnt_unit, &pump[id].daily_pump_flowmeter_counts);
 else pump[id].daily_pump_flowmeter_counts=0;
 char *MsgFormat= "P%d: Status:%s, Daily Volume:%1.1fl, Tmax:%1.1fC° Imax:%1.1fA Ttrip:%1.1fC° Treset:%1.1fC° FlowRate_min:%ddl/min, restart delay:%dmin auto_switch_on:%d remote:%d, CNT@low_flow:%d susp_res:%s\n";
 if (buf_size>strlen(MsgFormat)+16) sprintf(message,MsgFormat,id+1,PUMP_status_str[get_pump_id_state(id)],convertCNT2Liter(pump[id].daily_pump_flowmeter_counts),pump[id].T_max,pump[id].I_max,get_T_trip(id),get_T_reset(id),get_flow_rate_protection_limit_dl_per_min(id),get_restart_delay(id),get_autoSwitchON(id),get_remotePump(id),pump[id].Flow_CNT_at_err,susp_reason_str);
}

void getpumptimechanges(int id, char* message, int buf_size)
{
			for (int i=P_UNKNOWN;i<=P_ON;i++)
			{
				if (buf_size>64) sprintf(message+strlen(message),"ID:%d %s: %lld\n",id,PUMP_status_str[i],pump_status_changes[id].status_change_time[i]);
			}
}

void set_restart_delay(int id, int restart_delay)
{
 pump[id].pump_restart_delay=restart_delay;
}

int get_restart_delay(int id)
{
  return (pump[id].pump_restart_delay);
}
  
void get_LEVEL_string_for_id(int id,char* result_string)
{
	sprintf(result_string+strlen(result_string),"%0.1fcm off=%0.1fcm on=%0.1fcm fill=%ds sink=%ds \n",water_level,pump[id].protection_level_off,pump[id].protection_level_on,(int)pump[id].fill_time,(int)pump[id].sink_time); 
}

int getsinktime(int id)
{
  return ((int)pump[id].sink_time);
}

int getfilltime(int id)
{
  return ((int)pump[id].fill_time);
}


int measure_flowrate_on_local_pump(int pump_ID)
{
  if(pump_ID!=-1)
  {  
  static uint64_t TimePastVolumeMeasured=0;
	if(TimePastVolumeMeasured==0)  TimePastVolumeMeasured=esp_timer_get_time();
	uint64_t Volume_measure_delta_time; 
  
	int delta_volume_cnt1=0;
  int delta_volume_cnt2=0;
	ESP_ERROR_CHECK(pcnt_unit_get_count(pump[pump_ID].pcnt_unit, &pump[pump_ID].daily_pump_flowmeter_counts));
  delta_volume_cnt1=pump[pump_ID].daily_pump_flowmeter_counts-pump[pump_ID].prev_daily_pump_flowmeter_counts;
  pump[pump_ID].prev_daily_pump_flowmeter_counts=pump[pump_ID].daily_pump_flowmeter_counts;
  delta_volume_cnt2=pump[pump_ID].daily_pump_flowmeter_counts-pump[pump_ID].prev_daily_pump_flowmeter_counts_flowmeter;
  
	
	if (((Volume_measure_delta_time=(esp_timer_get_time() - TimePastVolumeMeasured)) >= Volume_measure_interval_us) && (delta_volume_cnt2>5))
    {
	   char message[32];  
     float volume_rate_liter_per_min;	
     volume_rate_liter_per_min= 60*convertCNT2Liter(delta_volume_cnt2)/(1.0*Volume_measure_delta_time/1000000.0);
     sprintf(message,"%0.1f l/min %0.1f l",volume_rate_liter_per_min,convertCNT2Liter(pump[pump_ID].daily_pump_flowmeter_counts-pump[pump_ID].cnt_at_pump_start));
     Write_Msg_toDisplay(5,message);
     pump[pump_ID].prev_daily_pump_flowmeter_counts_flowmeter=pump[pump_ID].daily_pump_flowmeter_counts;
	   TimePastVolumeMeasured = esp_timer_get_time(); // get next publish time
    }
	return delta_volume_cnt1;
  }
  else return 0;
}





bool check_flowrate(int pump_id,int looptime_ms) 
{
  int delta_cnt=0;
  static int lastCNT=-162;
  int actCNT;
	ESP_ERROR_CHECK(pcnt_unit_get_count(pump[pump_id].pcnt_unit, &actCNT));
  delta_cnt=actCNT-lastCNT;
  lastCNT=actCNT;
  
  int limit=pump[pump_id].flow_rate_protection_limit_dl_per_min/10*YF_DN32_PULSE_PER_LITER/60*looptime_ms/1000;
	ESP_LOGI("DEBUG_TASK", "delta_cnt:%d limit:%d, looptime:%dms",delta_cnt,limit,looptime_ms);
  if(delta_cnt<limit) pump[pump_id].Flow_CNT_at_err=delta_cnt;
  return (delta_cnt>=limit);
}






void GetVolumeStringfor_pump(int id,char *result_string)
{
	sprintf(result_string+strlen(result_string),"Avg: %0.1f l/min",60*convertCNT2Liter(pump[id].daily_pump_flowmeter_counts-pump[id].cnt_at_pump_start)/(1.0*(now_pump() - pump[id].last_pump_on_time))); 
}


bool isPUMP_disabled_local(int id) {return(get_pump_id_state_array(id)==P_DISABLED);}


bool isPUMP_disabled(int id) {return(get_pump_id_state(id)==P_DISABLED);}

bool isPUMP_available(int id) {return(get_pump_id_state(id)>=P_OFF);}


bool getPUMP_prio(int id) {return(pump[id].prio);}
void setPUMP_prio(int id, bool val) {pump[id].prio=val;pump[other_pump(id)].prio=!val;}
bool getPUMP_switchbackifavailable(int id) {return(pump[id].switchbackifavailable);}
void setPUMP_switchbackifavailable(int id, bool val) {pump[id].prio=val;}

bool get_autoSwitchON(int id) {return(pump[id].Auto_switch_on_if_powered);}
void set_autoSwitchON(int id, bool val) {pump[id].Auto_switch_on_if_powered=val;}

bool get_remotePump(int id) {return(pump[id].remote_pump);}
void set_remotePump(int id, bool val) {pump[id].remote_pump=val;}


float getsinkvolume(int id) {return pump[id].sink_volume;}

float get_max_current(int id) {return pump[id].max_current;}
void  set_max_current(int id, float imax) 
{
  pump[id].max_current=imax;
  pump[id].T_trip=imax*imax*R_eq*R_th*6/5.5; 

}

float get_T_trip(int id) {return pump[id].T_trip;}
void  set_T_trip(int id, float T_trip) {pump[id].T_trip=T_trip;}

float get_T_reset(int id) {return pump[id].T_reset;}
void  set_T_reset(int id, float T_reset) {pump[id].T_reset=T_reset;}




void  set_flow_rate_protection_limit_dl_per_min(int id, int flow_min_dlper_min) {pump[id].flow_rate_protection_limit_dl_per_min=flow_min_dlper_min;}
int  get_flow_rate_protection_limit_dl_per_min(int id) {return(pump[id].flow_rate_protection_limit_dl_per_min);}





void check_pump_protection_GPIO_input(int id)
        {
         if(readDI(pump[id].GPIO_PROT))
          {
              if(get_pump_id_state(id)!=P_SUSPENDED)
              {		
              ESP_LOGI("PUMP","PUMP SUSPENDED");  
              pump[id].pump_protection_started_at=now_pump();
              pump[id].protection_level_off=water_level;
              pump[id].sink_time=now_pump()-pump[id].last_pump_on_time;
              pcnt_unit_get_count(pump[id].pcnt_unit, &pump[id].cnt_at_pump_suspend);
              pump[id].sink_volume=convertCNT2Liter(pump[id].cnt_at_pump_suspend-pump[id].cnt_at_pump_start);
              }
              if (running_pump_ID==id) active_pump_suspended=id;
              switch_pump_id_to_state(id,P_SUSPENDED);
          }
	        else
          {
            if	(get_pump_id_state(id)==P_SUSPENDED)
            {
              pump[id].protection_level_on=water_level;
              pump[id].fill_time=now_pump()-pump[id].pump_protection_started_at;
            }
            
            if	(get_pump_id_state(id)==P_SUSPENDED || get_pump_id_state(id)==P_DELAY)
            {

              if((now_pump()-pump[id].pump_protection_started_at)/60>=pump[id].pump_restart_delay)
              {
                ESP_LOGI(TAG,"PUMP_RESUMED");
                Write_Msg_toDisplay(2,"pump resumed");
                switch_pump_id_to_state(id,P_RESUMED);
              }
              else {
                char message[32];
                sprintf(message,"waiting:%llds",(int)pump[id].pump_restart_delay*60+pump[id].pump_protection_started_at-now_pump()); 
                ESP_LOGI(TAG,"WAITING FOR RESTART DELAY");
                Write_Msg_toDisplay(2,message);
                pump[id].protection_level_on=water_level;
                pump[id].fill_time=now_pump()-pump[id].pump_protection_started_at;
                switch_pump_id_to_state(id,P_DELAY);
              }
            }	
            vTaskDelay(1000 / portTICK_PERIOD_MS);
          }
        }






static void level_switch_monitoring_task(void* pvParameters)
{
  T_pump *actpump= (T_pump *)pvParameters;
  uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)==pdPASS) 
        {
          //ESP_LOGI("DEBUG_TASK", "GPIO[%"PRIu32"] intr, val: %d,pumpID:%d\n", io_num, gpio_get_level(io_num),actpump->ID);
          check_pump_protection_GPIO_input(actpump->ID);
        }
        //else ESP_LOGI("DEBUG_TASK", "level_switch_monitoring_task is waiting");
    }
}


void Chek_pump_current_and_flow_rate_task(void *pvParameters)
{
 T_pump *actpump= (T_pump *)pvParameters;
 //ESP_LOGI("DEBUG_TASK", "Chek_pump_current_and_flow_rate_task for pumpID:%d",actpump->ID);
 TickType_t xLastWakeTime;
 const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS; 
 xLastWakeTime = xTaskGetTickCount();
 for(long run_cnt=1;;run_cnt++)
 {
  vTaskDelayUntil( &xLastWakeTime, xFrequency );
  //ESP_LOGI("DEBUG_TASK", "Chek_pump_current_and_flow_rate_task for pumpID:%d",actpump->ID);
  if (ACS71020_initialized)
  {
   xSemaphoreTake(I2C_mutex, portMAX_DELAY);
   double irms= MeasuredValue(actpump->ACS71020_address, 0x20, 0x7fff0000, 0,16,14,30.0);
   double urms= MeasuredValue(actpump->ACS71020_address, 0x20, 0x00007fff, 0, 0,15,0.275*(R1_4+Rs)/Rs); 
  //double p=    MeasuredValue(actpump->ACS71020_address, 0x28, 0x0001ffff,15, 0,15,30.0*0.275*(R1_4+Rs)/Rs);
   xSemaphoreGive(I2C_mutex);
   if (actpump->I_max<irms ) actpump->I_max=irms;
  //ESP_LOGI("DEBUG_TASK", "irms:%lf limit:%f",irms,actpump->max_current);
   if (urms<180) switch_pump_id_to_state(actpump->ID,P_UNDERVOLTAGE); 
   else if(get_pump_id_state(actpump->ID)==P_UNDERVOLTAGE) switch_pump_id_to_state(actpump->ID,P_DELAY);

   if (!motor_protect_func(irms,actpump->T_trip,actpump->T_reset,xFrequency*portTICK_PERIOD_MS,get_pump_id_state(actpump->ID)==P_ON, &pump[actpump->ID].T_max)) switch_pump_id_to_state(actpump->ID,PROT_T_TRIP);
   else if(get_pump_id_state(actpump->ID)==PROT_T_TRIP) switch_pump_id_to_state(actpump->ID,PROT_T_RESET);
   else if(get_pump_id_state(actpump->ID)==PROT_T_RESET) switch_pump_id_to_state(actpump->ID,P_DELAY);
  }


  if (pump[actpump->ID].GPIO_CNT!=-1)
  {
    int limit,sec;
    for (sec=1;sec<20;sec++) if ((limit=sec*pump[actpump->ID].flow_rate_protection_limit_dl_per_min/10*YF_DN32_PULSE_PER_LITER/60)>=2) break;	
    //ESP_LOGI("TEST","%d %d",limit,sec);

    if ((run_cnt%sec==0) && (get_pump_id_state(actpump->ID)==P_ON) && (!check_flowrate(actpump->ID,sec*xFrequency*portTICK_PERIOD_MS))) 
    {
      ESP_LOGI("DEBUG_TASK","Too low flow rate"); 
      if (actpump->just_turned_on) actpump->just_turned_on=false;
      else switch_pump_id_to_state(actpump->ID,P_FLOW_PROT);
    }
    if(get_pump_id_state(actpump->ID)==P_FLOW_PROT) switch_pump_id_to_state(actpump->ID,P_DELAY);

  }
  if  (actpump->GPIO_PROT!=-1)
  {
    if (get_pump_id_state(actpump->ID)==P_DELAY) 
    {
    if((now_pump()-pump[actpump->ID].pump_protection_started_at)/60>=pump[actpump->ID].pump_restart_delay)
                {
                  ESP_LOGI(TAG,"PUMP_RESUMED");
                  Write_Msg_toDisplay(2,"pump resumed");
                  switch_pump_id_to_state(actpump->ID,P_RESUMED);
                }
    else
    {
                  char message[32];
                  sprintf(message,"waiting:%llds",(int)pump[actpump->ID].pump_restart_delay*60+pump[actpump->ID].pump_protection_started_at-now_pump()); 
                  ESP_LOGI(TAG,"WAITING FOR RESTART DELAY");
                  Write_Msg_toDisplay(2,message);
                  pump[actpump->ID].protection_level_on=water_level;
                  pump[actpump->ID].fill_time=now_pump()-pump[actpump->ID].pump_protection_started_at;
    }           
    }
  }
  if (get_autoSwitchON(actpump->ID) && (get_pump_id_state(actpump->ID)==P_RESUMED)) 
  {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    switch_pump_id_to_state(actpump->ID,P_ON);
  }

}
}


