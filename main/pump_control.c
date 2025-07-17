#include "pump_control.h"
#include <string.h>
#include "esp_timer.h"
#include "DIO.h"

extern void Write_Msg_toDisplay(int line, char *Msg);
extern float water_level;

void install_pcnt();
#define EXAMPLE_PCNT_HIGH_LIMIT 32767
#define EXAMPLE_PCNT_LOW_LIMIT -1

T_pump pump[2];
char* PUMP_status_str[]={"DISABLED","SUSPENDED","DELAY","OFF","RESUMED","ON"};

static const char *TAG = "pump";
int pump_num=0;
int running_pump_ID=-1;

time_t now_pump()
{
 
 time_t now;
 time(&now);
 return(now);
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
		pump[id].daily_pump_volume=0;
		pcnt_unit_clear_count(pump[id].pcnt_unit);
  }
 }
 prev_now=now;
}




T_pump_states get_pump_id_state(int id)
{
 return (pump[id].status);
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
   pcnt_unit_get_count(pump[id].pcnt_unit, &pump[id].cnt_at_pump_start);
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
 pump[id].status_change_time[new_state]=now_pump();
 switch (new_state)
 {
  case P_ON:              switch_pump_ch_relay(id,true);	 
                          break;
  default:                switch_pump_ch_relay(id,false);	           
                          break;                
 }
}

void init_pump(int id, int GPIO_PUMP, int GPIO_PROT,int GPIO_CNT,bool prio, bool switchbackifavailable)
{
	pump[id].ID=id;
  pump[id].pump_running=false;
	pump[id].GPIO_PUMP=GPIO_PUMP;
	pump[id].GPIO_PROT=GPIO_PROT;
	pump[id].GPIO_CNT=GPIO_CNT;
 
    pump[id].sink_time=0;
    pump[id].fill_time=0;	
	pump[id].last_pump_on_time=0;
    pump[id].protection_level_off=0;
    pump[id].protection_level_on=0;
	pump[id].pump_restart_delay=10;
    pump[id].prio=prio;
	pump[id].switchbackifavailable=switchbackifavailable;
    pump[id].pcnt_unit = NULL;
    pump[id].daily_pump_volume=0;
    pump[id].prev_daily_pump_volume=0;
	install_pcnt(id);
  set_DIO_direction(GPIO_PUMP,GPIO_MODE_OUTPUT);
  set_DIO_direction(GPIO_PROT,GPIO_MODE_INPUT);
   switch_pump_id_to_state(id,P_OFF);
	pump_num++;
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
void switch_pump(bool on_state)
{
 int pump2switch;
 if (running_pump_ID>-1) pump2switch=running_pump_ID;	 
 else
 {
  int higher_prio_pump=0;
  if ((pump_num==2) && (pump[1].prio)) higher_prio_pump=1;
  pump2switch=higher_prio_pump;
  if ((pump[higher_prio_pump].status==P_DISABLED) || (pump[higher_prio_pump].status==P_SUSPENDED))
  pump2switch=(higher_prio_pump==1)?0:1;
 }


 switch_pump_id_to_state(pump2switch,on_state?P_ON:P_OFF);
}

bool is_low_prio_pump_running()
{
 if (pump_num<2) return false;
 if (running_pump_ID==-1) return false;
 return (pump[running_pump_ID].prio?false:true);
}

int other_pump(int id) {return((id==0)?1:0);}



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
  typedef enum {PUMP1,PUMP2,NO}T_active_pump_suspended;
  T_active_pump_suspended active_pump_suspended=NO; 
  for (int id=0;id<pump_num;id++)	
  {
	if(readDI(pump[id].GPIO_PROT))
	{
			if(pump[id].status!=P_SUSPENDED)
			{		
			ESP_LOGI("PUMP","PUMP SUSPENDED");  
			pump[id].pump_protection_started_at=now_pump();
			pump[id].protection_level_off=water_level;
			pump[id].sink_time=now_pump()-pump[id].last_pump_on_time;
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
	}
 }

 if ((pump_num==2) && (active_pump_suspended!=NO)) switch_pump_id_to_state(other_pump(active_pump_suspended),P_ON); //switch to an other pump

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

 return ((pump_num==2)?(pump[0].status>pump[1].status)?pump[0].status:pump[1].status:pump[0].status);
}

bool isPUMP_disabled_or_suspended()
{
  for (int id=0;id<pump_num;id++)	
  {
   if ((pump[id].status!=P_DISABLED) && (pump[id].status!=P_SUSPENDED)) return false;
  }
  return true;
}

float convertCNT2Liter(int delta_volume_cnt)
{
  return(1.0*delta_volume_cnt/YF_DN32_PULSE_PER_LITER);
}

void GetPumpStatusString(int id, char* message, int buf_size)
{
 pcnt_unit_get_count(pump[id].pcnt_unit, &pump[id].daily_pump_volume);
 char *MsgFormat= "P%d: Status:%s, Daily Volume:%1.1f";
 if (buf_size>strlen(MsgFormat)+8) sprintf(message,"P%d: Status:%s, Daily Volume:%f",id+1,PUMP_status_str[pump[id].status],convertCNT2Liter(pump[id].daily_pump_volume));
}

void getpumptimechanges(int id, char* message, int buf_size)
{
			for (int i=0;i<sizeof(T_pump_states);i++)
			{
				if (buf_size>64) sprintf(message+strlen(message),"ID:%d %s: %lld\n",id,PUMP_status_str[i],pump[id].status_change_time[i]);
			}
}



T_pump_states GetPumpStatus(int id)
{
 return(pump[id].status);
}

void set_restart_delay(int id, int restart_delay)
{
 pump[id].pump_restart_delay=restart_delay;
}
  
void get_LEVEL_string(char* result_string)
{
   *result_string=0;
  for (int id=0;id<pump_num;id++)	
  {
	sprintf(result_string+strlen(result_string),"%0.1fcm off=%0.1fcm on=%0.1fcm fill=%ds sink=%ds \n",water_level,pump[id].protection_level_off,pump[id].protection_level_on,(int)pump[id].fill_time,(int)pump[id].sink_time); 
  }	


}







int measure_flowrate()
{
  clear_volumes_at_midnight();
  if(running_pump_ID!=-1)
  {  
  static uint64_t TimePastVolumeMeasured=0;
	if(TimePastVolumeMeasured==0)  TimePastVolumeMeasured=esp_timer_get_time();
	uint64_t Volume_measure_delta_time; 
  
	int delta_volume_cnt=0;
	ESP_ERROR_CHECK(pcnt_unit_get_count(pump[running_pump_ID].pcnt_unit, &pump[running_pump_ID].daily_pump_volume));
  delta_volume_cnt=pump[running_pump_ID].daily_pump_volume-pump[running_pump_ID].prev_daily_pump_volume;
  pump[running_pump_ID].prev_daily_pump_volume=pump[running_pump_ID].daily_pump_volume;
	
	if (((Volume_measure_delta_time=(esp_timer_get_time() - TimePastVolumeMeasured)) >= Volume_measure_interval_us) && (delta_volume_cnt>5))
    {
	   char message[32];  
     float volume_rate_liter_per_min;	
     volume_rate_liter_per_min= 60*convertCNT2Liter(delta_volume_cnt)/(1.0*Volume_measure_delta_time/1000000.0);
     sprintf(message,"%0.1f l/min %0.1f l",volume_rate_liter_per_min,convertCNT2Liter(pump[running_pump_ID].daily_pump_volume-pump[running_pump_ID].cnt_at_pump_start));
     Write_Msg_toDisplay(5,message);
	   TimePastVolumeMeasured = esp_timer_get_time(); // get next publish time
    }
	return delta_volume_cnt;
  }
  else return 0;
}

void install_pcnt(int id)
{
	ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
		.flags.accum_count = true,
    };
    
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pump[id].pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pump[id].pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = pump[id].GPIO_CNT,
		.level_gpio_num=-1,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pump[id].pcnt_unit, &chan_a_config, &pcnt_chan_a));
    

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,PCNT_CHANNEL_EDGE_ACTION_HOLD));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
  

    ESP_LOGI(TAG, "add watch points and register callbacks");
    int watch_points[] = {EXAMPLE_PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pump[id].pcnt_unit, watch_points[i]));
    }
    /*pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));*/

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pump[id].pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pump[id].pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pump[id].pcnt_unit));
}

void GetVolumeString(char *result_string)
{
  *result_string=0;
  for (int id=0;id<pump_num;id++)	
  {
	sprintf(result_string+strlen(result_string),"Avg: %0.1f l/min",60*convertCNT2Liter(pump[id].daily_pump_volume-pump[id].cnt_at_pump_start)/(1.0*(now_pump() - pump[id].last_pump_on_time))); 
  }

}

bool isPUMP_disabled(int id) {return(get_pump_id_state(id)==P_DISABLED);}

bool isPUMP_available(int id) {return(pump[id].status>=P_OFF);}


bool getPUMP_prio(int id) {return(pump[id].prio);}
void setPUMP_prio(int id, bool val) {pump[id].prio=val;pump[other_pump(id)].prio=!val;}
bool getPUMP_switchbackifavailable(int id) {return(pump[id].switchbackifavailable);}
void setPUMP_switchbackifavailable(int id, bool val) {pump[id].prio=val;}
