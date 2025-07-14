#ifndef _PUMP_H_
#define _PUMP_H_

#include <time.h>
#include "stdbool.h"
//#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"

#define Volume_measure_interval_us 11E6 
#define YF_DN32_PULSE_PER_LITER	27

typedef enum {P_DISABLED, P_SUSPENDED,P_DELAY,P_OFF,P_RESUMED,P_ON} T_pump_states;

typedef struct{
	int ID;
	int pump_restart_delay;
	bool prio;
	bool switchbackifavailable;
	T_pump_states status;
	int GPIO_PUMP;
	int GPIO_PROT;
	int GPIO_CNT;
    time_t sink_time;
    time_t fill_time;
	time_t last_pump_on_time;
	time_t pump_protection_started_at;	
	float protection_level_off;
    float protection_level_on;
	int cnt_at_pump_start;
	pcnt_unit_handle_t pcnt_unit;
    int daily_pump_volume;
    int prev_daily_pump_volume;
} T_pump; 

//extern T_pump pump[2];
extern int pump_num;

void switch_pump(bool on_state);
void switch_pump_ch(int id,bool on_state);
void enable_pump(int ch,bool enable);
void init_pump(int id, int GPIO_PUMP, int GPIO_PROT,int GPIO_CNT,bool prio, bool switchbackifresumed);
bool isPUMP_disabled_or_suspended();
T_pump_states check_pump_protection();
char *GetPumpStatusString(int id);
void set_restart_delay(int ch, int restart_delay);
void get_LEVEL_string(char* result_string);
int measure_flowrate();
void GetVolumeString(char *result_string);
bool isPUMP_disabled(int id);
bool isPUMP_available(int id);
bool getPUMP_prio(int id);
void setPUMP_prio(int id, bool val);
bool getPUMP_switchbackifavailable(int id);
void setPUMP_switchbackifavailable(int id, bool val);

#endif