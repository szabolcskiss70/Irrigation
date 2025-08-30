#ifndef _PUMP_H_
#define _PUMP_H_

#include <time.h>
#include "stdbool.h"
//#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "flow_meter.h"
#include "pump_states.h"

#define Volume_measure_interval_us 11E6 





typedef struct{
	int ID;
	int pump_restart_delay; //minimum time for pump restart from pump OFF
	bool prio; //priority of pump
	bool switchbackifavailable; // cwitch back to prio pump if possible
	float max_current;
	float T_trip;
	float T_reset;
	float T_max;
	float I_max;
	float last_T;
	int Flow_CNT_at_err;
	T_pump_states status; //actual pump status
	int GPIO_PUMP; //GPIO of pump relay
	int GPIO_PROT; //gpio of protection imput
	int GPIO_CNT;  //gpio of flow meter
    time_t sink_time; //sink time from pump start to suspended state
	float sink_volume;
    time_t fill_time; //time from suspended to protection OFF
	time_t last_pump_on_time; //timestamp of last pump switch ON
	time_t pump_protection_started_at;	//timestamp of pump protection started
	float protection_level_off;
    float protection_level_on;
	int cnt_at_pump_start; //count value at pump start
	int cnt_at_pump_suspend; //count value at pump suspend
    int daily_pump_flowmeter_counts; // daily pump counts
    int prev_daily_pump_flowmeter_counts; //daily pump counts in previous read cycle
	int prev_daily_pump_flowmeter_counts_flowmeter; ////daily pump counts in previous measure cycle
	time_t status_change_time[P_ON-PROT_T_TRIP+1]; // timestamps of state changes
	bool pump_running; // actual pump state
	int flow_rate_protection_limit_dl_per_min;
	pcnt_unit_handle_t pcnt_unit; // flow meter counter handle
	TaskHandle_t CurrentMonitoringTaskHAndle;
	int ACS71020_address;
	bool Auto_switch_on_if_powered;
	bool remote_pump;
	bool just_turned_on;
	int suspend_reason;
} T_pump; 

extern T_pump pump[3];


T_pump_states get_pump_id_state(int id);

void switch_pump_ch(int id,bool on_state);
void init_pump(int id, int GPIO_PUMP, int GPIO_PROT,int GPIO_CNT,bool prio, bool switchbackifresumed,int ACS71020_address );

T_pump_states check_pump_protection();

void set_restart_delay(int ch, int restart_delay);
void get_LEVEL_string_for_id(int id,char* result_string);
void GetVolumeStringfor_pump(int id,char* result_string);
bool isPUMP_disabled(int id);
bool isPUMP_available(int id);
void switch_pump_id_to_state(int id, T_pump_states new_state);
int measure_flowrate_on_local_pump(int pump_ID);


#endif