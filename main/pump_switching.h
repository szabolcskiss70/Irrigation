#ifndef PUMP_SW_H_  
#define PUMP_SW_H_
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdbool.h"
#include "pump_states.h"
#include "pump_params.h"
#include <time.h>

typedef enum {PUMP1,PUMP2,BOTH}T_pump_list;
typedef enum {P0,P1,NO}T_active_pump_suspended;
typedef enum {ONLY_SLAVE_RELAY,CURR_PROTECTED,VIA_LORA_FUNC,CLONING,modeLAST} T_dual_mode_flags;
extern T_active_pump_suspended active_pump_suspended; 
extern int pump_num;
extern int running_pump_ID;
extern QueueHandle_t pump_request_queue;
extern SemaphoreHandle_t pump_array_mutex;
extern int dual_mode_flags;
extern bool Cloned_buffer_valid;

void set_pump_default_params(int id,char* ldata);
int getINTvaluefromslave(char* msg);
T_pump_states get_pump_id_state_array(int id);
int other_pump(int id);
void init_pump_switching(int dual_mode_flags);
void gen_switch_pump_id_to_state(int id, T_pump_states new_state);
void init_single_pump(int id, int GPIO_PUMP, int GPIO_PROT,int GPIO_CNT,bool prio, bool switchbackifresumed,int ACS71020_address, bool pump_current_prot );
void get_LEVEL_string(char* result_string);
void GetVolumeString(char *result_string);
bool isPUMP_disabled_or_suspended();
T_pump_states check_pump_protection();
int measure_flowrate();
time_t now_pump();

typedef struct 
{
  bool state;
  T_pump_list assigned_pump;  
}
T_pump_switching_request;

#endif
