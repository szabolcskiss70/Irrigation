#ifndef PUMP_PARAM_H_  
#define PUMP_PARAM_H_


bool getPUMP_prio(int id);
void setPUMP_prio(int id, bool val);
bool getPUMP_switchbackifavailable(int id);
void setPUMP_switchbackifavailable(int id, bool val);
float get_max_current(int id);
void  set_max_current(int id, float imax);
void  set_flow_rate_protection_limit_dl_per_min(int id, int flow_min_dlper_min);
int   get_flow_rate_protection_limit_dl_per_min(int id);
float get_T_trip(int id);
void  set_T_trip(int id, float T_trip);
float get_T_reset(int id);
void  set_T_reset(int id, float T_reset);
bool get_autoSwitchON(int id);
void set_autoSwitchON(int id, bool val);
bool get_remotePump(int id);
void set_remotePump(int id, bool val);
void set_restart_delay(int id, int restart_delay);
int get_restart_delay(int id);
void getpumptimechanges(int id, char* message, int buf_size);
void GetPumpStatusString(int id, char* message, int buf_size);
void getpumptimechanges(int id, char* message, int buf_size);
void enable_pump(int ch,bool enable);
bool isPUMP_disabled_local(int id);
int getsinktime(int id);
int getfilltime(int id);
float getsinkvolume(int id);

#endif