#ifndef FLOW_METER_H_  
#define FLOW_METER_H_

#include "driver/pulse_cnt.h"
#include "stdbool.h"


#define YF_DN32_PULSE_PER_LITER	27
#define EXAMPLE_PCNT_HIGH_LIMIT 32767
#define EXAMPLE_PCNT_LOW_LIMIT -1

void install_pcnt(int id,pcnt_unit_handle_t *pcnt_unit,int GPIO_CNT);


#endif