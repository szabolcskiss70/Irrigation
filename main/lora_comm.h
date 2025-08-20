#ifndef LORA_COMM_H_  
#define LORA_COMM_H_
#include <stdint.h>
#include "lora.h"


extern uint8_t lora_transmit_buf[256];
extern uint8_t lora_receive_buf[256];
extern int INT_result;
void init_lora();
void task_rx(void *p);

#define LORA_RX_BIT    0x01



#endif