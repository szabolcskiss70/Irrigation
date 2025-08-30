#ifndef LORA_COMM_H_  
#define LORA_COMM_H_
#include "freertos/FreeRTOS.h"
#include <stdint.h>
#include "lora.h"


extern uint8_t lora_transmit_buf[256];
extern uint8_t lora_receive_buf[256];
extern int INT_result;
extern bool lora_comm_initialized;
int init_lora();
void task_rx(void *p);

#define LORA_RX_BIT    0x01
extern QueueHandle_t lora_ans_evt_queue;

int my_lora_packet_rssi();
void my_lora_send_packet(uint8_t *buf, int size);



#endif