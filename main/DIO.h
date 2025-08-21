#ifndef _DIO_H_
#define _DIO_H_

// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"

#include "mcp23018.h"

// I2C driver
#include "driver/i2c.h"

// Error library
#include "esp_err.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include <stdio.h>

void Init_DIO(i2c_master_bus_handle_t MCP_bus_handle);
void writeDO(int portbit, bool value);
bool readDI(int portbit);
void set_DIO_direction(int portbit,gpio_mode_t mode);
void set_DIO_interrupt(int portbit,gpio_mode_t mode, gpio_int_type_t intr_type);
extern QueueHandle_t gpio_evt_queue;


#endif