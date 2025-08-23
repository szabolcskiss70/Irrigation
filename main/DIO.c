#include <dio.h>
#include "driver/gpio.h"

i2c_master_dev_handle_t MCP_dev_handle;

#define ESP_INTR_FLAG_DEFAULT 0


QueueHandle_t gpio_evt_queue;
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void writeDO(int portbit, bool value)
{
 uint8_t lora_send_buf[32];   
 switch (portbit)
 {
  case 0 ... 39:        gpio_set_level(portbit,value);
                        break;
  case 100 ... 115:     portbit-=100;              
                        mcp23018_write_bit  (MCP_dev_handle,(portbit<=7)?portbit:portbit-8, MCP23018_GPIO, (portbit<=7)?GPIOA:GPIOB,value);
                        break;    
  case 1000 ... 1039:   portbit-=1000;  
                        sprintf((char*)lora_send_buf,"DIO%d:%d",portbit,(int)value);
                        lora_send_packet(lora_send_buf,sizeof(lora_send_buf)-1);
                        break;

 }
}


bool readDI(int portbit)
{
 bool ret;
 switch (portbit)
 {
  case 0 ... 39: ret=gpio_get_level(portbit);
             break;

  case 100 ... 115:  portbit-=100;   
                 mcp23018_read_bit( MCP_dev_handle,(portbit<=7)?portbit:portbit-8,  MCP23018_GPIO,  (portbit<=7)?GPIOA:GPIOB, &ret);
                 break;   
 
 }
 return (ret);
}


void Init_DIO(i2c_master_bus_handle_t MCP_bus_handle)
{
    esp_err_t ret = mcp23018_init(MCP_bus_handle,MCP23018_DEFAULT_ADDR,&MCP_dev_handle);
    ESP_ERROR_CHECK(ret);
}

void set_DIO_direction(int portbit,gpio_mode_t mode)
{
 switch (portbit)
 {
  case 0 ... 39: gpio_set_direction(portbit ,mode);
             break;

  case 100 ... 115:  portbit-=100;   
                 mcp23018_write_bit  (MCP_dev_handle,(portbit<=7)?portbit:portbit-8, MCP23018_IODIR, (portbit<=7)?GPIOA:GPIOB,((mode&GPIO_MODE_DEF_OUTPUT)!=0)?0:1);
                 break;   
 
 }   
}


void set_DIO_interrupt(int portbit,gpio_mode_t mode, gpio_int_type_t intr_type)
{
    ESP_LOGI("DEBUG","set_DIO_interrupt:%d,%d,%d",portbit,mode,intr_type);
    gpio_config_t io_conf;
    io_conf.intr_type = intr_type;
    io_conf.pin_bit_mask = (1ULL<<portbit);
    io_conf.mode = mode;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(portbit, gpio_isr_handler, (void*) portbit);
}
