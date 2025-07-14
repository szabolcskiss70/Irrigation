#include <dio.h>
#include "driver/gpio.h"

i2c_master_dev_handle_t MCP_dev_handle;


void writeDO(int portbit, bool value)
{
 switch (portbit)
 {
  case 0 ... 39: gpio_set_level(portbit,value);
             break;
  case 100 ... 115:  portbit-=100;              
                 mcp23018_write_bit  (MCP_dev_handle,(portbit<=7)?portbit:portbit-8, MCP23018_GPIO, (portbit<=7)?GPIOA:GPIOB,value);
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
