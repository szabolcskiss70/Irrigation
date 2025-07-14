#include "mcp23018.h"

#include <driver/gpio.h>
#include <driver/i2c_master.h>

#include "esp_log.h"

static const char* TAG = "MCP23018";

static gpio_num_t i2c_gpio_sda = 4;
static gpio_num_t i2c_gpio_scl = 15;
#define I2C_TOOL_TIMEOUT_VALUE_MS (50)
static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;

i2c_master_bus_handle_t MCP_bus_handle=NULL;


/**
 * Converts generic register and group (A/B) to register address
 * @param reg the generic register index
 * @param group the group (A/B) to compute offset
 * @return The register address specified by the parameters
*/
uint8_t mcp23018_register(mcp23018_reg_t reg, mcp23018_gpio_t group) {
   return (group == GPIOA)?(reg << 1):(reg << 1) | 1;
}

/**
 * Initializes the MCP23018
 * @param mcp the MCP23018 interface structure
 * @return an error code or MCP23018_ERR_OK if no error encountered
*/
mcp23018_err_t mcp23018_init(i2c_master_bus_handle_t in_tool_bus_handle,int chip_addr,i2c_master_dev_handle_t *dev_handle) {

	if (in_tool_bus_handle==NULL)
	{	
	 i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .scl_io_num = i2c_gpio_scl,
        .sda_io_num = i2c_gpio_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
     };

     if (i2c_new_master_bus(&i2c_bus_config, &MCP_bus_handle) != ESP_OK) {
        
        ESP_LOGE(TAG,"i2c_new_master_bus failed");
        return 0;
     }
	}
	else MCP_bus_handle=in_tool_bus_handle; //already initialized by other device, just use the handle for ACS71020

	
    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = chip_addr,
    };
  
    if (i2c_master_bus_add_device(MCP_bus_handle, &i2c_dev_conf, dev_handle) != ESP_OK) 
    {
      ESP_LOGE(TAG,"i2c_master_bus_add_device failed");
      return 0;
    }
 
 
	ESP_LOGI(TAG,"I2C DRIVER INSTALLED");
	
	// make PORTA I/O's output
	//if (mcp23018_write_register(*dev_handle, MCP23018_IODIR, GPIOA, 0x00)!=MCP23018_ERR_OK) ESP_LOGE(TAG,"mcp23018_write_register failed");
	return MCP23018_ERR_OK;
}

/**
 * Writes a value to an MCP23018 register
 * @param mcp the MCP23018 interface structure
 * @param reg A generic register index
 * @param group the group (A/B) to compute register address offset
 * @param v the value to write to the register
 * @return an error code or MCP23018_ERR_OK if no error encountered
*/
mcp23018_err_t mcp23018_write_register(i2c_master_dev_handle_t dev_handle, mcp23018_reg_t reg, mcp23018_gpio_t group, uint8_t v) {
   uint8_t r = mcp23018_register(reg, group);
   int len = 1;
	uint8_t *data = malloc(len + 1);
   data[0] = r;
	data[1] = v;
   esp_err_t ret = i2c_master_transmit(dev_handle, data, len + 1, I2C_TOOL_TIMEOUT_VALUE_MS);
  	free(data);
   if (ret == ESP_FAIL) {
      ESP_LOGE(TAG,"ERROR: unable to write to register");
      return MCP23018_ERR_FAIL;
   }
   return MCP23018_ERR_OK;
}

/**
 * Reads a value to an MCP23018 register
 * @param mcp the MCP23018 interface structure
 * @param reg A generic register index
 * @param group the group (A/B) to compute register address offset
 * @param data a pointer to an 8 bit value to be read from the device
 * @return an error code or MCP23018_ERR_OK if no error encountered
*/
mcp23018_err_t mcp23018_read_register(i2c_master_dev_handle_t dev_handle, mcp23018_reg_t reg, mcp23018_gpio_t group, uint8_t *data) {
   // from the generic register and group, derive register address
   uint8_t r = mcp23018_register(reg, group);
   
   int len = 1;
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, (uint8_t*)&r, 1, data, len, I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) return MCP23018_ERR_OK;
    else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGW(TAG, "Read failed");
    }
    return MCP23018_ERR_FAIL;

}
	
	

/**
 * Clears a bit from a current register value
 * @param mcp address of the MCP23018 data structure
 * @param bit The number of the bit to set
 * @param reg A generic register index
 * @param group the group (A/B) to compute register address offset
 * @return an error code or MCP23018_ERR_OK if no error encountered
*/
mcp23018_err_t mcp23018_set_bit(i2c_master_dev_handle_t dev_handle, uint8_t bit, mcp23018_reg_t reg, mcp23018_gpio_t group) {   
   uint8_t current_value;
   if( mcp23018_read_register(dev_handle, reg, group, &current_value) != MCP23018_ERR_OK ) {
      uint8_t r = mcp23018_register(reg, group);
      ESP_LOGE(TAG, "ERROR: unable to read current value of register %02x",r);
      return MCP23018_ERR_FAIL;
   }
   current_value |= 1 << bit;
   if( mcp23018_write_register(dev_handle, reg, group, current_value) != MCP23018_ERR_OK ) {
      uint8_t r = mcp23018_register(reg, group);
      ESP_LOGE(TAG, "ERROR: unable to write new value %02X to register %02x",current_value, r);
      return MCP23018_ERR_FAIL;
   }
   return MCP23018_ERR_OK;
}

/**
 * Clears a bit from a current register value
 * @param mcp address of the MCP23018 data structure
 * @param bit The number of the bit to set
 * @param reg A generic register index
 * @param group the group (A/B) to compute register address offset
 * @return an error code or MCP23018_ERR_OK if no error encountered
*/
mcp23018_err_t mcp23018_clear_bit(i2c_master_dev_handle_t dev_handle, uint8_t bit, mcp23018_reg_t reg, mcp23018_gpio_t group) {   
   uint8_t current_value;
   if( mcp23018_read_register(dev_handle, reg, group, &current_value) != MCP23018_ERR_OK ) {
      uint8_t r = mcp23018_register(reg, group);
      ESP_LOGE(TAG, "ERROR: unable to read current value of register %02x",r);
      return MCP23018_ERR_FAIL;
   }
   current_value &= ~(1 << bit);
   if( mcp23018_write_register(dev_handle, reg, group, current_value) != MCP23018_ERR_OK ) {
      uint8_t r = mcp23018_register(reg, group);
      ESP_LOGE(TAG, "ERROR: unable to write new value %02X to register %02x",current_value, r);
      return MCP23018_ERR_FAIL;
   }
   return MCP23018_ERR_OK;
}

mcp23018_err_t mcp23018_read_bit(i2c_master_dev_handle_t dev_handle, uint8_t bit, mcp23018_reg_t reg, mcp23018_gpio_t group, bool *val) {   
   uint8_t current_value;
   if( mcp23018_read_register(dev_handle, reg, group, &current_value) != MCP23018_ERR_OK ) {
      uint8_t r = mcp23018_register(reg, group);
      ESP_LOGE(TAG, "ERROR: unable to read current value of register %02x",r);
      return MCP23018_ERR_FAIL;
   }
   ESP_LOGE(TAG, "current_value %d",current_value);
   *val=((current_value & (1 << bit))!=0)?true:false;
   return MCP23018_ERR_OK;
}

mcp23018_err_t mcp23018_write_bit(i2c_master_dev_handle_t dev_handle, uint8_t bit, mcp23018_reg_t reg, mcp23018_gpio_t group, bool value) {   
   uint8_t current_value;
   if( mcp23018_read_register(dev_handle, reg, group, &current_value) != MCP23018_ERR_OK ) {
      uint8_t r = mcp23018_register(reg, group);
      ESP_LOGE(TAG, "ERROR: unable to read current value of register %02x",r);
      return MCP23018_ERR_FAIL;
   }
   if (value)  current_value |= 1 << bit;
   else        current_value &= ~(1 << bit);;
   current_value |= 1 << bit;
   if( mcp23018_write_register(dev_handle, reg, group, current_value) != MCP23018_ERR_OK ) {
      uint8_t r = mcp23018_register(reg, group);
      ESP_LOGE(TAG, "ERROR: unable to write new value %02X to register %02x",current_value, r);
      return MCP23018_ERR_FAIL;
   }
   return MCP23018_ERR_OK;
}

