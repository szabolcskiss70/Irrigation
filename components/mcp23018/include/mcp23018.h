#ifndef _MCP23018_H_
#define _MCP23018_H_
 
 
 // Error library
#include "freertos/FreeRTOS.h"
#include "esp_err.h"

// I2C driver
#include "driver/i2c_master.h"

// FreeRTOS (for delay)
#include "freertos/task.h"

// registers
#define MCP23018_IODIRA		0x00
#define MCP23018_IPOLA 		0x02
#define MCP23018_GPINTENA 	0x04
#define MCP23018_DEFVALA 	0x06
#define MCP23018_INTCONA 	0x08
#define MCP23018_IOCONA 	0x0A
#define MCP23018_GPPUA 		0x0C
#define MCP23018_INTFA 		0x0E
#define MCP23018_INTCAPA 	0x10
#define MCP23018_GPIOA 		0x12
#define MCP23018_OLATA 		0x14


#define MCP23018_IODIRB 	0x01
#define MCP23018_IPOLB 		0x03
#define MCP23018_GPINTENB 	0x05
#define MCP23018_DEFVALB 	0x07
#define MCP23018_INTCONB 	0x09
#define MCP23018_IOCONB 	0x0B
#define MCP23018_GPPUB 		0x0D
#define MCP23018_INTFB 		0x0F
#define MCP23018_INTCAPB 	0x11
#define MCP23018_GPIOB 		0x13
#define MCP23018_OLATB 		0x15

#define MCP23018_DEFAULT_ADDR	0x20




/*
   mcp23018_err_t
   
   Specifies an error code returned by functions
   in the MCP23018 API
*/
typedef enum {
   MCP23018_ERR_OK      = 0x00,
   MCP23018_ERR_CONFIG  = 0x01,
   MCP23018_ERR_INSTALL = 0x02,
   MCP23018_ERR_FAIL    = 0x03
} mcp23018_err_t;

/*
   R/W bits
*/
#ifndef WRITE_BIT
#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#endif

#ifndef READ_BIT
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#endif

// bit to enable checking for ACK
#ifndef ACK_CHECK_EN
#define ACK_CHECK_EN   0x1
#endif

/*
   mcp23018_reg_t

   Specifies a generic register which
   can point to either group A or
   group B depending on an offset that
   can be applied.
*/
typedef enum {
	MCP23018_IODIR		= 0x00,
	MCP23018_IPOL		= 0x01,
	MCP23018_GPINTEN	= 0x02,
	MCP23018_DEFVAL	= 0x03,
	MCP23018_INTCON	= 0x04,
	MCP23018_IOCON		= 0x05,
	MCP23018_GPPU		= 0x06,
	MCP23018_INTF		= 0x07,
	MCP23018_INTCAP	= 0x08,
	MCP23018_GPIO		= 0x09,
	MCP23018_OLAT		= 0x0A
} mcp23018_reg_t;

/*
   mcp23018_gpio_t
   
   Specifies a group of GPIO pins, either
   group A or group B
*/
typedef enum {
	GPIOA = 0x00,
	GPIOB = 0x01
} mcp23018_gpio_t;

/*
   mcp23018_t
   
   Specifies an interface configuration
*/
typedef struct {
	uint8_t i2c_addr;
	i2c_port_t port;
	uint8_t sda_pin;
	uint8_t scl_pin;
	gpio_pullup_t sda_pullup_en;
	gpio_pullup_t scl_pullup_en;
} mcp23018_t;

/*

   Function prototypes
   
*/
mcp23018_err_t mcp23018_init(i2c_master_bus_handle_t in_tool_bus_handle,int chip_addr,i2c_master_dev_handle_t *dev_handle);
mcp23018_err_t mcp23018_write_register(i2c_master_dev_handle_t dev_handle, mcp23018_reg_t reg, mcp23018_gpio_t group, uint8_t v);
mcp23018_err_t mcp23018_read_register(i2c_master_dev_handle_t dev_handle, mcp23018_reg_t reg, mcp23018_gpio_t group, uint8_t *data);
mcp23018_err_t mcp23018_set_bit(i2c_master_dev_handle_t dev_handle, uint8_t bit, mcp23018_reg_t reg, mcp23018_gpio_t group);
mcp23018_err_t mcp23018_clear_bit(i2c_master_dev_handle_t dev_handle, uint8_t bit, mcp23018_reg_t reg, mcp23018_gpio_t group);
mcp23018_err_t mcp23018_read_bit(i2c_master_dev_handle_t dev_handle, uint8_t bit, mcp23018_reg_t reg, mcp23018_gpio_t group, bool *val); 
mcp23018_err_t mcp23018_write_bit(i2c_master_dev_handle_t dev_handle, uint8_t bit, mcp23018_reg_t reg, mcp23018_gpio_t group, bool value);


#endif