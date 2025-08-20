/**
 * @file ACS71020.h
 * 
 * @author Usman Mehmood (usmanmehmood55@gmail.com)
 * 
 * @brief The Allegro ACS71020 power monitoring IC greatly simplifies
 * the addition of power monitoring to many AC powered systems.
 * The sensor may be powered from the same supply as the
 * system's MCU, eliminating the need for multiple power supplies
 * and expensive digital isolation ICs. The device's construction
 * includes a copper conduction path that generates a magnetic field
 * proportional to applied current. The magnetic field is sensed
 * differentially to reject errors introduced by common mode fields.
 * 
 * @version 0.1
 * @date 2022-04-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _ACS71020_H_
#define _ACS71020_H_

#include <stdio.h>
#include <stdint.h>
#include "ACS71020_eeprom.h"
#include "ACS71020_volatile.h"



/**
 * register maps and bit fields are in ACS71020_eeprom.h and ACS71020_volatile.h
 * this file will contain the functions to read and write to the registers, and
 * to perform power related operations
 */

 //extern int read_ACS71020(int chip_addr, int data_addr, int *X0, int *X1,int *X2,int *X3);
extern double MeasuredValue(int ACS71020_address, int reg_address, long mask, int shiftleft,int shiftright,int fractional, float fullscale );
//extern void readEeprom(int ACS71020_address_default);
//extern void readShadow(int ACS71020_address_default);
extern int init_ACS71020(i2c_master_bus_handle_t in_tool_bus_handle,int chip_addr);
extern int write_ACS71020(int chip_addr, int data_addr, int regValue);
extern long read_ACS71020_register(int ACS71020_address, int reg_address, long mask, int shiftleft,int shiftright);
void read_ACS71020_register2(int reg_addr,long value);


#define ACS71020_address_default 0x66 
//slave: 0x61
#define Rs 1000.0
#define R1_4 2000000.0

#endif // _ACS71020_H_