//-------------------------------------------
// Name: vcnl_4040.h
// Author: Gregor Morrison
// Description: This driver file contains functions for communication
// with the proximity sensor over I2C.
//
// Referenced code: https://github.com/sparkfun/SparkFun_VCNL4040_Arduino_Library/blob/master/src/SparkFun_VCNL4040_Arduino_Library.cpp
//-------------------------------------------

#ifndef VCNL4040_H
#define VCNL4040_H

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h> 
#include "nrf_drv_twi.h"	//I2C driver library
#include "app_error.h"
#include "nrf_delay.h"
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// TWI instance ID
#define TWI_INSTANCE_ID 1

#define PROX_THRESHOLD 5000

// Proximity sensor address (for I2C)
#define VCNL4040_ADDR 0x60

// Proximity sensor register addresses
// Note: The VCNL4040 uses 2-byte command registers,
// with one upper and one lower byte. The datasheet
// distinguishes between upper and lower bytes by
// giving each byte a different name (i.e. CONF1 and CONF2)
// for the 2 bytes located at 0x03. This convention is followed
// here (see datasheet for more info).
#define VCNL4040_PS_CONF1 0x03 //Lower
#define VCNL4040_PS_CONF2 0x03 //Upper
#define VCNL4040_PS_CONF3 0x04 //Lower
#define VCNL4040_PS_MS 0x04 //Upper
#define VCNL4040_PS_DATA 0x08

#define NORMAL_MODE 0U

extern const nrf_drv_twi_t twi;

extern volatile bool m_xfer_done;

void twi_init(void);

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);

void vcnl4040_config(void);

void vcnl4040_read_sensor_data(void);

#endif // VCNL4040_H