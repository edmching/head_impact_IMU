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

// Proximity sensor address
#define VCNL4040_ADDR 0x60U //U >> 1

// Proximity sensor command codes
#define VCNL4040_PS_CONF1 0x03U //Lower
#define VCNL4040_PS_CONF2 0x03U //Upper
#define VCNL4040_PS_CONF3 0x04U //Lower
#define VCNL4040_PS_MS 0x04U //Upper
#define VCNL4040_PS_DATA 0x08U

#define NORMAL_MODE 0U

extern const nrf_drv_twi_t twi;

extern volatile bool m_xfer_done;

void twi_init(void);

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);

void vcnl4040_config(void);

void vcnl4040_read_sensor_data(void);

#endif // VCNL4040_H