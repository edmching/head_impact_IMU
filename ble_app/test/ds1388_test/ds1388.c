#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"	//I2C driver library
#include "nrf_delay.h"
#include "nrf52832_mdk.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// TWI instance ID
#define TWI_INSTANCE_ID     0

//Slave addresses
#define DS1388_ADDRESS         0x68  //b1101000, for RTC and WD
#define EEPROM_ADDRESS_1    0x69  //b1101001, EEPROM block 1
#define EEPROM_ADDRESS_2    0x6A  //b1101010, EEPROM block 2

//Registers
#define HUNDRED_SEC_REG     0x00  //hundredth of seconds
#define SEC_REG             0x01  //seconds
#define MIN_REG             0x02  //minutes
#define HOUR_REG            0x03  //hour
#define DAY_REG             0x04  //day
#define DATE_REG            0x05  //date
#define MONTH_REG           0x06  //month
#define YEAR_REG            0x07  //year
#define WD_HUNDRED_SEC_REG  0x08  //watchdog hundredth seconds
#define WD_SEC_REG          0x09  //watchdog seconds
#define TRICKLE_CHG_REG     0x0A  //trickle charger
#define FLAG_REG            0x0B  //flags
#define CONTROL_REG         0x0C  //control

//Control Register
#define EN_OSCILLATOR       0x00  
#define DIS_OSCILLATOR      0x80
#define EN_WD_COUNTER       0x02
#define DIS_WD_COUNTER      0x00    
#define WD_RST              0x01  //trigger reset if WD counter is enable and counter reach 0

//Time format
#define HOUR_MODE_12        0x40
#define HOUR_MODE_24        0x00
#define AM                  0x00
#define PM                  0x20

static uint8_t date[8];
static uint8_t byte;
static uint8_t time_format =  HOUR_MODE_24; //select either 12-HOUR FORMAT or 24-HOUR FORMAT, if 12-HOUR FORMAT, use together with AM, PM eg: HOUR_MODE_12 | PM
static uint8_t init_time[8] = {20, 2, 11, 2, 7, 26, 10, 0};
//init_time[0] = 20; (year)
//init_time[1] = 2;  (month)
//init_time[2] = 1;  (date)
//init_time[3] = 6;  (day of the week, MONDAY: 1, TUESDAY: 2, ... SUNDAY: 7)
//init_time[4] = 4;  (hour) 
//init_time[5] = 25; (minutes) 
//init_time[6] = 0;  (seconds)
//init_time[7] = 0;  (hundredth of second)
 
/* Indicates if operation on I2C has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t ds_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**
 * @brief Function for setting active mode on VCNL4040 proximity sensor
 */
uint8_t dec2hex(uint8_t val) {
  val = val + 6 * (val / 10);
  return val;
}

/**
 * @brief Function for setting active mode on VCNL4040 proximity sensor
 */
uint8_t hex2dec(uint8_t val) {
  val = val - 6 * (val >> 4);
  return val;
}

/**
 * @brief Function for setting active mode on VCNL4040 proximity sensor
 */
void ds_config(void)
{	
	uint8_t reg0[2] = {CONTROL_REG, (EN_OSCILLATOR | DIS_WD_COUNTER)};
    nrf_drv_twi_tx(&ds_twi, DS1388_ADDRESS, reg0, sizeof(reg0), false);
    while (m_xfer_done == false);
    
    uint8_t reg1[9] = {HUNDRED_SEC_REG,
                        dec2hex(init_time[7]),
                        dec2hex(init_time[6]),
                        dec2hex(init_time[5]),
                        (dec2hex(init_time[4]) | time_format),
                        dec2hex(init_time[3]),
                        dec2hex(init_time[2]),
                        dec2hex(init_time[1]),
                        dec2hex(init_time[0])
    };

    nrf_drv_twi_tx(&ds_twi, DS1388_ADDRESS, reg1, sizeof(reg1), false);
    while (m_xfer_done == false);
}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in]  Real-time data
 */
__STATIC_INLINE void data_handler(uint8_t rtc_data[8])
{
    NRF_LOG_INFO("Year:   20%d", rtc_data[0]);
    NRF_LOG_INFO("Month:  %d", rtc_data[1]);
    NRF_LOG_INFO("Date:   %d", rtc_data[2]);
    NRF_LOG_INFO("Day:    %d", rtc_data[3]);
    NRF_LOG_INFO("Hour:   %d", rtc_data[4]);
    NRF_LOG_INFO("Minute: %d", rtc_data[5]);
    NRF_LOG_INFO("Second: %d\n", rtc_data[6]);
    //NRF_LOG_INFO("Hundreth of a Second: %d", rtc_data[7]);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            //m_xfer_done = true;
            NRF_LOG_INFO("\r\nDATA NACK ERROR");
            NRF_LOG_FLUSH();
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            //m_xfer_done = true;
            NRF_LOG_INFO("\r\nADDRESS NACK ERROR");
            NRF_LOG_FLUSH();
            break;
        default:
            //m_xfer_done = true
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = DS1388_SCL,
       .sda                = DS1388_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&ds_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&ds_twi);
}

uint8_t readRegister(uint8_t reg_addr)
{
    do
        {
            __WFE();
        }while (m_xfer_done == false);

    m_xfer_done = false;

    nrf_drv_twi_xfer_desc_t const ds_desc = {NRFX_TWI_XFER_TXRX, DS1388_ADDRESS, sizeof(reg_addr), sizeof(byte), &reg_addr, &byte};

    nrf_drv_twi_xfer(&ds_twi, &ds_desc, false);
    
    while(nrf_drv_twi_is_busy(&ds_twi) == true);

    return byte;
}

uint8_t get_time()
{
  uint8_t ret;
  
  date[0] = readRegister(YEAR_REG);
  date[1] = readRegister(MONTH_REG);
  date[2] = readRegister(DATE_REG);
  date[3] = readRegister(DAY_REG);
  date[4] = readRegister(HOUR_REG);
  date[5] = readRegister(MIN_REG);
  date[6] = readRegister(SEC_REG);
  date[7] = readRegister(HUNDRED_SEC_REG);
  
  //Time processing 
  date[0] = hex2dec(date[0]);
  date[1] = hex2dec(date[1]);
  date[2] = hex2dec(date[2]);
  date[5] = hex2dec(date[5]);
  date[6] = hex2dec(date[6]);
  date[7] = hex2dec(date[7]);

  
  if ((date[4] & 0x40) == HOUR_MODE_24)
  {
    date[4] = hex2dec(date[4]);
    ret = 2;
    return ret;
  }
  else
  { 
    ret = (date[4] & 0x20 )>> 5;
    date[4] = hex2dec(date[4] & 0x1F);
    return ret;
  }
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example started.");
    NRF_LOG_FLUSH();
    twi_init();
    nrf_delay_ms(1000);
	ds_config();
    nrf_delay_ms(1000);

    while (true)
    {
        nrf_delay_ms(1000);
        get_time();
        data_handler(date);
        NRF_LOG_FLUSH();
    }
}

/** @} */
