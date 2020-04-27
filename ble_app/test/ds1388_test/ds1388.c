//-------------------------------------------
// Name: ds1388.c
// Author: Gregor Morrison
// Description: This file initiates I2C communication
// with the ds1388 RTC, configures the device,
// and then  prints out the date and time serially.
// This code requires manual setting of the RTC - for
// autoset functionality, reference "\ble_app\test\ds1388_autoset_test"
//
// Referenced code: https://github.com/DelfiSpace/DS1388/blob/master/DS1388.cpp
//-------------------------------------------

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"    // I2C driver library
#include "nrf_delay.h"
#include "nrf52832_mdk.h"   // Board (pin mapping) header file for breadboard setup
#include "ds1388.h"         // RTC header file with register/command defines

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// TWI instance ID
#define TWI_INSTANCE_ID     0

// Struct type with all necessary date and time variables
// Each variable is byte-wide to ensure compatibility with
// the ds1388's internal memory structure (see datasheet)
typedef struct{
    uint8_t year;
    uint8_t month;
    uint8_t date;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t hundreth;
} ds1388_data_t;

ds1388_data_t date; // declaration of above struct
static uint8_t byte;
static uint8_t time_format =  HOUR_MODE_24; // select either 12-HOUR FORMAT or 24-HOUR FORMAT, if 12-HOUR FORMAT, use together with AM, PM eg: HOUR_MODE_12 | PM

// This array holds numerical values corresponding to the 
// date or time variables as indicated below. Manually change
// the values in the array to modify the date/time as desired.
// Note: see the file "ds1388_auto.c" for autosetting the RTC
static uint8_t init_time[8] = {20, 2, 11, 2, 7, 26, 10, 0};
//init_time[0] = 20; (year)
//init_time[1] = 2;  (month)
//init_time[2] = 11;  (date)
//init_time[3] = 2;  (day of the week, MONDAY: 1, TUESDAY: 2, ... SUNDAY: 7)
//init_time[4] = 7;  (hour) 
//init_time[5] = 26; (minutes) 
//init_time[6] = 10;  (seconds)
//init_time[7] = 0;  (hundredth of second)
 
/* Indicates if operation on I2C has ended. */
static volatile bool m_xfer_done_ds = false;

/* TWI instance. */
static const nrf_drv_twi_t ds_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**
 * @brief Function for converting decimal number to hexadecimal
 */
uint8_t dec2hex(uint8_t val) {
  val = val + 6 * (val / 10);
  return val;
}

/**
 * @brief Function for converting hexadecimal number to decimal
 */
uint8_t hex2dec(uint8_t val) {
  val = val - 6 * (val >> 4);
  return val;
}

/**
 * @brief Function for configuring the ds1388. Writes the appropriate control
 * variables into the control register and then updates the internal RTC
 * registers with the date and time from the init_time[8] array
 */
void ds_config(void)
{	
    // Prepares target register (CONTROL_REG) and data to be written (EN_OSCILLATOR | DIS_WD_COUNTER)
    // In this case, the RTC oscillator is enabled and the watch dog counter is disabled
	uint8_t reg0[2] = {CONTROL_REG, (EN_OSCILLATOR | DIS_WD_COUNTER)};
    nrf_drv_twi_tx(&ds_twi, DS1388_ADDRESS, reg0, sizeof(reg0), false); // initates I2C transfer to ds1388
    while (m_xfer_done_ds == false); // waits for the I2C transfer to complete
    
    // Prepares a multibyte write starting at the target register (HUNDRED_SEC_REG) and
    // writes the time and date data in successive cycles. The RTC's internal address pointer
    // automatically increments the write address (see datasheet)
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

    nrf_drv_twi_tx(&ds_twi, DS1388_ADDRESS, reg1, sizeof(reg1), false); // initates I2C transfer to ds1388
    while (m_xfer_done_ds == false); // waits for the I2C transfer to complete
}

/**
 * @brief Function for handling data from RTC for logging to serial.
 *
 * @param[in]  Real-time data
 */
__STATIC_INLINE void data_handler(ds1388_data_t rtc_data)
{
    NRF_LOG_INFO("Year:   20%d", rtc_data.year);
    NRF_LOG_INFO("Month:  %d", rtc_data.month);
    NRF_LOG_INFO("Date:   %d", rtc_data.date);
    NRF_LOG_INFO("Day:    %d", rtc_data.day);
    NRF_LOG_INFO("Hour:   %d", rtc_data.hour);
    NRF_LOG_INFO("Minute: %d", rtc_data.minute);
    NRF_LOG_INFO("Second: %d\n", rtc_data.second);
    //NRF_LOG_INFO("Hundreth of a Second: %d", rtc_data[7]);
}

/**
 * @brief TWI events handler.
 */
void twi_handler_ds(nrf_drv_twi_evt_t const * p_event, void * p_context)
{

    // Should specific errors occur in I2C communication (such as slave NACK),
    // corresponding error message is printed to serial

    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done_ds = true;
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            NRF_LOG_INFO("\r\nDATA NACK ERROR");
            NRF_LOG_FLUSH();
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            NRF_LOG_INFO("\r\nADDRESS NACK ERROR");
            NRF_LOG_FLUSH();
            break;
        default:
            break;
    }
}

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    // Initializes the I2C connection to 400 kHz,
    // using the SDA and SCL pins for the board
    // (see README to change which board/platform is in use)

    const nrf_drv_twi_config_t twi_ds1388_config = {
       .scl                = I2C_SCL,
       .sda                = I2C_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };
    
    err_code = nrf_drv_twi_init(&ds_twi, &twi_ds1388_config, twi_handler_ds, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&ds_twi);
}
/**
 * @brief Performs a one-byte RTC internal register read.
 */
uint8_t readRegister(uint8_t reg_addr)
{
    do
        {
            __WFE();
        }while (m_xfer_done_ds == false);

    m_xfer_done_ds = false;

    // The descriptor below is used to properly handle the ds1388's read protocol
    // The ds1388 device address is selected, the register to read is passed and the read byte is returned
    // See the nrf SDK reference guide for more information on nrf_drv_twi_xfer_desc_t
    nrf_drv_twi_xfer_desc_t const ds_desc = {NRFX_TWI_XFER_TXRX, DS1388_ADDRESS, sizeof(reg_addr), sizeof(byte), &reg_addr, &byte};
    nrf_drv_twi_xfer(&ds_twi, &ds_desc, false); // initiates transfer using I2C descriptor above
    while(nrf_drv_twi_is_busy(&ds_twi) == true); // waits for the transfer to complete

    return byte; // read byte is returned
}

/**
 * @brief Reads all of the date and time registers, converts their hex
 * values to decimal, and then stores them within the date struct. 
 * Returns:
 * 0 if AM mode
 * 1 if PM mode
 * 2 if 24-hour mode
 */
uint8_t get_time()
{
  uint8_t ret;
  
  date.year = readRegister(YEAR_REG);
  date.month = readRegister(MONTH_REG);
  date.date = readRegister(DATE_REG);
  date.day = readRegister(DAY_REG);
  date.hour = readRegister(HOUR_REG);
  date.minute = readRegister(MIN_REG);
  date.second = readRegister(SEC_REG);
  date.hundreth = readRegister(HUNDRED_SEC_REG);
  
  //Time processing 
  date.year = hex2dec(date.year);
  date.month = hex2dec(date.month);
  date.date = hex2dec(date.date);
  date.minute = hex2dec(date.minute);
  date.second = hex2dec(date.second);
  date.hundreth = hex2dec(date.hundreth);

  
  if ((date.hour & 0x40) == HOUR_MODE_24)
  {
    date.hour = hex2dec(date.hour);
    ret = 2;
    return ret;
  }
  else
  { 
    ret = (date.hour & 0x20 )>> 5;
    date.hour = hex2dec(date.hour & 0x1F);
    return ret;
  }
}

/**
 * @brief Function for main application entry. Initializes I2C,
 * configures RTC, and prints current date and time to serial
 * once every second.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nRTC test code started");
    NRF_LOG_FLUSH();
    twi_init(); // initializes I2C
	ds_config(); // configures the ds1388

    while (true)
    {
        nrf_delay_ms(1000);
        get_time(); // gets the current time and date stored by the RTC
        data_handler(date); // prints the current time and date to serial
        NRF_LOG_FLUSH();
    }
}

/** @} */