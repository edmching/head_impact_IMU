//-------------------------------------------
// Name: ds1388.c
// Author: Gregor Morrison
// Description: This driver file contains functions for communication
// with the real-time clock over I2C.
// This code on its own requires manual setting of the RTC - for
// autoset functionality, reference "\ble_app\test\ds1388_autoset_test"
//
// Referenced code: https://github.com/DelfiSpace/DS1388/blob/master/DS1388.cpp
//-------------------------------------------
#include "ds1388.h"

/* RTC variables. */
static uint8_t byte;
// This array holds numerical values corresponding to the 
// date or time variables as indicated below. Manually change
// the values in the array to modify the date/time as desired.
// Note: see the file "ds1388_auto.c" for autosetting the RTC
static uint8_t time_format =  HOUR_MODE_24; //select either 12-HOUR FORMAT or 24-HOUR FORMAT, if 12-HOUR FORMAT, use together with AM, PM eg: HOUR_MODE_12 | PM
static uint8_t init_time[8] = {20, 4, 26, 6, 21, 53, 25, 0};

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
void ds1388_config(void)
{	
    // Prepares target register (CONTROL_REG) and data to be written (EN_OSCILLATOR | DIS_WD_COUNTER)
    // In this case, the RTC oscillator is enabled and the watch dog counter is disabled
	  uint8_t reg0[2] = {CONTROL_REG, (EN_OSCILLATOR | DIS_WD_COUNTER)};

    //Sets RST HIGH to enable the RTC
    nrf_gpio_cfg_output(RTC_RST_PIN);
    nrf_gpio_pin_set(RTC_RST_PIN);
    nrf_delay_ms(500);

    m_xfer_done = false;
    nrf_drv_twi_tx(&twi, DS1388_ADDRESS, reg0, sizeof(reg0), false);// initiates transfer using I2C descriptor above
    while (m_xfer_done == false);// waits for the transfer to complete
  
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

    nrf_drv_twi_tx(&twi, DS1388_ADDRESS, reg1, sizeof(reg1), false);
    while (m_xfer_done == false);

    NRF_LOG_INFO("RTC initialized");
}

/**
 * @brief Performs a one-byte RTC internal register read.
 */
uint8_t ds1388_readRegister(uint8_t reg_addr)
{
    do
        {
            __WFE();
        }while (m_xfer_done == false);

    m_xfer_done = false;

    // The descriptor below is used to properly handle the ds1388's read protocol
    // The ds1388 device address is selected, the register to read is passed and the read byte is returned
    // See the nrf SDK reference guide for more information on nrf_drv_twi_xfer_desc_t
    nrf_drv_twi_xfer_desc_t const ds_desc = {NRFX_TWI_XFER_TXRX, DS1388_ADDRESS, sizeof(reg_addr), sizeof(byte), &reg_addr, &byte};

    nrf_drv_twi_xfer(&twi, &ds_desc, false);
    
    while(nrf_drv_twi_is_busy(&twi) == true);

    return byte;
}
/**
 * @brief Reads all of the date and time registers, converts their hex
 * values to decimal, and then stores them within the date struct. 
 * Returns:
 * 0 if AM mode
 * 1 if PM mode
 * 2 if 24-hour mode
 */
uint8_t ds1388_get_time(ds1388_data_t* date)
{
  uint8_t ret;
  
  date->year = ds1388_readRegister(YEAR_REG);
  date->month = ds1388_readRegister(MONTH_REG);
  date->date = ds1388_readRegister(DATE_REG);
  date->day = ds1388_readRegister(DAY_REG);
  date->hour = ds1388_readRegister(HOUR_REG);
  date->minute = ds1388_readRegister(MIN_REG);
  date->second = ds1388_readRegister(SEC_REG);
  date->hundreth = ds1388_readRegister(HUNDRED_SEC_REG);
  
  //Time processing 
  date->year = hex2dec(date->year);
  date->month = hex2dec(date->month);
  date->date = hex2dec(date->date);
  date->minute = hex2dec(date->minute);
  date->second = hex2dec(date->second);
  date->hundreth = hex2dec(date->hundreth);

  
  if ((date->hour & 0x40) == HOUR_MODE_24)
  {
    date->hour = hex2dec(date->hour);
    ret = 2;
    return ret;
  }
  else
  { 
    ret = (date->hour & 0x20 )>> 5;
    date->hour = hex2dec(date->hour & 0x1F);
    return ret;
  }
}


