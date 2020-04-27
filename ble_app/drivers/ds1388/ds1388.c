#include "ds1388.h"

/* RTC variables. */
static uint8_t byte;
static uint8_t time_format =  HOUR_MODE_24; //select either 12-HOUR FORMAT or 24-HOUR FORMAT, if 12-HOUR FORMAT, use together with AM, PM eg: HOUR_MODE_12 | PM
static uint8_t init_time[8] = {20, 2, 12, 3, 4, 1, 0, 0};
//init_time[0] = 20; (year)
//init_time[1] = 2;  (month)
//init_time[2] = 1;  (date)
//init_time[3] = 6;  (day of the week, MONDAY: 1, TUESDAY: 2, ... SUNDAY: 7)
//init_time[4] = 4;  (hour) 
//init_time[5] = 25; (minutes) 
//init_time[6] = 0;  (seconds)
//init_time[7] = 0;  (hundredth of second)
 
/**
 * @brief Function for converting from dec to hex
 */
uint8_t dec2hex(uint8_t val) {
  val = val + 6 * (val / 10);
  return val;
}

/**
 * @brief Function for converting from hex to dec
 */
uint8_t hex2dec(uint8_t val) {
  val = val - 6 * (val >> 4);
  return val;
}

/**
 * @brief Function for setting active mode on DS1388 RTC
 */
void ds1388_config(void)
{	
	uint8_t reg0[2] = {CONTROL_REG, (EN_OSCILLATOR | DIS_WD_COUNTER)};
    nrf_drv_twi_tx(&twi, DS1388_ADDRESS, reg0, sizeof(reg0), false);
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

    nrf_drv_twi_tx(&twi, DS1388_ADDRESS, reg1, sizeof(reg1), false);
    while (m_xfer_done == false);

    NRF_LOG_INFO("RTC initialized");
}

uint8_t ds1388_readRegister(uint8_t reg_addr)
{
    do
        {
            __WFE();
        }while (m_xfer_done == false);

    m_xfer_done = false;

    nrf_drv_twi_xfer_desc_t const ds_desc = {NRFX_TWI_XFER_TXRX, DS1388_ADDRESS, sizeof(reg_addr), sizeof(byte), &reg_addr, &byte};

    nrf_drv_twi_xfer(&twi, &ds_desc, false);
    
    while(nrf_drv_twi_is_busy(&twi) == true);

    return byte;
}

uint8_t ds1388_get_time(ds1388_data_t* date)
{
  uint8_t ret;
  
  date->year = readRegister(YEAR_REG);
  date->month = readRegister(MONTH_REG);
  date->date = readRegister(DATE_REG);
  date->day = readRegister(DAY_REG);
  date->hour = readRegister(HOUR_REG);
  date->minute = readRegister(MIN_REG);
  date->second = readRegister(SEC_REG);
  date->hundreth = readRegister(HUNDRED_SEC_REG);
  
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


