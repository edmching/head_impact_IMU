#include "ds1388.h"

/* RTC variables. */
static uint8_t byte;
static uint8_t time_format =  HOUR_MODE_24; //select either 12-HOUR FORMAT or 24-HOUR FORMAT, if 12-HOUR FORMAT, use together with AM, PM eg: HOUR_MODE_12 | PM
static uint8_t init_time[8] = {20, 4, 26, 6, 21, 53, 25, 0};

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
    nrf_gpio_cfg_output(RTC_RST_PIN);
    nrf_gpio_pin_set(RTC_RST_PIN);
    nrf_delay_ms(500);
    m_xfer_done = false;
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


