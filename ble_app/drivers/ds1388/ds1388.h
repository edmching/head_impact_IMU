#ifndef DS1388_H
#define DS1388_H

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

uint8_t ds1388_get_time(ds1388_data_t* date);

uint8_t ds1388_readRegister(uint8_t reg_addr);

void ds1388_config(void);

uint8_t dec2hex(uint8_t val);

uint8_t hex2dec(uint8_t val);

#endif //DS1388_H