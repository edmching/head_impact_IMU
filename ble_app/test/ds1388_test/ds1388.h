//-------------------------------------------
// Name: ds1388.h
// Author: UBC Capstone Team 48 - 2019/2020
// Description: This header file contains the
// addresses, registers and commands needed
// to read/write to the ds1388 RTC using 
// ds1388.c
// See ds1388 datasheet for more information 
//
// Referenced code: https://github.com/DelfiSpace/DS1388/blob/master/DS1388.h
//-------------------------------------------

//DS1388 address
#define DS1388_ADDRESS      0x68  // b1101000, for RTC and WD

//DS1388 Registers
#define HUNDRED_SEC_REG     0x00  // hundredth of seconds
#define SEC_REG             0x01  // seconds
#define MIN_REG             0x02  // minutes
#define HOUR_REG            0x03  // hour
#define DAY_REG             0x04  // day
#define DATE_REG            0x05  // date
#define MONTH_REG           0x06  // month
#define YEAR_REG            0x07  // year
#define WD_HUNDRED_SEC_REG  0x08  // watchdog hundredth seconds
#define WD_SEC_REG          0x09  // watchdog seconds
#define TRICKLE_CHG_REG     0x0A  // trickle charger
#define FLAG_REG            0x0B  // flags
#define CONTROL_REG         0x0C  // control

// DS1388 Control Register
#define EN_OSCILLATOR       0x00 // enables internal RTC oscillator
#define DIS_OSCILLATOR      0x80 // disables internal RTC oscillator
#define EN_WD_COUNTER       0x02
#define DIS_WD_COUNTER      0x00    
#define WD_RST              0x01 

// Time formats
#define HOUR_MODE_12        0x40
#define HOUR_MODE_24        0x00
#define AM                  0x00
#define PM                  0x20