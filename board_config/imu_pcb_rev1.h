/**
* Copyright (c) 2018 makerdiary
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*
* * Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above
*   copyright notice, this list of conditions and the following
*   disclaimer in the documentation and/or other materials provided
*   with the distribution.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
#ifndef IMU_PCB_REV1_H
#define IMU_PCB_REV1_H 

#ifdef __cplusplus
extern "C" {
#endif
#ifdef USE_BLE_CLI
    //temporary definitions in order to run ble_app_cli
    // PCB does not have any leds or buttons avaliable for use
    // LEDs definitions for nRF52832-MDK
    #define LEDS_NUMBER    3
    
    #define LED_START      22
    #define LED_1          22
    #define LED_2          23
    #define LED_3          24
    #define LED_STOP       24
    
    #define LEDS_ACTIVE_STATE 0
    
    #define LEDS_INV_MASK  LEDS_MASK
    
    #define LEDS_LIST { LED_1, LED_2, LED_3 }
    
    #define BSP_LED_0      LED_1
    #define BSP_LED_1      LED_2
    #define BSP_LED_2      LED_3
    
    #define BUTTONS_NUMBER 4
    #define BUTTON_1       27    // Connect Grove-Button at Base Dock Grove Port#1
    #define BUTTON_2       29    // Connect Grove-Button at Base Dock Grove Port#2
    #define BUTTON_3       31    // Connect Grove-Button at Base Dock Grove Port#3
    #define BUTTON_4       3     // Connect Grove-Button at Base Dock Grove Port#4
    
    #define BUTTON_PULL    NRF_GPIO_PIN_PULLDOWN
    
    #define BUTTONS_ACTIVE_STATE 1
    
    #define BUTTONS_LIST { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4}
    
    #define BSP_BUTTON_0   BUTTON_1
    #define BSP_BUTTON_1   BUTTON_2
    #define BSP_BUTTON_2   BUTTON_3
    #define BSP_BUTTON_3   BUTTON_4
#endif


#define RX_PIN_NUMBER  23
#define TX_PIN_NUMBER  24
#define HWFC           false

//===================SPI PINOUT=========
//gyro - PIN names are mapped wrong in this PCB rev1.
//This is corrected in PCB rev2
//gyro's SDI should be mapped to GYRO_MOSI (Pin 0.00)
//gyro's SDO should be mapped to GYRO_MISO (Pin 0.01)
//So these pins are corrected below for this rev1
#define SPI_GYRO_MOSI_PIN            1 
#define SPI_GYRO_MISO_PIN            0 
#define SPI_GYRO_SCK_PIN             2
#define SPI_GYRO_CS_PIN              3

#define SPI_ACCEL_MOSI_PIN            10
#define SPI_ACCEL_MISO_PIN            9
#define SPI_ACCEL_SCK_PIN             11
#define SPI_ACCEL_CS_PIN              8

//flash - pin names are mapped wrong in this PCB rev1
//This is corrected in PCB rev2
// flash's DQ0 should be mapped to MEM_MOSI (pin 0.16)
// flash's DQ1 should be mapped to MEM_MISO (pin 0.12)
//So these pins are corrected below for this rev1
#define SPI_FLASH_MOSI_PIN      12 
#define SPI_FLASH_MISO_PIN      16
#define SPI_FLASH_SCK_PIN       13
#define SPI_FLASH_CS_PIN        15

//===================I2C PINOUT========================//
#define I2C_SCL                 19
#define I2C_SDA                 20
#define RTC_RST_PIN             22

#ifdef __cplusplus
}
#endif

#endif // IMU_PCB_REV1_H
