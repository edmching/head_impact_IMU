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
#define RX_PIN_NUMBER  23
#define TX_PIN_NUMBER  24
#define HWFC           false

//===================SPI PINOUT========================//
#define SPI_GYRO_MOSI_PIN            1 //gyro - pins 0 & 1 are flipped
#define SPI_GYRO_MISO_PIN            0 
#define SPI_GYRO_SCK_PIN             2
#define SPI_GYRO_CS_PIN              3

#define SPI_ACCEL_MOSI_PIN            10
#define SPI_ACCEL_MISO_PIN            9
#define SPI_ACCEL_SCK_PIN             11
#define SPI_ACCEL_CS_PIN              8

#define SPI_FLASH_MOSI_PIN      19
#define SPI_FLASH_MISO_PIN      15
#define SPI_FLASH_SCK_PIN       14
#define SPI_FLASH_CS_PIN        18

//===================I2C PINOUT========================//
#define I2C_SCL                 19
#define I2C_SDA                 20
#define RTC_RST_PIN             22

#ifdef __cplusplus
}
#endif

#endif // IMU_PCB_REV1_H
