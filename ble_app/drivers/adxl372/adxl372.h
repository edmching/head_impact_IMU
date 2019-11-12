/***************************************************************************//**
 *   @file   adxl372.h
 *   @brief  ADXL372 device driver.
********************************************************************************
 * Copyright 2017(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*******************************************************************************/

#ifndef ADXL372_H_
#define ADXL372_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h> 
#include <nrf_drv_spi.h>

#define L_ENDIAN

/* Register address */
#define ADI_ADXL372_ADI_DEVID           0x00u   /* Analog Devices, Inc., accelerometer ID */
#define ADI_ADXL372_MST_DEVID          	0x01u   /* Analog Devices MEMS device ID */
#define ADI_ADXL372_DEVID               0x02u   /* Device ID */
#define ADI_ADXL372_REVID               0x03u   /* product revision ID*/
#define ADI_ADXL372_STATUS_1            0x04u   /* Status register 1 */
#define ADI_ADXL372_STATUS_2            0x05u   /* Status register 2 */
#define ADI_ADXL372_FIFO_ENTRIES_2	0x06u   /* Valid data samples in the FIFO */
#define ADI_ADXL372_FIFO_ENTRIES_1	0x07u   /* Valid data samples in the FIFO */
#define ADI_ADXL372_X_DATA_H            0x08u   /* X-axis acceleration data [11:4] */
#define ADI_ADXL372_X_DATA_L            0x09u   /* X-axis acceleration data [3:0] | dummy LSBs */
#define ADI_ADXL372_Y_DATA_H            0x0Au   /* Y-axis acceleration data [11:4] */
#define ADI_ADXL372_Y_DATA_L		0x0Bu   /* Y-axis acceleration data [3:0] | dummy LSBs */
#define ADI_ADXL372_Z_DATA_H            0x0Cu   /* Z-axis acceleration data [11:4] */
#define ADI_ADXL372_Z_DATA_L            0x0Du   /* Z-axis acceleration data [3:0] | dummy LSBs */
#define ADI_ADXL372_X_MAXPEAK_H         0x15u   /* X-axis MaxPeak acceleration data [15:8] */
#define ADI_ADXL372_X_MAXPEAK_L         0x16u   /* X-axis MaxPeak acceleration data [7:0] */
#define ADI_ADXL372_Y_MAXPEAK_H        	0x17u   /* X-axis MaxPeak acceleration data [15:8] */
#define ADI_ADXL372_Y_MAXPEAK_L         0x18u   /* X-axis MaxPeak acceleration data [7:0] */
#define ADI_ADXL372_Z_MAXPEAK_H         0x19u   /* X-axis MaxPeak acceleration data [15:8] */
#define ADI_ADXL372_Z_MAXPEAK_L         0x1Au   /* X-axis MaxPeak acceleration data [7:0] */
#define ADI_ADXL372_OFFSET_X 	        0x20u   /* X axis offset */
#define ADI_ADXL372_OFFSET_Y    	0x21u   /* Y axis offset */
#define ADI_ADXL372_OFFSET_Z	        0x22u   /* Z axis offset */
#define ADI_ADXL372_X_THRESH_ACT_H      0x23u   /* X axis Activity Threshold [15:8] */
#define ADI_ADXL372_X_THRESH_ACT_L	0x24u   /* X axis Activity Threshold [7:0] */
#define ADI_ADXL372_Y_THRESH_ACT_H      0x25u   /* Y axis Activity Threshold [15:8] */
#define ADI_ADXL372_Y_THRESH_ACT_L      0x26u   /* Y axis Activity Threshold [7:0] */
#define ADI_ADXL372_Z_THRESH_ACT_H	0x27u   /* Z axis Activity Threshold [15:8] */
#define ADI_ADXL372_Z_THRESH_ACT_L	0x28u   /* Z axis Activity Threshold [7:0] */
#define ADI_ADXL372_TIME_ACT	        0x29u   /* Activity Time */
#define ADI_ADXL372_X_THRESH_INACT_H    0x2Au   /* X axis Inactivity Threshold [15:8] */
#define ADI_ADXL372_X_THRESH_INACT_L	0x2Bu   /* X axis Inactivity Threshold [7:0] */
#define ADI_ADXL372_Y_THRESH_INACT_H    0x2Cu   /* Y axis Inactivity Threshold [15:8] */
#define ADI_ADXL372_Y_THRESH_INACT_L    0x2Du   /* Y axis Inactivity Threshold [7:0] */
#define ADI_ADXL372_Z_THRESH_INACT_H	0x2Eu   /* Z axis Inactivity Threshold [15:8] */
#define ADI_ADXL372_Z_THRESH_INACT_L	0x2Fu   /* Z axis Inactivity Threshold [7:0] */
#define ADI_ADXL372_TIME_INACT_H        0x30u   /* Inactivity Time [15:8] */
#define ADI_ADXL372_TIME_INACT_L        0x31u   /* Inactivity Time [7:0] */
#define ADI_ADXL372_X_THRESH_ACT2_H     0x32u   /* X axis Activity2 Threshold [15:8] */
#define ADI_ADXL372_X_THRESH_ACT2_L	0x33u   /* X axis Activity2 Threshold [7:0] */
#define ADI_ADXL372_Y_THRESH_ACT2_H     0x34u   /* Y axis Activity2 Threshold [15:8] */
#define ADI_ADXL372_Y_THRESH_ACT2_L     0x35u   /* Y axis Activity2 Threshold [7:0] */
#define ADI_ADXL372_Z_THRESH_ACT2_H	0x36u   /* Z axis Activity2 Threshold [15:8] */
#define ADI_ADXL372_Z_THRESH_ACT2_L	0x37u   /* Z axis Activity2 Threshold [7:0] */
#define ADI_ADXL372_HPF 	        0x38u   /* High Pass Filter */
#define ADI_ADXL372_FIFO_SAMPLES        0x39u   /* FIFO Samples */
#define ADI_ADXL372_FIFO_CTL	        0x3Au   /* FIFO Control */
#define ADI_ADXL372_INT1_MAP            0x3Bu   /* Interrupt 1 mapping control */
#define ADI_ADXL372_INT2_MAP            0x3Cu   /* Interrupt 2 mapping control */
#define ADI_ADXL372_TIMING	        0x3Du   /* Timing */
#define ADI_ADXL372_MEASURE		0x3Eu   /* Measure */
#define ADI_ADXL372_POWER_CTL           0x3Fu   /* Power control */
#define ADI_ADXL372_SELF_TEST           0x40u   /* Self Test */
#define ADI_ADXL372_SRESET              0x41u   /* Reset */
#define ADI_ADXL372_FIFO_DATA		0x42u   /* FIFO Data */

#define ADI_ADXL372_ADI_DEVID_VAL       0xADu   /* Analog Devices, Inc., accelerometer ID */
#define ADI_ADXL372_MST_DEVID_VAL       0x1Du   /* Analog Devices MEMS device ID */
#define ADI_ADXL372_DEVID_VAL           0xFAu   /* Device ID */
#define ADI_ADXL372_REVID_VAL           0x02u   /* product revision ID*/
#define ADI_ADXL372_RESET_CODE          0x52u	/* Writing code 0x52 resets the device */

/* ADXL372_MEASURE */
#define MEASURE_AUTOSLEEP_MASK		0xBF
#define MEASURE_AUTOSLEEP_POS		6
#define MEASURE_BANDWIDTH_MASK		0xF8
#define MEASURE_BANDWIDTH_POS       0
#define MEASURE_ACTPROC_MASK		0xCF
#define MEASURE_ACTPROC_POS		    4

/* ADXL372_TIMING */
#define TIMING_ODR_MASK			    0x1F
#define TIMING_ODR_POS			    5
#define TIMING_WUR_MASK 		    0xE3
#define TIMING_WUR_POS 			    2

/* ADXL372_POWER_CTL */
#define PWRCTRL_OPMODE_MASK		    0xFC
#define PWRCTRL_OPMODE_POS          0
#define PWRCTRL_INSTON_THRESH_MASK	0xDF
#define PWRCTRL_FILTER_SETTLE_MASK	0xEF
#define PWRCTRL_FILTER_SETTLE_POS	4

#define INSTAON_THRESH_POS		    5

/* ADXL372_FIFO_CTL */
#define FIFO_CRL_SAMP8_POS		    0
#define FIFO_CRL_MODE_POS		    1
#define FIFO_CRL_FORMAT_POS		    3


#define DATA_RDY	  1
#define FIFO_RDY	  2
#define FIFO_FULL	  4
#define FIFO_OVR	  8
#define FIFO_OVR_POS  3
#define FIFO_FULL_POS 2
#define FIFO_RDY_POS  1


#define ADXL_SPI_RNW    1 /* Sets the Read bit R/W */

/*Acceleremoter configuration*/
#define ACT_VALUE          30     /* Activity threshold value */

#define INACT_VALUE        30     /* Inactivity threshold value */

#define ACT_TIMER          1    /* Activity timer value in multiples of 3.3ms */

#define INACT_TIMER        1     /* Inactivity timer value in multiples of 26ms */

#define ADXL_INT1_PIN     7 //not used currently
#define ADXL_INT2_PIN     5 //not used currently

typedef nrf_drv_spi_t  adxl372_spi_handle_t;

typedef enum {
    STAND_BY = 0,
    WAKE_UP,
    INSTANT_ON,
    FULL_BW_MEASUREMENT
} adxl372_op_mode_t;

typedef enum {
    ODR_400HZ = 0,
    ODR_800HZ,
    ODR_1600HZ,
    ODR_3200HZ,
    ODR_6400HZ
} adxl372_odr_t;

typedef enum {
    BW_200HZ = 0,
    BW_400HZ,
    BW_800HZ,
    BW_1600HZ,
    BW_3200HZ
} adxl372_bw_t;

typedef enum {
    WUR_52MS = 0,
    WUR_104MS,
    WUR_208MS,
    WUR_512MS,
    WUR_2048MS,
    WUR_4096MS,
    WUR_8192MS,
    WUR_24576MS
} adxl372_wakeup_rate_t;


typedef enum {
    DEF = 0,
    LINKED,
    LOOPED
} adxl372_act_proc_mode_t;


typedef enum {
    XYZ_FIFO = 0,
    X_FIFO,
    Y_FIFO,
    XY_FIFO,
    Z_FIFO,
    XZ_FIFO,
    YZ_FIFO,
    XYZ_PEAK_FIFO
} adxl372_fifo_format_t;

typedef enum {
    BYPASSED = 0,
    STREAMED,
    TRIGGERED,
    OLDEST_SAVED
} adxl372_fifo_mode_t;

typedef struct {
    unsigned short samples;
    adxl372_fifo_mode_t mode;
    adxl372_fifo_format_t format;
} fifo_config_t;

typedef enum {
    ADXL_INSTAON_LOW_THRESH = 0,
    ADXL_INSTAON_HIGH_THRESH
} adxl372_instaon_thresh_t;

typedef enum {
    FILTER_SETTLE_16 = 0,
    FILTER_SETTLE_370
} adxl372_filter_settle_t;


typedef struct {
    int16_t x; 
    int16_t y;
    int16_t z;
} adxl372_accel_data_t; 


struct adxl372_device {
    adxl372_spi_handle_t *spi;
    fifo_config_t fifo_config;
};

void adxl372_init (void);

uint8_t* adxl372_read_reg( uint8_t reg_addr);

void adxl372_write_reg( uint8_t reg_addr, uint8_t reg_data);

uint8_t* adxl372_multibyte_read_reg( uint8_t reg_addr, uint16_t num_bytes);

void adxl372_write_mask(uint8_t reg_addr, uint32_t mask, uint32_t pos, uint8_t val);

void adxl372_set_op_mode(adxl372_op_mode_t mode);

void adxl372_set_odr(adxl372_odr_t odr);

void adxl372_set_wakeup_rate(adxl372_wakeup_rate_t wur);

void adxl372_set_bandwidth(adxl372_bw_t bw);

void adxl372_set_autosleep(bool enable);

void adxl372_set_act_proc_mode (adxl372_act_proc_mode_t mode);

void adxl372_set_instaon_threshold(adxl372_instaon_thresh_t mode);

void adxl372_set_activity_threshold(uint16_t  thresh, bool referenced, bool enable);

void adxl372_set_activity2_threshold(uint16_t  thresh, bool referenced, bool enable);

void adxl372_set_inactivity_threshold(uint16_t thresh, bool referenced, bool enable);

void adxl372_set_activity_time(uint8_t time);

void adxl372_set_inactivity_time(uint8_t time);

void adxl372_set_filter_settle(adxl372_filter_settle_t mode);

uint8_t adxl372_get_dev_ID(void);

uint8_t adxl372_get_status_reg(void);

uint8_t adxl372_get_activity_status_reg(void);

void adxl372_get_highest_peak_accel_data(adxl372_accel_data_t* max_peak);

void adxl372_get_accel_data(adxl372_accel_data_t* accel_data);

void adxl372_reset(void);

int32_t adxl372_configure_fifo (struct adxl372_device* dev, uint16_t fifo_samples, adxl372_fifo_mode_t fifo_mode, adxl372_fifo_format_t fifo_format);

int32_t adxl372_get_fifo_data(struct adxl372_device *dev, adxl372_accel_data_t *fifo_data);

void adxl372_set_interrupts(void);

#endif /* ADXL372_H_ */

