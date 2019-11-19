#include "adxl372.h"
#include "spi_driver.h"
#include "nrf_delay.h"
#include "nrf_log.h"


void adxl372_init (void)
{
    //Setup SPI 
    spi_init();
    /*
    GPIO for INT1 pin and INT2 pin,
    CS lines
    gpio_init();
    */

    //initialize device settings
    /* set up measurement mode */
    adxl372_set_op_mode(STAND_BY);
    adxl372_set_hpf_disable(true);
    adxl372_set_lpf_disable(true);
    adxl372_set_bandwidth(BW_3200HZ);
    adxl372_set_odr(ODR_6400HZ);
    adxl372_set_filter_settle(FILTER_SETTLE_16);
    adxl372_set_op_mode(FULL_BW_MEASUREMENT);

}


/*
 * Read register from adxl372
 * @param reg_addr - register address
 * @return the register data
 */
uint8_t* adxl372_read_reg( uint8_t reg_addr) 
{
    uint8_t read_addr;

    read_addr = ((reg_addr & 0xFF) << 1 | 0x01); //set R bit to 1

    return spi_write_and_read(&read_addr, 1);
}

/*
 * Write to an adxl372 register
 * @param reg_addr - The register address to write to
 * @param reg_data - The register data to send to
 * @return none
 */
void adxl372_write_reg(uint8_t reg_addr, uint8_t reg_data)
{
    uint8_t tx_buf[2]; // write address + write data

    tx_buf[0] = (reg_addr & 0xFF) << 1; //addr is 7-bits
    tx_buf[1] = reg_data;
    spi_write_and_read( tx_buf, 2);
}

/*
 * Read multiple bytes from an adxl372 register
 * @param reg_addr - The register address to read from
 * @param num_bytes - number of bytes to read
 * @return the register data
 */
uint8_t* adxl372_multibyte_read_reg( uint8_t reg_addr, uint16_t num_bytes) 
{
    uint8_t read_addr;
    
    read_addr = ((reg_addr & 0xFF) << 1 | 0x01); //set R bit to 1

    return spi_write_and_read(&read_addr, num_bytes);
}

void adxl372_write_mask(uint8_t reg_addr, uint32_t mask, uint32_t pos, uint8_t val)
{
    uint8_t reg_data = 0;

    reg_data &= mask; 
    reg_data |= (val << pos) & ~mask;

    adxl372_write_reg(reg_addr, reg_data);
}

/* ADXL372 Register Functions*/
void adxl372_set_op_mode(adxl372_op_mode_t mode)
{
    adxl372_write_mask(ADI_ADXL372_POWER_CTL, PWRCTRL_OPMODE_MASK, PWRCTRL_OPMODE_POS, mode);
}

void adxl372_set_hpf_disable(bool set)
{
    adxl372_write_mask(ADI_ADXL372_POWER_CTL, PWRCTRL_HPF_DISABLE_MASK, PWRCTRL_HPF_DISABLE_POS, set);
}

void adxl372_set_lpf_disable(bool set)
{
    adxl372_write_mask(ADI_ADXL372_POWER_CTL, PWRCTRL_LPF_DISABLE_MASK, PWRCTRL_LPF_DISABLE_POS, set);
}

void adxl372_set_odr(adxl372_odr_t odr)
{
    adxl372_write_mask(ADI_ADXL372_TIMING, TIMING_ODR_MASK, TIMING_ODR_POS, odr);
}

void adxl372_set_wakeup_rate(adxl372_wakeup_rate_t wur)
{
    adxl372_write_mask(ADI_ADXL372_TIMING, TIMING_WUR_MASK, TIMING_WUR_POS, wur);
}

void adxl372_set_bandwidth(adxl372_bw_t bw)
{
    adxl372_write_mask(ADI_ADXL372_MEASURE, MEASURE_BANDWIDTH_MASK, MEASURE_BANDWIDTH_POS, bw);
}

void adxl372_set_autosleep(bool enable)
{
    adxl372_write_mask(ADI_ADXL372_MEASURE, MEASURE_AUTOSLEEP_MASK, MEASURE_AUTOSLEEP_POS,enable);
}

void adxl372_set_act_proc_mode (adxl372_act_proc_mode_t mode)
{
    adxl372_write_mask(ADI_ADXL372_MEASURE, MEASURE_ACTPROC_MASK, MEASURE_ACTPROC_POS, mode);
}

void adxl372_set_instaon_threshold(adxl372_instaon_thresh_t mode)
{
    adxl372_write_mask(ADI_ADXL372_POWER_CTL, PWRCTRL_INSTON_THRESH_MASK, INSTAON_THRESH_POS, mode);
}

void adxl372_set_activity_threshold(uint16_t  thresh, bool referenced, bool enable)
{
    adxl372_set_op_mode(STAND_BY);

    adxl372_write_reg(ADI_ADXL372_X_THRESH_ACT_H, thresh >> 3);
    adxl372_write_reg(ADI_ADXL372_X_THRESH_ACT_L, 
                        (thresh << 5) | (referenced << 1) | enable);
    adxl372_write_reg(ADI_ADXL372_Y_THRESH_ACT_H, thresh >> 3);
    adxl372_write_reg(ADI_ADXL372_Y_THRESH_ACT_L, (thresh << 5) | enable);
    adxl372_write_reg(ADI_ADXL372_Z_THRESH_ACT_H, thresh >> 3);
    adxl372_write_reg(ADI_ADXL372_Z_THRESH_ACT_L, (thresh << 5) | enable);
}

void adxl372_set_activity2_threshold(uint16_t  thresh, bool referenced, bool enable)
{
    adxl372_set_op_mode(STAND_BY);
    adxl372_write_reg(ADI_ADXL372_X_THRESH_ACT2_H, thresh >> 3);
    adxl372_write_reg(ADI_ADXL372_X_THRESH_ACT2_L,
                        (thresh << 5) | (referenced << 1) | enable);
    adxl372_write_reg( ADI_ADXL372_Y_THRESH_ACT2_H, thresh >> 3);
    adxl372_write_reg(ADI_ADXL372_Y_THRESH_ACT2_L, (thresh << 5) | enable);
    adxl372_write_reg(ADI_ADXL372_Z_THRESH_ACT2_H, thresh >> 3);
    adxl372_write_reg(ADI_ADXL372_Z_THRESH_ACT2_L, (thresh << 5) | enable);

}

void adxl372_set_inactivity_threshold(uint16_t thresh, bool referenced, bool enable)
{
    adxl372_set_op_mode(STAND_BY); 

    adxl372_write_reg(ADI_ADXL372_X_THRESH_INACT_H, thresh >> 3);
    adxl372_write_reg(ADI_ADXL372_X_THRESH_INACT_L,
                        (thresh << 5) | (referenced << 1) | enable);
    adxl372_write_reg(ADI_ADXL372_Y_THRESH_INACT_H, thresh >> 3);
    adxl372_write_reg(ADI_ADXL372_Y_THRESH_INACT_L, (thresh << 5) | enable);
    adxl372_write_reg(ADI_ADXL372_Z_THRESH_INACT_H, thresh >> 3);
    adxl372_write_reg(ADI_ADXL372_Z_THRESH_INACT_L, (thresh << 5) | enable);
}

void adxl372_set_activity_time(uint8_t time)
{
    adxl372_write_reg(ADI_ADXL372_TIME_ACT, time);
}

void adxl372_set_inactivity_time(uint8_t time)
{
    adxl372_write_reg(ADI_ADXL372_TIME_INACT_H, time >> 8);
    adxl372_write_reg(ADI_ADXL372_TIME_INACT_L, time & 0xFF);
}

void adxl372_set_filter_settle(adxl372_filter_settle_t mode)
{
    adxl372_write_mask(ADI_ADXL372_POWER_CTL, PWRCTRL_FILTER_SETTLE_MASK, PWRCTRL_FILTER_SETTLE_POS, mode);
}

/*
 *  Gets the device ID 
 *  @return device ID of adxl372
 */
uint8_t adxl372_get_dev_ID(void)
{
    uint8_t* device_ID;
    device_ID = adxl372_read_reg(ADI_ADXL372_ADI_DEVID);
    
    return device_ID[0];
}

uint8_t adxl372_get_status_reg(void)
{
    uint8_t* status_reg;

    status_reg = adxl372_read_reg(ADI_ADXL372_STATUS_1);

    return status_reg[0];    
}

uint8_t adxl372_get_activity_status_reg(void)
{
    uint8_t* activity_status_reg;

    activity_status_reg = adxl372_read_reg(ADI_ADXL372_STATUS_2);

    return activity_status_reg[0];    
}

void adxl372_get_highest_peak_accel_data(adxl372_accel_data_t* max_peak)
{
    uint8_t *buf;
    uint8_t status;

    do{
        status = adxl372_get_status_reg();
    }while((!status & DATA_RDY));
    
    buf = adxl372_multibyte_read_reg(ADI_ADXL372_X_MAXPEAK_H, 6);

    // shift the MSB by 4 bits to make space for the 4 LSB (total = 12bits)
    max_peak->x = (buf[0] << 4) | (buf[1] >> 4); 
    max_peak->y = (buf[2] << 4) | (buf[3] >> 4);
    max_peak->z = (buf[4] << 4) | (buf[5] >> 4);

}

void adxl372_get_accel_data(adxl372_accel_data_t* accel_data)
{
    uint8_t status;
    uint8_t *buf;

    do{
        status = adxl372_get_status_reg();
        status = status & 0x1; // check first bit(data ready)
    }while((!status & DATA_RDY)); //loop while status = 0

    buf = adxl372_multibyte_read_reg(ADI_ADXL372_X_DATA_H, 6);

    // shift the MSB by 4 bits to make space for the 4 LSB (total = 12bits)
    accel_data->x = (buf[0] << 4) | (buf[1] >> 4); 
    accel_data->y = (buf[2] << 4) | (buf[3] >> 4);
    accel_data->z = (buf[4] << 4) | (buf[5] >> 4);
}

void adxl372_reset(void)
{
    adxl372_set_op_mode(STAND_BY);
    adxl372_write_reg(ADI_ADXL372_SRESET, ADI_ADXL372_RESET_CODE);
    nrf_delay_ms(1);
}

/**
 * configure fifo in adxl372
 * 
 * @return 1 if successful otherwise -1 
 */
int32_t adxl372_configure_fifo (struct adxl372_device *dev, uint16_t fifo_samples, adxl372_fifo_mode_t fifo_mode, adxl372_fifo_format_t fifo_format)
{
    uint8_t config;

    /*
	 * All FIFO modes must be configured while in standby mode.
	 */
    adxl372_set_op_mode(STAND_BY);

     if (fifo_samples > 512)
        return -1;

    fifo_samples -= 1; //leave space so it doesnt overflow

    config = ((uint8_t)fifo_mode << FIFO_CRL_MODE_POS) |
             ((uint8_t)fifo_format << FIFO_CRL_FORMAT_POS) |
             (((fifo_samples > 0xFF) ? 1:0)); //greater than 256?   

    adxl372_write_reg(ADI_ADXL372_FIFO_SAMPLES, fifo_samples & 0xFF);

    adxl372_write_reg(ADI_ADXL372_FIFO_CTL, config);

    dev->fifo_config.samples = fifo_samples + 1;
    dev->fifo_config.mode = fifo_mode;
    dev->fifo_config.format = fifo_format;

    return 1;
}

/* TODO: figure out how big the buffer should be
 *  Do we change how data in m_rx_buf is moved??
 *
 * Get accelerometer fifo data in adxl372
 * 
 * @return 1 if successful otherwise -1 
 */
int32_t adxl372_get_fifo_data(struct adxl372_device *dev, adxl372_accel_data_t *fifo_data)
{
    uint8_t fifo_status;
    fifo_status = adxl372_get_status_reg();
   
    //checks fifo overun status
    if(fifo_status & FIFO_OVR)
    {
        NRF_LOG_INFO("FIFO overrun \n");
        return -1; //fifo overrun
    }

    if(dev->fifo_config.mode != BYPASSED)
    {
        if( (fifo_status & FIFO_RDY) || (fifo_status & FIFO_FULL) )
        {
           /*
            * FIFO holds 512 Samples,
            * Each sample is 2 bytes
            */
            uint8_t *buf;
            buf = adxl372_multibyte_read_reg(ADI_ADXL372_FIFO_DATA, dev->fifo_config.samples*2);

            //set samples from data read
            for (int i = 0; i< dev->fifo_config.samples*2; i +=6)
            {
                fifo_data->x = (buf[i] << 4)   | (buf[i+1] >> 4); 
                fifo_data->y = (buf[i+2] << 4) | (buf[i+3] >> 4); 
                fifo_data->y = (buf[i+4] << 4) | (buf[i+5] >> 4); 
            }
        }
    }


    return 1;
}

void adxl372_set_interrupts(void)
{
    //TODO once we have INT1 and INT2 pins setup
}