#include "adxl372.h"
#include "spi_driver.h"
#include "nrf_delay.h"

void adxl372_init (void)
{
    /*
    GPIO for INT1 pin and INT2 pin,
    gpio_init();
    */

    //initialize device settings
    /* set up measurement mode */
    adxl372_reset();
    adxl372_set_op_mode(STAND_BY);
    //Please refer to figure 36 User offset trim profile for more info
    //For ADXL372 Vs=3.3V, x_offset = 0, y_offset=2, z_offset=5 
    adxl372_set_x_offset(0);
    adxl372_set_y_offset(2); //+10 LSB
    adxl372_set_z_offset(5); //+35 LSB
    adxl372_set_hpf_disable(true);
    adxl372_set_lpf_disable(true);
    adxl372_set_bandwidth(BW_3200HZ);
    adxl372_set_odr(ODR_6400HZ);
    adxl372_set_filter_settle(FILTER_SETTLE_16);
    adxl372_write_mask(ADI_ADXL372_MEASURE, MEASURE_LOW_NOISE_MASK, MEASURE_LOW_NOISE_POS, LOW_NOISE);
    adxl372_set_op_mode(FULL_BW_MEASUREMENT);

}

void adxl372_init_instant_on_mode(struct adxl372_device *dev, uint16_t num_samples)
{
    //initialize device settings

    /* set up measurement mode */
    adxl372_reset();
    adxl372_set_op_mode(STAND_BY);
    adxl372_configure_fifo(dev, num_samples, OLDEST_SAVED, XYZ_FIFO);
    //Please refer to figure 36 User offset trim profile for more info
    //For ADXL372 Vs=3.3V, x_offset = 0, y_offset=2, z_offset=5 
    adxl372_set_x_offset(0);
    adxl372_set_y_offset(2); //+10 LSB
    adxl372_set_z_offset(5); //+35 LSB
    adxl372_set_hpf_disable(true);
    adxl372_set_lpf_disable(true);
    adxl372_set_bandwidth(BW_3200HZ);
    adxl372_set_odr(ODR_6400HZ);
    adxl372_set_filter_settle(FILTER_SETTLE_16);
    //adxl372_set_instaon_threshold(ADXL_INSTAON_HIGH_THRESH); //sets to 30g
    //adxl372_write_mask(ADI_ADXL372_MEASURE, MEASURE_LOW_NOISE_MASK, MEASURE_LOW_NOISE_POS, LOW_NOISE);
    adxl372_set_op_mode(FULL_BW_MEASUREMENT);
    //nrf_delay_ms(16);
    //adxl372_set_op_mode(INSTANT_ON);
}


/*
 * Read register from adxl372
 * @param reg_addr - register address
 * @param reg_data - register data
 * @return 0 if success otherwise -1
 */
int8_t adxl372_read_reg( uint8_t reg_addr, uint8_t *reg_data)
{
    uint8_t read_addr;
    uint8_t buf[2]; //first byte is 0x00 and second byte is reg value
    int8_t ret;

    read_addr = ((reg_addr & 0xFF) << 1) | ADXL_SPI_RNW; //set R/W bit to 1 for reading

    ret = spi_write_and_read(SPI_ADXL372_CS_PIN, &read_addr, 1, buf, 2);
    if (ret < 0)
        return ret;
    
    *reg_data = buf[1];

    return ret;
}

/*
 * Write to an adxl372 register
 * @param reg_addr - The register address to write to
 * @param reg_data - The register data to send to
 * @return 0 if success otherwise -1
 */
int8_t adxl372_write_reg(uint8_t reg_addr, uint8_t reg_data)
{
    uint8_t tx_buf[2]; // write address + write data
    uint8_t rx_buf[2]; 

    tx_buf[0] = (reg_addr & 0xFF) << 1; //addr is 7-bits
    tx_buf[1] = reg_data;

    return spi_write_and_read(SPI_ADXL372_CS_PIN, tx_buf, 2, rx_buf, 2);
}

/*
 * Read multiple bytes from an adxl372 register
 * @param reg_addr - The register address to read from
 * @param reg_data - The register data to send to
 * @return 0 if success otherwise -1
 */
int8_t adxl372_multibyte_read_reg( uint8_t reg_addr, uint8_t* reg_data, uint8_t num_bytes) 
{
    uint8_t read_addr;
    uint8_t rx_buf[257]; 
    int8_t ret;
    
    if(num_bytes > 256)
        return -1;

    read_addr = (reg_addr << 1) | 0x01; //set R bit to 1
    memset(rx_buf, 0x00, num_bytes + 1);

    ret = spi_write_and_read(SPI_ADXL372_CS_PIN, &read_addr, 1, rx_buf, num_bytes + 1 );
    if (ret < 0)
        return ret;
    
    memcpy(reg_data, &rx_buf[1], num_bytes);

    return ret;
}

int8_t adxl372_write_mask(uint8_t reg_addr, uint32_t mask, uint32_t pos, uint8_t val)
{
    uint8_t reg_data;
    int ret;

    ret = adxl372_read_reg(reg_addr, &reg_data);
    if (ret < 0)
        return ret;

    //modifies the bit in the specified position
    reg_data &= mask;                  
    reg_data |= (val << pos) & ~mask; 

    return adxl372_write_reg(reg_addr, reg_data);
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
    adxl372_write_mask(ADI_ADXL372_MEASURE, MEASURE_AUTOSLEEP_MASK, MEASURE_AUTOSLEEP_POS, enable);
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
    uint8_t device_ID = 0;
    adxl372_read_reg(ADI_ADXL372_ADI_DEVID, &device_ID);
    
    return device_ID;
}

uint8_t adxl372_get_status_reg(void)
{
    uint8_t status_reg = 0;

    adxl372_read_reg(ADI_ADXL372_STATUS_1, &status_reg);

    return status_reg;    
}

uint8_t adxl372_get_activity_status_reg(void)
{
    uint8_t activity_status_reg = 0;

    adxl372_read_reg(ADI_ADXL372_STATUS_2, &activity_status_reg);

    return activity_status_reg;    
}

void adxl372_get_highest_peak_accel_data(adxl372_accel_data_t* max_peak)
{
    uint8_t buf[6];
    uint8_t status;

    do{
        status = adxl372_get_status_reg();
    }while((!status & DATA_RDY));
    
    adxl372_multibyte_read_reg(ADI_ADXL372_X_MAXPEAK_H, buf, 6);

    // shift the MSB by 8 bits to make space for LSB (12bit left justified)
    max_peak->x = (buf[0] << 8) | buf[1]; 
    max_peak->y = (buf[2] << 8) | buf[3];
    max_peak->z = (buf[4] << 8) | buf[5];

    max_peak->x = (max_peak->x /16) *100;
    max_peak->y = (max_peak->y /16) *100;
    max_peak->z = (max_peak->z /16) *100;

}

void adxl372_get_accel_data(adxl372_accel_data_t *accel_data)
{
    uint8_t status;
    uint8_t buf[6];

    do{
        status = adxl372_get_status_reg();
        status = status & 0x1; // check first bit(data ready)
    }while((!status & DATA_RDY)); //loop while status = 0

    adxl372_multibyte_read_reg(ADI_ADXL372_X_DATA_H, buf, 6);

    // shift the MSB by 8 bits to make space for LSB (12bit left justified)
    accel_data->x = (buf[0] << 8) | (buf[1] & 0xF0); 
    accel_data->y = (buf[2] << 8) | (buf[3] & 0xF0);
    accel_data->z = (buf[4] << 8) | (buf[5] & 0xF0);

    //convert from 12 bit to 16bit and then to mg
    accel_data->x = (accel_data->x /16) *100;
    accel_data->y = (accel_data->y /16) *100;
    accel_data->z = (accel_data->z /16) *100;
}

//Please refer to figure 36 User offset trim profile for more info
void adxl372_set_x_offset(uint8_t offset)
{
    adxl372_write_reg(ADI_ADXL372_OFFSET_X, offset);
}

//Please refer to figure 36 User offset trim profile for more info
void adxl372_set_y_offset(uint8_t offset)
{
    adxl372_write_reg(ADI_ADXL372_OFFSET_Y, offset);
}

//Please refer to figure 36 User offset trim profile for more info
void adxl372_set_z_offset(uint8_t offset)
{
    adxl372_write_reg(ADI_ADXL372_OFFSET_Z, offset);
}

void adxl372_reset(void)
{
    adxl372_set_op_mode(STAND_BY);
    adxl372_write_reg(ADI_ADXL372_SRESET, ADI_ADXL372_RESET_CODE);
    nrf_delay_ms(1);
}

int32_t adxl372_configure_fifo (struct adxl372_device *dev, uint16_t fifo_samples, adxl372_fifo_mode_t fifo_mode, adxl372_fifo_format_t fifo_format)
{
    uint8_t config;

	//All FIFO modes must be configured while in standby mode.
    adxl372_set_op_mode(STAND_BY);

     if (fifo_samples > 512)
        return -1;

    fifo_samples -= 1; //leave space so it doesnt overflow

    config = ((uint8_t)fifo_mode << FIFO_CTL_MODE_POS) |
             ((uint8_t)fifo_format << FIFO_CTL_FORMAT_POS) |
             (((fifo_samples > 0xFF) ? 1:0));

    adxl372_write_reg(ADI_ADXL372_FIFO_CTL, config);

    adxl372_write_reg(ADI_ADXL372_FIFO_SAMPLES, fifo_samples & 0xFF);


    dev->fifo_config.samples = fifo_samples + 1;
    dev->fifo_config.mode = fifo_mode;
    dev->fifo_config.format = fifo_format;

    return 1;
}

int8_t adxl372_get_fifo_data(struct adxl372_device *dev, adxl372_accel_data_t *samples)
{
    uint8_t fifo_status;
    uint8_t buf[1024];
    fifo_status = adxl372_get_status_reg();
    int8_t ret;
   
    //checks fifo overun status
    if(fifo_status & FIFO_OVR)
        return -1; //fifo overrun

    if(dev->fifo_config.mode != BYPASSED)
    {
        if( (fifo_status & FIFO_FULL) ) //(fifo_status & FIFO_RDY) || 
        {
            // FIFO holds 512 Samples,
            // Each sample is 2 bytes
            // TODO: if fifo sample is 512 we must do multibyte 4 times since max num_byte is 255
            //for(int i = 0; i < dev->fifo_config.samples*2; i+=255)
            ret = adxl372_multibyte_read_reg(ADI_ADXL372_FIFO_DATA, buf, dev->fifo_config.samples*2);
            if (ret < 0)
                return ret;
        
            for (int i = 0; i< dev->fifo_config.samples*2; i +=6)
            {
                samples->x = (buf[i+0] << 8) | (buf[i+1] & 0xF0); 
                samples->y = (buf[i+2] << 8) | (buf[i+3] & 0xF0); 
                samples->z = (buf[i+4] << 8) | (buf[i+5] & 0xF0); 

                //convert from 12 bit to 16bit and then to mg
                samples->x = (samples->x/16)*100;
                samples->y = (samples->y/16)*100;
                samples->z = (samples->z/16)*100;
                samples++; 
            }

        }
        else
        {
            return -3;//ERROR fifo no ready
        }
    }
    else
    {
        return -2; //ERROR in bypass mode
    }


    return 0;
}

void adxl372_set_interrupts(void)
{
    //TODO once we have INT1 and INT2 pins setup
}
