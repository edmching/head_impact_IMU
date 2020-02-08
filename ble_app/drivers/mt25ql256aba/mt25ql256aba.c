#include "mt25ql256aba.h"
#include "spi_driver.h"

int8_t mt25ql256aba_read(uint8_t command_code, uint8_t* address, uint8_t* reg_data, uint8_t num_bytes) 
{
    uint8_t DQ0[4];
    uint8_t DQ1[257]; //first byte is 0x00 and second byte is reg value
    int8_t ret;

    if(num_bytes > 255)
        return -1;

    DQ0[0] = command_code & 0xFF;
    DQ0[1] = address[0] & 0xFF;
    DQ0[2] = address[1] & 0xFF;
    DQ0[3] = address[2] & 0xFF;

    memset(DQ1, 0x00, num_bytes + 1);

    ret = spi_write_and_read(SPI_MT25QL256ABA_CS_PIN, DQ0, 4, DQ1, num_bytes + 1);
    if (ret < 0)
        return ret;
    
    memcpy( reg_data, &DQ1[1], num_bytes);
    return ret;
}

int8_t mt25ql256aba_write_op(uint8_t command_code, uint8_t address)
{
    //uint8_t tx_msg[2];
    //uint8_t rx_buf[2];
    //tx_msg[0] = command_code;
    //tx_msg[1] = address;

    //return spi_write_and_read(, tx_msg, 2, rx_buf, 2 ); // send 2 bytes
    return -1; //TODO
}