#include "mt25ql256aba.h"
#include "spi_driver.h"

/*
 * Read to an mt25ql256aba register
 * @param command_code  - command specifies the register
 * @param address       - pointer to the flash address 
 * @param address_size  - the size of the address in bytes (either 
 *                          0 for no address or
 *                          3 or 4 bytes address mode)
 * @param reg_data      - a pointer to store the register data
 * @param rx_num_bytes  - max number is 251 for 3 byte address mode
 *                          or 250 for 4 byte address mode
 * @return 0            - if success otherwise -1
 */
int8_t mt25ql256aba_read_op(uint8_t command_code, uint8_t* address, uint8_t address_size, uint8_t* reg_data, uint8_t rx_num_bytes) 
{
    uint8_t DQ0[5] = {0};
    uint8_t DQ1[257] = {0}; //first byte is 0x00 and second byte is reg value
    int8_t ret = 0;

    if(1 + address_size + rx_num_bytes > 255)
        return -1;
    if(address_size == 0 || address_size == 3 || address_size == 4)
    {
        DQ0[0] = command_code;
        memcpy(DQ0 + 1, address, address_size);

        memset(DQ1, 0x00, rx_num_bytes + 1);

        //Include a starting byte while DQ0 to transfers to slave 
        ret = spi_write_and_read(SPI_MT25QL256ABA_CS_PIN, DQ0, 1 + address_size, DQ1, rx_num_bytes + 1 + address_size);
        if (ret < 0)
            return ret;
    
        memcpy(reg_data, DQ1 + 1 + address_size, rx_num_bytes);
    }
    else
    {
        return -2;
    }

    return ret;
}

/*
 * Write to an mt25ql256aba register
 * @param command_code  - command specifies the register
 * @param address       - pointer to the flash address 
 * @param address_size  - the size of the address in bytes (either 
 *                          0 for no address or
 *                          3 or 4 bytes address mode)
 * @param data          - pointer to data to write to
 * @param data_size     - the size of data in bytes (MAX 251 bytes 
 *                         for 3 byte address mode or MAX 250 
 *                         bytes for 4)
 * @return 0            - if success otherwise -1
 */
int8_t mt25ql256aba_write_op(uint8_t command_code, uint8_t* address, uint8_t address_size, uint8_t* data, uint8_t data_size)
{
    uint8_t DQ0[256] = {0};
    int8_t ret = 0;
    //uint8_t DQ1; //output contains no valuable data

    if((1 + address_size + data_size) > 255)
        return -1;
    if(address_size == 0 || address_size == 3 || address_size == 4)
    {
        DQ0[0] = command_code;
        memcpy(DQ0 + 1, address, address_size);
        memcpy(DQ0 + 1 + address_size, data, data_size);

        ret = spi_write_and_read(SPI_MT25QL256ABA_CS_PIN, DQ0,
                 1 + address_size + data_size, NULL, 0);
    }   
    else{
        return -2;
    }

    return ret; 
}

int8_t mt25ql256aba_write_enable(void)
{
    int8_t ret;
    ret =  mt25ql256aba_write_op(MT25QL256ABA_WRITE_ENABLE, NULL, 0, NULL, 0);

    return ret;
}

int8_t mt25ql256aba_write_disable(void)
{
    int8_t ret;
    ret =  mt25ql256aba_write_op(MT25QL256ABA_WRITE_DISABLE, NULL, 0, NULL, 0);

    return ret;
}
