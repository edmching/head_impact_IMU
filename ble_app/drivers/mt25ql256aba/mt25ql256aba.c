#include "mt25ql256aba.h"

const nrf_drv_spi_t flash_spi = NRF_DRV_SPI_INSTANCE(FLASH_SPI_INSTANCE);

nrf_drv_spi_config_t const flash_spi_config = {
        .ss_pin       = SPI_FLASH_CS_PIN,
        .miso_pin     = SPI_FLASH_MISO_PIN,
        .mosi_pin     = SPI_FLASH_MOSI_PIN,
        .sck_pin      = SPI_FLASH_SCK_PIN,
        .irq_priority = SPI_IRQ_PRIORITY,
        .orc          = 0xFF,
        .frequency    = NRF_DRV_SPI_FREQ_1M,
        .mode         = NRF_DRV_SPI_MODE_0,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };

/*
 * Read from an mt25ql256aba register
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
    uint8_t DQ0[5];
    uint8_t DQ1[257]; 
    int8_t ret = 0;

    if(1 + address_size + rx_num_bytes > 255)
        return -1;
    if(address_size == 0 || address_size == 3 || address_size == 4)
    {
        DQ0[0] = command_code;
        memcpy(DQ0 + 1, address, address_size);

        memset(DQ1, 0x00, rx_num_bytes + address_size + 1);

        //Include a starting byte while DQ0 to transfers to slave 
        ret = spi_write_and_read(&flash_spi, SPI_FLASH_CS_PIN,
                                       DQ0, (1 + address_size),
                                       DQ1, (rx_num_bytes + 1 + address_size)
                                       );
        if (ret < 0)
            return ret;
    
        memcpy(reg_data, (DQ1 + 1 + address_size), rx_num_bytes);
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
    uint8_t DQ0[256];
    int8_t ret = 0;
    //uint8_t DQ1; //output contains no valuable data

    if((1 + address_size + data_size) > 255)
        return -1;
    if(address_size == 0 || address_size == 3 || address_size == 4)
    {
        DQ0[0] = command_code;
        memcpy((DQ0 + 1), address, address_size);
        memcpy((DQ0 + 1 + address_size), data, data_size);

        ret = spi_write_and_read(&flash_spi, SPI_FLASH_CS_PIN, DQ0,
                                       (1 + address_size + data_size),
                                       NULL, 0);
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

void mt25ql256aba_read_flag_reg(flag_reg_t *flag_reg)
{
    uint8_t flag_register;
    mt25ql256aba_read_op(MT25QL256ABA_READ_FLAG_STATUS_REGISTER, NULL, 0, &flag_register, sizeof(flag_register));
    flag_reg->program_erase_controller = flag_register >> 7;
    flag_reg->program = (flag_register >> 4) & 0x1; 
    flag_reg->byte_addressing = flag_register & 0x1;
    flag_reg->erase = (flag_register >> 5) & 0x1; 
    flag_reg->protection = (flag_register >> 1) & 0x1;
    /*
    NRF_LOG_INFO("");
    NRF_LOG_INFO("CHECKING FLAG REG.....");
    NRF_LOG_INFO("program_erase controller %d, erase %d, program %d", 
                    program_erase_controller, erase, program);
    NRF_LOG_INFO("protection %d, byte %d",protection, byte_addressing);
    */
}

void mt25ql256aba_check_write_in_progress_flag(void)
{
    uint8_t flash_ready; 

    do{
       mt25ql256aba_read_op(MT25QL256ABA_READ_STATUS_REGISTER, NULL, 0, &flash_ready, sizeof(flash_ready));
       flash_ready = flash_ready & 0x1;
    }while(flash_ready == 1);
}

void mt25ql256aba_erase_subsector(uint32_t address)
{
    uint8_t addr_buf[3];

    NRF_LOG_INFO("");
    NRF_LOG_INFO("ERASING SUBSECTOR....");
    mt25ql256aba_check_write_in_progress_flag();
    convert_4byte_address_to_3byte_address(address, addr_buf);
    mt25ql256aba_write_enable();
    mt25ql256aba_write_op(MT25QL256ABA_ERASE_4KB_SUBSECTOR, addr_buf, sizeof(addr_buf), NULL, 0);
}

void mt25ql256aba_reset_device(void)
{
    NRF_LOG_INFO("");
    NRF_LOG_INFO("RESETING DEVICE....");
    mt25ql256aba_check_write_in_progress_flag();
    mt25ql256aba_read_op(MT25QL256ABA_RESET_ENABLE, NULL, 0, NULL, 0);
    mt25ql256aba_read_op(MT25QL256ABA_RESET_MEMORY, NULL, 0, NULL, 0);
}

void mt25ql256aba_bulk_erase(void)
{
    NRF_LOG_INFO("");
    NRF_LOG_INFO("PERFORMING BULK ERASE");
    mt25ql256aba_check_write_in_progress_flag();
    mt25ql256aba_write_enable();
    mt25ql256aba_write_op(MT25QL256ABA_BULK_ERASE, NULL, 0, NULL, 0);
}

/*
 * converts 4 byte address to 3 byte address
 * @param address - 4 byte flash address
 * @param address_tx_buffer - 3 byte address stored in a buffer 
 * @return none 
 */
void convert_4byte_address_to_3byte_address(uint32_t address, uint8_t* address_tx_buffer)
{
    uint8_t* address_ptr = (uint8_t*)&address;
    address_tx_buffer[0] = address_ptr[2]; //high address value
    address_tx_buffer[1] = address_ptr[1];
    address_tx_buffer[2] = address_ptr[0]; //low address value
}

void mt25ql256aba_startup_test(void)
{
    uint8_t val[3];

    NRF_LOG_INFO("");
    NRF_LOG_INFO("PERFORMING FLASH TEST....");
    mt25ql256aba_read_op(MT25QL256ABA_READ_ID, NULL, 0, val, sizeof(val));

    NRF_LOG_INFO("1: device id = 0x%x (0x20)", val[0]);
    if(val[0] != 0x20)
    {
        NRF_LOG_INFO("FLASH READ TEST FAIL");
        while(1)
        {
            __WFE();
        }
    }

    nrf_delay_ms(100);
    NRF_LOG_INFO("2:memory type = 0x%x (0xBA)", val[1]);
    if(val[1] != 0xBA)
    {
        NRF_LOG_INFO("FLASH READ TEST FAIL");
        while(1)
        {
            __WFE();
        }
    }

    nrf_delay_ms(100);
    NRF_LOG_INFO("3:memory capacity = 0x%x (0x19)", val[2]);
    if(val[2]!= 0x19)
    {
        NRF_LOG_INFO("FLASH READ TEST FAIL");
        while(1)
        {
            __WFE();
        }
    }
}

