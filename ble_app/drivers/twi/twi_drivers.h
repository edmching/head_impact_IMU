
// TWI instance ID
#define TWI_INSTANCE_ID 1

extern const nrf_drv_twi_t twi;

void twi_init(void);

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
