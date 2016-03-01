
#include "__uz2400.h"
#include "__uz_type.h"
#include "__uz_data.h"
#include <stdlib.h>

#define SUCCESS 1
#define FAILURE 0

//Security Mode
#define AES_CTR         	0x01
#define ASE_CCM_128     	0x02
#define AES_CCM_64      	0x03
#define AES_CCM_32      	0x04
#define AES_CBC_MAC_128 0x05
#define AES_CBC_MAC_64  0x06
#define AES_CBC_MAC_32  0x07

//Batter Monitor Threshold, Do not use other value
#define BATH_35 0xe0
#define BATH_33 0xd0
#define BATH_32 0xc0
#define BATH_31 0xb0
#define BATH_28 0xa0
#define BATH_27 0x90
#define BATH_26 0x80
#define BATH_25 0x70

//Address Mode
#define ADDR_NONE 0x00
#define ADDR_16 0x02 
#define ADDR_64 0x03

/**************************************************************

	Sample API List

**************************************************************/

void uz_init();

void uz_set_cca(UINT8 CCAMode, UINT8 CSThreshold, UINT8 EDThreshold);

void uz_set_mac_address(const uint8_t *MacAddress);

void uz_soft_reset();

void uz_rx_error_mode();

void uz_rx_flush();

void uz_reg_wakeup();

void uz_power_wakeup();

void uz_timed_sleep(UINT8 MS);

void uz_force_tx_mode();

void uz_enable_turbo_mode();

void uz_disable_turbo_mode();

int uz_prepare(UINT8 *DataPtr, UINT8 Length);

UINT8 uz_transmit();

/**************************************************************

	Other Functions

**************************************************************/

void spi_sw(UINT8 Address, UINT8 Value);

UINT8 spi_sr(UINT8 Address);

void spi_lw(UINT16 Address, UINT8 Value);

UINT8 spi_lr(UINT16 Address);

void spi_fill_fifo(UINT16 Address, UINT8 *DataPtr, UINT8 Length);

void spi_dump_fifo(UINT16 Address, UINT8 *DataPtr, UINT8 Length);

void spi_rd_rx_fifo(UINT8 *DataPtr, UINT8 *RxLength);


void disable_rf_int(void);
void enable_rf_int(void);
void uz2400_isr_handler(void);

void wait_uz2400();
void uz2400_wakup();

