#include "../ntu32_config.h"
#include "__uz_type.h"
#include "__uz_data.h"
#include "__uz_srf.h"
#include "__uz2400.h"

static void SPI_Proc(unsigned char *data, uint16_t len, uint8_t last){
	/* set count by byte */
	PDMA_BCR1 = len;
	PDMA_BCR2 = len;
	/* set source address */
	PDMA_SAR1 = (uint32_t)data;
	/* set destination address */
	PDMA_DAR2 = (uint32_t)data;
	/*  Trggle DMA */
	PDMA_CSR1 |= 0x00800000UL;
	PDMA_CSR2 |= 0x00800000UL;
	/* SPI_SS to low */
	SPI0_SSR &= ~0x00000004UL;
	/* SPI DMA triggle */
	SPI0_DMA |= 0x00000003UL;
	while(PDMA_CSR1&0x00800000UL);
	while(PDMA_CSR2&0x00800000UL);
	/* SPI_SS to high */
	if(last)
		SPI0_SSR |= 0x00000004UL;
}

void spi_sw(UINT8 Address, UINT8 Value){
  	UINT8 TX_Buffer[2];
  
  	TX_Buffer[0] = (Address << 1) | 0x0001; //Shfit the Address
  	TX_Buffer[1] = Value;
 	disable_rf_int();
 	SPI_Proc(TX_Buffer, 2, 1);
 	enable_rf_int();
}

UINT8 spi_sr(UINT8 Address){
  	UINT8 TX_Buffer[2];

	TX_Buffer[0] = (Address << 1);
	disable_rf_int();
 	SPI_Proc(TX_Buffer, 2, 1);
  	enable_rf_int();
  	return TX_Buffer[1];
}

void spi_lw(UINT16 Address, UINT8 Value){
  	UINT8 *p = (UINT8 *)&Address;
  	UINT8 TX_Buffer[3];

  	Address = (Address << 5) | 0x8010;
	TX_Buffer[2] = Value;
  	TX_Buffer[1] = *p++;
	TX_Buffer[0] = *p;
  	disable_rf_int();
	SPI_Proc(TX_Buffer, 3, 1);
  	enable_rf_int();
}

UINT8 spi_lr(UINT16 Address){
  	UINT8 *p = (UINT8 *)&Address;
  	UINT8 TX_Buffer[3];

  	Address = (Address << 5) | 0x8000;
  	TX_Buffer[1] = *p++;
	TX_Buffer[0] = *p;
  	disable_rf_int();
  	SPI_Proc(TX_Buffer, 3, 1);
  	enable_rf_int();
  	return TX_Buffer[2];
}

void spi_fill_fifo(UINT16 Address, UINT8 *DataPtr, UINT8 Length){
  	UINT8 *p = (UINT8 *)&Address;
  	UINT8 TX_Buffer[2];

  	Address = (Address << 5) | 0x8010;
  	TX_Buffer[1] = *p++;
	TX_Buffer[0] = *p;
	
  	disable_rf_int();
  	SPI_Proc(TX_Buffer, 2, 0);
  	SPI_Proc(DataPtr, Length, 1);
  	enable_rf_int();
}


void spi_dump_fifo(UINT16 Address, UINT8 *DataPtr, UINT8 Length){
  	UINT8 *p = (UINT8 *)&Address;
  	UINT8 TX_Buffer[2];
  	
  	Address = (Address << 5) | 0x8000;
  	TX_Buffer[1] = *p++;
	TX_Buffer[0] = *p;
 	disable_rf_int();
	SPI_Proc(TX_Buffer, 2, 0);
	SPI_Proc(DataPtr, Length, 1);
	enable_rf_int();
}


void spi_rd_rx_fifo(UINT8 *DataPtr, UINT8 *RxLength){
	UINT8 TX_Buffer[3],Length;
  
  	disable_rf_int();
  	TX_Buffer[0] = 0xE0;
  	TX_Buffer[1] = 0x00;
  	SPI_Proc(TX_Buffer, 3, 0);
  	if(TX_Buffer[2]>141)
  		TX_Buffer[2] = 141;
	*RxLength = Length = TX_Buffer[2]+2;
	SPI_Proc(DataPtr, Length, 1);  
  	enable_rf_int();
}

