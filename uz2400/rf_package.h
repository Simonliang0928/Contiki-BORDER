#ifndef __RF_PACKAGE__
#define __RF_PACKAGE__

//extern unsigned short PANID;
extern unsigned char RFMAC[8];
//extern unsigned short NWKADDR;

#define HEADER_LENGTH         (11)
#define MAX_TX_PACKAGE        (128)
#define MAX_RX_PACKAGE        (128)

extern unsigned char rf_rx_length;
extern unsigned char rf_tx_buffer[MAX_TX_PACKAGE];
extern unsigned char rf_rx_buffer[MAX_RX_PACKAGE];

void init_package(unsigned short panid,unsigned short nwkaddr);
void set_tx_dst_panid(unsigned short panid);
void set_tx_dst_nwk_addr(unsigned short nwkaddr);
void set_tx_package_seq(unsigned char seq);

unsigned char get_rx_package_seq();
unsigned short get_rx_src_panid();
unsigned short get_rx_src_nwk_addr();
unsigned short get_rx_dst_panid();
unsigned short get_rx_dst_nwk_addr();

#endif
