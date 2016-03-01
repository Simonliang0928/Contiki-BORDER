#include "rf_package.h"

//unsigned short PANID=0x1111;
unsigned char RFMAC[8]={0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11};
//unsigned short NWKADDR = 0x1111;

unsigned char rf_rx_length;
unsigned char rf_tx_buffer[MAX_TX_PACKAGE];
unsigned char rf_rx_buffer[MAX_RX_PACKAGE];

void set_tx_package_seq(unsigned char seq){
	rf_tx_buffer[2]=seq;	
}

unsigned char get_rx_package_seq(){
	return rf_rx_buffer[2];
}

void set_tx_dst_panid(unsigned short panid){
  rf_tx_buffer[3]=(unsigned char)(panid&0x00ff);//dst
  rf_tx_buffer[4]=(unsigned char)(panid>>8);
}

void set_tx_dst_nwk_addr(unsigned short nwkaddr){
  rf_tx_buffer[5]=(unsigned char)(nwkaddr&0x00ff);//dst
  rf_tx_buffer[6]=(unsigned char)(nwkaddr>>8);
}

unsigned short get_rx_src_panid(){
  unsigned short temp=0;
  temp=rf_rx_buffer[8];
  temp<<=8;
  temp+=rf_rx_buffer[7];
  return temp;
}

unsigned short get_rx_src_nwk_addr(){
  unsigned short temp=0;
  temp=rf_rx_buffer[10];
  temp<<=8;
  temp+=rf_rx_buffer[9];
  return temp;
}

unsigned short get_rx_dst_panid(){
  unsigned short temp=0;
  temp=rf_rx_buffer[4];
  temp<<=8;
  temp+=rf_rx_buffer[3];
  return temp;
}

unsigned short get_rx_dst_nwk_addr(){
  unsigned short temp=0;
  temp=rf_rx_buffer[6];
  temp<<=8;
  temp+=rf_rx_buffer[5];
  return temp;
}

void init_package(unsigned short panid,unsigned short nwkaddr){
  rf_tx_buffer[0]=0x01;//fcf
  rf_tx_buffer[1]=0x88;
  
  rf_tx_buffer[2]=0x01;//serial number
  
  set_tx_dst_panid(0xffff);
  set_tx_dst_nwk_addr(0xffff);
  
  rf_tx_buffer[7]=(unsigned char)(panid&0x00ff);//dst
  rf_tx_buffer[8]=(unsigned char)(panid>>8);
  
  rf_tx_buffer[9]=(unsigned char)(nwkaddr&0x00ff);//dst
  rf_tx_buffer[10]=(unsigned char)(nwkaddr>>8);
}

