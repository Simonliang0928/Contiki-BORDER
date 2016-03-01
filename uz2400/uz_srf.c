#include "../ntu32_config.h"
#include "rf.h"

extern void (*fp)(void);

void set_interrupt_function(void (*func))
{
    fp = func;
}

void set_uz_output_freq(unsigned char freq){
  switch(freq){
  case F2p5MHZ:
    spi_lw(RFCTRL7, 0x80);
    break;
  case F5MHZ:
    spi_lw(RFCTRL7, 0x81);
    break;
  case F10MHZ:
    spi_lw(RFCTRL7, 0x82);
    break;
  case F20MHZ:
    spi_lw(RFCTRL7, 0x83);
    break;
  }
}

void uz_wakeup(){
	uz2400_wakeup_set();

	wait_uz2400();
	wait_uz2400();
	wait_uz2400();
	wait_uz2400();
	wait_uz2400();

	uz2400_wakeup_reset();

	while(!IntStatus.Wakeup);
	IntStatus.Wakeup=0;
}

void uz_reset(){
	uz2400_reset_reset();

	wait_uz2400();
	wait_uz2400();
	wait_uz2400();
	wait_uz2400();
	wait_uz2400();

	uz2400_reset_set();
}

void init_rf_pin(){
}

void wait_uz2400(){
  UINT16 i=0;
  for(i=10;i>0;i--);
}

/**************************************************************
	Function Name: uz_init()
	Description: Initial UZ2400
	Parameters: None
	Return Value: None
**************************************************************/

void uz_init(){
  UINT8 i;

  uz2400_reset_reset();
  for(i=0;i<255;i++){}
  uz2400_reset_set();
  for(i=0;i<255;i++){}

  do{
    spi_lw(TX_N_BASE,0x55);
  }while(spi_lr(TX_N_BASE)!=0x55);

  do{
    spi_lw(TX_N_BASE,0xAA);
  }while(spi_lr(TX_N_BASE)!=0xAA);

  // for new version "UZ2400 Silicon Version D"
  // from DS-2400-53_v1_1_RN_Register_Comparison.pdf
  // p36 4.1 Initialization
  spi_sw(GATECLK,0x20);
  spi_sw(PACON1,0x08);
  spi_sw(FIFOEN,0x94);
  spi_sw(TXPEMISP,0x95);
  spi_sw(BBREG3,0x50);
  spi_sw(BBREG5,0x07);
  spi_sw(BBREG6,0x40);
  spi_lw(RFCTRL0,0x03);
  spi_lw(RFCTRL1,0x02);
  spi_lw(RFCTRL2,0x66);
  spi_lw(RFCTRL4,0x06);
  spi_lw(RFCTRL6,0x30);
  spi_lw(RFCTRL7,0xEC);
  spi_lw(RFCTRL8,0x8C);
  spi_sw(GPIODIR,0x00);
  spi_lw(SECCTRL,0x20);
  spi_lw(RFCTRL50,0x05);
  spi_lw(RFCTRL51,0xC0);
  spi_lw(RFCTRL52,0x01);
  spi_lw(RFCTRL59,0x00);
  spi_lw(RFCTRL73,0x40);
  spi_lw(RFCTRL74,0xC5);
  spi_lw(RFCTRL75,0x13);
  spi_lw(RFCTRL76,0x07);
  spi_sw(INTMSK,0x00);
  spi_sw(SOFTRST,0x02);
  // Normal operation
  spi_sw(RFCTRL,0x00);
  //==============
  // Hold RF state machine in Reset
  // spi_sw(RFCTRL,0x04);

/*
  //flush the RX fifo
  spi_sw(RXFLUSH,0x01);

  //Set CCA mode to ED
  spi_sw(BBREG2, 0x80);

  //ED threshold for CCA
  spi_sw(RSSITHCCA, 0x60);

  //20Mhz clock recovery time
  spi_sw(SLPACK, 0x5f);

  //Apend RSSI value in Rx packets, Tx filter control, 20MHz clock recovery time control
  spi_sw(BBREG6, 0xd0);

  //RF optimized control
  spi_lw(RFCTRL2, 0x80); //RF-PLL Control

  spi_lw(RFCTRL7, 0x80); //Sleep clock selection, [6:7] = 10 : use internal ring oscillator

  spi_lw(RFCTRL8, 0x10); // RF VCO control

  //Sleep clock frequency control
  spi_lw(SCLKDIV, 0x01);

  //Enable the Interrupt
  spi_sw(INTMSK, 0);

//  do{
  //    test = spi_lr(RFSTATE);
 // }while((test&0xA0) != 0xA0);

  //Errata Issue
  spi_sw(FFOEN, 0x98);

  spi_sw(TXPEMISP, 0x95);

  //Reset RF state machine
  spi_sw(RFCTL, 0x04);
  spi_sw(RFCTL, 0x00);

   */
}
/**************************************************************
	Function Name: uz_enable_pa()
	Description: Enable Power Amplifier (UP2202)
	Parameters: None
	Return Value: None
For U-Power 500

Long register0x203=0x00
Long register0x22F=0x0F

//RFCTRL3     		0x203
//TESTMODE 		0x22f 

For U-Power 1000

Long register0x203=0x04
Long register0x22F=0x0F


**************************************************************/

void uz_enable_pa(unsigned char mode){
  switch(mode){
  case PA500M:
    spi_lw(RFCTRL3,  0x00);
    break;
  case PA1000M:
    spi_lw(RFCTRL3,  0x04);
    break;
  default:
    spi_lw(RFCTRL3,  0x00);
    spi_lw(TESTMODE,  spi_lr(TESTMODE) & 0xf8);
    return;
  }
  //spi_lw(RFCTRL3,  0x04);
  spi_lw(TESTMODE,  spi_lr(TESTMODE) |0x0f);
}

void uz_disable_pa(){
  spi_lw(RFCTRL3,  0x00);
  spi_lw(TESTMODE,  spi_lr(TESTMODE) & 0xf8);
}

/**************************************************************
	Function Name:  uz_set_tx_power()
	Description: Set Power Level
	Parameters: dB -> Power Level
	Return Value: None
	Note: If dB > 0 ,will reduce power level
**************************************************************/

void uz_set_tx_power(UINT8 d){ // Value Range: 0 ~ -40
  UINT8 Value = 0x00;
  signed short Operator=0;
  signed short dB=d;
  
  if(dB > 40 || dB < 0) return;
  
  if(dB == 0){ //Maximum Tx Power
          Value = 0;
  }else{
    if((Operator = dB - 40) >=0){ //dB = 40
      Value |= 0xc0;
      spi_lw(RFCTRL3, Value); //Set Power
      Value &= 0x00;
      Value |=0x40;
    }else if((Operator = dB - 30) >=0){//30 <= dB < 40
      Value |= 0xc0;
    }else if((Operator = dB - 20) >=0){//20 <= dB < 30
      Value |= 0x80;
    }else if((Operator = dB - 10) >=0){//10 <= dB < 20
      Value |= 0x40;
    }else{                                            // 0 < dB < 10
      Operator = dB;
    }
  
    if(Operator != 0){
      if(Operator == 1){
        Value |= 0x08;
      }else if(Operator == 2 || Operator == 3){
        Value |= 0x10;
      }else if(Operator == 4){
        Value |= 0x18;
      }else if(Operator == 5){
        Value |= 0x20;
      }else if(Operator == 6){
        Value |= 0x28;
      }else if(Operator == 7){
        Value |= 0x30;
      }else{//Operator == 8 or 9
        Value |= 0x38;	
      }
    }
  }
  
  spi_lw(RFCTRL3, Value);

}

/**************************************************************
	Function Name: uz_set_channel()
	Description: Set Channel
	Parameters: NewChannel -> Logical Channel you want modify, Value Range 11~26
	Return Value: None
**************************************************************/

void uz_set_channel(UINT8 NewChannel){ 

  if (NewChannel > 26 || NewChannel < 11) return; //Check Channel Range
  
  spi_lw(RFCTRL0, ((NewChannel - 11) << 4) + 0x02); //// Shift logic channel, Program Channel
  
  spi_sw(GATECLK,0x01);

  int i=0;
  for (i=0;i<4700;i++);
  spi_sw(RFCTL, 0x10);

  for (i=0;i<4700;i++);
  spi_sw(RFCTL, 0x01);

  for (i=0;i<4700;i++);
  spi_sw(RFCTL, 0x0);

  for (i=0;i<4700;i++);
  spi_sw(GATECLK,0x0);

  // Old Version:
  //Reset RF state machine
  //spi_sw(RFCTL, 0x04);
  //spi_sw(RFCTL, 0x00);

   // for new version "UZ2400 Silicon Version D"
   // from DS-2400-53_v1_1_RN_Register_Comparison.pdf
   // p36 4.1 Initialization  //Reset RF state machine
   //Hold RF state machine in Reset
   // spi_sw(RFCTL, 0x04);

}

/**************************************************************
	Function Name: uz_set_cca()
	Description: Setup CSMA/CA Mode and Threshold
	Parameters: CCAMode -> CSMA/CA mechanism, CS only, ED only or CS |ED hybrid
			     CS_TH -> Threshold for Carrier Sense Mode
			     ED_TH -> Threshold for Energy Detect Mode
	Return Value: None
**************************************************************/

#define CCA_ED_MODE 0x80
#define CCA_CS_MODE 0x40

void uz_set_cca(UINT8 CCAMode, UINT8 CS_TH, UINT8 EDT_TH){
  UINT8 Value = 0x00;
  
  Value |= CCAMode;
  
  if(CCAMode != CCA_ED_MODE){ // Set Threshold for CS or Hybrid
    Value |= ((CS_TH & 0x0f) << 2);
  }
  
  if(CCAMode != CCA_CS_MODE){ // Set Threshold for ED or Hybrid
    spi_sw(RSSITHCCA, EDT_TH);
  }
  
  spi_sw(BBREG2, Value); //Program Channel
}

/**************************************************************
	Function Name: uz_read_rssi()
	Description: Read current RSSI value
	Parameters: None
	Return Value: RSSI Value
**************************************************************/

UINT8 uz_read_rssi(){
  spi_sw(BBREG6, spi_sr(BBREG6) | 0x80); // Issue RSSI Request
  
  while(!(spi_sr(BBREG6) & 0x1)); //Wait Value Ready
   
  return  spi_lr(RSSI);
}

/**************************************************************
	Function Name: uz_set_mac_address()
	Description: Set Mac Address
	Parameters: MacAddress -> 64bits Mac Address Pointer
	Return Value: None
	Note: Mac Address is used as the hardware filter of RX Normal Mode 
**************************************************************/

void uz_set_mac_address(const uint8_t *MacAddress){
  uint8_t i;
  
  for (i=0; i<8; i++)
  spi_sw(EADR0+(7-i), *(MacAddress+i)); //Set Value 
}

/**************************************************************
	Function Name: uz_set_panId()
	Description: Set Pan Id
	Parameters: PanId -> 16bits PAN identifier
	Return Value: None
	Note: Pan Id is used as the hardware filter of RX Normal Mode 
**************************************************************/

void uz_set_panId(UINT16 PanId){
  spi_sw(PANIDL,(unsigned char)(PanId&0x00ff));
  spi_sw(PANIDH,(unsigned char)(PanId>>8));
}

/**************************************************************
	Function Name: uz_set_nwk_addr()
	Description: Set Network(16bits) Address
	Parameters: NwkAddr -> 16bits short Address
	Return Value: None
**************************************************************/

void uz_set_nwk_addr(UINT16 NwkAddr){
  spi_sw(SADRL,(unsigned char)(NwkAddr&0x00ff));
  spi_sw(SADRH,(unsigned char)(NwkAddr>>8));
}

/**************************************************************
	Function Name: uz_soft_reset()
	Description: Reset UZ2400 by Software
	Parameters: None
	Return Value: None
**************************************************************/

void uz_soft_reset(){
  //Reset Power management, Base band, Mac
  spi_sw(SOFTRST, 0x07);
}

/**************************************************************
	Function Name: UzSetUnslotMode()
	Description: Set UZ2400 work in unslot mode
	Parameters: None
	Return Value: None
**************************************************************/

void UzSetUnslotMode(){
  spi_sw(ORDER, 0xff);              
  spi_sw(TXMCR, spi_sr(TXMCR) & ~0x20); //Set the hardware sloted bit
}


/**************************************************************
	Function Name: 
	Description: 
	Parameters: None
	Return Value: None
**************************************************************/

void UzSetCoordinator(){
  spi_sw(RXMCR, spi_sr(RXMCR) |0x0c); //Set the PAN coordinator Bit
}

/**************************************************************
	Function Name: uz_rx_promiscouos_mode()
	Description: Set Promiscouos Mode of Rx
	Parameters: None
	Return Value: None
**************************************************************/

void uz_rx_promiscouos_mode(){
  spi_sw(RXMCR, spi_sr(RXMCR) | 0x01); // Accept all packets with CRC OK
}

/**************************************************************
	Function Name: uz_rx_error_mode()
	Description: Set Error Mode of Rx
	Parameters: None
	Return Value: None
**************************************************************/

void uz_rx_error_mode(){
  spi_sw(RXMCR, spi_sr(RXMCR) | 0x03); // Accept all kinds of pkt(even CRC error)
} 

/**************************************************************
	Function Name: uz_rx_normal_mode()
	Description: Set Normal Mode of Rx
	Parameters: None
	Return Value: None
**************************************************************/

void uz_rx_normal_mode(){
  spi_sw(RXMCR, spi_sr(RXMCR) & ~0x03); // Accept packets with crossponding Pan Id , Mac Address or Network Addrss
}

/**************************************************************
	Function Name: uz_enable_turbo_mode()
	Description: Enable The Turbo Mode of UZ2400
	Parameters: None
	Return Value: None
**************************************************************/

void uz_enable_turbo_mode(){
  spi_sw(BBREG0, spi_sr(BBREG0) | 0x01);
  spi_sw(BBREG3, spi_sr(BBREG3) | 0x38);
  spi_sw(BBREG4, spi_sr(BBREG4) | 0x5C);
}

/**************************************************************
	Function Name: uz_disable_turbo_mode()
	Description: Disable The Turbo Mode of UZ2400
	Parameters: None
	Return Value: None
**************************************************************/

void uz_disable_turbo_mode(){
  spi_sw(BBREG0, spi_sr(BBREG0) & ~0x01);
  spi_sw(BBREG3, spi_sr(BBREG3) & ~0x38);
  spi_sw(BBREG4, spi_sr(BBREG4) & ~0x5C);
}

/**************************************************************
	Function Name: uz_force_tx_mode()
	Description: 
	Parameters: None
	Return Value: None
**************************************************************/

void uz_force_tx_mode(){
  spi_sw(RFCTL, spi_sr(RFCTL) | 0x02);
}

static bool retransmit;
static UINT8 dataLength;

int uz_prepare(UINT8 *DataPtr, UINT8 Length) {
	
  //printf("uz_prepare length: %d\n", Length);
  spi_lw(TX_N_LEN, Length); // Fill Data Length Into TXFIFO
  spi_fill_fifo(TX_N_BASE, DataPtr, Length); //Fill Data Into TXFIFO
  
  if(*(DataPtr) & 0x20) retransmit = TRUE;
	else retransmit = FALSE;
  dataLength = Length;
  return SUCCESS;
}

UINT8 uz_transmit() {
  UINT8 Value;
  UINT32 Timer;
  
  //printf("uz_transmit length: %d\n", dataLength);
  Value = spi_sr(TXNMTRIG);
  
  if(retransmit==TRUE){
    Value |= 0x05; //Set Ackreq(SREG0x1B[2]) if re-transmission is required
    
    //Set Wait Time
    Timer = ((640 + 32*dataLength + 912)*3 + 2816) * 7; //(CCA + Tx*Length + AckWaitDuration)*MaxRetry + Backoff (us) , System Clock: 10MHz
  }else{
    Value &= ~0x07; //Clear
    Value |= 0x01;
    
    //Set Wait Time
    Timer = ((640 + 32*dataLength + 912)*3) * 7; //(CCA + Tx*Length)*MaxRetry + Backoff (us) , System Clock: 10MHz
  }
  
  spi_sw(TXNMTRIG, Value);  //Set trigger bit(SREG0x1B[0]) to send packet This bit will be automatically cleared.
    
  UINT8 cnt;
RETRY_ENTRY:
  if(!(spi_sr(TXSR) & 0x01)) {// determine whether TX success
    //printf(".....uz_transmit successed.....\n", dataLength);
	return 0;
  } else {
	if(++cnt < 5) {
      int nop;
      for(nop = 1000; nop > 0; nop--) asm("nop"); 
      //printf(".....uz_transmit going to retry.....\n", dataLength);
	  goto RETRY_ENTRY;
	} else {
      //printf(".....uz_transmit successed.....\n", dataLength);
	  return 1;
    }
  }
  /*//Wait Interrupt
  while(Timer > 0){
    if(IntStatus.TxN){
      IntStatus.TxN = 0; //Reset Status
      if(!(spi_sr(TXSR) & 0x01)) {
        printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!uz_transmit successed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", dataLength);
        return SUCCESS;
      }else{
        break;
      }			
    }
    
    Timer--;
  }
  printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!uz_transmit failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", dataLength);
  
  return FAILURE;*/
}

/**************************************************************
	Function Name: uz_tx()
	Description: Send Raw Data
	Parameters: DataPtr -> Out Data Pointer
	                   Length -> Out data Length
	                   
	Return Value: SUCCESS -> Send Data Successful
	                     FAILURE -> Send Data Failure 
**************************************************************/

UINT8 uz_tx(UINT8 *DataPtr, UINT8 Length){
	uz_prepare(DataPtr, Length);
	return uz_transmit();

}

/**************************************************************
	Function Name:  uz_rx()
	Description: Receive Data fron RXFIFO
	Parameters: RecvBuff -> Data buff that put received data
	Return Value: reveived data length
**************************************************************/

UINT8 uz_rx(UINT8 *RecvBuff){ 
  UINT8 Length;
  spi_rd_rx_fifo(RecvBuff, &Length); //Receive Data from RxFIFO
  
  if(IntStatus.Rx ==1){ //Check Interrupt Status
    IntStatus.Rx =0; //Reset Status
  }
  
  return Length;
  
  return 0;
}

/**************************************************************
	Function Name: uz_rxOnlyBeacon()
	Description: Let Rx Receive Only IEEE 802.15.4 Beacon Frame
	Parameters: None
	Return Value: None
**************************************************************/

void uz_rxOnlyBeacon(){
  spi_sw(RXFLUSH, (spi_sr(RXFLUSH) & ~0x0e) | 0x02);
}

/**************************************************************
	Function Name: uz_rxOnlyCommand()
	Description: Let Rx Receive Only IEEE 802.15.4 Command Frame
	Parameters: None
	Return Value: None
**************************************************************/

void uz_rxOnlyCommand(){
  spi_sw(RXFLUSH, (spi_sr(RXFLUSH) & ~0x0e) | 0x08);
}

/**************************************************************
	Function Name: uz_rxOnlyData()
	Description: Let Rx Receive Only IEEE 802.15.4 Data Frame
	Parameters: None
	Return Value: None
**************************************************************/

void uz_rxOnlyData(){
  spi_sw(RXFLUSH, (spi_sr(RXFLUSH) & ~0x0e) | 0x04);
}

/**************************************************************
	Function Name: uz_rxAllFrame()
	Description: Let Rx Receive all IEEE 802.15.4 Frame
	Parameters: None
	Return Value: None
**************************************************************/

void uz_rxAllFrame(){
  spi_sw(RXFLUSH, spi_sr(RXFLUSH) & ~0x0e);
}

/**************************************************************
	Function Name: uz_rx_flush()
	Description: Drop data in RXFIFO 
	Parameters: None
	Return Value: None
**************************************************************/

void uz_rx_flush(){
  spi_sw(RXFLUSH, spi_sr(RXFLUSH) | 0x01);// Trigger drop data
}

/**************************************************************
	Function Name: uz_sleep()
	Description: Force UZ2400 into Sleep Mode
	Parameters: None
	Return Value: None
**************************************************************/

void uz_sleep(){
  spi_sw(SLPACK ,spi_sr(SLPACK) | 0x80);
}

/**************************************************************
	Function Name: uz_timed_sleep()
	Description: Periodic Sleep Mode of UZ2400
	Parameters: MS -> Sleep Time
	Return Value: None
**************************************************************/

void uz_timed_sleep(UINT8 MS){  // Range 0~2000 MS, use in un-sloted mode
  INT16 Main, Remain;
  div_t Temp;
  
  Temp = div(1000000*MS, 30517);  //If sleep clock = 32.768KHz
  Main = Temp.quot;
  
  Temp = div(1000*Temp.rem, 50); // System clock = 20MHz
  Remain = Temp.quot;
  
  //Set Main Counter
  spi_lw(TXMAINCNTH0, 0);
  
  spi_lw(TXMAINCNTM, (Main & 0xff00) >> 8);
  
  spi_lw(TXMAINCNTL, (Main & 0x00ff));
  
  //Set Re-main Counter
  spi_lw(TXREMCNTH, (Remain & 0xff00) >> 8);
  
  spi_lw(TXREMCNTL, (Remain & 0x00ff));
  
  //WakeCnt
  spi_sw(SLPACK ,spi_sr(SLPACK) |0x41);
  
  spi_sw(RFCTL, spi_sr(RFCTL) & ~0x18);
  
  //Trigger sleep
  spi_lw(TXMAINCNTH1, spi_lr(TXMAINCNTH1) | 0x80);

}

/**************************************************************
	Function Name: uz_reg_wakeup()
	Description: Register Wake Up Mode, wake Up UZ2400
	Parameters: None
	Return Value: None
**************************************************************/

void uz_reg_wakeup(){
  UINT8 Value;
  
  //Disable ext. wake up (im. wakeup mode), Register wake up
  Value = spi_sr(TXBCNINTL);
  Value &= ~0x80;
  
  Value |= 0x40;
  spi_sw(TXBCNINTL, Value); 
  
  //Flag be released by software
  Value &= ~0x40;
  spi_sw(TXBCNINTL, Value);
}

/**************************************************************
	Function Name: uz_power_wakeup()
	Description: Power Management Wake Up Mode, wake Up UZ2400
	Parameters: None
	Return Value: None
**************************************************************/

void uz_power_wakeup(){
  
  //Disable ext. wake up
  spi_sw(TXBCNINTL, spi_sr(TXBCNINTL) & ~0x80);
  
  //Reset power management
  spi_sw(SOFTRST, spi_sr(SOFTRST) | 0x04);
}

/**************************************************************
	Function Name: uz_enable_ext_wakeup()
	Description: Enable External Wake Up
	Parameters: None
	Return Value: None
**************************************************************/

void uz_enable_ext_wakeup(){
  
  spi_sw(TXBCNINTL, spi_sr(TXBCNINTL) | 0x80);
  
  spi_sw(RXFLUSH, spi_sr(RXFLUSH) | 0x60);
}

/**************************************************************
	Function Name: UzEnabBatteryLifeEx()
	Description: Set IEEE 802.15.4 Battey Life Extension
	Parameters: None
	Return Value: None
**************************************************************/

void UzEnabBatteryLifeEx(){
  spi_sw(TXMCR, spi_sr(TXMCR) | 0x40);
}

/**************************************************************
	Function Name: UzDisBatteryLifeEx()
	Description: Set IEEE 802.15.4 Battey Life Extension
	Parameters: None
	Return Value: None
**************************************************************/

void UzDisBatteryLifeEx(){
  spi_sw(TXMCR, spi_sr(TXMCR) & ~0x40);
}

/**************************************************************
	Function Name:  UzSecureTx()
	Description: Transmit Secure Data
	Parameters: SecMode -> Encryption Mode
	                   *SecKey -> Security Key, defined by user
	                   *DataPtr -> Data Pointer
	                   Length -> Data Length
	                   HeaderLength -> Length of Data Header	                
	Return Value: SUCCESS -> Send Data Successful
	                     FAILURE -> Send Data Failure 
**************************************************************/

UINT8 UzSecureTx(UINT8 SecMode, UINT8 *SecKey, UINT8 *DataPtr, UINT8 Length, UINT8 HeaderLength){
  UINT8 Value;
  UINT32 Timer; 
  
  //Fill TXFIFO
  spi_lw(TX_N_HDR, HeaderLength); //Header Length
  spi_lw(TX_N_LEN, Length); //Data Length
  spi_fill_fifo(TX_N_BASE, DataPtr, Length); //Data
  
  //Fill Security key
  spi_fill_fifo(KEY_TX_N, SecKey, 16);
  
  //Fill in cipher mode
  spi_sw(SECCR0, (spi_sr(SECCR0) & ~0x07) | SecMode);
  
  Value = spi_sr(TXNMTRIG);
  
  if(*(DataPtr) & 0x20){
    Value |= 0x05; //Set Ackreq(SREG0x1B[2]) if re-transmission is required
    
    //Set Wait Time
    Timer = ((640 + 32*Length + 912)*3 + 2816)*2; //(CCA + Tx*Length + AckWaitDuration)*MaxRetry + Backoff (us) , System Clock: 20MHz
  }else{
    Value &= ~0x07; //Clear
    Value |= 0x01;
    
    //Set Wait Time
    Timer = ((640 + 32*Length + 912)*3)*2; //(CCA + Tx*Length)*MaxRetry + Backoff (us) , System Clock: 20MHz
    }
  
  spi_sw(TXNMTRIG, Value); //Trigger Tx start with security
  
  while(Timer > 0){
    if(IntStatus.TxN == 1){
      IntStatus.TxN = 0;
      if(!(spi_sr(TXSR) & 0x01)){ //Check TXFIFO release state
        return SUCCESS;
      }else{
        break;
      }			
    }
    
    Timer -= 1;
  }
  
  return FAILURE;
}

/**************************************************************
	Function Name: UzSecureRx()
	Description: Receive Secure Data
	Parameters: SecMode -> Decryption Mode
	                   *SecKey -> Security Key, defined by user
	                   *InBuff -> Data Buffer Pointer
	Return Value: Length -> Received Data Length
**************************************************************/

UINT8 UzSecureRx(UINT8 SecMode, UINT8 *SecKey, UINT8 *InBuff){
  UINT8 Length;
  
  if(IntStatus.Sec == 1){
  IntStatus.Sec = 0;
  
  spi_fill_fifo(KEY_RX_N, SecKey, 16);//Fill Secure key into FIFO
  
  spi_sw(SECCR0, (spi_sr(SECCR0) & ~0x38) | SecMode); //Fill cipher mode
  
  //Security start
  spi_sw(SECCR0, spi_sr(SECCR0) | 0x40); // Trigger Security Process 
  
  //Wait Interrupt
  while(1){
    if(IntStatus.Rx == 1){
      spi_rd_rx_fifo(InBuff, &Length); //Fetch Data from RXFIFO
      IntStatus.Rx =0; //Reset Interrupt Flag
      return Length;
      }
    }		
  }
  
  return 0;

}


void rf_init(unsigned short panid,unsigned short nwkaddr,unsigned char channel){
	uz_init();
	uz_set_tx_power(0);
	uz_enable_ext_wakeup();

	uz_set_channel(channel);
	uz_set_panId(panid);
	uz_set_nwk_addr(nwkaddr);

	init_package(panid,nwkaddr);
	set_tx_dst_panid(0xffff);
	set_tx_dst_nwk_addr(0xffff);

	uz_rx_normal_mode();

}
