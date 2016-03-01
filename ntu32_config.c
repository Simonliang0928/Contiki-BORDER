#include "ntu32_config.h"
#include "uz2400/rf.h"

EXTI_InitTypeDef RF_EXTI_InitStructure;
void (*fp)(void);
void (*usart_fp)(int c);

void uart1_set_input(void (*func))
{
    usart_fp = func;
//    halDigioIntSetEdge(&pinRadio_GPIO0, HAL_DIGIO_INT_RISING_EDGE);
//    halDigioIntConnect(&pinRadio_GPIO0, pfISR);
//    halDigioIntEnable(&pinRadio_GPIO0);

    // And clear the exception
//    CLEAR_EXC_RX_FRM_DONE();
}

/***************************************************************************//**
 * @brief Setup clocks
 ******************************************************************************/
void SetupClock()
{
    if(!(SYS_RegLockAddr&0x00000001UL)) {
        SYS_RegLockAddr = 0x59;
        SYS_RegLockAddr = 0x16;
        SYS_RegLockAddr = 0x88;
    }
	/* Set HCLK source form HIRC and divide 1  */
	CLK_CLKSEL0 |= 0x00000007UL;
    CLK_CLKDIV0 = (CLK_CLKDIV0 & ~0x0000000FUL);
   	/* Enable  HIRC */
	CLK_PWRCTL |= 0x00000004UL;
	/* Waiting for HIRC clock ready */
	while((CLK_CLKSTATUS&0x00000010UL)!=0x00000010UL);
	
 	/*  Set HCLK frequency 42MHz ,from HIRC, NR=16, NF=56, NO=1, (source = 12M)*/
 	CLK_PLLCTL = (CLK_PLLCTL&~0x0002133FUL)|0x00020318UL;
 	CLK_PLLCTL &= ~0x00010000UL;
 	/* Waiting for PLL ready */
	while((CLK_CLKSTATUS&0x00000004UL)!=0x00000004UL);
	/* Change HCLK source form PLL and  divide 1  */
	CLK_CLKSEL0 = (CLK_CLKSEL0 & ~0x00000007UL)|0x00000002UL;
    CLK_CLKDIV0 = (CLK_CLKDIV0 & ~0x0000000FUL);
    
    /* Select UART clock source from HIRC, and DIV=1*/
    CLK_CLKSEL1 = (CLK_CLKSEL1&~0x00000003UL)|0x00000002UL;
    CLK_CLKDIV0 = (CLK_CLKDIV0 & ~0x00000F00UL);
    /* Select SPI0 clock source from HCLK, and DIV=1*/
    CLK_CLKSEL2 |= 0x00100000UL;
 	/* Enable UART0, SPI0 IP clock */
 	CLK_APBCLK |= 0x00011000UL;
 	/* Enable DMA IP clock */
 	CLK_AHBCLK |= 0x00000002UL;
 	
 	/*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD  */
    SYS_PB_L_MFP = (SYS_PB_L_MFP&~0x00000077UL)|0x00000011UL;
    /* Set multi function pin for SPI0 */
    SYS_PC_L_MFP = (SYS_PC_L_MFP&~0x00007777UL)|0x00001111UL;    
 	/* Lock protected registers */
 	SYS_RegLockAddr = 0;
}


void SetupIO(void){
	/* init RF INT pin9, RF Reset pin10, RF wakeup pin11 */
	GPIOB_PMD = 0x00500000UL;
	//RF WAKEUP to low
	GPIOB11 = 0x00000000UL;
	//RF RESET to low
	GPIOB10 = 0x00000000UL;
}




/***************************************************************************//**
 * @brief Init UART
 ******************************************************************************/
void SetupUART(void)
{
	/* Init UART0 to 8n1 FIFO depth = 1 and select UART */
	UART0_TLCTL = 0x00000003UL;
	UART0_FUN_SEL &= ~0x00000003UL;
	/* 115200 = 42M/(364+1) */
	UART0_BAUD = 364;	
	/* RX interrupt enable */
	UART0_IER = 0x00000001UL;
}

void SetupI2C(void){
	for(;;){
		UART0_DATA = 0x31;
		while(!(UART0_FSR&0x00000200UL));
	}
}


void SetupRfSpi(void){
	/* CLK keep low whan idle, TX send data on falling edge, RX get data on
	 * rising data, data width 8 bit, SPI Master mode
	 */
	SPI0_CTL = 0x00000044UL;
	/* baud rate , because PLL is 42M, so SPI CLK = 42/(9+1) = 4.2M*/
	SPI0_CLKDIV = 0x00000009UL;
	/* Disable AUTO SS, SPI_SS to high */
	SPI0_SSR = 0x00000005UL;
	/* enable DMA channel 1& 2 */
	DMA_GCRCSR = 0x00000600UL;	
	/* Set SPI width by byte*/
	PDMA_CSR1 = 0x00080000UL;
	PDMA_CSR2 = 0x00080000UL;	
	/* set DAM timeout , disable DMA timer*/
	PDMA_TCR2 = 0x5555;
	PDMA_CSR2 = (PDMA_CSR2&~0x00001000UL);
	/* set DMA mux of SPI0 periphal */
	DMA_DSSR0 = (DMA_DSSR0&~0x001F1F00UL)|0x00100000UL;
	/* set DMA1 mem to APB */
	PDMA_CSR1 = (PDMA_CSR1&~0x0000000CUL)|0x00000008UL|0x00000001UL;
	/* set DMA2 APB to mem */
	PDMA_CSR2 = (PDMA_CSR2&~0x0000000CUL)|0x00000004UL|0x00000001UL;	
	/* set source address */
	PDMA_DAR1 = SPI0_TX0;
	PDMA_CSR1 = (PDMA_CSR1&~0x000000F0UL)|0x00000080UL;
	/* set destination address */
	PDMA_SAR2 = SPI0_RX0;
	PDMA_CSR2 = (PDMA_CSR2&~0x000000F0UL)|0x00000020UL;	
}

void SetupIsr(void){
	/* NVIC of UART0 enable */
	NVIC_ISER |= 1<<12;
	// RF INT falling trigger
	GPIOB_IMD &= ~0x00000200UL;
	GPIOB_IER = (GPIOB_IMD&~0x02000000UL)|0x00000200UL;
	/* NVIC of RF int enable */
	NVIC_ISER |= 1<<4;
	/*---------------------------------------------------------------------------------------------------------*/
    /* SysTick initial                                                                                         */
    /*---------------------------------------------------------------------------------------------------------*/
    /* source is PLL 42Mh, target Tick time is 10ms */ 
    SYST_RVR = 420000;
    /* clear current SysTick value */
    SYST_CVR = 0;
    /* Enable SysTick interrupt, source from internal , start to counting */
    SYST_CTL = 0x00000007UL;
}

void EnableRfIsr(void){
	GPIOB_IER |= 0x00000200UL;
}
void DisableRfIsr(void){
	GPIOB_IER &= ~0x00000200UL;
}

void cc2520_isr_handler() {	
    if(fp!=0) fp();
}

INTERRUPT_STS IntStatus;
void uz2400_isr_handler(void){
	*((uint8_t *)&IntStatus) = spi_sr(ISRSTS);

	if(IntStatus.TxN){ //Transmit frame success
	}

	if(IntStatus.TxG1){
	}

	if(IntStatus.TxG2){
	}

	if(IntStatus.Rx){ //Received a frame
		if(fp!=0) fp();
	}

	if(IntStatus.Sec){ //Received a secure frame , need key to decrypt
	}

	if(IntStatus.Timer){
	}

	if(IntStatus.Wakeup){
	}

	if(IntStatus.Sleep){

	}
}
void I2C_ByteWrite(I2C_TypeDef* I2Cx, u8 data,u8 devAddr, u8 WriteAddr)
{
#if 0
	/* Send STRAT condition */
	I2C_GenerateSTART(I2Cx, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send EEPROM address for write */
	I2C_Send7bitAddress(I2Cx, devAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Send the EEPROM's internal address to write to */
	I2C_SendData(I2Cx, WriteAddr);

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send the byte to be written */
	I2C_SendData(I2Cx, data);

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STOP condition */
	I2C_GenerateSTOP(I2Cx, ENABLE);
#endif	
}

void I2C_BufferRead(I2C_TypeDef* I2Cx,u8* pBuffer,u8 devAddr, u8 ReadAddr, u16 NumByteToRead)
{
#if 0		
  /* Send START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2Cx, devAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(I2Cx, ENABLE);

  /* Send the EEPROM's internal address to write to */
  I2C_SendData(I2Cx, ReadAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STRAT condition a second time */
  I2C_GenerateSTART(I2Cx, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send EEPROM address for read */
  I2C_Send7bitAddress(I2Cx, devAddr, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2Cx, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(I2Cx, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the EEPROM */
      *pBuffer = I2C_ReceiveData(I2Cx);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2Cx, ENABLE);
#endif  
}



int pressed = 0;
extern button_pressed();

//USER ISR
void EINT0_IRQHandler(void){
    if (EXTI_GetITStatus(EXTI_Line2) != RESET){
		if(pressed) pressed = 0;
		  else pressed = 1;
		button_pressed();
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

//ACC ISR
void EINT1_IRQHandler(void){
    if (EXTI_GetITStatus(EXTI_Line3) != RESET){
    	HandleMMA7660Isr();
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

//RF ISR
void  GPABC_IRQHandler(void){
	if(GPIOB_ISRC&0x00000200UL){
		disable_rf_int();
		uz2400_isr_handler();
		enable_rf_int();
	}
	GPIOB_ISRC = GPIOB_ISRC;		
#if 0	
    if (EXTI_GetITStatus(EXTI_Line4) != RESET){
		#ifdef SELECT_CC2520
			halRfDisableRxInterrupt();
			cc2520_isr_handler();
			halRfEnableRxInterrupt();
		#else
			//disable_rf_int();
			uz2400_isr_handler();
			//enable_rf_int();
		#endif
		EXTI4_ClearBit();
    }
#endif    
}

//UART0 ISR
void UART0_IRQHandler(void)
{
	if(usart_fp!=0) usart_fp(UART0_DATA);
#if 0	
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)	        
	{
			int c = USART_ReceiveData(USART1);
			if(usart_fp!=0) usart_fp(c);
	}
#endif		
}
