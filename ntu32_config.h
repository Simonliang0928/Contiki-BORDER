#ifndef __D95543007_ARM_CONFIG_H__
#define __D95543007_ARM_CONFIG_H__

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h"
#include "misc.h"

#include "sensor/mma7660.h"

/* IRQ Channels --------------------------------------------------------------*/
#define WWDG_IRQChannel              ((u8)0x00)  /* Window WatchDog Interrupt */
#define PVD_IRQChannel               ((u8)0x01)  /* PVD through EXTI Line detection Interrupt */
#define TAMPER_IRQChannel            ((u8)0x02)  /* Tamper Interrupt */
#define RTC_IRQChannel               ((u8)0x03)  /* RTC global Interrupt */
#define FLASH_IRQChannel             ((u8)0x04)  /* FLASH global Interrupt */
#define RCC_IRQChannel               ((u8)0x05)  /* RCC global Interrupt */
#define EXTI0_IRQChannel             ((u8)0x06)  /* EXTI Line0 Interrupt */
#define EXTI1_IRQChannel             ((u8)0x07)  /* EXTI Line1 Interrupt */
#define EXTI2_IRQChannel             ((u8)0x08)  /* EXTI Line2 Interrupt */
#define EXTI3_IRQChannel             ((u8)0x09)  /* EXTI Line3 Interrupt */
#define EXTI4_IRQChannel             ((u8)0x0A)  /* EXTI Line4 Interrupt */
#define DMA1_Channel1_IRQChannel     ((u8)0x0B)  /* DMA1 Channel 1 global Interrupt */
#define DMA1_Channel2_IRQChannel     ((u8)0x0C)  /* DMA1 Channel 2 global Interrupt */
#define DMA1_Channel3_IRQChannel     ((u8)0x0D)  /* DMA1 Channel 3 global Interrupt */
#define DMA1_Channel4_IRQChannel     ((u8)0x0E)  /* DMA1 Channel 4 global Interrupt */
#define DMA1_Channel5_IRQChannel     ((u8)0x0F)  /* DMA1 Channel 5 global Interrupt */
#define DMA1_Channel6_IRQChannel     ((u8)0x10)  /* DMA1 Channel 6 global Interrupt */
#define DMA1_Channel7_IRQChannel     ((u8)0x11)  /* DMA1 Channel 7 global Interrupt */
#define ADC1_2_IRQChannel            ((u8)0x12)  /* ADC1 et ADC2 global Interrupt */
#define USB_HP_CAN_TX_IRQChannel     ((u8)0x13)  /* USB High Priority or CAN TX Interrupts */
#define USB_LP_CAN_RX0_IRQChannel    ((u8)0x14)  /* USB Low Priority or CAN RX0 Interrupts */
#define CAN_RX1_IRQChannel           ((u8)0x15)  /* CAN RX1 Interrupt */
#define CAN_SCE_IRQChannel           ((u8)0x16)  /* CAN SCE Interrupt */
#define EXTI9_5_IRQChannel           ((u8)0x17)  /* External Line[9:5] Interrupts */
#define TIM1_BRK_IRQChannel          ((u8)0x18)  /* TIM1 Break Interrupt */
#define TIM1_UP_IRQChannel           ((u8)0x19)  /* TIM1 Update Interrupt */
#define TIM1_TRG_COM_IRQChannel      ((u8)0x1A)  /* TIM1 Trigger and Commutation Interrupt */
#define TIM1_CC_IRQChannel           ((u8)0x1B)  /* TIM1 Capture Compare Interrupt */
#define TIM2_IRQChannel              ((u8)0x1C)  /* TIM2 global Interrupt */
#define TIM3_IRQChannel              ((u8)0x1D)  /* TIM3 global Interrupt */
#define TIM4_IRQChannel              ((u8)0x1E)  /* TIM4 global Interrupt */
#define I2C1_EV_IRQChannel           ((u8)0x1F)  /* I2C1 Event Interrupt */
#define I2C1_ER_IRQChannel           ((u8)0x20)  /* I2C1 Error Interrupt */
#define I2C2_EV_IRQChannel           ((u8)0x21)  /* I2C2 Event Interrupt */
#define I2C2_ER_IRQChannel           ((u8)0x22)  /* I2C2 Error Interrupt */
#define SPI1_IRQChannel              ((u8)0x23)  /* SPI1 global Interrupt */
#define SPI2_IRQChannel              ((u8)0x24)  /* SPI2 global Interrupt */
#define USART1_IRQChannel            ((u8)0x25)  /* USART1 global Interrupt */
#define USART2_IRQChannel            ((u8)0x26)  /* USART2 global Interrupt */
#define USART3_IRQChannel            ((u8)0x27)  /* USART3 global Interrupt */
#define EXTI15_10_IRQChannel         ((u8)0x28)  /* External Line[15:10] Interrupts */
#define RTCAlarm_IRQChannel          ((u8)0x29)  /* RTC Alarm through EXTI Line Interrupt */
#define USBWakeUp_IRQChannel         ((u8)0x2A)  /* USB WakeUp from suspend through EXTI Line Interrupt */
#define TIM8_BRK_IRQChannel          ((u8)0x2B)  /* TIM8 Break Interrupt */
#define TIM8_UP_IRQChannel           ((u8)0x2C)  /* TIM8 Update Interrupt */
#define TIM8_TRG_COM_IRQChannel      ((u8)0x2D)  /* TIM8 Trigger and Commutation Interrupt */
#define TIM8_CC_IRQChannel           ((u8)0x2E)  /* TIM8 Capture Compare Interrupt */
#define ADC3_IRQChannel              ((u8)0x2F)  /* ADC3 global Interrupt */
#define FSMC_IRQChannel              ((u8)0x30)  /* FSMC global Interrupt */
#define SDIO_IRQChannel              ((u8)0x31)  /* SDIO global Interrupt */
#define TIM5_IRQChannel              ((u8)0x32)  /* TIM5 global Interrupt */
#define SPI3_IRQChannel              ((u8)0x33)  /* SPI3 global Interrupt */
#define UART4_IRQChannel             ((u8)0x34)  /* UART4 global Interrupt */
#define UART5_IRQChannel             ((u8)0x35)  /* UART5 global Interrupt */
#define TIM6_IRQChannel              ((u8)0x36)  /* TIM6 global Interrupt */
#define TIM7_IRQChannel              ((u8)0x37)  /* TIM7 global Interrupt */
#define DMA2_Channel1_IRQChannel     ((u8)0x38)  /* DMA2 Channel 1 global Interrupt */
#define DMA2_Channel2_IRQChannel     ((u8)0x39)  /* DMA2 Channel 2 global Interrupt */
#define DMA2_Channel3_IRQChannel     ((u8)0x3A)  /* DMA2 Channel 3 global Interrupt */
#define DMA2_Channel4_5_IRQChannel   ((u8)0x3B)  /* DMA2 Channel 4 and DMA2 Channel 5 global Interrupt */

#define SENSOR_I2C				I2C2

//USER BUTTON
#define USER_INT_PORT_SOURCE	GPIO_PortSourceGPIOC
#define USER_INT_PIN_SOURCE		GPIO_PinSource2
#define USER_INT_LINE			EXTI_Line2
//RF
#define RF_INT_PORT_SOURCE		GPIO_PortSourceGPIOC
#define RF_INT_PIN_SOURCE		GPIO_PinSource4
#define RF_INT_LINE				EXTI_Line4
#define RF_WAKEUP_PORT			GPIOC
#define RF_WAKEUP_PIN			GPIO_Pin_5
#define RF_RESET_PORT			GPIOB
#define RF_RESET_PIN			GPIO_Pin_2
#define RF_SPI					SPI1
#define uz2400_reset_set()		GPIO_WriteBit(RF_RESET_PORT,RF_RESET_PIN,Bit_SET)
#define uz2400_reset_reset()	GPIO_WriteBit(RF_RESET_PORT,RF_RESET_PIN,Bit_RESET)
#define uz2400_wakeup_set()		GPIO_WriteBit(RF_WAKEUP_PORT,RF_WAKEUP_PIN,Bit_SET)
#define uz2400_wakeup_reset()	GPIO_WriteBit(RF_WAKEUP_PORT,RF_WAKEUP_PIN,Bit_RESET)
#define uz2400_select()			GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_RESET)
#define uz2400_deselect()		GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_SET)
#define uz2400_wait_spi_w_ready()	while(SPI_I2S_GetFlagStatus(RF_SPI,SPI_I2S_FLAG_TXE)==RESET);
#define uz2400_wait_spi_r_ready()	while(SPI_I2S_GetFlagStatus(RF_SPI,SPI_I2S_FLAG_RXNE)==RESET);
void EnableRfIsr(void);
void DisableRfIsr(void);
extern EXTI_InitTypeDef RF_EXTI_InitStructure;


//LED
#define LED_PORT	GPIOC
#define LED_A		GPIO_Pin_6
#define LED_B		GPIO_Pin_7
#define LED_C		GPIO_Pin_8
#define LED_D		GPIO_Pin_9

void SetupClock(void);
void SetupIO(void);
void SetupI2C(void);
void SetupUART(void);
void SetupIsr(void);
void SetupRfSpi(void);

void I2C_ByteWrite(I2C_TypeDef* I2Cx, u8 data,u8 devAddr, u8 WriteAddr);
void I2C_BufferRead(I2C_TypeDef* I2Cx,u8* pBuffer,u8 devAddr, u8 ReadAddr, u16 NumByteToRead);
///////////////////////////////////////////////////////////////////////////////////////////////
//NANO100LE3BN
#define PDMA_CSR1 	*(volatile unsigned long*)(0x50008100)
#define PDMA_SAR1 	*(volatile unsigned long*)(0x50008104)
#define PDMA_DAR1 	*(volatile unsigned long*)(0x50008108)
#define PDMA_BCR1 	*(volatile unsigned long*)(0x5000810C)
#define PDMA_TCR1 	*(volatile unsigned long*)(0x50008128)
#define PDMA_CSR2 	*(volatile unsigned long*)(0x50008200)
#define PDMA_SAR2 	*(volatile unsigned long*)(0x50008204)
#define PDMA_DAR2 	*(volatile unsigned long*)(0x50008208)
#define PDMA_BCR2 	*(volatile unsigned long*)(0x5000820C)
#define PDMA_TCR2 	*(volatile unsigned long*)(0x50008228)
#define DMA_GCRCSR 	*(volatile unsigned long*)(0x50008F00)
#define DMA_DSSR0 	*(volatile unsigned long*)(0x50008F04)
#define DMA_DSSR1 	*(volatile unsigned long*)(0x50008F08)

#define NVIC_ISER		*(volatile unsigned long*)(0xE000E000+0x100)

#define SPI0_RX0 	(0x40030010)
#define SPI0_TX0 	(0x40030020)
#define SPI0_CTL 	*(volatile unsigned long*)(0x40030000+0x00)
#define SPI0_STATUS	*(volatile unsigned long*)(0x40030000+0x04)
#define SPI0_CLKDIV	*(volatile unsigned long*)(0x40030000+0x08)
#define SPI0_SSR 	*(volatile unsigned long*)(0x40030000+0x0C)
#define SPI0_DMA 	*(volatile unsigned long*)(0x40030000+0x38)

#define GPIOB_PMD	*(volatile unsigned long*)(0x50004000+0x040)
#define GPIOB_IMD	*(volatile unsigned long*)(0x50004000+0x058)
#define GPIOB_IER	*(volatile unsigned long*)(0x50004000+0x05C)
#define GPIOB_ISRC	*(volatile unsigned long*)(0x50004000+0x060)
#define GPIOB9		*(volatile unsigned long*)(0x50004000+0x264)
#define GPIOB10		*(volatile unsigned long*)(0x50004000+0x268)
#define GPIOB11		*(volatile unsigned long*)(0x50004000+0x26C)

#define UART0_DATA		*(volatile unsigned long*)(0x40050000+0x00)
#define UART0_TLCTL		*(volatile unsigned long*)(0x40050000+0x08)
#define UART0_IER		*(volatile unsigned long*)(0x40050000+0x0C)
#define UART0_FSR		*(volatile unsigned long*)(0x40050000+0x18)
#define UART0_BAUD 		*(volatile unsigned long*)(0x40050000+0x24)
#define UART0_FUN_SEL 	*(volatile unsigned long*)(0x40050000+0x38)
#define SYS_PB_L_MFP	*(volatile unsigned long*)(0x50000000+0x038)
#define SYS_PC_L_MFP	*(volatile unsigned long*)(0x50000000+0x040)
#define SYS_RegLockAddr	*(volatile unsigned long*)(0x50000000+0x100)

#define CLK_PWRCTL		*(volatile unsigned long*)(0x50000200+0x00)
#define CLK_AHBCLK		*(volatile unsigned long*)(0x50000200+0x04)
#define CLK_APBCLK		*(volatile unsigned long*)(0x50000200+0x08)
#define CLK_CLKSTATUS	*(volatile unsigned long*)(0x50000200+0x0C)
#define CLK_CLKSEL0		*(volatile unsigned long*)(0x50000200+0x10)
#define CLK_CLKSEL1		*(volatile unsigned long*)(0x50000200+0x14)
#define CLK_CLKSEL2		*(volatile unsigned long*)(0x50000200+0x18)
#define CLK_CLKDIV0		*(volatile unsigned long*)(0x50000200+0x1C)
#define CLK_PLLCTL		*(volatile unsigned long*)(0x50000200+0x24)

#define SYST_CTL		*(volatile unsigned long*)(0xE000E000+0x10)
#define SYST_RVR		*(volatile unsigned long*)(0xE000E000+0x14)
#define SYST_CVR		*(volatile unsigned long*)(0xE000E000+0x18)
#define SCS_ICSR		*(volatile unsigned long*)(0xE000ED00+0x04)

#define uz2400_reset_set()		do{GPIOB10=0x00000001UL;}while(0)
#define uz2400_reset_reset()	do{GPIOB10=0x00000000UL;}while(0)
#define uz2400_wakeup_set()		do{GPIOB11=0x00000001UL;}while(0)
#define uz2400_wakeup_reset()	do{GPIOB11=0x00000000UL;}while(0)
#endif
