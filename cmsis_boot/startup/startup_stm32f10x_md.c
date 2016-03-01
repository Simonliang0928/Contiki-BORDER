/**
 ******************************************************************************
 * @file      startup_stm32f10x_md.c
 * @author    Coocox
 * @version   V1.0
 * @date      12/23/2009
 * @brief     STM32F10x Medium Density Devices Startup code.
 *            This module performs:
 *                - Set the initial SP
 *                - Set the vector table entries with the exceptions ISR address
 *                - Initialize data and bss
 *                - Setup the microcontroller system.
 *                - Call the application's entry point.
 *            After Reset the Cortex-M3 processor is in Thread mode,
 *            priority is Privileged, and the Stack is set to Main.
 *******************************************************************************
 */
 

/*----------Stack Configuration-----------------------------------------------*/  
//#define STACK_SIZE       0x00000100      /*!< Stack size (in Words)           */
//__attribute__ ((section(".co_stack")))
//unsigned long pulStack[STACK_SIZE];      


/*----------Heap Configuration-----------------------------------------------*/  
#define HEAP_SIZE       0x00000480
__attribute__ ((section(".co_heap")))
unsigned long pulHeap[HEAP_SIZE];
extern unsigned long __cs3_heap_start;
extern unsigned long __cs3_heap_end;

/*----------Macro definition--------------------------------------------------*/  
#define WEAK __attribute__ ((weak))           


/*----------Declaration of the default fault handlers-------------------------*/  
/* System exception vector handler */
void WEAK  Reset_Handler(void);
void WEAK  NMI_Handler(void);
void WEAK  HardFault_Handler(void);
void WEAK  MemManage_Handler(void);
void WEAK  BusFault_Handler(void);
void WEAK  UsageFault_Handler(void);
void WEAK  SVC_Handler(void);
void WEAK  DebugMon_Handler(void);
void WEAK  PendSV_Handler(void);
void WEAK  SysTick_Handler(void);
void WEAK  BOD_IRQHandler(void);
void WEAK  WDT_IRQHandler(void);
void WEAK  EINT0_IRQHandler(void);
void WEAK  EINT1_IRQHandler(void);
void WEAK  GPABC_IRQHandler(void);
void WEAK  GPDEF_IRQHandler(void);
void WEAK  PWM0_IRQHandler(void);
void WEAK  PWM1_IRQHandler(void);
void WEAK  TMR0_IRQHandler(void);
void WEAK  TMR1_IRQHandler(void);
void WEAK  TMR2_IRQHandler(void);
void WEAK  TMR3_IRQHandler(void);
void WEAK  UART0_IRQHandler(void);
void WEAK  UART1_IRQHandler(void);
void WEAK  SPI0_IRQHandler(void);
void WEAK  SPI1_IRQHandler(void);
void WEAK  SPI2_IRQHandler(void);
void WEAK  HIRC_IRQHandler(void);
void WEAK  I2C0_IRQHandler(void);
void WEAK  I2C1_IRQHandler(void);
void WEAK  SC2_IRQHandler(void);
void WEAK  SC0_IRQHandler(void);
void WEAK  SC1_IRQHandler(void); 
void WEAK  USBD_IRQHandler(void); 
void WEAK  LCD_IRQHandler(void); 
void WEAK  PDMA_IRQHandler(void); 
void WEAK  USBD_IRQHandler(void); 
void WEAK  I2S_IRQHandler(void); 
void WEAK  PDWU_IRQHandler(void); 
void WEAK  ADC_IRQHandler(void); 
void WEAK  DAC_IRQHandler(void); 
void WEAK  RTC_IRQHandler(void); 


/*----------Symbols defined in linker script----------------------------------*/  
extern unsigned long _sidata;    /*!< Start address for the initialization 
                                      values of the .data section.            */
extern unsigned long _sdata;     /*!< Start address for the .data section     */    
extern unsigned long _edata;     /*!< End address for the .data section       */    
extern unsigned long _sbss;      /*!< Start address for the .bss section      */
extern unsigned long _ebss;      /*!< End address for the .bss section        */      
extern void _eram;               /*!< End address for ram                     */


/*----------Function prototypes-----------------------------------------------*/  
extern int main(void);           /*!< The entry point for the application.    */
extern void SystemInit(void);    /*!< Setup the microcontroller system(CMSIS) */
void Default_Reset_Handler(void);   /*!< Default reset handler                */
static void Default_Handler(void);  /*!< Default exception handler            */
static void Default_Handler_2(void);  

extern unsigned int _estack;
/**
  *@brief The minimal vector table for a Cortex M3.  Note that the proper constructs
  *       must be placed on this to ensure that it ends up at physical address
  *       0x00000000.  
  */
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{       
  /*----------Core Exceptions-------------------------------------------------*/
//  (void *)&pulStack[STACK_SIZE-1],     /*!< The initial stack pointer         */
  (void*)&_estack-4, 
  Reset_Handler,                /*!< Reset Handler                            */
  NMI_Handler,                  /*!< NMI Handler                              */
  HardFault_Handler,            /*!< Hard Fault Handler                       */
  0,0,0,0,0,0,0,                /*!< Reserved                                 */
  SVC_Handler,                  /*!< SVCall Handler                           */
  0,0,                          /*!< Reserved                                 */
  PendSV_Handler,               /*!< PendSV Handler                           */
  SysTick_Handler,              /*!< SysTick Handler                          */
  
  /*----------External Exceptions---------------------------------------------*/
  BOD_IRQHandler,              	/*!<  0: Brownout low voltage detected interrupt */
  WDT_IRQHandler,               /*!<  1: Watch Dog Timer interrupt */
  EINT0_IRQHandler,             /*!<  2: External signal interrupt from PB.14 pin */
  EINT1_IRQHandler,             /*!<  3: External signal interrupt from PB.15 pin */
  GPABC_IRQHandler,             /*!<  4: External interrupt from PA[15:0]/PB[15:0]/PC[15:0] */
  GPDEF_IRQHandler,             /*!<  5: External interrupt from PD[15:0]/PE[15:0]/PF[7:0] */
  PWM0_IRQHandler,             	/*!<  6: PWM 0 interrupt */
  PWM1_IRQHandler,             	/*!<  7: PWM 1 interrupt */
  TMR0_IRQHandler,             	/*!<  8: Timer 0 interrupt */
  TMR1_IRQHandler,             	/*!<  9: Timer 1 interrupt */
  TMR2_IRQHandler,             	/*!< 10: Timer 2 interrupt */
  TMR3_IRQHandler,     			/*!< 11: Timer 3 interrupt */
  UART0_IRQHandler,     		/*!< 12: UART0 interrupt */
  UART1_IRQHandler,     		/*!< 13: UART1 interrupt */
  SPI0_IRQHandler,     			/*!< 14: SPI0 interrupt */
  SPI1_IRQHandler,     			/*!< 15: SPI1 interrupt */
  SPI2_IRQHandler,     			/*!< 16: SPI2 interrupt */
  HIRC_IRQHandler,     			/*!< 17: HIRC interrupt */
  I2C0_IRQHandler,            	/*!< 18: I2C0 interrupt */
  I2C1_IRQHandler,    			/*!< 19: I2C1 interrupt */
  SC2_IRQHandler,   			/*!< 20: SC2 interrupt */
  SC0_IRQHandler,          		/*!< 21: SC0 interrupt */
  SC1_IRQHandler,          		/*!< 22: SC1 interrupt */
  USBD_IRQHandler,           	/*!< 23: USB FS Device interrupt */
  0,          					/*!< 24: */
  LCD_IRQHandler,           	/*!< 25: LCD interrupt */
  PDMA_IRQHandler,      		/*!< 26: PDMA interrupt */
  I2S_IRQHandler,           	/*!< 27: I2S interrupt */
  PDWU_IRQHandler,              /*!< 28: Power Down Wake up interrupt */
  ADC_IRQHandler,              	/*!< 29: ADC interrupt */
  DAC_IRQHandler,              	/*!< 30: DAC interrupt */
  RTC_IRQHandler,           	/*!< 31: Real time clock interrupt */
//  (void *)0xF108F85F            /*!< Boot in RAM mode                         */
};        


/**
  * @brief  This is the code that gets called when the processor first
  *         starts execution following a reset event. Only the absolutely
  *         necessary set is performed, after which the application
  *         supplied main() routine is called. 
  * @param  None
  * @retval None
  */
void Default_Reset_Handler(void)
{
  /* Initialize data and bss */
  unsigned long *pulSrc, *pulDest;

  /* Copy the data segment initializers from flash to SRAM */
  pulSrc = &_sidata;

  for(pulDest = &_sdata; pulDest < &_edata; )
  {
    *(pulDest++) = *(pulSrc++);
  }
#if 0  
  /* Zero fill the bss segment.  This is done with inline assembly since this
     will clear the value of pulDest if it is not kept in a register. */
  __asm("  ldr     r0, =_sbss\n"
        "  ldr     r1, =_ebss\n"
        "  mov     r2, #0\n"
        "  .thumb_func\n"
        "zero_loop:\n"
        "    cmp     r0, r1\n"
        "    it      lt\n"
        "    strlt   r2, [r0], #4\n"
        "    blt     zero_loop");
#else
  __asm("  ldr     r0, =_sbss\n"
        "  ldr     r1, =_ebss\n"
        "  mov     r2, #0\n"
        "zero_loop:\n"
        "    str     r2, [r0]\n"
        "    add     r0, r0, #4\n" 	
        "    cmp     r0, r1\n"
        "    blt     zero_loop");
#endif  
  /* Zero fill the heap */
  pulHeap[          0] = 0xFF; // One access, so compiler/linker cannot optimize out the heap!
  pulHeap[HEAP_SIZE-1] = 0xAA;  // values are equal! just for checking the ranges in the memory window
 
#if 0
// and now - quick erase the heap on system-startup / if desired
  __asm("  ldr     r0, =__cs3_heap_start\n"
        "  ldr     r1, =__cs3_heap_end\n"
        "  mov     r2, #0\n"
        "  .thumb_func\n"
        "heap_zero_loop:\n"
        "    cmp     r0, r1\n"
        "    it      lt\n"
        "    strlt   r2, [r0], #4\n"
        "    blt     heap_zero_loop");
#else        
  __asm("  ldr     r0, =__cs3_heap_start\n"
        "  ldr     r1, =__cs3_heap_end\n"
        "  mov     r2, #0\n"
        "heap_zero_loop:\n"
        "    str     r2, [r0]\n"
        "    add     r0, r0, #4\n"
        "    cmp     r0, r1\n"
        "    blt     heap_zero_loop");
#endif        
  /* Setup the microcontroller system. */
  //SystemInit();
    
  /* Call the application's entry point.*/
  main();
}

/**
  *@brief Provide weak aliases for each Exception handler to the Default_Handler. 
  *       As they are weak aliases, any function with the same name will override 
  *       this definition.
  */
#pragma weak Reset_Handler = Default_Reset_Handler  
#pragma weak NMI_Handler = Default_Handler
#pragma weak HardFault_Handler = Default_Handler_2
#pragma weak MemManage_Handler = Default_Handler
#pragma weak BusFault_Handler = Default_Handler
#pragma weak UsageFault_Handler = Default_Handler
#pragma weak SVC_Handler = Default_Handler
#pragma weak DebugMon_Handler = Default_Handler
#pragma weak PendSV_Handler = Default_Handler
#pragma weak SysTick_Handler = Default_Handler
#pragma weak BOD_IRQHandler = Default_Handler
#pragma weak WDT_IRQHandler = Default_Handler
#pragma weak EINT0_IRQHandler = Default_Handler
#pragma weak EINT1_IRQHandler = Default_Handler
#pragma weak GPABC_IRQHandler = Default_Handler_2
#pragma weak GPDEF_IRQHandler = Default_Handler
#pragma weak PWM0_IRQHandler = Default_Handler
#pragma weak PWM1_IRQHandler = Default_Handler
#pragma weak TMR0_IRQHandler = Default_Handler
#pragma weak TMR1_IRQHandler = Default_Handler
#pragma weak TMR2_IRQHandler = Default_Handler
#pragma weak TMR3_IRQHandler = Default_Handler
#pragma weak UART0_IRQHandler = Default_Handler
#pragma weak UART1_IRQHandler = Default_Handler
#pragma weak SPI0_IRQHandler = Default_Handler
#pragma weak SPI1_IRQHandler = Default_Handler
#pragma weak SPI2_IRQHandler = Default_Handler
#pragma weak HIRC_IRQHandler = Default_Handler
#pragma weak I2C0_IRQHandler = Default_Handler
#pragma weak I2C1_IRQHandler = Default_Handler
#pragma weak SC2_IRQHandler = Default_Handler
#pragma weak SC0_IRQHandler = Default_Handler
#pragma weak SC1_IRQHandler = Default_Handler
#pragma weak USBD_IRQHandler = Default_Handler
#pragma weak LCD_IRQHandler = Default_Handler
#pragma weak PDMA_IRQHandler = Default_Handler
#pragma weak I2S_IRQHandler = Default_Handler
#pragma weak PDWU_IRQHandler = Default_Handler
#pragma weak ADC_IRQHandler = Default_Handler
#pragma weak DAC_IRQHandler = Default_Handler
#pragma weak RTC_IRQHandler = Default_Handler


/**
  * @brief  This is the code that gets called when the processor receives an 
  *         unexpected interrupt.  This simply enters an infinite loop, 
  *         preserving the system state for examination by a debugger.
  * @param  None
  * @retval None  
  */
static void Default_Handler(void) 
{
  /* Go into an infinite loop. */
  printf("Default handlerx loop.\n");
  while (1) 
  {
  }
}

static void Default_Handler_2(void) 
{
  /* Go into an infinite loop. */
  printf("Default ISR handler2 loop.\r\n");
  while (1) 
  {
  }
}
/*********************** (C) COPYRIGHT 2009 Coocox ************END OF FILE*****/
