#ifndef __UZ_DATA_HIDE__
#define __UZ_DATA_HIDE__

// Interrupt Structure
typedef struct _interrupt_flag_{
  UINT8 TxN    : 1;
  UINT8 TxG1   : 1;
  UINT8 TxG2   : 1;
  UINT8 Rx     : 1;
  UINT8 Sec    : 1;
  UINT8 Timer  : 1;
  UINT8 Wakeup : 1;
  UINT8 Sleep  : 1;
}INTERRUPT_STS;

#endif
