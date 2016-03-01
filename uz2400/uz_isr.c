#include "../ntu32_config.h"
#include "__uz_srf.h"

/*******************************************************************************
 Function Name: disable_rf_int
 Parameters: None
 Return Data: None
 Description: Disable the external interrupt by CPU interrupt mask
********************************************************************************/

unsigned char isRfIsrEnable=0;
void disable_rf_int(void){
	DisableRfIsr();
}

void enable_rf_int(void){
	EnableRfIsr();
}

