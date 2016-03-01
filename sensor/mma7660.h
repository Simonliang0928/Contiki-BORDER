#ifndef __D95543007_MMA7660_H__
#define	__D95543007_MMA7660_H__

#define MMA7660_I2C_ADDR	0x98

//USER BUTTON
#define MMA7660_INT_PORT_SOURCE		GPIO_PortSourceGPIOC
#define MMA7660_INT_PIN_SOURCE		GPIO_PinSource3
#define MMA7660_INT_LINE			EXTI_Line3

//REGISTER
#define ACC_REG_ADDR_XOUT		0x00
#define ACC_REG_ADDR_YOUT		0x01
#define ACC_REG_ADDR_ZOUT		0x02
#define ACC_REG_ADDR_TILT		0x03
#define ACC_REG_ADDR_SRST		0x04
#define ACC_REG_ADDR_SPCNT		0x05
#define ACC_REG_ADDR_INTSU		0x06
#define ACC_REG_ADDR_MODE		0x07
#define ACC_REG_ADDR_SR			0x08
#define ACC_REG_ADDR_PDET		0x09
#define ACC_REG_ADDR_PD			0x0A
//ALERT
#define ACC_ALERT				0x40

void SetupMMA7660(void);
void GetAccFromMMA7660(signed char* xyz);
void HandleMMA7660Isr(void);
unsigned char IsMMA7660DataReady(void);

#endif
