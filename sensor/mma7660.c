#include "mma7660.h"
#include "ntu32_config.h"

static unsigned char isDataReady=0;

void SetupMMA7660(void){
	isDataReady=0;
	I2C_ByteWrite(SENSOR_I2C,0x01,MMA7660_I2C_ADDR,ACC_REG_ADDR_SPCNT);
	I2C_ByteWrite(SENSOR_I2C,0xFF,MMA7660_I2C_ADDR,ACC_REG_ADDR_INTSU);
	I2C_ByteWrite(SENSOR_I2C,0x49,MMA7660_I2C_ADDR,ACC_REG_ADDR_MODE);
	I2C_ByteWrite(SENSOR_I2C,0x01,MMA7660_I2C_ADDR,ACC_REG_ADDR_SR);
	I2C_ByteWrite(SENSOR_I2C,0x00,MMA7660_I2C_ADDR,ACC_REG_ADDR_PDET);
	I2C_ByteWrite(SENSOR_I2C,0x00,MMA7660_I2C_ADDR,ACC_REG_ADDR_PD);
}
void GetAccFromMMA7660(signed char* xyz){
	do{
		I2C_BufferRead(SENSOR_I2C,(u8*)xyz,MMA7660_I2C_ADDR,ACC_REG_ADDR_XOUT,3);
	}while((xyz[0] & ACC_ALERT));

	xyz[0]<<=2;
	xyz[1]<<=2;
	xyz[2]<<=2;
	isDataReady=0;
}
void HandleMMA7660Isr(void){
	isDataReady=1;
}
unsigned char IsMMA7660DataReady(void){
	return isDataReady;
}
