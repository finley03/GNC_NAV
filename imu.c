#include "imu.h"


void imu_init() {
	// set i2c disable bit
		
	// set SS low
	REG_PORT_OUTCLR0 = SPI_IMU_SS;
	// select address USER_CTRL
	spi_command(0x03);
	// set i2c disable bit
	spi_command(0b00010000);
	// set SS high
	REG_PORT_OUTSET0 = SPI_IMU_SS;
}


uint8_t imu_check() {
	// read WHO_AM_I address
	
	// set SS low
	REG_PORT_OUTCLR0 = SPI_IMU_SS;
	// select address WHO_AM_I on read mode
	spi_command(0x00 | 0b10000000);
	// read return
	uint8_t who_am_i = spi_command(0);
	// set SS high
	REG_PORT_OUTSET0 = SPI_IMU_SS;
	
	// wait while SS up
	delay_ms(1);
	
	// test i2c disable bit
	
	REG_PORT_OUTCLR0 = SPI_IMU_SS;
	
	spi_command(0x06 | 0b10000000);
	// read return
	uint8_t pwr_mgmt_1 = spi_command(0);
	// set SS high
	REG_PORT_OUTSET0 = SPI_IMU_SS;
	
	
	if (who_am_i == 0xea && pwr_mgmt_1 == 0x41) return 0;
	else return 1;
}