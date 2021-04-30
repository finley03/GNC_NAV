#include "imu.h"


void imu_write(uint8_t address, uint8_t data) {
	// set SS low
	REG_PORT_OUTCLR0 = SPI_IMU_SS;
	// select address
	spi_command(address & IMU_WRITE_MASK);
	// send data
	spi_command(data);
	// set ss high
	REG_PORT_OUTSET0 = SPI_IMU_SS;
}


uint8_t imu_read(uint8_t address) {
	// set SS low
	REG_PORT_OUTCLR0 = SPI_IMU_SS;
	// select address
	spi_command(address | IMU_READ_MASK);
	// read data
	uint8_t out = spi_command(0);
	// set SS high
	REG_PORT_OUTSET0 = SPI_IMU_SS;
	
	return out;
}


uint16_t imu_read_16(uint8_t address) {
	// set SS low
	REG_PORT_OUTCLR0 = SPI_IMU_SS;
	//delay_us(2);
	// send address
	spi_command(address | IMU_READ_MASK);
	
	// get return data MSB first
	uint8_t out0 = spi_command(0); // MSB
	uint8_t out1 = spi_command(0); // LSB
	
	// set SS high
	//delay_us(2);
	REG_PORT_OUTSET0 = SPI_IMU_SS;
	//delay_us(2);
	
	// combine to one value
	uint16_t out = ((uint16_t) out0 << 8) | (uint16_t) out1;
	
	return out;
}


void imu_user_bank(uint8_t bank) {
	if (bank < 4) {
		imu_write(REG_BANK_SEL, (bank << 4));
	}
}


uint8_t imu_mag_read(uint8_t address) {
	imu_user_bank(3);
	
	// set i2c slave address to 0x0c (magnetometer)
	imu_write(I2C_SLV0_ADDR, 0x0c | 0x80);
	
	// set address of register to read in magnetometer
	imu_write(I2C_SLV0_REG, address);
	
	// enable i2c and request byte
	imu_write(I2C_SLV0_CTRL, 0x80 | 0x01);
	
	// wait for transfer to complete
	delay_ms(1);
	
	imu_user_bank(0);
	
	uint8_t out = imu_read(EXT_SLV_SENS_DATA_00);
	
	return out;
}


void imu_mag_write(uint8_t address, uint8_t data) {
	imu_user_bank(3);
	
	// set slave address
	imu_write(I2C_SLV0_ADDR, 0x0c);
	
	// set address to write in magnetometer
	imu_write(I2C_SLV0_REG, address);
	
	// store data to write
	imu_write(I2C_SLV0_DO, data);
	
	// enable i2c and send byte
	imu_write(I2C_SLV0_CTRL, 0x80 | 0x01);
	
	
	imu_user_bank(0);
	
	delay_ms(1);
	
}


void imu_init_magnetometer() {
	// make sure user bank is 0;
	imu_user_bank(0);
	
	//delay_ms(1); // critical delay, no idea why
	
	// enable i2c master
	imu_write(USER_CTRL, 0b00100000);
	
	//delay_ms(1);

	
	// change bank
	imu_user_bank(3);
	
	//delay_ms(1);
	
	// set i2c master clock frequency to type 7 (recommended mode)
	imu_write(I2C_MST_CTRL, 0x07);
	
	//delay_ms(1);
	
	// enable delay odr for i2c slave 0
	imu_write(I2C_MST_DELAY_CTRL, 0x01);
	
	//delay_ms(1);

	imu_user_bank(0);
	
	//delay_ms(1);
}


void imu_init() {
	// set i2c disable bit
	imu_write(USER_CTRL, 0b00010000);
	
	// exit sleep mode
	imu_write(PWR_MGMT_1, 0b00000001);
	
	// wait to exit sleep mode
	delay_ms(1);
	
	
	imu_init_magnetometer();
	
	//// power down magnetometer
	//imu_mag_write(MAG_CNTL2, 0x00);
	//
	//// reset IMU
	//imu_write(PWR_MGMT_1, 0x80);
	//
	//// wait for imu to come back up
	//delay_ms(1);
	//
	//// reset magnetometer
	//imu_mag_write(MAG_CNTL3, 0x01);
	//
	//// set clock source
	//imu_write(PWR_MGMT_1, 0b00000001);
	//
	//// init mag again
	//imu_init_magnetometer();
}


uint8_t imu_check() {
	// read WHO_AM_I address
	
	uint8_t who_am_i = imu_read(WHO_AM_I);
	
	uint8_t mag_who_am_i = imu_mag_read(MAG_WAI1);
	
	if (who_am_i == 0xea && mag_who_am_i == 0x48) return 0;
	else return 1;
}


IMU_Data imu_get_data() {
	IMU_Raw_Data imu_raw_data;
	
	imu_raw_data.accel_x = imu_read_16(ACCEL_XOUT_H);
	imu_raw_data.accel_y = imu_read_16(ACCEL_YOUT_H);
	imu_raw_data.accel_z = imu_read_16(ACCEL_ZOUT_H);
	
	imu_raw_data.gyro_x = imu_read_16(GYRO_XOUT_H);
	imu_raw_data.gyro_y = imu_read_16(GYRO_YOUT_H);
	imu_raw_data.gyro_z = imu_read_16(GYRO_ZOUT_H);
	
	imu_raw_data.temp = imu_read_16(TEMP_OUT_H);
	
	
	IMU_Data imu_data;
	
	// convert values to SI unit floats
	
	#define G 9.80665f // acceleration due to gravity
	#define ACCEL_MAX 32768 // signed 16 bit int
	#define ACCEL_RANGE 2.0f
	#define ACCEL_MULTIPLIER ((G*ACCEL_RANGE)/ACCEL_MAX)
	
	#define GYRO_MAX 32768
	#define GYRO_RANGE 250.0f
	#define GYRO_MULTIPLIER (GYRO_RANGE/GYRO_MAX)
	
	
	imu_data.accel_x = (float) imu_raw_data.accel_x * ACCEL_MULTIPLIER;
	imu_data.accel_y = (float) imu_raw_data.accel_y * ACCEL_MULTIPLIER;
	imu_data.accel_z = (float) imu_raw_data.accel_z * ACCEL_MULTIPLIER;
	
	imu_data.gyro_x = (float) imu_raw_data.gyro_x * GYRO_MULTIPLIER;
	imu_data.gyro_y = (float) imu_raw_data.gyro_y * GYRO_MULTIPLIER;
	imu_data.gyro_z = (float) imu_raw_data.gyro_z * GYRO_MULTIPLIER;
	
	#define TEMP_MAX 32768
	#define TEMP_RANGE 62.5 // unsigned range
	#define TEMP_OFFSET 22.5
	#define TEMP_MULTIPLIER (TEMP_RANGE/TEMP_MAX)
	
	imu_data.temp = (float) imu_raw_data.temp * TEMP_MULTIPLIER + TEMP_OFFSET;
	
	
	return imu_data;
}