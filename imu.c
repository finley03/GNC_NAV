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


// function swaps units of two bytes
// writes back to array of bytes
// size must be multiple of 2
void imu_read_16_n(uint8_t address, uint8_t writeback[], uint8_t size) { // size is in bytes
	// set SS low
	REG_PORT_OUTCLR0 = SPI_IMU_SS;
	// send address
	spi_command(address | IMU_READ_MASK);

	// read data in units of 2 bytes, swapping them
	for (uint16_t i = 0; i < size; i += 2) {
		writeback[i + 1] = spi_command(0);
		writeback[i] = spi_command(0);
	}
	
	// set SS high
	REG_PORT_OUTSET0 = SPI_IMU_SS;
}


void imu_read_n(uint8_t address, uint8_t writeback[], uint8_t size) {
	// set SS low
	REG_PORT_OUTCLR0 = SPI_IMU_SS;
	// send address
	spi_command(address | IMU_READ_MASK);

	// read data
	for (uint16_t i = 0; i < size; ++i) {
		writeback[i] = spi_command(0);
	}
	
	// set SS high
	REG_PORT_OUTSET0 = SPI_IMU_SS;
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
	delay_us(1000); // random failiures at 800us
	
	imu_user_bank(0);
	
	uint8_t out = imu_read(EXT_SLV_SENS_DATA_00);
	
	return out;
}


// max 7 bytes
void imu_mag_read_n(uint8_t address, uint8_t writeback[], uint8_t size) {
	// make sure value is within range
	size = (size < 8) ? size : 7;
	
	imu_user_bank(3);
	
	// set i2c slave address to 0x0c (magnetometer)
	imu_write(I2C_SLV0_ADDR, 0x0c | 0x80);
		
	// set address of register to read in magnetometer
	imu_write(I2C_SLV0_REG, address);
	
	// enable i2c and request byte
	imu_write(I2C_SLV0_CTRL, 0x80 | size);
	
	// wait for transfer to complete
	delay_us(1000);
	
	imu_user_bank(0);
	
	// collect data from IMU
	imu_read_n(EXT_SLV_SENS_DATA_00, writeback, size);
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
	
	delay_ms(10);
	
	
	imu_user_bank(0);
	
}


void imu_init_magnetometer() {
	// make sure user bank is 0;
	imu_user_bank(0);
	
	// enable i2c master
	imu_write(USER_CTRL, 0b00100000);
	
	// change bank
	imu_user_bank(3);
	
	// set i2c master clock frequency to type 7 (recommended mode)
	imu_write(I2C_MST_CTRL, 0x07);
	
	// enable delay odr for i2c slave 0
	imu_write(I2C_MST_DELAY_CTRL, 0x01);

	imu_user_bank(0);
	//
	//delay_ms(1);
	//
	// set magnetometer to continuous measurment at 100hz
	imu_mag_write(MAG_CNTL2, 0x08);
}


void imu_init() {
	// set i2c disable bit
	imu_write(USER_CTRL, 0b00010000);
	
	// exit sleep mode
	imu_write(PWR_MGMT_1, 0b00000001);
	
	// wait to exit sleep mode (unspecified in datasheet)
	delay_us(80);
	
	
	imu_init_magnetometer();
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
	// read all IMU data as one string
	imu_read_16_n(ACCEL_XOUT_H, imu_raw_data.reg, sizeof(imu_raw_data.reg));
	
	
	IMU_Data imu_data;
	
	// convert values to SI unit floats
	
	#define G 9.80665f // acceleration due to gravity
	#define ACCEL_MAX 32768 // signed 16 bit int
	#define ACCEL_RANGE 2.0f
	#define ACCEL_MULTIPLIER ((G*ACCEL_RANGE)/ACCEL_MAX)
	
	#define GYRO_MAX 32768
	#define GYRO_RANGE 250.0f
	#define GYRO_MULTIPLIER (GYRO_RANGE/GYRO_MAX)
	
	
	imu_data.accel_x = (float) imu_raw_data.bit.accel_x * ACCEL_MULTIPLIER;
	imu_data.accel_y = (float) imu_raw_data.bit.accel_y * ACCEL_MULTIPLIER;
	imu_data.accel_z = (float) imu_raw_data.bit.accel_z * ACCEL_MULTIPLIER;
	
	imu_data.gyro_x = (float) imu_raw_data.bit.gyro_x * GYRO_MULTIPLIER;
	imu_data.gyro_y = (float) imu_raw_data.bit.gyro_y * GYRO_MULTIPLIER;
	imu_data.gyro_z = (float) imu_raw_data.bit.gyro_z * GYRO_MULTIPLIER;
	
	#define TEMP_MAX 32768
	#define TEMP_RANGE 62.5 // unsigned range
	#define TEMP_OFFSET 22.5
	#define TEMP_MULTIPLIER (TEMP_RANGE/TEMP_MAX)
	
	imu_data.temp = (float) imu_raw_data.bit.temp * TEMP_MULTIPLIER + TEMP_OFFSET;
	
	
	return imu_data;
}


MAG_Data mag_get_data() {
	MAG_Raw_Data mag_raw_data;
	// read all mag registers as one string
	imu_mag_read_n(MAG_HXL, mag_raw_data.reg, sizeof(mag_raw_data.reg));
	// read ST2 register to indicate reading has finished and new data can be loaded
	imu_mag_read(MAG_ST2);
	
	
	MAG_Data mag_data;
	
	// convert values to uT floats
	
	#define MAG_MAX 32768 // 16 bit signed integer
	#define MAG_RANGE 4912.0f // +- maximum
	#define MAG_MULTIPLIER (MAG_RANGE/MAG_MAX)
	
	#define X_OFFSET -6.5f
	#define Y_OFFSET -21.5f
	#define Z_OFFSET 28.8f
	
	
	mag_data.mag_x = (float) mag_raw_data.bit.mag_x * MAG_MULTIPLIER - X_OFFSET;
	mag_data.mag_y = (float) mag_raw_data.bit.mag_y * MAG_MULTIPLIER - Y_OFFSET;
	mag_data.mag_z = (float) mag_raw_data.bit.mag_z * MAG_MULTIPLIER - Z_OFFSET;
	
	
	return mag_data;
}