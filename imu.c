#include "imu.h"
#include "util.h"


// create regression function for ellipsoid
#define NR_MAG_CAL_POINTS 100
#define DATA_SIZE NR_MAG_CAL_POINTS
#include "ellipsoidfit.h"


// calibration data
float mag_A[9];
float mag_b[3];
float accel_b[3];
float gyro_b[3];


// changes user bank accessible by spi
void imu_user_bank(uint8_t bank);


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


// max 7 bytes // edit: What? it works just fine with more!
void imu_mag_read_n(uint8_t address, uint8_t writeback[], uint8_t size) {
	// make sure value is within range
	//size = (size < 8) ? size : 7;
	
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
	delay_ms(1); // maybe remove
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
	
	if (who_am_i == 0xea) return 0;
	else return 1;
}


uint8_t mag_check() {
	// read WHO_AM_I address
	
	uint8_t who_am_i = imu_mag_read(MAG_WAI1);
	
	if (who_am_i == 0x48) return 0;
	else return 1;
}


IMU_Data imu_get_data_raw() {
	IMU_Raw_Data imu_raw_data;
	// read all IMU data as one string
	imu_read_16_n(ACCEL_XOUT_H, imu_raw_data.reg, sizeof(imu_raw_data.reg));
	
	
	
	// check and correct sensor scaling
	static uint8_t accel_fs_sel = ACCEL_FS_SEL_2;
	static uint8_t gyro_fs_sel = GYRO_FS_SEL_250;
	static float accel_range = 2.0f;
	static float gyro_range = 250.0f;
	
	// get maximum raw values from data
	// 32 bit values required because for some reason it fails otherwise
	int32_t max_accel = UMAX_3(imu_raw_data.bit.accel_x, imu_raw_data.bit.accel_y, imu_raw_data.bit.accel_z);
	int32_t max_gyro = UMAX_3(imu_raw_data.bit.gyro_x, imu_raw_data.bit.gyro_y, imu_raw_data.bit.gyro_z);
	//*debug = (float) max_gyro;
	
	imu_user_bank(2);
	
	// check if a value exceeds range
	// if true and range can be increased, increase range
	// -10 is to account for noise
	if (max_accel >= INT16_MAX - 10) {
		if (accel_fs_sel < ACCEL_FS_SEL_16) {
			// increment range code
			++accel_fs_sel;
			// write code to ACCEL_CONFIG
			// shift bits to left to account for position.
			imu_write(ACCEL_CONFIG, accel_fs_sel << 1);
			// update accel_range
			// case ACCEL_FS_SEL_2 can be ignored because this code can only be reached
			// if the value is incremented to ACCEL_FS_SEL_4 or higher
			switch(accel_fs_sel) {
				case ACCEL_FS_SEL_4:
					accel_range = 4.0f;
					break;
				case ACCEL_FS_SEL_8:
					accel_range = 8.0f;
					break;
				case ACCEL_FS_SEL_16:
					accel_range = 16.0f;
					break;
			}
		}
	}
	// case for if range decreases
	// - 100 is to prevent continuous switching due to noise
	else if (max_accel < INT16_MAX/2 - 100) {
		if (accel_fs_sel > ACCEL_FS_SEL_2) {
			// decrement range code
			--accel_fs_sel;
			// write code to ACCEL_CONFIG
			// shift bits left
			imu_write(ACCEL_CONFIG, accel_fs_sel << 1);
			// update accel_range
			switch(accel_fs_sel) {
				case ACCEL_FS_SEL_2:
					accel_range = 2.0f;
					break;
				case ACCEL_FS_SEL_4:
					accel_range = 4.0f;
					break;
				case ACCEL_FS_SEL_8:
					accel_range = 8.0f;
					break;
			}
		}
	}
	
	
	// corrections to gyro range
	if (max_gyro >= INT16_MAX - 10) {
		if (gyro_fs_sel < GYRO_FS_SEL_2000) {
			// increment range code
			++gyro_fs_sel;
			// write code to ACCEL_CONFIG
			// shift bits to left to account for position.
			imu_write(GYRO_CONFIG_1, gyro_fs_sel << 1);
			// update accel_range
			// case ACCEL_FS_SEL_2 can be ignored because this code can only be reached
			// if the value is incremented to ACCEL_FS_SEL_4 or higher
			switch(gyro_fs_sel) {
				case GYRO_FS_SEL_500:
					gyro_range = 500.0f;
					break;
				case GYRO_FS_SEL_1000:
					gyro_range = 1000.0f;
					break;
				case GYRO_FS_SEL_2000:
					gyro_range = 2000.0f;
					break;
			}
		}
	}
	// case for if range decreases
	// - 100 is to prevent continuous switching due to noise
	else if (max_gyro < INT16_MAX/2 - 100) {
		if (gyro_fs_sel > GYRO_FS_SEL_250) {
			// decrement range code
			--gyro_fs_sel;
			// write code to ACCEL_CONFIG
			// shift bits left
			imu_write(GYRO_CONFIG_1, gyro_fs_sel << 1);
			// update accel_range
			switch(gyro_fs_sel) {
				case GYRO_FS_SEL_250:
					gyro_range = 250.0f;
					break;
				case GYRO_FS_SEL_500:
					gyro_range = 500.0f;
					break;
				case GYRO_FS_SEL_1000:
					gyro_range = 1000.0f;
					break;
			}
		}
	}
	
	imu_user_bank(0);
	
	
	
	IMU_Data imu_data;
	
	// convert values to SI unit floats
	
	#define G 9.80665f // acceleration due to gravity
	#define ACCEL_MAX 32768 // signed 16 bit int
	#define G_ACCEL_MAX (G/ACCEL_MAX)
	//#define ACCEL_RANGE 2.0f
	//#define ACCEL_MULTIPLIER ((G*ACCEL_RANGE)/ACCEL_MAX)
	float accel_multiplier = accel_range * G_ACCEL_MAX;
	
	#define GYRO_MAX 32768
	#define _GYRO_MAX (1.0f/GYRO_MAX)
	//#define GYRO_RANGE 250.0f
	//#define GYRO_MULTIPLIER (GYRO_RANGE/GYRO_MAX)
	float gyro_multiplier = gyro_range * _GYRO_MAX;
	
	
	// no transformations needed for accelerometer (north east down)
	imu_data.accel_x = (float) imu_raw_data.bit.accel_x * accel_multiplier;
	imu_data.accel_y = (float) imu_raw_data.bit.accel_y * accel_multiplier;
	imu_data.accel_z = (float) imu_raw_data.bit.accel_z * accel_multiplier;
	
	// no transformations needed for gyro
	imu_data.gyro_x = (float) imu_raw_data.bit.gyro_x * gyro_multiplier;
	imu_data.gyro_y = (float) imu_raw_data.bit.gyro_y * gyro_multiplier;
	imu_data.gyro_z = (float) imu_raw_data.bit.gyro_z * gyro_multiplier;
	
	#define TEMP_MAX 32768
	#define TEMP_RANGE 62.5 // unsigned range
	#define TEMP_OFFSET 22.5
	#define TEMP_MULTIPLIER (TEMP_RANGE/TEMP_MAX)
	
	imu_data.temp = (float) imu_raw_data.bit.temp * TEMP_MULTIPLIER + TEMP_OFFSET;
	
	
	return imu_data;
}


MAG_Data mag_get_data_raw() {
	MAG_Raw_Data mag_raw_data;
	// read all mag registers as one string
	// ST2 is automatically read
	imu_mag_read_n(MAG_HXL, mag_raw_data.reg, sizeof(mag_raw_data.reg));
	// read ST2 register to indicate reading has finished and new data can be loaded
	//imu_mag_read(MAG_ST2);
	
	
	MAG_Data mag_data;
	
	// convert values to uT floats
	
	#define MAG_MAX 32768 // 16 bit signed integer
	#define MAG_RANGE 4912.0f // +- maximum
	#define MAG_MULTIPLIER (MAG_RANGE/MAG_MAX)
	
	
	// transform magnetometer readings to north east down
	mag_data.bit.mag_x = (float) mag_raw_data.bit.mag_x * MAG_MULTIPLIER;
	mag_data.bit.mag_y = -((float) mag_raw_data.bit.mag_y * MAG_MULTIPLIER);
	mag_data.bit.mag_z = -((float) mag_raw_data.bit.mag_z * MAG_MULTIPLIER);
	
	
	return mag_data;
}


IMU_Data imu_get_data() {
	IMU_Data uncalibrated = imu_get_data_raw();
	IMU_Data calibrated;
	
	calibrated.accel_x = uncalibrated.accel_x;
	calibrated.accel_y = uncalibrated.accel_y;
	calibrated.accel_z = uncalibrated.accel_z;
	calibrated.gyro_x = uncalibrated.gyro_x - gyro_b[0];
	calibrated.gyro_y = uncalibrated.gyro_y - gyro_b[1];
	calibrated.gyro_z = uncalibrated.gyro_z - gyro_b[2];
	calibrated.temp = uncalibrated.temp;
	
	return calibrated;
}


MAG_Data mag_get_data() {
	MAG_Data uncalibrated = mag_get_data_raw();
	MAG_Data calibrated;
	
	float centered[3];
	centered[0] = uncalibrated.reg[0] - mag_b[0];
	centered[1] = uncalibrated.reg[1] - mag_b[1];
	centered[2] = uncalibrated.reg[2] - mag_b[2];
	
	mat_multiply(mag_A, 3, 3, centered, 3, 1, calibrated.reg);
	
	return calibrated;
}

#define MAG_CAL_TIME 20.0f // seconds to calibrate magnetometer
#define MAG_CAL_ITERATION_DELAY (MAG_CAL_TIME * 1000 / NR_MAG_CAL_POINTS)
#define CCAT2(x, y) x##y
#define CCAT(x, y) CCAT2(x, y)
#define FN(x) CCAT(x, NR_MAG_CAL_POINTS)
void mag_cal() {
	// array for magnetometer data
	float data[NR_MAG_CAL_POINTS * 3];
	
	for (uint16_t i = 0; i < NR_MAG_CAL_POINTS; ++i) {
		LED_TOGGLE();
		
		// get magnetometer data
		MAG_Data mag_data = mag_get_data_raw();
		
		// copy data to "data" register
		mat_copy(mag_data.reg, 3, data + i * 3);
		
		// wait 100 ms so 10s for entire calibration
		delay_ms(MAG_CAL_ITERATION_DELAY);
	}
	
	// regress ellipsoid to data and get correction matrices
	//ellipsoidcorrection_100(data, mag_A, mag_b);
	FN(ellipsoidcorrection_)(data, mag_A, mag_b);
	
	LED_OFF();
}
#undef CCAT2
#undef CCAT
#undef FN

#define NR_GYRO_CAL_POINTS 1000
#define GYRO_CAL_TIME 10.0f // seconds to calibrate gyro
#define GYRO_CAL_ITERATION_DELAY (GYRO_CAL_TIME * 1000 / NR_GYRO_CAL_POINTS)
void gyro_cal() {
	// array for gyro axes
	double data[3] = { 0.0, 0.0, 0.0 };
	
	for (uint16_t i = 0; i < NR_GYRO_CAL_POINTS; ++i) {
		if (i % 20 == 0) LED_TOGGLE();
		
		// get imu data
		IMU_Data imu_data = imu_get_data_raw();
		
		// add data to gyro_b
		data[0] += imu_data.gyro_x;
		data[1] += imu_data.gyro_y;
		data[2] += imu_data.gyro_z;
		
		delay_ms(GYRO_CAL_ITERATION_DELAY);
	}
	
	// write correct values to gyro_b
	gyro_b[0] = data[0] / NR_GYRO_CAL_POINTS;
	gyro_b[1] = data[1] / NR_GYRO_CAL_POINTS;
	gyro_b[2] = data[2] / NR_GYRO_CAL_POINTS;
	
	LED_OFF();
}


//void accel_mag_cal() {
	//// arrays for data
	//float mag_array[NR_CAL_POINTS * 3];
	//float accel_array[NR_CAL_POINTS * 3];
	//
	//for (uint16_t i = 0; i < NR_CAL_POINTS; ++i) {
		//// get data
		//MAG_Data mag_data = mag_get_data_raw();
		//IMU_Data imu_data = imu_get_data_raw();
		//float accel_data[3] = { imu_data.accel_x, imu_data.accel_y, imu_data.accel_z };
		//
		//mat_copy(mag_data.reg, 3, mag_array + i * 3);
		//mat_copy(accel_data, 3, accel_array + i * 3);
		//
		//delay_ms(150);
	//}
	//
	//float dummyvector[9];
	//ellipsoidcorrection_100(mag_array, mag_A, mag_b);
	//ellipsoidcorrection_100(accel_array, dummyvector, accel_b);
//}