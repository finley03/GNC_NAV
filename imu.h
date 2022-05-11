#ifndef IMU_H
#define IMU_H

#include "util.h"
#include "spi.h"
#include "mat.h"

#define IMU_WRITE_MASK 0x7f
#define IMU_READ_MASK 0x80


#define REG_BANK_SEL 0x7f

// user bank 0

#define WHO_AM_I 0x00
#define USER_CTRL 0x03
#define LP_CONFIG 0x05
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2 0x07
#define INT_PIN_CFG 0x0f
#define INT_ENABLE 0x10
#define INT_ENABLE_1 0x11
#define INT_ENABLE_2 0x12
#define INT_ENABLE_3 0x13
#define I2C_MST_STATUS 0x17
#define INT_STATUS 0x19
#define INT_STATUS_1 0x1a
#define INT_STATUS_2 0x1b
#define INT_STATUS_3 0x1c
#define DELAY_TIMEH 0x28
#define DELAY_TIMEL 0x29
#define ACCEL_XOUT_H 0x2d
#define ACCEL_XOUT_L 0x2e
#define ACCEL_YOUT_H 0x2f
#define ACCEL_YOUT_L 0x30
#define ACCEL_ZOUT_H 0x31
#define ACCEL_ZOUT_L 0x32
#define GYRO_XOUT_H 0x33
#define GYRO_XOUT_L 0x34
#define GYRO_YOUT_H 0x35
#define GYRO_YOUT_L 0x36
#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38
#define TEMP_OUT_H 0x39
#define TEMP_OUT_L 0x3a
#define EXT_SLV_SENS_DATA_00 0x3b

// user bank 2

#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG_1 0x01
#define GYRO_CONFIG_2 0x02
#define XG_OFFS_USRH 0x03
#define XG_OFFS_USRL 0x04
#define YG_OFFS_USRH 0x05
#define YG_OFFS_USRL 0x06
#define ZG_OFFS_USRH 0x07
#define ZG_OFFS_USRL 0x08
#define ODR_ALIGN_EN 0x09
#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11
#define ACCEL_INTEL_CTRL 0x12
#define ACCEL_WOM_THR 0x13
#define ACCEL_CONFIG 0x14
#define ACCEL_CONFIG_2 0x15
#define FSYNC_CONFIG 0x52
#define TEMP_CONFIG 0x53
#define MOD_CTRL_USR 0x54

// user bank 3

#define I2C_MST_ODR_CONFIG 0x00
#define I2C_MST_CTRL 0x01
#define I2C_MST_DELAY_CTRL 0x02
#define I2C_SLV0_ADDR 0x03
#define I2C_SLV0_REG 0x04
#define I2C_SLV0_CTRL 0x05
#define I2C_SLV0_DO 0x06



// magnetometer

#define MAG_WAI1 0x00
#define MAG_WAI2 0x01
#define MAG_RSV1 0x02
#define MAG_RSV2 0x03
#define MAG_ST1 0x10
#define MAG_HXL 0x11
#define MAG_HXH 0x12
#define MAG_HYL 0x13
#define MAG_HYH 0x14
#define MAG_HZL 0x15
#define MAG_HZH 0x16
#define MAG_TMPS 0x17
#define MAG_ST2 0x18
#define MAG_CNTL1 0x30
#define MAG_CNTL2 0x31
#define MAG_CNTL3 0x32
#define MAG_TS1 0x33
#define MAG_TS2 0x34



#define ACCEL_FS_SEL_2 0x00
#define ACCEL_FS_SEL_4 0x01
#define ACCEL_FS_SEL_8 0x02
#define ACCEL_FS_SEL_16 0x03
#define GYRO_FS_SEL_250 0x00
#define GYRO_FS_SEL_500 0x01
#define GYRO_FS_SEL_1000 0x02
#define GYRO_FS_SEL_2000 0x03



typedef struct {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	
	int16_t temp;
} IMU_Raw_Data_Type;


typedef union {
	IMU_Raw_Data_Type bit;
	
	uint8_t reg[sizeof(IMU_Raw_Data_Type)];
} IMU_Raw_Data;


typedef struct {
	float accel_x;
	float accel_y;
	float accel_z;
	
	float gyro_x;
	float gyro_y;
	float gyro_z;
	
	float temp;
} IMU_Data;


typedef struct {
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
	
	uint8_t tmps; // dummy register
	uint8_t st2; // must be read to end read transaction
} MAG_Raw_Data_Type;


typedef union {
	MAG_Raw_Data_Type bit;
	
	uint8_t reg[sizeof(MAG_Raw_Data_Type)];
} MAG_Raw_Data;


typedef struct {
	float mag_x;
	float mag_y;
	float mag_z;
} MAG_Data_Type;


typedef union {
	MAG_Data_Type bit;
	
	float reg[sizeof(MAG_Data_Type) / sizeof(float)];
} MAG_Data;


//typedef struct {
	//float bias_x;
	//float bias_y;
	//float bias_z;
	//float scale_x;
	//float scale_y;
	//float scale_z;
//} MAG_Cal_Data;


// IMU must be initialised to prevent
// switching to i2c mode
void imu_init();


// checks IMU by checking returned identifier
// returns 0 if pass
uint8_t imu_check();



// get formatted IMU data
IMU_Data imu_get_data();


// this function is extremely slow, and does not utilise DMA
// should only be used periocially to avoid hanging the system
MAG_Data mag_get_data();

// get magnetometer calibration data
// takes large amount of time to gather data
// requires user to move board
//MAG_Cal_Data mag_cal();
void mag_cal();

// calibrate gyro
// board must remain still
void gyro_cal();

//void accel_mag_cal();


uint8_t mag_check();



#endif