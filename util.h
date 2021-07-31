#ifndef UTIL_H
#define UTIL_H

#include "samd21e18a.h"
#include "time.h"

#include <math.h>
#include <stdbool.h>

#define _STD_G 9.80665

typedef enum {
	_NAV_PARAM_START = 16384,
	_KALMAN_POSITION_UNCERTAINTY,
	_KALMAN_VELOCITY_UNCERTAINTY,
	_KALMAN_ORIENTATION_UNCERTAINTY,
	_KALMAN_ORIENTATION_MEASUREMENT_UNCERTAINTY,
	_KALMAN_GNSS_HORIZONTAL_UNCERTAINTY_MUL,
	_KALMAN_GNSS_VERTICAL_UNCERTAINTY_MUL,
	_KALMAN_BARO_VARIANCE,
	_KALMAN_ACCEL_VARIANCE,
	_KALMAN_ANGULARVELOCITY_VARIANCE,
	_KALMAN_GNSS_ZEROLAT,
	_KALMAN_GNSS_ZEROLONG,
	_MAG_A_1, // mag distortion matrix
	_MAG_A_2,
	_MAG_A_3,
	_MAG_B, // mag offset vector
	_ACCEL_B, // accel offset vector
	_GYRO_B, // gyro offset vector
	
	_NAV_VOLATILE_PARAM_START = 24576,
	//_KALMAN_RUN
} NAV_Param;

#define LED PORT_PA02
#define LED_ON() (REG_PORT_OUTSET0 = LED);
#define LED_OFF() (REG_PORT_OUTCLR0 = LED);

#define ABS(a) ((a < 0) ? -(a) : a)
#define MAX_2(a, b) ((a > b) ? a : b)
#define MIN_2(a, b) ((b > a) ? a : b);
#define MAX_3(a, b, c) (MAX_2(MAX_2(a, b), c))
#define MIN_3(a, b, c) (MIN_2(MIN_2(a, b), c))
#define UMAX_2(a, b) MAX_2(ABS(a), ABS(b))
#define UMIN_2(a, b) MIN_2(ABS(a), ABS(b))
#define UMAX_3(a, b, c) MAX_3(ABS(a), ABS(b), ABS(c))
#define UMIN_3(a, b, c) MIN_3(ABS(a), ABS(b), ABS(c))

#define pow(a, b) exp2(b * log2(a))
#define powf(a, b) exp2f(b * log2f(a))

#define radians(x) ((x) * 0.01745329251994329576923690768489)
#define degrees(x) ((x) * 57.295779513082320876798154814105)

void LED_print_8(uint8_t data);

void crc_init();

#define CRC32_CHECK 0x2144DF1C
uint32_t crc32(uint8_t* data, uint32_t data_size);

//void correct_value(float* value, float* A, float* b, float* writeback);

void nav_set_value(NAV_Param parameter, float* value);
void nav_read_value(NAV_Param parameter, float* value);


#define DEVICE_ID 0xd5d5


typedef enum {
	NAV_REQUEST_HEADER_START,
	NAV_SET_VEC3_REQUEST_HEADER,
	NAV_READ_VEC3_REQUEST_HEADER,
	NAV_SET_SCALAR_REQUEST_HEADER,
	NAV_READ_SCALAR_REQUEST_HEADER
} NAV_RequestHeader;


typedef struct __attribute__((aligned(4))) { // aligned is for CRC calculation
	uint16_t device_id;
	
	// kalman filter output
	
	float position_x;
	float position_y;
	float position_z;
	
	float velocity_x;
	float velocity_y;
	float velocity_z;
	
	float accel_x;
	float accel_y;
	float accel_z;
	
	float accelraw_x;
	float accelraw_y;
	float accelraw_z;
	
	float orientation_x;
	float orientation_y;
	float orientation_z;
	
	float accelmagorientation_x;
	float accelmagorientation_y;
	float accelmagorientation_z;
	
	float angularvelocity_x;
	float angularvelocity_y;
	float angularvelocity_z;
	
	float mag_x;
	float mag_y;
	float mag_z;
	
	// gps output
	
	float latitude;
	float longitude;
	float gps_height;
	float h_acc; // horizontal accuracy
	float v_acc; // vertical accuracy
	uint16_t gps_satellites; // number of satellites
	uint32_t gps_watchdog; // time since last data in ms
	
	// raw data output
	
	float pressure; // pa
	float imu_temperature;
	float baro_temperature;
	
	float debug1;
	float debug2;
	
	
	uint32_t crc;
} Nav_Data_Packet_Type;


typedef union __attribute__((aligned(4))) { // union to ease accessing of data
	Nav_Data_Packet_Type bit;
	
	uint8_t reg[sizeof(Nav_Data_Packet_Type)];
} NAV_Data_Packet;


typedef struct __attribute__((aligned(4))) {
	uint16_t device_id;
	
	uint8_t device_code;
	uint8_t imu_code;
	uint8_t mag_code;
	uint8_t baro_code;
	
	uint32_t crc;
} NAV_Selftest_Packet_Type;


typedef union __attribute__((aligned(4))) {
	NAV_Selftest_Packet_Type bit;
	
	uint8_t reg[sizeof(NAV_Selftest_Packet_Type)];
} NAV_Selftest_Packet;


#define NAV_ACK_OK 0x0000
#define NAV_ACK_ERROR 0xFFFF

// packet returned to CTRL computer when more data is required to fulfill request
typedef struct __attribute__((aligned(4))) {
	uint16_t device_id;
	
	uint16_t status_code;
	
	uint32_t crc;
} NAV_ACK_Packet_Type;

typedef union __attribute__((aligned(4))) {
	NAV_ACK_Packet_Type bit;
	
	uint8_t reg[sizeof(NAV_ACK_Packet_Type)];
} NAV_ACK_Packet;


typedef struct __attribute__((aligned(4))) {
	uint16_t header;
	
	uint16_t parameter; // cast from CTRL_Param
	float data[3];
	
	uint32_t crc;
} Set_Vec3_Request_Type;


typedef union __attribute__((aligned(4))) {
	Set_Vec3_Request_Type bit;
	
	uint8_t reg[sizeof(Set_Vec3_Request_Type)];
} Set_Vec3_Request;


typedef struct __attribute__((aligned(4))) {
	uint16_t header;
	
	uint16_t parameter;
	
	uint32_t crc;
} Read_Vec3_Request_Type;

typedef union __attribute__((aligned(4))) {
	Read_Vec3_Request_Type bit;
	
	uint8_t reg[sizeof(Read_Vec3_Request_Type)];
} Read_Vec3_Request;

typedef struct __attribute__((aligned(4))) {
	uint16_t device_id;
	
	float data[3];
	
	uint32_t crc;
} Read_Vec3_Response_Type;

typedef union __attribute__((aligned(4))) {
	Read_Vec3_Response_Type bit;
	
	uint8_t reg[sizeof(Read_Vec3_Response_Type)];
} Read_Vec3_Response;


typedef struct __attribute__((aligned(4))) {
	uint16_t header;
	
	uint16_t parameter; // cast from CTRL_Param
	float data;
	
	uint32_t crc;
} Set_Scalar_Request_Type;


typedef union __attribute__((aligned(4))) {
	Set_Scalar_Request_Type bit;
	
	uint8_t reg[sizeof(Set_Scalar_Request_Type)];
} Set_Scalar_Request;

typedef struct __attribute__((aligned(4))) {
	uint16_t header;
	
	uint16_t parameter;
	
	uint32_t crc;
} Read_Scalar_Request_Type;

typedef union __attribute__((aligned(4))) {
	Read_Scalar_Request_Type bit;
	
	uint8_t reg[sizeof(Read_Scalar_Request_Type)];
} Read_Scalar_Request;

typedef struct __attribute__((aligned(4))) {
	uint16_t device_id;
	
	float data;
	
	uint32_t crc;
} Read_Scalar_Response_Type;

typedef union __attribute__((aligned(4))) {
	Read_Scalar_Response_Type bit;
	
	uint8_t reg[sizeof(Read_Scalar_Response_Type)];
} Read_Scalar_Response;


#endif