#ifndef KALMAN_H
#define KALMAN_H

#include "util.h"
#include "mat.h"
#include "imu.h"


// assume acceleration standard deviation of 3m/s
#define KALMAN_ACCEL_VARIANCE 9
// assume angle standard deviation of 10 degrees/s
#define KALMAN_ANGULARVELOCITY_VARIANCE 100



// data arrangement of position state vector
typedef struct __attribute__((packed)) {
	float position_x;
	float position_y;
	float position_z;
	
	float velocity_x;
	float velocity_y;
	float velocity_z;
} Position_State_Type;

// union for position state vector
typedef union {
	Position_State_Type bit;
	
	float reg[sizeof(Position_State_Type)];
} Position_State;

// data arrangement for measurement data vector
typedef struct __attribute__((packed)) {
	float accel_x;
	float accel_y;
	float accel_z;
} Accel_Data_Type;

// union for measurement data vector
typedef union {
	Accel_Data_Type bit;
	
	float reg[sizeof(Accel_Data_Type) / sizeof(float)];
} Accel_Data;

// data arrangement for position measurement vector
typedef struct __attribute__((packed)) {
	float position_x;
	float position_y;
	float position_z;
} Position_Data_Type;

// union for positon measurement vector
typedef union {
	Position_Data_Type bit;
	
	float reg[sizeof(Position_Data_Type) / sizeof(float)];
} Position_Data;



// data arrangement of orientation state
typedef struct __attribute__((packed)) {
	float orientation_x;
	float orientation_y;
	float orientation_z;
} Orientation_State_Type;

// union for orientation state vector
typedef union {
	Orientation_State_Type bit;
	
	float reg[sizeof(Orientation_State_Type)];
} Orientation_State;


typedef struct __attribute__((packed)) {
	float rotation_x;
	float rotation_y;
	float rotation_z;
} Gyro_Data_Type;


typedef union {
	Gyro_Data_Type bit;
	
	float reg[sizeof(Gyro_Data_Type)];
} Gyro_Data;


void kalman_init();

// position kalman filter

// 6 dimensional state (position, velocity)
// 3 dimensional predict measurements (acceleration)
// 3 dimensional update measurements (position)

// tested
Accel_Data kalman_predict_position(Position_State* state, Accel_Data data, Orientation_State orientation);

// tested
void kalman_update_position(Position_State* state, Position_Data data, Orientation_State orientation, float* measurement_uncertainty);

// tested
void kalman_position_measurement_uncertainty(float* writeback, float hAcc, float vAcc);

void kalman_position_measurement_uncertainty_baro(float* writeback, float hAcc);


// orientation kalman filter

// 3 dimensional state (orientation in euler angles), z: yaw, y: pitch, x: roll
// 3 dimensional predict measurements (angular velocity relative to body frame)
// 3 dimensional update measurements (orientation in very imprecise euler angles)

void kalman_predict_orientation(Orientation_State* state, Gyro_Data data);

void kalman_update_orientation(Orientation_State* state, Orientation_State data);

Orientation_State kalman_orientation_generate_state(MAG_Data mag_data, Accel_Data accel_data);

void kalman_set_axes_range(float* array);




#endif