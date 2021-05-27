#ifndef KALMAN_H
#define KALMAN_H

#include "time.h"
#include "mat.h"



// assume acceleration standard deviation of 3
#define KALMAN_ACCEL_VARIANCE 9



// data arrangement of state vector
typedef struct __attribute__((packed)) {
	float position_x;
	float position_y;
	float position_z;
	
	float velocity_x;
	float velocity_y;
	float velocity_z;
} Position_State_Type;

// union for state vector
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



void kalman_predict_position(Position_State* state, Accel_Data data, float* estimate_uncertainty);

float kalman_update_position(Position_State* state, Position_Data data, float* estimate_uncertainty, float* measurement_uncertainty);

void kalman_measurement_uncertainty(float* writeback, float hAcc, float vAcc);




#endif