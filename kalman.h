#ifndef KALMAN_H
#define KALMAN_H

#include "time.h"
#include "mat.h"


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
	
	float reg[sizeof(Accel_Data_Type)];
} Accel_Data;


void kalman_predict_position(Position_State* state, Accel_Data data);




#endif