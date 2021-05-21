#include "kalman.h"


void kalman_predict_position(Position_State* state, Accel_Data data) {
	// get time since last calculation
	float i_time = read_timer_s();
	// reset timer
	start_timer();
	// get square value
	float i_time_squared = i_time * i_time;
	
	// create state transition matrix
	float F[36] = {
		1, 0, 0, i_time, 0, 0,
		0, 1, 0, 0, i_time, 0,
		0, 0, 1, 0, 0, i_time,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1
	};
	
	// create control matrix
	float G[18] = {
		0.5 * i_time_squared, 0, 0,
		0, 0.5 * i_time_squared, 0,
		0, 0, 0.5 * i_time_squared,
		i_time, 0, 0,
		0, i_time, 0,
		0, 0, i_time
	};
	
	
	// multiplication result from state multiplied by state transition
	float Fx[6];
	// multiplication result from data multiplied by input translation
	float Gu[6];
	
	// multiply state by state transition matrix and write to Fx
	mat_multiply(F, 6, 6, state->reg, 6, 1, Fx);
	
	// multiply data by data translation matrix and write to Gu
	mat_multiply(G, 6, 3, data.reg, 3, 1, Gu);
	
	// add resulting vectors and write back to state
	mat_add(Fx, Gu, 6, state->reg);
	
}