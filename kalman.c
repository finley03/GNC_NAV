#include "kalman.h"


// global file variable for last time
uint32_t predict_previous_time;

void kalman_predict_position(Position_State* state, Accel_Data data) {
	//// get time since last calculation
	//float i_time = read_timer_s();
	//// reset timer
	//start_timer();
	//// get square value
	//float i_time_squared = i_time * i_time;
	
	// get current time
	uint32_t current_time = read_timer_20ns();
	// calculate time difference
	uint32_t delta_time = current_time - predict_previous_time;
	// reset previous time
	predict_previous_time = current_time;
	// convert previous time to float
	float i_time = delta_time * TIMER_S_MULTIPLIER;
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



uint32_t update_previous_time;

void kalman_update_position(Position_State* state, Position_State measured_state, float* kalman_gain) {
	// get current time
	uint32_t current_time = read_timer_20ns();
	// calculate time difference
	uint32_t delta_time = current_time - update_previous_time;
	// reset previous time
	update_previous_time = current_time;
	// convert previous time to float
	float i_time = delta_time * TIMER_S_MULTIPLIER;
	// get square value
	float i_time_squared = i_time * i_time;
	
	
	// create observation matrix (yes it's an identity matrix right now)
	float H[36] = {
		1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1
	};
	
	
	// multiplication result from state multiplied by observation matrix
	float Hx[6];
	// innovation
	float i[6];
	// multiplication result from multiplying innovation by kalman gain
	float Ki[6];
	
	// writebackarray for state
	Position_State state_writeback;
	
	// multiply state by observation matrix and write to Hx
	mat_multiply(H, 6, 6, state->reg, 6, 1, Hx);
	
	// subtract Hx from measured state
	mat_subtract(measured_state.reg, Hx, 6, i);
	
	// multiply innovation by kalman gain and write to Ki
	mat_multiply(kalman_gain, 6, 6, i, 6, 1, Ki);
	
	// write final result to state
	mat_add(state->reg, Ki, 6, state_writeback.reg);
	
	mat_copy(state_writeback.reg, 6, state->reg);
	
}