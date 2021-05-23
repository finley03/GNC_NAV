#include "kalman.h"


// global file variable for last time
uint32_t predict_previous_time;

void kalman_predict_position(Position_State* state, Accel_Data data, float* estimate_uncertainty, float* process_uncertainty) {
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
	
	
	//----------STATE EXTRAPOLATION----------//
	
	
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
	
	
	//----------UNCERTAINTY EXTRAPOLATION----------//
	
	
	// current estimate uncertainty multiplied by state transition matrix
	float FP[36];
	// transpose of state transition matrix
	float F_t[36];
	// result of multpilication block
	float FPF_t[36];
	
	
	// get transpose of F
	mat_transpose(F, 6, 6, F_t);
	
	// multiply P by F
	mat_multiply(F, 6, 6, estimate_uncertainty, 6, 6, FP);
	
	// multiply F_t by FP
	mat_multiply(FP, 6, 6, F_t, 6, 6, FPF_t);
	
	// add process noise and write back to estimate uncertainty
	mat_add(FPF_t, process_uncertainty, 36, estimate_uncertainty);
	
}



uint32_t update_previous_time;

void kalman_update_position(Position_State* state, Position_Data data, float* estimate_uncertainty, float* measurement_uncertainty) {
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
	
	
	//----------CALCULATE KALMAN GAIN----------//
		
	
	// kalman gain matrix
	float K[18];
	
	// observation matrix
	float H[18] = {
		1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0
	};
	
	// transposed observation matrix
	float H_t[18];
	// estimate uncertainty multiplied by observation matrix transpose
	float PH_t[18];
	// 3x3 estimate uncertainty
	float HPH_t[9];
	// innovation
	float S[9];
	// S^-1
	float Sinv[9];
	
	
	// get H transpose
	mat_transpose(H, 3, 6, H_t);
	
	// multiply estimate uncertainty by transpose of observation matrix
	mat_multiply(estimate_uncertainty, 6, 6, H_t, 6, 3, PH_t);
	
	// multiply result by H
	mat_multiply(H, 3, 6, PH_t, 6, 3, HPH_t);
	
	// calculate innovation
	mat_add(HPH_t, measurement_uncertainty, 9, S);
	
	// calculate inverse of innovation
	mat_inverse_3x3(S, Sinv);
	
	// multiply Sinv by PH_t
	mat_multiply(PH_t, 6, 3, Sinv, 3, 3, K);
	
	
	//----------UPDATE STATE----------//
	
	
	// state multiplied by observation matrix
	float Hx[3];
	
	// innovation
	float innovation[3];
	
	// change
	float Ki[6];
	
	// new state
	float new_state[6];
	
	
	// multiply state by observation matrix
	mat_multiply(H, 3, 6, state->reg, 6, 1, Hx);
	
	// subtract
	mat_subtract(data.reg, Hx, 3, innovation);
	
	// multiply by kalman gain
	mat_multiply(K, 6, 3, innovation, 3, 1, Ki);
	
	// add values
	mat_add(state->reg, Ki, 3, new_state);
	
	// copy to state matrix
	mat_copy(new_state, 6, state->reg);
	
	
	//----------UPDATE ESTIMATE UNCERTAINTY----------
	
	
	// kalman gain multiplied by observation matrix
	float KH[36];
	
	// identity
	float I6[36] = {
		1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1
	};
	
	// identity minus KH
	float I6_KH[36];
	
	// transpose
	float I6_KH_t[36];
	
	// first element
	float I6_KHP[36];
	
	// combined
	float I6_KHPI6_KH_t[36];
	
	// k transpose
	float K_t[36];
	
	float KR[18];
	
	float KRK_t[36];
	
	
	// calculate KH
	mat_multiply(K, 6, 3, H, 3, 6, KH);
	
	// subtract from identity
	mat_subtract(I6, KH, 36, I6_KH);
	
	// calculate transpose
	mat_transpose(I6_KH, 6, 6, I6_KH_t);
	
	
	mat_multiply(I6_KH, 6, 6, estimate_uncertainty, 6, 6, I6_KHP);
	
	
	mat_multiply(I6_KHP, 6, 6, I6_KH_t, 6, 6, I6_KHPI6_KH_t);
	
	
	
	
	// multiply kalman gain by measurement uncertainty
	mat_multiply(K, 6, 3, measurement_uncertainty, 3, 3, KR);
	
	// multiply K_t by result
	mat_multiply(KR, 6, 3, K_t, 3, 6, KRK_t);
	
	
	
	// add final result
	mat_add(I6_KHPI6_KH_t, KRK_t, 36, estimate_uncertainty);

	
}