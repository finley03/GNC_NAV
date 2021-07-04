#include "kalman.h"


#include <math.h>


// Position Kalman Filter


//// global file variable for last time
//uint32_t predict_position_previous_time;

Accel_Data kalman_predict_position(Position_State* state, Accel_Data data, Orientation_State orientation, float* estimate_uncertainty) {
	static uint32_t predict_position_previous_time = 0;
	// get current time
	uint32_t current_time = read_timer_20ns();
	// calculate time difference
	uint32_t delta_time = current_time - predict_position_previous_time;
	// reset previous time
	predict_position_previous_time = current_time;
	// convert previous time to float
	float i_time = delta_time * TIMER_S_MULTIPLIER;
	// get square value
	float i_time_squared = i_time * i_time;
	// get cubed value
	float i_time_cubed = i_time_squared * i_time;
	// get value ^ 4
	float i_time_4 = i_time_squared * i_time_squared;
	
	
	//----------STATE EXTRAPOLATION----------//
	
	
	// euler kinematic matrix
	// transforms acceleration from accelerometer in board frame
	// to acceleration in world frame.
	
	float pi_180 = 0.01745329;
	
	float sinx = sin(orientation.bit.orientation_x * pi_180);
	float siny = sin(orientation.bit.orientation_y * pi_180);
	float sinz = sin(orientation.bit.orientation_z * pi_180);
	float cosx = cos(orientation.bit.orientation_x * pi_180);
	float cosy = cos(orientation.bit.orientation_y * pi_180);
	float cosz = cos(orientation.bit.orientation_z * pi_180);
	
	// correct error in variables
	float euler_mat[9] = {
		cosy*cosz, sinx*siny*cosz-cosx*sinz, cosx*siny*cosz+sinx*sinz,
		cosy*sinz, sinx*siny*sinz+cosx*cosz, cosx*siny*sinz-sinx*cosz,
		-siny, sinx*cosy, cosx*cosy
	};
	
	
	// vector of transformed accel data
	float accel_data[3];
	
	mat_multiply(euler_mat, 3, 3, data.reg, 3, 1, accel_data);
	
	// add gravity offset to Z axis
	// z is down therefore gravity gives negative acceleration
	accel_data[2] += 9.80665;
	
	
	
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
	mat_multiply(G, 6, 3, accel_data, 3, 1, Gu);
	
	// add resulting vectors and write back to state
	mat_add(Fx, Gu, 6, state->reg);
	
	
	//----------CALCULATE PROCESS UNCERTAINTY----------//
	
	
	// calculate position variance
	float x_var = i_time_4 * 0.25 * KALMAN_ACCEL_VARIANCE;
	// calculate velocity variance
	float v_var = i_time_squared * KALMAN_ACCEL_VARIANCE;
	// calculate position velocity covariance
	float xv_cov = i_time_cubed * 0.5 *	KALMAN_ACCEL_VARIANCE;
	
	// process uncertainty matrix
	float Q[36] = {
		x_var, 0, 0, xv_cov, 0, 0,
		0, x_var, 0, 0, xv_cov, 0,
		0, 0, x_var, 0, 0, xv_cov,
		xv_cov, 0, 0, v_var, 0, 0,
		0, xv_cov, 0, 0, v_var, 0,
		0, 0, xv_cov, 0, 0, v_var
	};
	
	
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
	mat_add(FPF_t, Q, 36, estimate_uncertainty);
	
	
	Accel_Data transformed_accel_data;
	transformed_accel_data.bit.accel_x = accel_data[0];
	transformed_accel_data.bit.accel_y = accel_data[1];
	transformed_accel_data.bit.accel_z = accel_data[2];
	return transformed_accel_data;
}


//
//uint32_t update_position_previous_time;

void kalman_update_position(Position_State* state, Position_Data data, Orientation_State orientation, float* estimate_uncertainty, float* measurement_uncertainty, float* accelerometer_bias) {
	static uint32_t update_position_previous_time = 0;
	// get current time
	uint32_t current_time = read_timer_20ns();
	// calculate time difference
	uint32_t delta_time = current_time - update_position_previous_time;
	// reset previous time
	update_position_previous_time = current_time;
	// convert previous time to float
	float i_time = delta_time * TIMER_S_MULTIPLIER;
	// get square value
	//float i_time_squared = i_time * i_time;
	
	
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
	
	// add values (corrected dimension error)
	mat_add(state->reg, Ki, 6, new_state);
	
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
	
	// untested
	
	
	// calculate transpose of K
	mat_transpose(K, 6, 3, K_t);
	
	
	// multiply K_t by result
	mat_multiply(KR, 6, 3, K_t, 3, 6, KRK_t);
	
	
	
	// add final result
	mat_add(I6_KHPI6_KH_t, KRK_t, 36, estimate_uncertainty);
	

	////----------CALCULATE ACCELEROMETER BIAS CORRECTIONS----------//
	//
	//
	//// corrections to velocity calculated by the kalman filter
	//float v_corrections[3] = { Ki[3], Ki[4], Ki[5] };
	//
	//// convert to accelerometer correction values in world space
	//float a_corrections[3];
	//float a_corrections_transformed[3];
	//
	//mat_scalar_product(v_corrections, 1/(i_time * 500), 3, a_corrections);
	//
	//// euler kinematic matrix
	//// transforms acceleration from accelerometer in board frame
	//// to acceleration in world frame.
		//
	//float pi_180 = 0.01745329;
		//
	//float sinx = sin(orientation.bit.orientation_x * pi_180);
	//float siny = sin(orientation.bit.orientation_y * pi_180);
	//float sinz = sin(orientation.bit.orientation_z * pi_180);
	//float cosx = cos(orientation.bit.orientation_x * pi_180);
	//float cosy = cos(orientation.bit.orientation_y * pi_180);
	//float cosz = cos(orientation.bit.orientation_z * pi_180);
		//
	//// correct error in variables
	//float euler_mat[9] = {
		//cosy*cosz, sinx*siny*cosz-cosx*sinz, cosx*siny*cosz+sinx*sinz,
		//cosy*sinz, sinx*siny*sinz+cosx*cosz, cosx*siny*sinz-sinx*cosz,
		//-siny, sinx*cosy, cosx*cosy
	//};
	//
	//// by taking the inverse, we get the equation that will transform
	//// world space acceleration into local space acceleration
	//
	//float euler_mat_inv[9];
	//
	//mat_inverse_3x3(euler_mat, euler_mat_inv);
	//
	//// multiply a_corrections by inverse of euler mat and write back
	//mat_multiply(euler_mat_inv, 3, 3, a_corrections, 3, 1, a_corrections_transformed);
	//
	//// add transformed corrections to bias
	//mat_add(accelerometer_bias, a_corrections_transformed, 3, accelerometer_bias);
}


void kalman_position_measurement_uncertainty(float* writeback, float hAcc, float vAcc) {
	// given values for accuracy should be standard deviation
	// square values to get variance
	float hAcc_2 = hAcc * hAcc;
	float vAcc_2 = vAcc * vAcc;
	
	// x axis variance
	// set covariance values to 0
	writeback[0] = hAcc_2 * 0.5;
	writeback[1] = 0;
	writeback[2] = 0;
	writeback[3] = 0;
	// y axis variance
	writeback[4] = hAcc_2 * 0.5;
	writeback[5] = 0;
	writeback[6] = 0;
	writeback[7] = 0;
	// z axis variance
	//writeback[8] = vAcc_2;
	writeback[8] = 2000;
}




// Orientation Kalman Filter

//uint32_t predict_orientation_previous_time;

void kalman_predict_orientation(Orientation_State* state, Gyro_Data data, float* estimate_uncertainty) {
	static uint32_t predict_orientation_previous_time = 0;
	// get current time
	uint32_t current_time = read_timer_20ns();
	// calculate time difference
	uint32_t delta_time = current_time - predict_orientation_previous_time;
	// reset previous time
	predict_orientation_previous_time = current_time;
	// convert previous time to float
	float i_time = delta_time * TIMER_S_MULTIPLIER;
	// get square value
	float i_time_squared = i_time * i_time;
	
	
	
	//----------STATE EXTRAPOLATION----------//
	
	
	// create state transition matrix (no velocity or acceleration therefore identity)
	float F[9] = {
		1, 0, 0,
		0, 1, 0,
		0, 0, 1
	};
	
	// create control matrix
	// convert angular velocity in body frame to change in euler angles in north east down frame
	
	const float pi_180 = 0.01745329;
	
	//float sinx = sin(state->bit.orientation_x * pi_180);
	//float siny = sin(state->bit.orientation_y * pi_180);
	//float sinz = sin(state->bit.orientation_z * pi_180);
	//float cosx = cos(state->bit.orientation_x * pi_180);
	//float cosy = cos(state->bit.orientation_y * pi_180);
	//float cosz = cos(state->bit.orientation_z * pi_180);
	float sinx = sin(state->bit.orientation_x * pi_180);
	float cosx = cos(state->bit.orientation_x * pi_180);
	float cosy = cos(state->bit.orientation_y * pi_180);
	float tany = tan(state->bit.orientation_y * pi_180);
	float secy = 1 / cosy;
	
	float G[9] = {
		(1)*i_time, (sinx*tany)*i_time, (cosx*tany)*i_time,
		0, (cosx)*i_time, (-sinx)*i_time,
		0, (sinx*secy)*i_time, (cosx*secy)*i_time
	};
	//float G[9] = {
		//(cosy*cosz)*i_time, (sinx*siny*cosz-cosx*sinz)*i_time, (cosx*siny*cosz+sinx*sinz)*i_time,
		//(cosy*sinz)*i_time, (sinz*siny*sinz+cosx*cosz)*i_time, (cosx*siny*sinz-sinx*cosz)*i_time,
		//(-siny)*i_time, (sinx*cosy)*i_time, (cosx*cosy)*i_time
	//};
	
	
	// multiplication result from state multiplied by state transition
	float Fx[3];
	// multiplication result from data multiplied by input translation
	float Gu[3];
	
	// multiply state by state transition matrix and write to Fx
	mat_multiply(F, 3, 3, state->reg, 3, 1, Fx);
	
	// multiply data by data translation matrix and write to Gu
	mat_multiply(G, 3, 3, data.reg, 3, 1, Gu);
	
	// add resulting vectors and write back to state
	mat_add(Fx, Gu, 3, state->reg);
	
	// set angle range
	kalman_set_axes_range(state->reg);
	
	
	//----------CALCULATE PROCESS UNCERTAINTY----------//
	
	
	// calculate orientation variance
	float r_var = i_time_squared * KALMAN_ANGULARVELOCITY_VARIANCE;

	
	// process uncertainty matrix
	float Q[9] = {
		r_var, 0, 0,
		0, r_var, 0,
		0, 0, r_var
	};
	
	
	//----------UNCERTAINTY EXTRAPOLATION----------//
	
	
	// current estimate uncertainty multiplied by state transition matrix
	float FP[9];
	// transpose of state transition matrix
	float F_t[9];
	// result of multpilication block
	float FPF_t[9];
	
	
	// get transpose of F
	mat_transpose(F, 3, 3, F_t);
	
	// multiply P by F
	mat_multiply(F, 3, 3, estimate_uncertainty, 3, 3, FP);
	
	// multiply F_t by FP
	mat_multiply(FP, 3, 3, F_t, 3, 3, FPF_t);
	
	// add process noise and write back to estimate uncertainty
	mat_add(FPF_t, Q, 9, estimate_uncertainty);
	
}


//uint32_t update_orientation_previous_time;

void kalman_update_orientation(Orientation_State* state, Orientation_State data, float* estimate_uncertainty, float* measurement_uncertainty) {
	static uint32_t update_orientation_previous_time = 0;
	// get current time
	uint32_t current_time = read_timer_20ns();
	// calculate time difference
	uint32_t delta_time = current_time - update_orientation_previous_time;
	// reset previous time
	update_orientation_previous_time = current_time;
	// convert previous time to float
	float i_time = delta_time * TIMER_S_MULTIPLIER;
	// get square value
	//float i_time_squared = i_time * i_time;
	
	
	
	if (state->bit.orientation_x > data.bit.orientation_x + 180) data.bit.orientation_x += 360;
	if (state->bit.orientation_x < data.bit.orientation_x - 180) data.bit.orientation_x -= 360;
	
	if (state->bit.orientation_z > data.bit.orientation_z + 180) data.bit.orientation_z += 360;
	if (state->bit.orientation_z < data.bit.orientation_z - 180) data.bit.orientation_z -= 360;
	
		
		
	//----------CALCULATE KALMAN GAIN----------//
		
		
	// kalman gain matrix
	float K[9];
		
	// observation matrix
	float H[9] = {
		1, 0, 0,
		0, 1, 0,
		0, 0, 1
	};
		
	// transposed observation matrix
	float H_t[9];
	// estimate uncertainty multiplied by observation matrix transpose
	float PH_t[9];
	// 3x3 estimate uncertainty
	float HPH_t[9];
	// innovation
	float S[9];
	// S^-1
	float Sinv[9];
		
		
	// get H transpose
	mat_transpose(H, 3, 3, H_t);
		
	// multiply estimate uncertainty by transpose of observation matrix
	mat_multiply(estimate_uncertainty, 3, 3, H_t, 3, 3, PH_t);
		
	// multiply result by H
	mat_multiply(H, 3, 3, PH_t, 3, 3, HPH_t);
		
	// calculate innovation
	mat_add(HPH_t, measurement_uncertainty, 9, S);
		
	// calculate inverse of innovation
	mat_inverse_3x3(S, Sinv);
		
	// multiply Sinv by PH_t
	mat_multiply(PH_t, 3, 3, Sinv, 3, 3, K);
		
		
	//----------UPDATE STATE----------//
		
		
	// state multiplied by observation matrix
	float Hx[3];
		
	// innovation
	float innovation[3];
		
	// change
	float Ki[3];
		
	// new state
	float new_state[3];
		
		
	// multiply state by observation matrix
	mat_multiply(H, 3, 3, state->reg, 3, 1, Hx);
		
	// subtract
	mat_subtract(data.reg, Hx, 3, innovation);
		
	// multiply by kalman gain
	mat_multiply(K, 3, 3, innovation, 3, 1, Ki);
		
	// add values (corrected dimension error)
	mat_add(state->reg, Ki, 3, new_state);
		
	// copy to state matrix
	mat_copy(new_state, 3, state->reg);
	
	// set angle range
	kalman_set_axes_range(state->reg);
		
		
		
	//----------UPDATE ESTIMATE UNCERTAINTY----------
		
		
	// kalman gain multiplied by observation matrix
	float KH[9];
		
	// identity
	float I3[9] = {
		1, 0, 0,
		0, 1, 0,
		0, 0, 1
	};
		
	// identity minus KH
	float I3_KH[9];
		
	// transpose
	float I3_KH_t[9];
		
	// first element
	float I3_KHP[9];
		
	// combined
	float I3_KHPI3_KH_t[9];
		
	// k transpose
	float K_t[9];
		
	float KR[9];
		
	float KRK_t[9];
		
		
	// calculate KH
	mat_multiply(K, 3, 3, H, 3, 3, KH);
		
	// subtract from identity
	mat_subtract(I3, KH, 9, I3_KH);
		
	// calculate transpose
	mat_transpose(I3_KH, 3, 3, I3_KH_t);
		
		
	mat_multiply(I3_KH, 3, 3, estimate_uncertainty, 3, 3, I3_KHP);
		
		
	mat_multiply(I3_KHP, 3, 3, I3_KH_t, 3, 3, I3_KHPI3_KH_t);
		
		
		
		
	// multiply kalman gain by measurement uncertainty
	mat_multiply(K, 3, 3, measurement_uncertainty, 3, 3, KR);
		
	// untested
		
		
	// calculate transpose of K
	mat_transpose(K, 3, 3, K_t);
		
		
	// multiply K_t by result
	mat_multiply(KR, 3, 3, K_t, 3, 3, KRK_t);
		
		
		
	// add final result
	mat_add(I3_KHPI3_KH_t, KRK_t, 9, estimate_uncertainty);
		
}


Orientation_State kalman_orientation_generate_state(MAG_Data mag_data, Accel_Data accel_data) {
	// define vectors for calculating orientation
	float north[3], east[3], down[3];
	
	//// transform accelerometer east-north-up to north-east-down and negate to get down vector from gravity vector
	// no transformations needed other than negation
	float accel[3];
	//accel[0] = -accel_data.bit.accel_y;
	//accel[1] = -accel_data.bit.accel_x;
	//accel[2] = accel_data.bit.accel_z;
	accel[0] = -accel_data.bit.accel_x;
	accel[1] = -accel_data.bit.accel_y;
	accel[2] = -accel_data.bit.accel_z;
	
	// normalize accel data and write to down
	// gravity should somewhat follow down
	mat_3_normalize(accel, down);
	
	// populate mag data array
	float mag[3];
	mag[0] = mag_data.bit.mag_x;
	mag[1] = mag_data.bit.mag_y;
	mag[2] = mag_data.bit.mag_z;
	
	// normalize mag array and write back to mag
	mat_3_normalize(mag, mag);
	
	// take cross product down x mag to find east
	mat_crossp(down, mag, east);
	
	// normalize east
	mat_3_normalize(east, east);
	
	// take cross product east x down to find north
	mat_crossp(east, down, north);
	
	// double check north is fully normalized
	mat_3_normalize(north, north);
	
	
	// calculate Euler angles from pseudo
	// direction cosine matrix made from north east down vectors
	// extend inverse trig functions to +- 180
	
	Orientation_State orientation_state;
	
	float a180_pi = 57.29577951;
	
	orientation_state.bit.orientation_x = atan(down[1] / down[2]) * a180_pi;
	if (down[2] < 0) {
		if (down[1] > 0) orientation_state.bit.orientation_x += 180;
		else if (down[1] < 0) orientation_state.bit.orientation_x -= 180;
	}
	
	orientation_state.bit.orientation_y = -asin(down[0]) * a180_pi;
	
	orientation_state.bit.orientation_z = atan(east[0] / north[0]) * a180_pi;
	if (north[0] < 0) {
		if (east[0] > 0) orientation_state.bit.orientation_z += 180;
		else if (east[0] < 0) orientation_state.bit.orientation_z -= 180;
	}
	
	return orientation_state;
}



void kalman_set_axes_range(float* array) {
	// set x axis
	if (array[0] > 180) array[0] -= 360;
	else if (array[0] < -180) array[0] += 360;
	
	// set z axis
	if (array[2] > 180) array[2] -= 360;
	else if (array[2] < -180) array[2] += 360;
	
	// set y axis
	if (array[1] > 90) {
		array[1] = 180 - array[1];
		
		if (array[0] >= 0) array[0] -= 180;
		else if (array[0] < 0) array[0] += 180;
		
		if (array[2] >= 0) array[2] -= 180;
		else if (array[2] < 0) array[2] += 180;
	}
	else if (array[1] < -90) {
		array[1] = -180 - array[1];
		
		if (array[0] >= 0) array[0] -= 180;
		else if (array[0] < 0) array[0] += 180;
		
		if (array[2] >= 0) array[2] -= 180;
		else if (array[2] < 0) array[2] += 180;
	}
}