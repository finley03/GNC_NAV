#include "util.h"
#include "mat.h"


//extern float kalman_run;
extern float kalman_gnss_horizontal_uncertainty_mul;
extern float kalman_gnss_vertical_uncertainty_mul;
extern float kalman_accel_variance;
extern float kalman_angularvelocity_variance;
extern float kalman_baro_variance;
extern float gnss_zerolat;
extern float gnss_zerolong;
extern float orientation_measurement_uncertainty[9];
extern float position_estimate_uncertainty[36];
extern float orientation_estimate_uncertainty[9];



void LED_print_8(uint8_t data) {
	for (uint16_t i = 0; i < 8; ++i) {
		uint8_t bit = data & 0x80;
		
		if (bit != 0) {
			REG_PORT_OUTSET0 = LED;
		}
		
		delay_ms(250);
		
		REG_PORT_OUTCLR0 = LED;
		
		delay_ms(250);
		
		data = (data << 1);
	}
}


void crc_init() {
	// enable writing to DSU
	PAC1->WPCLR.reg = 0x2;
	// define CRC type to 32bit
	DMAC->CRCCTRL.bit.CRCPOLY = 0x1;
}


uint32_t crc32(uint8_t* data, uint32_t data_size) {
	*((volatile unsigned int*) 0x41007058) &= ~0x30000UL;
	
	// fill data register with starting value
	//DSU->DATA.reg = 0xffffffff;
	DSU->DATA.reg = 0xffffffff;
	
	DSU->ADDR.reg = (uint32_t) data;
	DSU->LENGTH.reg = DSU_LENGTH_LENGTH((data_size / 4));
	
	// clear done bit
	DSU->STATUSA.bit.DONE = 1;
	
	// start CRC calculation
	DSU->CTRL.bit.CRC = 1;
	
	// wait until done
	while(!DSU->STATUSA.bit.DONE);
	
	uint32_t out = DSU->DATA.reg ^ 0xffffffff;
	
	*((volatile unsigned int*) 0x41007058) |= 0x20000UL;
	
	return out;
}


void nav_set_value(NAV_Param parameter, float* value) {
	switch (parameter) {
		case _KALMAN_POSITION_UNCERTAINTY:
		position_estimate_uncertainty[0] = value[0];
		position_estimate_uncertainty[7] = value[1];
		position_estimate_uncertainty[14] = value[2];
		break;
		case _KALMAN_VELOCITY_UNCERTAINTY:
		position_estimate_uncertainty[21] = value[0];
		position_estimate_uncertainty[28] = value[1];
		position_estimate_uncertainty[35] = value[2];
		break;
		case _KALMAN_ORIENTATION_UNCERTAINTY:
		orientation_estimate_uncertainty[0] = value[0];
		orientation_estimate_uncertainty[4] = value[1];
		orientation_estimate_uncertainty[8] = value[2];
		break;
		case _KALMAN_ORIENTATION_MEASUREMENT_UNCERTAINTY:
		orientation_measurement_uncertainty[0] = value[0];
		orientation_measurement_uncertainty[4] = value[1];
		orientation_measurement_uncertainty[8] = value[2];
		break;
		case _KALMAN_GNSS_HORIZONTAL_UNCERTAINTY_MUL:
		kalman_gnss_horizontal_uncertainty_mul = *value;
		break;
		case _KALMAN_GNSS_VERTICAL_UNCERTAINTY_MUL:
		kalman_gnss_vertical_uncertainty_mul = *value;
		break;
		case _KALMAN_BARO_VARIANCE:
		kalman_baro_variance = *value;
		break;
		case _KALMAN_ACCEL_VARIANCE:
		kalman_accel_variance = *value;
		break;
		case _KALMAN_ANGULARVELOCITY_VARIANCE:
		kalman_angularvelocity_variance = *value;
		break;
		case _KALMAN_GNSS_ZEROLAT:
		gnss_zerolat = *value;
		break;
		case _KALMAN_GNSS_ZEROLONG:
		gnss_zerolong = *value;
		break;
		//case _KALMAN_RUN:
		//kalman_run = *value;
		//break;
		default:
		break;
	}
}

void nav_read_value(NAV_Param parameter, float* value) {
	switch (parameter) {
		case _KALMAN_POSITION_UNCERTAINTY:
		value[0] = position_estimate_uncertainty[0];
		value[1] = position_estimate_uncertainty[7];
		value[2] = position_estimate_uncertainty[14];
		break;
		case _KALMAN_VELOCITY_UNCERTAINTY:
		value[0] = position_estimate_uncertainty[21];
		value[1] = position_estimate_uncertainty[28];
		value[2] = position_estimate_uncertainty[35];
		break;
		case _KALMAN_ORIENTATION_UNCERTAINTY:
		value[0] = orientation_estimate_uncertainty[0];
		value[1] = orientation_estimate_uncertainty[4];
		value[2] = orientation_estimate_uncertainty[8];
		break;
		case _KALMAN_ORIENTATION_MEASUREMENT_UNCERTAINTY:
		value[0] = orientation_measurement_uncertainty[0];
		value[1] = orientation_measurement_uncertainty[4];
		value[2] = orientation_measurement_uncertainty[8];
		break;
		case _KALMAN_GNSS_HORIZONTAL_UNCERTAINTY_MUL:
		*value = kalman_gnss_horizontal_uncertainty_mul;
		break;
		case _KALMAN_GNSS_VERTICAL_UNCERTAINTY_MUL:
		*value = kalman_gnss_vertical_uncertainty_mul;
		break;
		case _KALMAN_BARO_VARIANCE:
		*value = kalman_baro_variance;
		break;
		case _KALMAN_ACCEL_VARIANCE:
		*value = kalman_accel_variance;
		break;
		case _KALMAN_ANGULARVELOCITY_VARIANCE:
		*value = kalman_angularvelocity_variance;
		break;
		case _KALMAN_GNSS_ZEROLAT:
		*value = gnss_zerolat;
		break;
		case _KALMAN_GNSS_ZEROLONG:
		*value = gnss_zerolong;
		break;
		//case _KALMAN_RUN:
		//*value = kalman_run;
		//break;
		default:
		break;
	}
}