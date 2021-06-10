#include "main.h"


void init();
void txc_data();
void system_check();


NAV_Selftest_Packet nav_selftest_packet;
NAV_Data_Packet nav_data_packet;


int main(void) {
	init();
	
	nav_data_packet.bit.device_id = DEVICE_ID;
	
	MAG_Cal_Data mag_cal_data = mag_cal();
	
	gps_init_dma();
	
	// initialise timer for first iteration
	start_timer();
	
	
	MAG_Data mag_data;
	Accel_Data accel_data;
	
	
	// create position state variable
	Position_State position_state;
	position_state.bit.position_x = 0;
	position_state.bit.position_y = 0;
	position_state.bit.position_z = 0;
	position_state.bit.velocity_x = 0;
	position_state.bit.velocity_y = 0;
	position_state.bit.velocity_z = 0;
	
	
	// set initial position estimate uncertainty;
	float position_estimate_uncertainty[36] = {
		1000, 0, 0, 0, 0, 0,
		0, 1000, 0, 0, 0, 0,
		0, 0, 1000, 0, 0, 0,
		0, 0, 0, 1000, 0, 0,
		0, 0, 0, 0, 1000, 0,
		0, 0, 0, 0, 0, 1000
	};
	
	// create position measurement uncertainty variable
	float position_measurement_uncertainty[9];
	
	
	
	// create orientation state variable
	Orientation_State orientation_state;
	orientation_state.bit.orientation_x = 0;
	orientation_state.bit.orientation_y = 0;
	orientation_state.bit.orientation_z = 0;
	
	
	// set initial orientation estimate uncertainty;
	float orientation_estimate_uncertainty[9] = {
		8000, 0, 0,
		0, 8000, 0,
		0, 0, 8000
	};
	
	// set orientation measurement uncertainty
	float orientation_measurement_uncertainty[9] = {
		8000, 0, 0,
		0, 8000, 0,
		0, 0, 8000
	};
	
	// accelerometer magnetometer orientation estimate
	Orientation_State accel_mag_orientation;
	
	
	while(1) {
		delay_ms(2);
		
		if (gps_dma_check_complete()) {
			float latitude = ubx_nav_pvt.bit.lat * 1E-7;
			float longitude = ubx_nav_pvt.bit.lon * 1E-7;
			float height = ubx_nav_pvt.bit.height * 1E-3;
			float hAcc = ubx_nav_pvt.bit.hAcc * 1E-3;
			float vAcc = ubx_nav_pvt.bit.vAcc * 1E-3;
			
			
			nav_data_packet.bit.latitude = latitude;
			nav_data_packet.bit.longitude = longitude;
			nav_data_packet.bit.gps_height = height;
			nav_data_packet.bit.h_acc = hAcc;
			nav_data_packet.bit.v_acc = vAcc;
			nav_data_packet.bit.gps_satellites = ubx_nav_pvt.bit.numSV;
			
			
			if (nav_data_packet.bit.h_acc >= 50) continue;
			
			
			// run position kalman update step
			
			// update measurement uncertainty
			kalman_position_measurement_uncertainty(position_measurement_uncertainty, hAcc, vAcc);
			
			// get cartesian coordinates or current position
			float x, y;
			
			gps_cartesian(latitude, longitude, &x, &y);
			
			// create measurement type
			Position_Data position_data;
			position_data.bit.position_x = x;
			position_data.bit.position_y = y;
			position_data.bit.position_z = -height;
			
			
			kalman_update_position(&position_state, position_data, position_estimate_uncertainty, position_measurement_uncertainty);
			
			
			
			// run orientation kalman update step
			
			
			
			accel_mag_orientation = kalman_orientation_generate_state(mag_data, accel_data);
			
			kalman_update_orientation(&orientation_state, accel_mag_orientation, orientation_estimate_uncertainty, orientation_measurement_uncertainty);
			
			
			// get barometer pressure readings
			nav_data_packet.bit.pressure = baro_get_pressure(&nav_data_packet.bit.baro_temperature);
			
		}
		
		
		IMU_Data imu_data = imu_get_data();
		
		
		MAG_Data mag_data_uncalibrated = mag_get_data();
		//MAG_Data mag_data;
		mag_data.mag_x = (mag_data_uncalibrated.mag_x - mag_cal_data.bias_x) * mag_cal_data.scale_x;
		mag_data.mag_y = (mag_data_uncalibrated.mag_y - mag_cal_data.bias_y) * mag_cal_data.scale_y;
		mag_data.mag_z = (mag_data_uncalibrated.mag_z - mag_cal_data.bias_z) * mag_cal_data.scale_z;
		
		// create accelerometer data type
		accel_data.bit.accel_x = imu_data.accel_x;
		accel_data.bit.accel_y = imu_data.accel_y;
		accel_data.bit.accel_z = imu_data.accel_z;
		
		Gyro_Data gyro_data;
		gyro_data.bit.rotation_x = imu_data.gyro_x;
		gyro_data.bit.rotation_y = imu_data.gyro_y;
		gyro_data.bit.rotation_z = imu_data.gyro_z;
		
		
		Accel_Data transformed_accel;
		
		if (nav_data_packet.bit.h_acc < 50) {
			// run kalman predict step
			transformed_accel = kalman_predict_position(&position_state, accel_data, orientation_state, position_estimate_uncertainty);
			kalman_predict_orientation(&orientation_state, gyro_data, orientation_estimate_uncertainty);
		}
		
		nav_data_packet.bit.position_x = position_state.bit.position_x;
		nav_data_packet.bit.position_y = position_state.bit.position_y;
		nav_data_packet.bit.position_z = position_state.bit.position_z;
		
		nav_data_packet.bit.velocity_x = position_state.bit.velocity_x;
		nav_data_packet.bit.velocity_y = position_state.bit.velocity_y;
		nav_data_packet.bit.velocity_z = position_state.bit.velocity_z;
		
		nav_data_packet.bit.orientation_x = orientation_state.bit.orientation_x;
		nav_data_packet.bit.orientation_y = orientation_state.bit.orientation_y;
		nav_data_packet.bit.orientation_z = orientation_state.bit.orientation_z;
		
		nav_data_packet.bit.accelmagorientation_x = accel_mag_orientation.bit.orientation_x;
		nav_data_packet.bit.accelmagorientation_y = accel_mag_orientation.bit.orientation_y;
		nav_data_packet.bit.accelmagorientation_z = accel_mag_orientation.bit.orientation_z;
		
		nav_data_packet.bit.angularvelocity_x = imu_data.gyro_x;
		nav_data_packet.bit.angularvelocity_y = imu_data.gyro_y;
		nav_data_packet.bit.angularvelocity_z = imu_data.gyro_z;
		
		nav_data_packet.bit.accel_x = transformed_accel.bit.accel_x;
		nav_data_packet.bit.accel_y = transformed_accel.bit.accel_y;
		nav_data_packet.bit.accel_z = transformed_accel.bit.accel_z;
		
		nav_data_packet.bit.accelraw_x = imu_data.accel_x;
		nav_data_packet.bit.accelraw_y = imu_data.accel_y;
		nav_data_packet.bit.accelraw_z = imu_data.accel_z;
		
		nav_data_packet.bit.mag_x = mag_data.mag_x;
		nav_data_packet.bit.mag_y = mag_data.mag_y;
		nav_data_packet.bit.mag_z = mag_data.mag_z;
		
		nav_data_packet.bit.imu_temperature = imu_data.temp;
		
		nav_data_packet.bit.debug1 = 1;
		nav_data_packet.bit.debug2 = 2;
		
		
		txc_data();
	}
	
	return 0;
}



void system_check() {
	nav_selftest_packet.bit.device_id = DEVICE_ID;
	nav_selftest_packet.bit.baro_code = baro_check();
	nav_selftest_packet.bit.imu_code = imu_check();
	nav_selftest_packet.bit.mag_code = mag_check();
	
	if (!nav_selftest_packet.bit.baro_code && !nav_selftest_packet.bit.imu_code && !nav_selftest_packet.bit.mag_code) {
		nav_selftest_packet.bit.device_code = 0;
	}
	else {
		nav_selftest_packet.bit.device_code = 1;
	}
}


void init() {
	set_clock_48m();
	init_timer();

	crc_init();
	
	spi_init();
	control_uart_init();
	gps_uart_init();
	//gps_init_dma();
	
	// wait for peripherals to initialize
	delay_ms(100);
	
	imu_init();
	baro_init();
	configure_gps();
	
	
	// set LED to output
	REG_PORT_DIRSET0 = LED;
	//REG_PORT_OUTSET0 = LED;
	
	

	delay_ms(100);
	
	
	//if (system_check() != 0) {
		//REG_PORT_OUTSET0 = LED;
		//delay_ms(1000);
		//REG_PORT_OUTCLR0 = LED;
	//}
	
}


void txc_data() {
	if (SERCOM0->USART.INTFLAG.bit.RXC) {
		uint8_t command = SERCOM0->USART.DATA.reg;
		
		while(SERCOM0->USART.INTFLAG.bit.RXC) SERCOM0->USART.DATA.reg;
		
		
		switch (command) {
			case 0x80: // selftest data packet
				system_check();
			
				nav_selftest_packet.bit.crc = gen_crc32((uint32_t) &nav_selftest_packet.reg[0], sizeof(nav_selftest_packet.reg) - 4);
				
				for (uint16_t i = 0; i < sizeof(nav_selftest_packet.reg); ++i) {
					control_uart_send(nav_selftest_packet.reg[i]);
				}
				break;
				
			case 0x81: // standard data packet
				nav_data_packet.bit.crc = gen_crc32((uint32_t) &nav_data_packet.reg[0], sizeof(nav_data_packet.reg) - 4);
				
				for (uint16_t i = 0; i < sizeof(nav_data_packet.reg); ++i) {
					control_uart_send(nav_data_packet.reg[i]);
				}
				break;
			//case 0x82: // magnetometer calibrate command
				//MAG_Cal_Data mag_cal_data = mag_cal();
		}
		
	}
}
