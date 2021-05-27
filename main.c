#include "main.h"


void init();
void txc_data();


NAV_Selftest_Packet nav_selftest_packet;
NAV_Data_Packet nav_data_packet;


int main(void) {
	init();
	
	nav_selftest_packet.bit.device_id = DEVICE_ID;
	nav_data_packet.bit.device_id = DEVICE_ID;
	nav_selftest_packet.bit.imu_code = imu_check();
	nav_selftest_packet.bit.baro_code = baro_check();
	
	//REG_PORT_OUTSET0 = LED;
	
	
	gps_init_dma();
	
	// initialise timer for first iteration
	start_timer();
	
	
	// create position state variable
	Position_State position_state;
	position_state.bit.position_x = 0;
	position_state.bit.position_y = 0;
	position_state.bit.position_z = 0;
	position_state.bit.velocity_x = 0;
	position_state.bit.velocity_y = 0;
	position_state.bit.velocity_z = 0;
	
	
	// set initial estimate uncertainty;
	float estimate_uncertainty[36] = {
		1000, 0, 0, 0, 0, 0,
		0, 1000, 0, 0, 0, 0,
		0, 0, 1000, 0, 0, 0,
		0, 0, 0, 1000, 0, 0,
		0, 0, 0, 0, 1000, 0,
		0, 0, 0, 0, 0, 1000
	};
	
	// create measurement uncertainty variable
	float measurement_uncertainty[9];
	
	
	while(1) {
		delay_ms(20);
		
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
			
			
			// run kalman update step
			
			// update measurement uncertainty
			kalman_measurement_uncertainty(measurement_uncertainty, hAcc, vAcc);
			
			// get cartesian coordinates or current position
			float x, y;
			
			gps_cartesian(latitude, longitude, &x, &y);
			
			// create measurement type
			Position_Data position_data;
			position_data.bit.position_x = x;
			position_data.bit.position_y = y;
			position_data.bit.position_z = height;
			
			
			nav_data_packet.bit.pressure = kalman_update_position(&position_state, position_data, estimate_uncertainty, measurement_uncertainty);
			
			
		}
		
		
		IMU_Data imu_data = imu_get_data();
		nav_data_packet.bit.accel_x = imu_data.accel_x;
		nav_data_packet.bit.accel_y = imu_data.accel_y;
		nav_data_packet.bit.accel_z = imu_data.accel_z;
		
		//nav_data_packet.bit.angularvelocity_x = imu_data.gyro_x;
		//nav_data_packet.bit.angularvelocity_y = imu_data.gyro_y;
		//nav_data_packet.bit.angularvelocity_z = imu_data.gyro_z;
		
		nav_data_packet.bit.temperature = imu_data.temp;
		
		//nav_data_packet.bit.pressure = baro_get_data();
		//nav_data_packet.bit.pressure = estimate_uncertainty[0];
		
		
		//MAG_Data mag_data = mag_get_data();
		nav_data_packet.bit.angularvelocity_x = imu_data.gyro_x;
		nav_data_packet.bit.angularvelocity_y = imu_data.gyro_y;
		nav_data_packet.bit.angularvelocity_z = imu_data.gyro_z;
		
		
		// create accelerometer data type
		Accel_Data accel_data;
		accel_data.bit.accel_x = imu_data.accel_x;
		accel_data.bit.accel_y = imu_data.accel_y;
		accel_data.bit.accel_z = imu_data.accel_z - 9.81; // bad gravity correction just for testing
		
		if (nav_data_packet.bit.h_acc < 50) {
			// run kalman predict step
			kalman_predict_position(&position_state, accel_data, estimate_uncertainty);
		}
		
		nav_data_packet.bit.position_x = position_state.bit.position_x;
		nav_data_packet.bit.position_y = position_state.bit.position_y;
		nav_data_packet.bit.position_z = position_state.bit.position_z;
		nav_data_packet.bit.velocity_x = position_state.bit.velocity_x;
		nav_data_packet.bit.velocity_y = position_state.bit.velocity_y;
		nav_data_packet.bit.velocity_z = position_state.bit.velocity_z;
		
		
		
		txc_data();
	}
	
	return 0;
}



int system_check() {
	uint8_t state = 0;
	
	if (imu_check() != 0) state = 1;
	if (baro_check() != 0) state = 1;
	
	return state;
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
	REG_PORT_OUTSET0 = LED;
	
	

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
			case 0x80:
				nav_selftest_packet.bit.crc = gen_crc32((uint32_t) &nav_selftest_packet.reg[0], sizeof(nav_selftest_packet.reg) - 4);
				
				for (uint16_t i = 0; i < sizeof(nav_selftest_packet.reg); ++i) {
					control_uart_send(nav_selftest_packet.reg[i]);
				}
				break;
				
			case 0x81:
				nav_data_packet.bit.crc = gen_crc32((uint32_t) &nav_data_packet.reg[0], sizeof(nav_data_packet.reg) - 4);
				
				for (uint16_t i = 0; i < sizeof(nav_data_packet.reg); ++i) {
					control_uart_send(nav_data_packet.reg[i]);
				}
				break;
		}
		
	}
}
