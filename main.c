#include "main.h"


#include "spi.h"
#include "uart.h"
#include "imu.h"
#include "baro.h"
#include "gps.h"
#include "dma.h"
#include "kalman.h"


void init();
void txc_data();
void system_check();


NAV_Selftest_Packet nav_selftest_packet;
NAV_Data_Packet nav_data_packet;
NAV_ACK_Packet nav_ack_packet;
//float kalman_run;
bool kalman_run;


int main(void) {
	init();
	
	nav_data_packet.bit.device_id = DEVICE_ID;
	
	//static float A[9];
	//static float b[3];
	//mag_cal(A, b);
	//mag_cal();
	//mag_cal();
	//MAG_Cal_Data mag_cal_data = mag_cal();
	
	gps_init_dma();
	
	// initialise timer for first iteration
	start_timer();
	
	
	static MAG_Data mag_data;
	static Accel_Data accel_data;
	
	
	// create position state variable
	static Position_State position_state;
	position_state.bit.position_x = 0;
	position_state.bit.position_y = 0;
	position_state.bit.position_z = 0;
	position_state.bit.velocity_x = 0;
	position_state.bit.velocity_y = 0;
	position_state.bit.velocity_z = 0;
	
	
	//// set initial position estimate uncertainty;
	//static float position_estimate_uncertainty[36] = {
		//1000, 0, 0, 0, 0, 0,
		//0, 1000, 0, 0, 0, 0,
		//0, 0, 1000, 0, 0, 0,
		//0, 0, 0, 1000, 0, 0,
		//0, 0, 0, 0, 1000, 0,
		//0, 0, 0, 0, 0, 1000
	//};
	
	// create position measurement uncertainty variable
	static float position_measurement_uncertainty[9];
	
	
	
	// create orientation state variable
	static Orientation_State orientation_state;
	orientation_state.bit.orientation_x = 0;
	orientation_state.bit.orientation_y = 0;
	orientation_state.bit.orientation_z = 0;
	
	
	//// set initial orientation estimate uncertainty;
	//static float orientation_estimate_uncertainty[9] = {
		//8000, 0, 0,
		//0, 8000, 0,
		//0, 0, 8000
	//};
	
	//// set orientation measurement uncertainty to 20 degrees
	//static float orientation_measurement_uncertainty[9] = {
		//400, 0, 0,
		//0, 400, 0,
		//0, 0, 400
	//};
	
	// accelerometer magnetometer orientation estimate
	static Orientation_State accel_mag_orientation;
	
	//float accelerometer_bias[3] = { 0, 0, 0 };
	
	
	while(1) {
		delay_ms(2);
		
		LED_ON();
		// data collection and predict moved to top to give kalman filter more accurate data
		IMU_Data imu_data = imu_get_data();
		LED_OFF();
				
				
		mag_data = mag_get_data();
		//MAG_Data mag_data;
		//mag_data.mag_x = (mag_data_uncalibrated.mag_x - mag_cal_data.bias_x) * mag_cal_data.scale_x;
		//mag_data.mag_y = (mag_data_uncalibrated.mag_y - mag_cal_data.bias_y) * mag_cal_data.scale_y;
		//mag_data.mag_z = (mag_data_uncalibrated.mag_z - mag_cal_data.bias_z) * mag_cal_data.scale_z;
		// shift values based on calculated correction
		//correct_value(mag_data_uncalibrated.reg, A, b, mag_data.reg);
				
		// create accelerometer data type
		accel_data.bit.accel_x = imu_data.accel_x;
		accel_data.bit.accel_y = imu_data.accel_y;
		accel_data.bit.accel_z = imu_data.accel_z;
				
		Gyro_Data gyro_data;
		gyro_data.bit.rotation_x = imu_data.gyro_x;
		gyro_data.bit.rotation_y = imu_data.gyro_y;
		gyro_data.bit.rotation_z = imu_data.gyro_z;
				
				
		Accel_Data transformed_accel;
				
		if (kalman_run) {
			if (nav_data_packet.bit.h_acc < 50) {
				// run kalman predict step
				transformed_accel = kalman_predict_position(&position_state, accel_data, orientation_state);
				kalman_predict_orientation(&orientation_state, gyro_data);
			}
		
		
		
			if (gps_dma_check_complete()) {
				float latitude = ubx_nav_pvt.bit.lat * 1E-7;
				float longitude = ubx_nav_pvt.bit.lon * 1E-7;
				float height = ubx_nav_pvt.bit.height * 1E-3;
				float hAcc = ubx_nav_pvt.bit.hAcc * 1E-3;
				float vAcc = ubx_nav_pvt.bit.vAcc * 1E-3;
			
				//float height = get_pressure_altitude(&nav_data_packet.bit.pressure, &nav_data_packet.bit.baro_temperature);
			
			
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
			
			
				kalman_update_position(&position_state, position_data, orientation_state, position_measurement_uncertainty);
			
			
			
				// run orientation kalman update step
			
			
				accel_mag_orientation = kalman_orientation_generate_state(mag_data, accel_data);
			
				kalman_update_orientation(&orientation_state, accel_mag_orientation);
			
			
				// get barometer pressure readings
				nav_data_packet.bit.pressure = baro_get_pressure(&nav_data_packet.bit.baro_temperature);
			
			}
		}
		

		
		
		// fill in data in data packet
		
		
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
		
		nav_data_packet.bit.mag_x = mag_data.bit.mag_x;
		nav_data_packet.bit.mag_y = mag_data.bit.mag_y;
		nav_data_packet.bit.mag_z = mag_data.bit.mag_z;
		
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

	nav_ack_packet.bit.device_id = DEVICE_ID;
	kalman_init();	
	kalman_run = false;
}


void txc_data() {
	if (SERCOM0->USART.INTFLAG.bit.RXC) {
		uint8_t command = SERCOM0->USART.DATA.reg;
		
		while(SERCOM0->USART.INTFLAG.bit.RXC) SERCOM0->USART.DATA.reg;
		
		
		switch (command) {
			// selftest data packet
			case 0x80:
			{
				system_check();
			
				nav_selftest_packet.bit.crc = crc32(nav_selftest_packet.reg, sizeof(nav_selftest_packet.reg) - 4);
				
				for (uint16_t i = 0; i < sizeof(nav_selftest_packet.reg); ++i) {
					control_uart_send(nav_selftest_packet.reg[i]);
				}
			}
			break;
				
			// standard data packet
			case 0x81:
			{
				nav_data_packet.bit.crc = crc32(nav_data_packet.reg, sizeof(nav_data_packet.reg) - 4);
				
				for (uint16_t i = 0; i < sizeof(nav_data_packet.reg); ++i) {
					control_uart_send(nav_data_packet.reg[i]);
				}
			}
			break;
				
			// set vec3 parameter
			case 0x82:
			{
				Set_Vec3_Request set_request;
				// send ack packet ok
				nav_ack_packet.bit.status_code = NAV_ACK_OK;
				nav_ack_packet.bit.crc = crc32(nav_ack_packet.reg, sizeof(nav_ack_packet.reg) - 4);
				control_uart_stream(nav_ack_packet.reg, sizeof(nav_ack_packet.reg));
				// wait for return packet
				control_uart_read(set_request.reg, sizeof(set_request.reg));
				// check packet is valid
				if (crc32(set_request.reg, sizeof(set_request.reg)) == CRC32_CHECK &&
					set_request.bit.header == NAV_SET_VEC3_REQUEST_HEADER) {
					nav_set_value((NAV_Param) set_request.bit.parameter, set_request.bit.data);
				}
				else {
					LED_ON();
					while(1);
				}
			}
			break;
				
			// read vec3 parameter
			case 0x83:
			{
				Read_Vec3_Request read_request;
				// send ack packet ok
				nav_ack_packet.bit.status_code = NAV_ACK_OK;
				nav_ack_packet.bit.crc = crc32(nav_ack_packet.reg, sizeof(nav_ack_packet.reg) - 4);
				control_uart_stream(nav_ack_packet.reg, sizeof(nav_ack_packet.reg));
				// wait for return packet
				control_uart_read(read_request.reg, sizeof(read_request.reg));
				// check packet is valid
				if (crc32(read_request.reg, sizeof(read_request.reg)) == CRC32_CHECK &&
				read_request.bit.header == NAV_READ_VEC3_REQUEST_HEADER) {
					Read_Vec3_Response read_packet;
					read_packet.bit.device_id = DEVICE_ID;
					nav_read_value((NAV_Param) read_request.bit.parameter, read_packet.bit.data);
					read_packet.bit.crc = crc32(read_packet.reg, sizeof(read_packet.reg) - 4);
					control_uart_stream(read_packet.reg, sizeof(read_packet.reg));
				}
				else {
					LED_ON();
					while(1);
				}
			}
			break;
			
			// set scalar parameter
			case 0x84:
			{
				Set_Scalar_Request set_request;
				// send ack packet ok
				nav_ack_packet.bit.status_code = NAV_ACK_OK;
				nav_ack_packet.bit.crc = crc32(nav_ack_packet.reg, sizeof(nav_ack_packet.reg) - 4);
				control_uart_stream(nav_ack_packet.reg, sizeof(nav_ack_packet.reg));
				// wait for return packet
				control_uart_read(set_request.reg, sizeof(set_request.reg));
				// check packet is valid
				if (crc32(set_request.reg, sizeof(set_request.reg)) == CRC32_CHECK &&
				set_request.bit.header == NAV_SET_SCALAR_REQUEST_HEADER) {
					nav_set_value((NAV_Param) set_request.bit.parameter, &set_request.bit.data);
				}
				else {
					LED_ON();
					while(1);
				}
			}
			break;
			
			// read scalar parameter
			case 0x85:
			{
				Read_Scalar_Request read_request;
				// send ack packet ok
				nav_ack_packet.bit.status_code = NAV_ACK_OK;
				nav_ack_packet.bit.crc = crc32(nav_ack_packet.reg, sizeof(nav_ack_packet.reg) - 4);
				control_uart_stream(nav_ack_packet.reg, sizeof(nav_ack_packet.reg));
				// wait for return packet
				control_uart_read(read_request.reg, sizeof(read_request.reg));
				// check packet is valid
				if (crc32(read_request.reg, sizeof(read_request.reg)) == CRC32_CHECK &&
				read_request.bit.header == NAV_READ_SCALAR_REQUEST_HEADER) {
					Read_Scalar_Response read_packet;
					read_packet.bit.device_id = DEVICE_ID;
					nav_read_value((NAV_Param) read_request.bit.parameter, &read_packet.bit.data);
					read_packet.bit.crc = crc32(read_packet.reg, sizeof(read_packet.reg) - 4);
					control_uart_stream(read_packet.reg, sizeof(read_packet.reg));
				}
				else {
					LED_ON();
					while(1);
				}
			}
			break;
			
			// calibrate magnetometer
			case 0x86:
			mag_cal();
			break;
			
			// enable kalman filter
			case 0x87:
			kalman_run = true;
			break;
			
			// disable kalman filter
			case 0x88:
			kalman_run = false;
			break;
			
			// reset computer
			case 0xFF:
			NVIC_SystemReset();
			break;
		}
		
	}
}
