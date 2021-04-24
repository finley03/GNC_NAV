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
	
	REG_PORT_OUTSET0 = LED;
	
	
	gps_init_dma();
	
	while(1) {
		delay_ms(2);
		
		if (gps_dma_check_complete()) {
			float latitude = ubx_nav_pvt.bit.lat * 1E-7;
			float longitude = ubx_nav_pvt.bit.lon * 1E-7;
			float height = ubx_nav_pvt.bit.height * 1E-3;
			//float speed = ubx_nav_pvt.bit.gSpeed * 1E-3;
			float hAcc = ubx_nav_pvt.bit.hAcc * 1E-3;
			float vAcc = ubx_nav_pvt.bit.vAcc * 1E-3;
			
			
			nav_data_packet.bit.latitude = latitude;
			nav_data_packet.bit.longitude = longitude;
			nav_data_packet.bit.gps_height = height;
			nav_data_packet.bit.h_acc = hAcc;
			nav_data_packet.bit.v_acc = vAcc;
			nav_data_packet.bit.gps_satellites = ubx_nav_pvt.bit.numSV;
		}
		
		
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
	
	

	delay_ms(100);
	
	
	if (system_check() != 0) {
		REG_PORT_OUTSET0 = LED;
		delay_ms(1000);
		REG_PORT_OUTCLR0 = LED;
	}
	
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
