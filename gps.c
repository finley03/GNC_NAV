#include "gps.h"


#include <math.h>


void gps_tx_msg(const uint8_t msg[], uint16_t msg_size) {
	for (uint16_t i = 0; i < msg_size; ++i) {
		gps_uart_send(msg[i]);
	}
}

// Set Baud to 921600

const uint8_t UBX_CFG_PRT_setbaud[] = {
	0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00,
	0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x10,
	0x0E, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x1B, 0x5A
};

// Disable NMEA GGA

const uint8_t UBX_CFG_MSG_disableGGA[] = {
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23
};

// Disable NMEA GLL

const uint8_t UBX_CFG_MSG_disableGLL[] = {
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A
};

// Disable NMEA GSA

const uint8_t UBX_CFG_MSG_disableGSA[] = {
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31
};

// Disable NMEA GSV

const uint8_t UBX_CFG_MSG_disableGSV[] = {
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38
};

// Disable NEMA RMC

const uint8_t UBX_CFG_MSG_disableRMC[] = {
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F
};

// Disable NMEA VTG

const uint8_t UBX_CFG_MSG_disableVTG[] = {
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46
};

// Enable UBX_NAV_PVT

const uint8_t UBX_CFG_MSG_enableUBX_NAV_PVT[] = {
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07,
	0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x1B, 0xEC
};

// set rate to 5hz

const uint8_t UBX_CFG_MSG_set_rate_5Hz[] = {
	0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00,
	0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A
};

// set rate to 14.94hz

const uint8_t UBX_CFG_MSG_set_rate_15Hz[] = {
	0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x43, 0x00,
	0x01, 0x00, 0x01, 0x00, 0x59, 0x4C
};





void configure_gps() {
	gps_tx_msg(UBX_CFG_PRT_setbaud, sizeof(UBX_CFG_PRT_setbaud));
	
	delay_ms(50);
	
	
	// update baud rate
	// disable USART
	SERCOM1->USART.CTRLA.bit.ENABLE = 0;
	
	// wait for synchronization
	while(SERCOM1->USART.SYNCBUSY.bit.ENABLE);
	
	// change baud rate to 921600
	SERCOM1->USART.BAUD.reg = 45403; // 45403
	
	// re-enable USART
	SERCOM1->USART.CTRLA.bit.ENABLE = 1;
	
	// wait for synchronization
	while(SERCOM1->USART.SYNCBUSY.bit.ENABLE);
	
	
	gps_tx_msg(UBX_CFG_MSG_disableGGA, sizeof(UBX_CFG_MSG_disableGGA));
	gps_tx_msg(UBX_CFG_MSG_disableGLL, sizeof(UBX_CFG_MSG_disableGLL));
	gps_tx_msg(UBX_CFG_MSG_disableGSA, sizeof(UBX_CFG_MSG_disableGSA));
	gps_tx_msg(UBX_CFG_MSG_disableGSV, sizeof(UBX_CFG_MSG_disableGSV));
	gps_tx_msg(UBX_CFG_MSG_disableRMC, sizeof(UBX_CFG_MSG_disableRMC));
	gps_tx_msg(UBX_CFG_MSG_disableVTG, sizeof(UBX_CFG_MSG_disableVTG));
	gps_tx_msg(UBX_CFG_MSG_enableUBX_NAV_PVT, sizeof(UBX_CFG_MSG_enableUBX_NAV_PVT));
	gps_tx_msg(UBX_CFG_MSG_set_rate_15Hz, sizeof(UBX_CFG_MSG_set_rate_15Hz));
}



void gps_cartesian(float latitude, float longitude, float* x, float* y) {
	float multiplier = 111194.9266;
	float zero_lat = 50.000000;
	float zero_long = 0.000000;
	float pi_180 = 0.01745329;
	
	*x = (latitude - zero_lat) * multiplier;
	*y = (longitude - zero_long) * multiplier * cos(latitude * pi_180);
}