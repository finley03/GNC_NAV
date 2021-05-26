#include "gps.h"


#include <math.h>


void gps_tx_msg(const uint8_t msg[], uint16_t msg_size) {
	for (uint16_t i = 0; i < msg_size; ++i) {
		gps_uart_send(msg[i]);
	}
}



//typedef struct __attribute__((aligned(16))) {
	//union {
		//struct {
			//uint16_t VALID:1;
			//uint16_t EVOSEL:2;
			//uint16_t BLOCKACT:2;
			//uint16_t :3;
			//uint16_t BEATSIZE:2;
			//uint16_t SRCINC:1;
			//uint16_t DSTINC:1;
			//uint16_t STEPSEL:1;
			//uint16_t STEPSIZE:3;
		//} bit;
		//
		//uint16_t reg;
	//} BTCTRL;
	//
	//uint16_t BTCNT;
	//uint32_t SRCADDR;
	//uint32_t DSTADDR;
	//uint32_t DESCADDR;
//} DMA_DESCRIPTOR_Type;
//
//volatile DMA_DESCRIPTOR_Type dma_descriptor[1];
//DMA_DESCRIPTOR_Type dma_descriptor_writeback[1];
//
//
//void gps_init_dma() {
	//// configure DMA descriptor
	//// DMA descriptor is valid for use
	//dma_descriptor[0].BTCTRL.bit.VALID = 1;
	//
	//// NOACT
	//dma_descriptor[0].BTCTRL.bit.BLOCKACT = 0;
	//
	//// don't increment source address
	//dma_descriptor[0].BTCTRL.bit.SRCINC = 0;
	//
	//// increment destination address
	//dma_descriptor[0].BTCTRL.bit.DSTINC = 1;
	//
	//// step size selection applies to destination
	//dma_descriptor[0].BTCTRL.bit.STEPSEL = 0;
	//
	//// set step size of sizeof beat
	//dma_descriptor[0].BTCTRL.bit.STEPSIZE = 0;
	//
	//// set beat size to one byte
	//dma_descriptor[0].BTCTRL.bit.BEATSIZE = 0;
	//
	//
	//dma_descriptor[0].BTCNT = sizeof(ubx_nav_pvt.reg);
	//dma_descriptor[0].DSTADDR = ((uint32_t) &(ubx_nav_pvt.reg[0]) + sizeof(ubx_nav_pvt.reg));
	//dma_descriptor[0].SRCADDR = (uint32_t) &(SERCOM1->USART.DATA.reg);
	//
	//
	//
	//// tell DMAC where descriptors are
	//DMAC->BASEADDR.reg = (uint32_t) &(dma_descriptor[0]);
	//DMAC->WRBADDR.reg = (uint32_t) &(dma_descriptor_writeback[0]);
	//
	//
	//
	//// enable DMA bus clocks
	//PM->AHBMASK.bit.DMAC_ = 1;
	//PM->APBBMASK.bit.DMAC_ = 1;
	//
	//
	//// enable DMAC
	//DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
	//
	//
	//// select DMAC channel 0
	//DMAC->CHID.reg = 0;
	//
	//
	////DMAC_CHCTRLB_Type dmac_chctrlb = {
	////// standard priority
	////.bit.LVL = 0,
	////
	////// set trigger action to beat
	////.bit.TRIGACT = 2,
	////
	////// set trigger source to SERCOM2 TX
	////.bit.TRIGSRC = 0x06
	////};
	////
	////
	////DMAC->CHCTRLB.reg = dmac_chctrlb.reg;
	//
	//DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0) | DMAC_CHCTRLB_TRIGSRC(SERCOM1_DMAC_ID_RX) | DMAC_CHCTRLB_TRIGACT_BEAT;
	//
	//
	////DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
	//
//
//}
//
//
//void gps_dma_transfer() {
	//// clear previously buffered data
	//while(SERCOM1->USART.INTFLAG.bit.RXC) SERCOM1->USART.DATA.reg;
	//DMAC->CHINTFLAG.bit.TCMPL = 1;
	//DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
//}
//
//
//uint8_t gps_dma_check_complete() {
	//if (DMAC->CHINTFLAG.bit.TCMPL) {
		//gps_dma_transfer();
		//return 1;
	//}
	//else return 0;
//}



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





void configure_gps() {
	gps_tx_msg(UBX_CFG_PRT_setbaud, sizeof(UBX_CFG_PRT_setbaud));
	
	
	delay_ms(100);
	
	
	// update baud rate
	// disable USART
	SERCOM1->USART.CTRLA.bit.ENABLE = 0;
	
	// wait for synchronisation
	while(SERCOM1->USART.SYNCBUSY.bit.ENABLE);
	
	// change baud rate to 921600
	SERCOM1->USART.BAUD.reg = 45403; // 45403
	
	// re-enable USART
	SERCOM1->USART.CTRLA.bit.ENABLE = 1;
	
	// wait for synchronisation
	while(SERCOM1->USART.SYNCBUSY.bit.ENABLE);
	
	
	delay_ms(100);
	
	
	
	
	gps_tx_msg(UBX_CFG_MSG_disableGGA, sizeof(UBX_CFG_MSG_disableGGA));
	gps_tx_msg(UBX_CFG_MSG_disableGLL, sizeof(UBX_CFG_MSG_disableGLL));
	gps_tx_msg(UBX_CFG_MSG_disableGSA, sizeof(UBX_CFG_MSG_disableGSA));
	gps_tx_msg(UBX_CFG_MSG_disableGSV, sizeof(UBX_CFG_MSG_disableGSV));
	gps_tx_msg(UBX_CFG_MSG_disableRMC, sizeof(UBX_CFG_MSG_disableRMC));
	gps_tx_msg(UBX_CFG_MSG_disableVTG, sizeof(UBX_CFG_MSG_disableVTG));
	gps_tx_msg(UBX_CFG_MSG_enableUBX_NAV_PVT, sizeof(UBX_CFG_MSG_enableUBX_NAV_PVT));
	
	
	
}



void gps_cartesian(float latitude, float longitude, float* x, float* y) {
	float multiplier = 111194.9266;
	float zero_lat = 50.000000;
	float zero_long = 0.000000;
	float pi_180 = 0.01745329;
	
	*y = (latitude - zero_lat) * multiplier;
	*x = (longitude - zero_long) * multiplier * cos(latitude * pi_180);
}