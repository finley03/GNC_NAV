#include "dma.h"


void gps_dma_transfer() {
	// select DMAC channel 0
	DMAC->CHID.reg = 0;
	
	// clear previously buffered data
	while(SERCOM1->USART.INTFLAG.bit.RXC) SERCOM1->USART.DATA.reg;
	DMAC->CHINTFLAG.bit.TCMPL = 1;
	DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}


uint8_t gps_dma_check_complete() {
	// select DMAC channel 0
	DMAC->CHID.reg = 0;

	if (DMAC->CHINTFLAG.bit.TCMPL) {
		gps_dma_transfer();
		return 1;
	}
	else return 0;
}


void gps_init_dma() {
	// configure DMA descriptor
	// DMA descriptor is valid for use
	dma_descriptor[0].BTCTRL.bit.VALID = 1;
	
	// NOACT
	dma_descriptor[0].BTCTRL.bit.BLOCKACT = 0;
	
	// don't increment source address
	dma_descriptor[0].BTCTRL.bit.SRCINC = 0;
	
	// increment destination address
	dma_descriptor[0].BTCTRL.bit.DSTINC = 1;
	
	// step size selection applies to destination
	dma_descriptor[0].BTCTRL.bit.STEPSEL = 0;
	
	// set step size of sizeof beat
	dma_descriptor[0].BTCTRL.bit.STEPSIZE = 0;
	
	// set beat size to one byte
	dma_descriptor[0].BTCTRL.bit.BEATSIZE = 0;
	
	
	dma_descriptor[0].BTCNT = sizeof(ubx_nav_pvt.reg);
	dma_descriptor[0].DSTADDR = ((uint32_t) &(ubx_nav_pvt.reg[0]) + sizeof(ubx_nav_pvt.reg));
	dma_descriptor[0].SRCADDR = (uint32_t) &(SERCOM1->USART.DATA.reg);
	
	
	
	// tell DMAC where descriptors are
	DMAC->BASEADDR.reg = (uint32_t) &(dma_descriptor[0]);
	DMAC->WRBADDR.reg = (uint32_t) &(dma_descriptor_writeback[0]);
	
	
	
	// enable DMA bus clocks
	PM->AHBMASK.bit.DMAC_ = 1;
	PM->APBBMASK.bit.DMAC_ = 1;
	
	
	// enable DMAC
	DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
	
	
	// select DMAC channel 0
	DMAC->CHID.reg = 0;
	
	// set GPS RX DMA to higher priority
	DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0x1) | DMAC_CHCTRLB_TRIGSRC(SERCOM1_DMAC_ID_RX) | DMAC_CHCTRLB_TRIGACT_BEAT;

	
	gps_dma_transfer();
}
