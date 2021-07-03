#include "util.h"


#include "mat.h"


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


uint32_t gen_crc32(uint32_t data_addr, uint32_t data_size) {
	*((volatile unsigned int*) 0x41007058) &= ~0x30000UL;
	
	// fill data register with starting value
	//DSU->DATA.reg = 0xffffffff;
	DSU->DATA.reg = 0xffffffff;
	
	DSU->ADDR.reg = data_addr;
	DSU->LENGTH.reg = DSU_LENGTH_LENGTH((data_size) / 4);
	
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


void correct_value(float* value, float* A, float* b, float* writeback) {
	float centered[3];
	centered[0] = value[0] - b[0];
	centered[1] = value[1] - b[1];
	centered[2] = value[2] - b[2];
	
	mat_multiply(A, 3, 3, centered, 3, 1, writeback);
}