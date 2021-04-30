#include "baro.h"


void baro_init() {
	// reset barometer
		
	// set SS low
	REG_PORT_OUTCLR0 = SPI_BARO_SS;
	// send command Reset
	spi_command(0x1e);
	// wait for reset to finish (2.8ms)
	delay_ms(3);
	// set SS high
	REG_PORT_OUTSET0 = SPI_BARO_SS;
}


uint8_t baro_check() {
	// convert D2 OSR=256 (temperature, lowest resolution)
	REG_PORT_OUTCLR0 = SPI_BARO_SS;
	// send command
	spi_command(0x50);
	// wait for conversion to finish (0.6ms)
	delay_ms(1);
	// set SS high
	REG_PORT_OUTSET0 = SPI_BARO_SS;
		
	// wait while SS up
	delay_ms(1);
	
	// adc read
		
	REG_PORT_OUTCLR0 = SPI_BARO_SS;
	// send command ADC READ
	spi_command(0x00);
		
	// receive bytes
	uint8_t data0 = spi_command(0);
	uint8_t data1 = spi_command(0);
	uint8_t data2 = spi_command(0);
		
	REG_PORT_OUTSET0 = SPI_BARO_SS;
	
	
	uint32_t data = ((uint32_t) data0 << 16) | ((uint32_t) data1 << 8) | (uint32_t) data2;
	
	// check data is not min or max value
	if (data != 0 && data != 0x00ffffff) return 0;
	else return 1;
}


float baro_get_data() {
	// convert D1 OSR=1024 (pressure, resolution 3)
	
	// SS low
	REG_PORT_OUTCLR0 = SPI_BARO_SS;
	// send command
	spi_command(0x44);
	// wait for conversion to finish
	delay_ms(3);
	// SS high
	REG_PORT_OUTSET0 = SPI_BARO_SS;
	
	// adc read
	delay_ms(1);
	
	REG_PORT_OUTCLR0 = SPI_BARO_SS;
	// send command ADC READ
	spi_command(0x00);
	// receive bytes
	uint8_t data0 = spi_command(0);
	uint8_t data1 = spi_command(0);
	uint8_t data2 = spi_command(0);
	
	REG_PORT_OUTSET0 = SPI_BARO_SS;
	
	
	uint32_t data = ((uint32_t) data0 << 16) | ((uint32_t) data1 << 8) | (uint32_t) data2;
	
	return (float) data;
}