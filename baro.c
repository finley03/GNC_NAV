#include "baro.h"


// takes delay in microseconds
void baro_send_command(uint8_t command, uint16_t delay) {
	// SS low
	REG_PORT_OUTCLR0 = SPI_BARO_SS;
	// send command
	spi_command(command);
	// wait with SS low
	delay_us(delay);
	// pull SS high
	REG_PORT_OUTSET0 = SPI_BARO_SS;
}


uint32_t baro_adc_read() {
	REG_PORT_OUTCLR0 = SPI_BARO_SS;
	// send command ADC READ
	spi_command(BARO_ADC_READ);
	// receive bytes
	uint8_t data0 = spi_command(0);
	uint8_t data1 = spi_command(0);
	uint8_t data2 = spi_command(0);
		
	REG_PORT_OUTSET0 = SPI_BARO_SS;
		
	uint32_t data = ((uint32_t) data0 << 16) | ((uint32_t) data1 << 8) | ((uint32_t) data2);
	return data;
}


uint32_t baro_get_data(uint8_t command, uint16_t delay) {
	baro_send_command(command, delay);
	return baro_adc_read();
}


uint16_t baro_read_16(uint8_t address) {
	REG_PORT_OUTCLR0 = SPI_BARO_SS;
	// send command ADC READ
	spi_command(address);
	// receive bytes
	uint8_t data0 = spi_command(0);
	uint8_t data1 = spi_command(0);
		
	REG_PORT_OUTSET0 = SPI_BARO_SS;
		
	uint16_t data = ((uint16_t) data0 << 8) | ((uint16_t) data1);
	return data;
}


void baro_prom_read() {
	baro_prom.sens = baro_read_16(BARO_PROM_SENS);
	baro_prom.off = baro_read_16(BARO_PROM_OFF);
	baro_prom.tcs = baro_read_16(BARO_PROM_TCS);
	baro_prom.tco = baro_read_16(BARO_PROM_TCO);
	baro_prom.tref = baro_read_16(BARO_PROM_TREF);
	baro_prom.tempsens = baro_read_16(BARO_PROM_TEMPSENS);
}


void baro_init() {
	// reset barometer
	baro_send_command(BARO_RESET, 3000);

	// get prom calibration data
	baro_prom_read();
}


uint8_t baro_check() {
	// convert D2 OSR=256 (temperature, lowest resolution)
	uint32_t data = baro_get_data(BARO_D2_256, 1000);
	//baro_send_command(BARO_D2_256, 1000);
	//uint32_t data = baro_adc_read();
	
	//REG_PORT_OUTCLR0 = SPI_BARO_SS;
	//// send command
	//spi_command(0x50);
	//// wait for conversion to finish (0.6ms)
	//delay_ms(1);
	//// set SS high
	//REG_PORT_OUTSET0 = SPI_BARO_SS;
		//
	//// wait while SS up
	//delay_ms(1);
	//
	//// adc read
		//
	//REG_PORT_OUTCLR0 = SPI_BARO_SS;
	//// send command ADC READ
	//spi_command(0x00);
		//
	//// receive bytes
	//uint8_t data0 = spi_command(0);
	//uint8_t data1 = spi_command(0);
	//uint8_t data2 = spi_command(0);
		//
	//REG_PORT_OUTSET0 = SPI_BARO_SS;
	//
	//
	//uint32_t data = ((uint32_t) data0 << 16) | ((uint32_t) data1 << 8) | (uint32_t) data2;
	
	// check data is not min or max value
	if (data != 0 && data != 0x00ffffff) return 0;
	else return 1;
}


float baro_get_pressure(float* temperature) {
	// perform all necessary data conversion
	// variable names are modeled after datasheet documentation
	// which is why they're 'non-descript'
	
	
	// get raw digital pressure value
	// 2.5ms used as delay value
	uint32_t D1 = baro_get_data(BARO_D1_1024, 2500);
	// get raw digital temperature value
	uint32_t D2 = baro_get_data(BARO_D2_1024, 2500);
	
	
	// get difference between actual and reference temperature
	int32_t dT = D2 - ((int32_t) baro_prom.tref * 256);
	// get actual temperature
	// units are hundredths of a degree
	// 8388608 = 2 ^ 23
	int32_t TEMP = 2000 + (dT * baro_prom.tempsens / 8388608);
	
	
	// offset at actual temperature
	// 131072 = 2 ^ 17
	int64_t OFF = ((int64_t) baro_prom.off * 131072) + (((int64_t) baro_prom.tco * (int64_t) dT) / 64);
	// sensitivity at actual temperature
	int64_t SENS = ((int64_t) baro_prom.sens * 65536) + (((int64_t) baro_prom.tcs * (int64_t) dT) / 128);
	// temperature compensated pressure
	// units are hundredths of mbar
	// 2097152 = 2 ^ 21
	int32_t P = ((D1 * SENS / 2097152) - OFF) / 32768;
	
	*temperature = ((float) TEMP) * 0.01;
	return ((float) P) * 0.01;
}