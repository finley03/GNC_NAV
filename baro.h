#ifndef BARO_H
#define BARO_H

#include "samd21e18a.h"
#include "time.h"
#include "spi.h"
#include "util.h"

#include <math.h>

// resolution conversion time:
// 256: 0.6ms
// 512: 1.17ms
// 1024: 2.28ms
// 2048: 4.54ms
// 4096: 9.04ms

#define BARO_RESET 0x1e
// d1 is pressure
#define BARO_D1_256 0x40
#define BARO_D1_512 0x42
#define BARO_D1_1024 0x44
#define BARO_D1_2048 0x46
#define BARO_D1_4096 0x48
// d1 is temperature
#define BARO_D2_256 0x50
#define BARO_D2_512 0x52
#define BARO_D2_1024 0x54
#define BARO_D2_2048 0x56
#define BARO_D2_4096 0x58

#define BARO_ADC_READ 0x00

// prom addresses
#define BARO_PROM_SETUP 0xa0
#define BARO_PROM_SENS 0xa2
#define BARO_PROM_OFF 0xa4
#define BARO_PROM_TCS 0xa6
#define BARO_PROM_TCO 0xa8
#define BARO_PROM_TREF 0xaa
#define BARO_PROM_TEMPSENS 0xac
#define BARO_PROM_CRC 0xae


typedef struct {
	uint16_t sens;
	uint16_t off;
	uint16_t tcs;
	uint16_t tco;
	uint16_t tref;
	uint16_t tempsens;
} Baro_Prom;

Baro_Prom baro_prom;


// barometer must be initialised to load
// PROM cal data to sensor
void baro_init();

uint8_t baro_check();

//float baro_get_data();
float baro_get_pressure(float* temperature);

float get_pressure_altitude(float* pressure, float* temperature);

#endif