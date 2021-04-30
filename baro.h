#ifndef BARO_H
#define BARO_H

#include "samd21e18a.h"
#include "time.h"
#include "spi.h"

// barometer must be initialised to load
// PROM cal data to sensor
void baro_init();

uint8_t baro_check();

float baro_get_data();

#endif