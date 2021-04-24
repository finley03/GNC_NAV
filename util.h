#ifndef UTIL_H
#define UTIL_H

#define LED PORT_PA02

#include "samd21e18a.h"
#include "time.h"

void LED_print_8(uint8_t data);

void crc_init();

uint32_t gen_crc32(uint32_t data_addr, uint32_t data_size);

#endif