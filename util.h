#ifndef UTIL_H
#define UTIL_H

#include "samd21e18a.h"
#include "time.h"

#include <fastmath.h>

#define LED PORT_PA02

#define ABS(a) ((a < 0) ? -a : a)
#define MAX_2(a, b) ((a > b) ? a : b)
#define MAX_3(a, b, c) (MAX_2(MAX_2(a, b), c))
#define UMAX_2(a, b) MAX_2(ABS(a), ABS(b))
#define UMAX_3(a, b, c) MAX_3(ABS(a), ABS(b), ABS(c))

void LED_print_8(uint8_t data);

void crc_init();

uint32_t gen_crc32(uint32_t data_addr, uint32_t data_size);

#endif