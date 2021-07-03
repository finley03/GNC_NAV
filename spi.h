#ifndef SPI_H
#define SPI_H

#include "util.h"

#define SPI_IMU_SS PORT_PA25
#define SPI_BARO_SS PORT_PA27


// SERCOM3
// PA22 - PA25

void spi_init();

uint8_t spi_command(uint8_t opcode);

#endif