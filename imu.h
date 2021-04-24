#ifndef IMU_H
#define IMU_H

#include "samd21e18a.h"
#include "time.h"
#include "spi.h"

#define IMU_WRITE_MASK 0x7f
#define IMU_READ_MASK 0x80


// IMU must be initialised to prevent
// switching to i2c mode
void imu_init();


// checks IMU by checking returned identifier
// returns 0 if pass
uint8_t imu_check();


#endif