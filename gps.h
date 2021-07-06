#ifndef GPS_H
#define GPS_H

#include "util.h"
#include "uart.h"


#define GPS_DEFAULT_BAUD 9600
#define GPS_BAUD 921600

#define UBX_HEADER_SIZE 2
#define UBX_CLASS_SIZE 1
#define UBX_ID_SIZE 1
#define UBX_LENGTH_SIZE 2
#define UBX_CHECKSUM_SIZE 2
#define UBX_PROTOCOL_SIZE (UBX_HEADER_SIZE+UBX_CLASS_SIZE+UBX_ID_SIZE+UBX_LENGTH_SIZE+UBX_CHECKSUM_SIZE)

typedef union {
	struct __attribute__((packed)) {
		uint16_t header;
		uint8_t class;
		uint8_t id;
		uint16_t length;
		
		uint32_t iTOW;
		uint16_t year;
		uint8_t month;
		uint8_t day;
		uint8_t hour;
		uint8_t min;
		uint8_t sec;
		uint8_t valid;
		uint32_t tAcc;
		int32_t nano;
		uint8_t fixType;
		uint8_t flags;
		uint8_t flags2;
		uint8_t numSV;
		int32_t lon;
		int32_t lat;
		int32_t height;
		int32_t hMSL;
		uint32_t hAcc;
		uint32_t vAcc;
		int32_t velN;
		int32_t velE;
		int32_t velD;
		int32_t gSpeed;
		int32_t headMot;
		uint32_t sAcc;
		uint32_t headAcc;
		uint16_t pDOP;
		uint8_t flags3;
		uint8_t reserved1;
		uint8_t reserved2;
		uint8_t reserved3;
		uint8_t reserved4;
		uint8_t reserved5;
		int32_t headVeh;
		int16_t magDec;
		uint16_t magAcc;
		
		uint8_t CK_A;
		uint8_t CK_B;
	} bit;

	uint8_t reg[100];
} UBX_NAV_PVT_Type;

volatile static UBX_NAV_PVT_Type ubx_nav_pvt;


void configure_gps();

void gps_cartesian(float latitude, float longitude, float* x, float* y);


//void gps_init_dma();
//
//void gps_dma_transfer();
//
//uint8_t gps_dma_check_complete();


#endif