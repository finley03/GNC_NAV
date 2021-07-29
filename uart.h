#ifndef UART_H
#define UART_H

#include "util.h"

// uart to main processor;
void control_uart_init();
void control_uart_flush();
void control_uart_read(uint8_t* addr, uint32_t n);
void control_uart_send(uint8_t data);
void control_uart_stream(uint8_t* addr, uint32_t nr_bytes);

void gps_uart_init();
void gps_uart_send(uint8_t data);

#endif