#ifndef UART_H
#define UART_H

#include "util.h"

// uart to main processor;
void control_uart_init();

void control_uart_send(uint8_t data);

void gps_uart_init();

void gps_uart_send(uint8_t data);

#endif