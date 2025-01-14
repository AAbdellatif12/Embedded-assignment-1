// uart.h
#ifndef UART_H
#define UART_H

#include <avr/io.h>

// UART Definitions
#define BAUD 9600
#define BRC ((F_CPU/16/BAUD) - 1)

// Function Prototypes
void UART_init(void);
void UART_transmit(unsigned char data);
unsigned char UART_receive(void);
void UART_sendString(const char* str);

#endif
