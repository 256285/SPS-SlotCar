/*
 * UART.h
 *
 *
 *  Created on: 28. 9. 2021
 *      Author: Ondrej Matula
 */
#ifndef UART_H_
#define UART_H_

#include "stdint.h"
#include <msp430.h>

void UART_init(void);     //initialization and initiation communication -- default baud rate is 19600
void UART_send_byte(uint8_t b);         // pošle jeden bajt (čeká na volný buffer)
void UART_send_string(const char *s);   // pošle řetězec zakončený nulou
void UART_send_int(int32_t v);          // pošle číslo jako text (dekadicky, se znaménkem)

#endif /* UART_H_ */
