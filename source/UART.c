/*
 * UART.c
 *
 *  Created on: 28. 9. 2021
 *      Author: Ondra
 */


#include "include/UART.h"

void UART_init(void)
{
    UCA1CTL1 = UCSWRST; //UCA1 softwarovy reset -> ON

    P5SEL |= BIT7;
    P5SEL |= BIT6;

    P5OUT &= ~BIT7;
    P5OUT &= ~BIT6;

    UCA1CTL1 |= UCSSEL__SMCLK; //nastaveni zdrojoveho frekvence SMCLK

   // UCA1BR0 = 0x34;           //16MHz / 19200 baud
   //UCA1BR1 = 0x00;
   //UCA1MCTL = 0x11;          //UCOS16 = 1 / UCBRFx = 1 -> dle nastaveni v tabulce datasheetu

   UCA1BR0 = 0x08;           //16MHz / 115200 baud
   UCA1BR1 = 0x00;
   UCA1MCTL = UCBRF_11 + UCOS16; //UCOS16 = 1 / UCBRFx = 11 -> dle nastaveni v tabulce datasheetu
   UCA1CTL1 &= ~UCSWRST;     //UCA1 softwarovy reset -> OFF

   // UCA1IE |= UCRXIE;      // vypnuto: v projektu není ISR pro USCI_A1, přijatý bajt by shodil program
}

void UART_send_byte(uint8_t b)
{
    while(!(UCA1IFG & UCTXIFG));    // počkat na volný vysílací buffer
    UCA1TXBUF = b;
}

void UART_send_string(const char *s)
{
    while(*s) UART_send_byte((uint8_t)*s++);
}

void UART_send_int(int32_t v)
{
    char tmp[11];
    uint8_t i = 0;

    if(v < 0)
    {
        UART_send_byte('-');
        v = -v;
    }
    if(v == 0) tmp[i++] = '0';
    while(v > 0)
    {
        tmp[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while(i) UART_send_byte((uint8_t)tmp[--i]);   // číslice se skládaly od konce
}