#include <msp430.h>
#include "include/logging.h"
#include "include/UART.h"

static int16_t  logZ[LOG_SIZE];
static volatile uint16_t logCount = 0;
static volatile uint8_t  sampleFlag = 0;

void Log_init(void)
{
    logCount = 0;
    sampleFlag = 0;

    TA1CCR0  = (16000000UL / 8 / SAMPLE_RATE_HZ) - 1;   // 2 MHz /100
    TA1CCTL0 = CCIE;
    TA1CTL   = TASSEL__SMCLK | ID__8 | MC__UP | TACLR;
}

uint8_t Log_sampleReady(void)
{
    if (sampleFlag)
    {
        sampleFlag = 0;
        return 1;
    }
    return 0;
}

void Log_add(int16_t value)
{
    if (logCount < LOG_SIZE)
        logZ[logCount++] = value;
}

void Log_send(void)
{
    uint16_t i;
    uint16_t n = Log_count();
    UART_send_string("#LOG\r\n");
    for (i = 0; i < n; i++)
    {
        UART_send_int(Log_get(i));
        UART_send_string("\r\n");
    }
}

uint8_t  Log_isFull(void)            { return logCount >= LOG_SIZE; }
uint16_t Log_count(void)             { return logCount; }
int16_t  Log_get(uint16_t index)     { return logZ[index]; }
void     Log_reset(void)             { logCount = 0; }

#pragma vector = TIMER1_A0_VECTOR
__interrupt void TA1_ISR(void)
{
    sampleFlag = 1;
}