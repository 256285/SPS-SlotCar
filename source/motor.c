/*
 * motor.c
 *
 *  Created on: 29. 9. 2020
 *      Author: dosedel
 */


#include "include/motor.h"

/*
 * Motor_init - nastaví piny H můstku a spustí PWM na ENABLE (P8.4 = TA0.4)
 * Po inicializaci motor stojí (střída 0 %), nastaven smer forward.
 */
void Motor_init(void)
{
   P10DIR |= 0x06;                  // MODE1 (P10.2) a PHASE (P10.1) jako výstupy
   P8DIR  |= BIT4;                  // ENABLE (P8.4) jako výstup
   P8SEL  |= BIT4;                  // P8.4 = výstup časovače TA0.4 (PWM)

   TA0CCR0  = MOTOR_PWM_PERIOD - 1; // perioda PWM
   TA0CCR4  = 0;                    // střída 0 % = motor stojí
   TA0CCTL4 = OUTMOD_7;             // reset/set: výstup HIGH od začátku periody do CCR4
   TA0CTL   = TASSEL__SMCLK | MC__UP | TACLR;   // SMCLK (16 MHz), up mode
   H_PHASE_FW();                    //nastavi smer na forward
}

/*
 * Motor_setSpeed - nastaví střídu PWM na ENABLE
 * percent - 0 - 100%, hodnoty nad 100 se oříznou na 100
 */
void Motor_setSpeed(int8_t percent)
{
    if(percent > 100)        percent = 100;
    
    TA0CCR4 = ((uint32_t)percent * MOTOR_PWM_PERIOD) / 100;
}
