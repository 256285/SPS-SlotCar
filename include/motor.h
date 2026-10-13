/*
 * motor.h
 *
 *  Created on: 29. 9. 2020
 *      Author: dosedel
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "stdint.h"
#include <msp430.h>

// macros definition
// H_Bridge port definition
#define H_MODE1_PORT  P10OUT     // break
#define H_ENABLE_PORT P8OUT     // in
#define H_PHASE_PORT  P10OUT     // direction

// H_Bridge pins definition
#define H_MODE1_PIN    BIT2
#define H_ENABLE_PIN   BIT4
#define H_PHASE_PIN    BIT1


#define H_MODE1_ON() (H_MODE1_PORT |= H_MODE1_PIN)      //hard stopping
#define H_MODE1_OFF() (H_MODE1_PORT &= ~H_MODE1_PIN)    //soft stopping


// POZOR: po Motor_init() je P8.4 přepnutý na výstup časovače (PWM),
// takže zápis do P8OUT na pin nemá žádný vliv. Rychlost se nastavuje
// funkcí Motor_setSpeed(), ne těmito makry.
#define H_ENABLE_ON() (H_ENABLE_PORT |= H_ENABLE_PIN)   //go
#define H_ENABLE_OFF() (H_ENABLE_PORT &= ~H_ENABLE_PIN) //stop


#define H_PHASE_FW() (H_PHASE_PORT |= H_PHASE_PIN)      //direction backward
#define H_PHASE_RV() (H_PHASE_PORT &= ~H_PHASE_PIN)     //direction forward

// PWM: Timer A0, SMCLK = 16 MHz, perioda 800 taktů -> 20 kHz
#define MOTOR_PWM_PERIOD  800

void Motor_init(void);
void Motor_setSpeed(uint8_t percent);   // střída PWM 0 - 100 %

#endif /* MOTOR_H_ */
