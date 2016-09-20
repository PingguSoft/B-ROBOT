/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/

#ifndef __CONFIG_H__
#define __CONFIG_H__
#include "common.h"

/*
*****************************************************************************************
* CONFIGURATION
*****************************************************************************************
*/
// controller
#define __OSC__         0
#define __MSP__         1

// sonar
#define __SONAR__       0

// motor test
#define __MOTOR_TEST__  0

// debug
#define __DEBUG__       0


#if __MOTOR_TEST__ && !__DEBUG__
#undef  __DEBUG__
#define __DEBUG__       1
#endif

#if defined(__AVR_ATmega328P__)
#define __STD_SERIAL__  0
#elif defined(__AVR_ATmega32U4__)
#define __STD_SERIAL__  1
#endif

/*
*****************************************************************************************
* PINS
*****************************************************************************************
*/

//---------------------------------------------------------------------------------------
#if defined(__AVR_ATmega328P__)

// MOTOR1
#define PIN_MOT_1_ENABLE        10

#define PIN_MOT_1_STEP          A0
#define PORT_MOT_1_STEP         PORTC
#define BIT_MOT_1_STEP          0

#define PIN_MOT_1_DIR           A1
#define PORT_MOT_1_DIR          PORTC
#define BIT_MOT_1_DIR           1

// MOTOR2
#define PIN_MOT_2_ENABLE        10

#define PIN_MOT_2_DIR           A2
#define PORT_MOT_2_DIR          PORTC
#define BIT_MOT_2_DIR           2

#define PIN_MOT_2_STEP          A3
#define PORT_MOT_2_STEP         PORTC
#define BIT_MOT_2_STEP          3

// OTHERS
#define PIN_LED                 13

#define PIN_SONAR_TRIG          5
#define PIN_SONAR_ECHO_1        6
#define PIN_SONAR_ECHO_2        7

#define PCINT_ECHO_PORT         PORTD
#define PCINT_ECHO_DDR          DDRD
#define PCINT_ECHO_MASK         PCMSK2
#define PCINT_ECHO              PCINT2_vect
#define PCINT_ECHO_PINS         PIND
#define PCINT_ECHO_IR_BIT       BV(2)

#define PIN_SERVO_1             3
#define PIN_SERVO_2             11

// ANALOG
#define PIN_ANALOG_VOLT         6       // A6

//---------------------------------------------------------------------------------------
#elif defined(__AVR_ATmega32U4__)

// MOTOR1
#define PIN_MOT_1_ENABLE        12

#define PIN_MOT_1_STEP          5
#define PORT_MOT_1_STEP         PORTC
#define BIT_MOT_1_STEP          6

#define PIN_MOT_1_DIR           4
#define PORT_MOT_1_DIR          PORTD
#define BIT_MOT_1_DIR           4


// MOTOR2
#define PIN_MOT_2_ENABLE        8

#define PIN_MOT_2_STEP          6
#define PORT_MOT_2_STEP         PORTD
#define BIT_MOT_2_STEP          7

#define PIN_MOT_2_DIR           7
#define PORT_MOT_2_DIR          PORTE
#define BIT_MOT_2_DIR           6

// OTHERS
#define PIN_LED                 A4

#define PIN_SONAR_TRIG          A5      // 19
#define PIN_SONAR_ECHO_1        3
#define PIN_SONAR_ECHO_2        2

#define PCINT_ECHO_PORT         PORTD
#define PCINT_ECHO_DDR          DDRD
#define PCINT_ECHO_MASK         PCMSK2
#define PCINT_ECHO              PCINT2_vect
#define PCINT_ECHO_PINS         PIND
#define PCINT_ECHO_IR_BIT       BV(2)

#define PIN_SERVO_1             13
#define PIN_SERVO_2             10

// ANALOG
#define PIN_ANALOG_VOLT         0       // A0

#endif

#endif
