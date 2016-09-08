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

// PINS
#define PORT_MOT_STEP           PORTC
#define PORT_MOT_DIR            PORTC

#define PIN_MOT_ENABLE          10

#define PIN_MOT_1_STEP          A0
#define BIT_MOT_1_STEP          0

#define PIN_MOT_1_DIR           A1
#define BIT_MOT_1_DIR           1

#define PIN_MOT_2_DIR           A2
#define BIT_MOT_2_DIR           2

#define PIN_MOT_2_STEP          A3
#define BIT_MOT_2_STEP          3

#define PIN_LED                 13

#define PIN_ANALOG_VOLT         6       // A6


#define PIN_SONAR_TRIG          5
#define PIN_SONAR_ECHO_1        6
#define PIN_SONAR_ECHO_2        7

#define PCINT_ECHO_PORT         PORTD
#define PCINT_ECHO_DDR          DDRD
#define PCINT_ECHO_MASK         PCMSK2
#define PCINT_ECHO              PCINT2_vect
#define PCINT_ECHO_PINS         PIND
#define PCINT_ECHO_IR_BIT       BV(2)

#endif
