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
* OPTIONS
*****************************************************************************************
*/
// board type
#define __BOARD_PRO_MINI__          1
#define __BOARD_UNO_SHIELD__        2
#define __BOARD_LEONARDO_SHIELD__   3

// controller
#define __CON_OSC__                 1
#define __CON_MSP__                 2


/*
*****************************************************************************************
* CONFIGURATION
*****************************************************************************************
*/
#define __FEATURE_BOARD__           __BOARD_PRO_MINI__
#define __FEATURE_CONTROLLER__      __CON_MSP__

// sonar
#define __FEATURE_SONAR__           1

// motor test
#define __FEATURE_MOTOR_TEST__      0

// debug
#define __FEATURE_DEBUG__           0


/*
*****************************************************************************************
* RULE CHECK
*****************************************************************************************
*/
#if __FEATURE_MOTOR_TEST__ && !__FEATURE_DEBUG__
#undef  __FEATURE_DEBUG__
#define __FEATURE_DEBUG__           1
#endif

#if defined(__AVR_ATmega328P__)
#define __STD_SERIAL__              0
#elif defined(__AVR_ATmega32U4__)
#define __STD_SERIAL__              1
#endif

#if defined(__AVR_ATmega32U4__) && (__FEATURE_BOARD__ != __BOARD_LEONARDO_SHIELD__)
#error "Check __FEATURE_BOARD__, it doesn't match with arduino board type"
#endif

#if defined(__AVR_ATmega328P__) && (__FEATURE_BOARD__ == __BOARD_LEONARDO_SHIELD__)
#error "Check __FEATURE_BOARD__, it doesn't match with arduino board type"
#endif

/*
*****************************************************************************************
* PINS
*****************************************************************************************
*/

//---------------------------------------------------------------------------------------
#if (__FEATURE_BOARD__ == __BOARD_PRO_MINI__)

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
#define BIT_SONAR_ECHO_1        6
#define PIN_SONAR_ECHO_2        7
#define BIT_SONAR_ECHO_2        7

#define PCINT_ECHO_PORT         PORTD
#define PCINT_ECHO_MASK         PCMSK2
#define PCINT_ECHO              PCINT2_vect
#define PCINT_ECHO_PINS         PIND
#define PCINT_ECHO_IR_BIT       BV(2)

#define PIN_SERVO_1             3
#define PIN_SERVO_2             11

// ANALOG
#define PIN_ANALOG_VOLT         6       // A6

// wifi module - default firmware (1), tcp2serial firmware (0)
#define __FEATURE_WIFI_DEFAULT__    0

//---------------------------------------------------------------------------------------
#elif (__FEATURE_BOARD__ == __BOARD_UNO_SHIELD__)

// MOTOR1
#define PIN_MOT_1_ENABLE        12

#define PIN_MOT_1_STEP          5
#define PORT_MOT_1_STEP         PORTD
#define BIT_MOT_1_STEP          5

#define PIN_MOT_1_DIR           4
#define PORT_MOT_1_DIR          PORTD
#define BIT_MOT_1_DIR           4


// MOTOR2
#define PIN_MOT_2_ENABLE        8

#define PIN_MOT_2_STEP          6
#define PORT_MOT_2_STEP         PORTD
#define BIT_MOT_2_STEP          6

#define PIN_MOT_2_DIR           7
#define PORT_MOT_2_DIR          PORTD
#define BIT_MOT_2_DIR           7

// OTHERS
#define PIN_LED                 13

#define PIN_SONAR_TRIG          A5      // 19
#define PIN_SONAR_ECHO_1        9
#define BIT_SONAR_ECHO_1        1
#define PIN_SONAR_ECHO_2        10
#define BIT_SONAR_ECHO_2        2

#define PCINT_ECHO_PORT         PORTB
#define PCINT_ECHO_MASK         PCMSK0
#define PCINT_ECHO              PCINT0_vect
#define PCINT_ECHO_PINS         PINB
#define PCINT_ECHO_IR_BIT       BV(0)

#define PIN_SERVO_1             3
#define PIN_SERVO_2             11

// ANALOG
#define PIN_ANALOG_VOLT         0       // A0

#define SERIAL_BPS              115200

// wifi module - default firmware (1), tcp2serial firmware (0)
#define __FEATURE_WIFI_DEFAULT__    1

//---------------------------------------------------------------------------------------
#elif (__FEATURE_BOARD__ == __BOARD_LEONARDO_SHIELD__)

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
#define PIN_SONAR_ECHO_1        9
#define BIT_SONAR_ECHO_1        5
#define PIN_SONAR_ECHO_2        11
#define BIT_SONAR_ECHO_2        7

#define PCINT_ECHO_PORT         PORTB
#define PCINT_ECHO_MASK         PCMSK0
#define PCINT_ECHO              PCINT0_vect
#define PCINT_ECHO_PINS         PINB
#define PCINT_ECHO_IR_BIT       BV(0)

#define PIN_SERVO_1             13
#define PIN_SERVO_2             10

// ANALOG
#define PIN_ANALOG_VOLT         0       // A0

// wifi module - default firmware (1), tcp2serial firmware (0)
#define __FEATURE_WIFI_DEFAULT__    1
#endif

/*
*****************************************************************************************
* COMMON SETTINGS
*****************************************************************************************
*/

#if __FEATURE_WIFI_DEFAULT__
#define SERIAL_BPS              115200
#else
#define SERIAL_BPS              57600
#endif
