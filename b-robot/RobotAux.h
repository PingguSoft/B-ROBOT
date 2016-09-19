/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is derived from deviationTx project for Arduino.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/

#ifndef _ROBOT_AUX_H_
#define _ROBOT_AUX_H_
#include <Arduino.h>
#include <avr/pgmspace.h>
#include "common.h"
#include "utils.h"
#include "config.h"


#define SERVO_MIN_PWM    550
#define SERVO_MAX_PWM   2250
#define SERVO_MID_PWM   ((SERVO_MIN_PWM + SERVO_MAX_PWM) / 2)

class RobotAux
{
#define VBAT_SMOOTH_LEVEL       16

public:
    void begin(void);
    u8   getBattVolt(void);
    void updateSonar(void);
    s16  getDist(u8 idx);

    void moveServo(u8 idx, u16 pwm);

private:
    void initServo(void);


    u16         mVoltBuf[VBAT_SMOOTH_LEVEL];
    u16         mVoltSum;
    u8          mVoltIdx;

    u32         mPingTS;
};

#endif
