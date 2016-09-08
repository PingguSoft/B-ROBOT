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

#include <stdarg.h>
#include "common.h"
#include "utils.h"
#include "RobotAux.h"

s16         mDist[2];
s16         mTempDist[2];
u32         mStartTS[2];

u8 RobotAux::getBattVolt(void)
{
    u16 v;

    v = analogRead(PIN_ANALOG_VOLT);

    mVoltSum += v;
    mVoltSum -= mVoltBuf[mVoltIdx];
    mVoltBuf[mVoltIdx++] = v;
    mVoltIdx %= VBAT_SMOOTH_LEVEL;

    return map(mVoltSum / VBAT_SMOOTH_LEVEL, 0, 1023, 0, 130);
}

void RobotAux::begin(void)
{
    mDist[0] = 0;
    mDist[1] = 0;

    pinMode(PIN_SONAR_TRIG, OUTPUT);
    digitalWrite(PIN_SONAR_TRIG, LOW);

    PCINT_ECHO_MASK |=  (BV(PIN_SONAR_ECHO_1) | BV(PIN_SONAR_ECHO_2));
    PCINT_ECHO_DDR  &= ~(BV(PIN_SONAR_ECHO_1) | BV(PIN_SONAR_ECHO_2));
    PCICR = PCICR | PCINT_ECHO_IR_BIT;

    for (u8 i = 0; i < VBAT_SMOOTH_LEVEL; i++)
        getBattVolt();
}


#define SONAR_BARO_LPF_LC             0.9f

void RobotAux::updateSonar(void)
{
    mDist[0] = mDist[0] * SONAR_BARO_LPF_LC + (mTempDist[0] + 1) * (1 - SONAR_BARO_LPF_LC);
    mDist[1] = mDist[1] * SONAR_BARO_LPF_LC + (mTempDist[1] + 1) * (1 - SONAR_BARO_LPF_LC);
    digitalWrite(PIN_SONAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_SONAR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_SONAR_TRIG, LOW);
}

s16 RobotAux::getDist(u8 idx)
{
    return mDist[idx];
}

ISR(PCINT_ECHO)
{
    static u8 ucLastPin;

    u8 pins = PCINT_ECHO_PINS;
    u8 mask = pins ^ ucLastPin;

    if (mask & BV(PIN_SONAR_ECHO_1)) {
        if (pins & BV(PIN_SONAR_ECHO_1)) {
            mStartTS[0] = micros();
        } else {
            s16 dist = (micros() - mStartTS[0]) / 58;
            if (dist < 400) {
                mTempDist[0] = dist;
            } else {
                mTempDist[0] = -1;
            }
        }
    }

    if (mask & BV(PIN_SONAR_ECHO_2)) {
        if (pins & BV(PIN_SONAR_ECHO_2)) {
            mStartTS[1] = micros();
        } else {
            s16 dist = (micros() - mStartTS[1]) / 58;
            if (dist < 400) {
                mTempDist[1] = dist;
            } else {
                mTempDist[1] = -1;
            }
        }
    }
    ucLastPin = pins;
}

