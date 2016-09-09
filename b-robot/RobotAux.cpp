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

#define SONAR_BARO_LPF_LC             0.8f

#define SONAR_NO_DETECT     -1
#define SONAR_PROGRESS      -2
#define SONAR_ECHO_DETECT   -3

s16          mDist[2];
volatile s16 mTempDist[2];
u32          mStartTS[2];
u32          mPingTS;

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

void RobotAux::initServo(void)
{
    pinMode(PIN_SERVO_1, OUTPUT);
    pinMode(PIN_SERVO_2, OUTPUT);

    // initialize timer2 as Fast PWM (mode3)
    TCCR2A = BV(COM2A1) | BV(COM2B1) | BV(WGM21) | BV(WGM20);
    TCCR2B = BV(CS22) | BV(CS21) | BV(CS20);                    // prescaler = 1024, 16MHz/1024/256 = 61Hz (16.3ms)
    TCNT2  = 0;
    moveServo(0, 1500);
    moveServo(1, 1500);
}

void RobotAux::moveServo(u8 idx, u16 pwm)
{
    u8 ocr = map(pwm, 0, 16384, 0, 256);

    if (idx == 0) {
        OCR2B = ocr;
    } else {
        OCR2A = ocr;
    }
}

void RobotAux::begin(void)
{
    mDist[0] = SONAR_NO_DETECT;
    mDist[1] = SONAR_NO_DETECT;

    initServo();

    for (u8 i = 0; i < VBAT_SMOOTH_LEVEL; i++)
        getBattVolt();

    pinMode(PIN_SONAR_TRIG, OUTPUT);
    digitalWrite(PIN_SONAR_TRIG, LOW);

    PCINT_ECHO_MASK |=  (BV(PIN_SONAR_ECHO_1) | BV(PIN_SONAR_ECHO_2));
    PCINT_ECHO_DDR  &= ~(BV(PIN_SONAR_ECHO_1) | BV(PIN_SONAR_ECHO_2));
    PCICR = PCICR | PCINT_ECHO_IR_BIT;
}

void RobotAux::updateSonar(void)
{
//    mDist[0] = mDist[0] * SONAR_BARO_LPF_LC + (mDist[0] + 1) * (1 - SONAR_BARO_LPF_LC);
//    mDist[1] = mDist[1] * SONAR_BARO_LPF_LC + (mDist[1] + 1) * (1 - SONAR_BARO_LPF_LC);
    s16 dist;

    if (micros() - mPingTS > 40000) {
        for (u8 i = 0; i < 2; i++) {
            if (mTempDist[i] < 0) {
                dist = mDist[i];
            } else {
                dist = mTempDist[i];
            }
            mDist[i] = dist; //(mDist[i] * SONAR_BARO_LPF_LC) + (mTempDist[i] * (1 - SONAR_BARO_LPF_LC));
            mTempDist[i] = SONAR_NO_DETECT;
        }
    }

    mPingTS = micros();
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
            mStartTS[0]  = micros();
            mTempDist[0] = SONAR_ECHO_DETECT;
        } else {
            mTempDist[0] = (micros() - mStartTS[0]) / 58;
        }
    }

    if (mask & BV(PIN_SONAR_ECHO_2)) {
        if (pins & BV(PIN_SONAR_ECHO_2)) {
            mStartTS[1]  = micros();
            mTempDist[1] = SONAR_ECHO_DETECT;
        } else {
            mTempDist[1] = (micros() - mStartTS[1]) / 58;
        }
    }
    ucLastPin = pins;
}

