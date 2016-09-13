#include <Arduino.h>
#include <stdarg.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <JJ_MPU6050_DMP_6Axis.h>  // Modified version of the MPU6050 library to work with DMP (see comments inside)

#include "common.h"
#include "config.h"
#include "utils.h"
#include "SerialProtocol.h"
#include "RobotAux.h"

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/
// NORMAL MODE PARAMETERS (MAXIMUN SETTINGS)
#define MAX_THROTTLE            580
#define MAX_STEERING            150
#define MAX_TARGET_ANGLE        12

// PRO MODE = MORE AGGRESSIVE (MAXIMUN SETTINGS)
#define MAX_THROTTLE_PRO        980     //680
#define MAX_STEERING_PRO        250
#define MAX_TARGET_ANGLE_PRO    40      //20

// Default control terms
#define KP                      0.19
#define KD                      28
#define KP_THROTTLE             0.07
#define KI_THROTTLE             0.04

// Control gains for raiseup (the raiseup movement requiere special control parameters)
#define KP_RAISEUP              0.16
#define KD_RAISEUP              36
#define KP_THROTTLE_RAISEUP     0       // No speed control on raiseup
#define KI_THROTTLE_RAISEUP     0.0

#define MAX_CONTROL_OUTPUT      500

// Servo definitions
#define SERVO_AUX_NEUTRO        1550    // Servo neutral position
#define SERVO_MIN_PULSEWIDTH    650
#define SERVO_MAX_PULSEWIDTH    2600

// Battery management [optional]. This is not needed for alkaline or Ni-Mh batteries but usefull for if you use lipo batteries
#define BATTERY_CHECK           0       // 0: No  check, 1: check (send message to interface)
#define LIPOBATT                0       // Default 0: No Lipo batt 1: Lipo batt (to adjust battery monitor range)
#define BATTERY_WARNING         105     // (10.5 volts) aprox [for lipo batts, disable by default]
#define BATTERY_SHUTDOWN        95      // (9.5 volts) [for lipo batts]
#define SHUTDOWN_WHEN_BATTERY_OFF 0     // 0: Not used, 1: Robot will shutdown when battery is off (BATTERY_CHECK SHOULD BE 1)

#define ZERO_SPEED              65535
#define MAX_ACCEL               7       // Maximun motor acceleration (MAX RECOMMENDED VALUE: 8) (default:7)

#define MICROSTEPPING           16      // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)

#define I2C_SPEED               400000L // 400kHz I2C speed

#define RAD2GRAD                57.2957795
#define GRAD2RAD                0.01745329251994329576923690768489

#define ITERM_MAX_ERROR         25      // Iterm windup constants for PI control //40
#define ITERM_MAX               8000    // 5000

#define MOT_1                   0
#define MOT_2                   1

#define CLK_PERIOD              2000000

#define MOTOR_TEST              0

/*
*****************************************************************************************
* MACROS
*****************************************************************************************
*/
#define CLR(x,y)                (x&=(~(1<<y)))
#define SET(x,y)                (x|=(1<<y))

#define ENABLE_MOTOR()          digitalWrite(PIN_MOT_ENABLE, LOW)
#define DISABLE_MOTOR()         digitalWrite(PIN_MOT_ENABLE, HIGH)

/*
*****************************************************************************************
* VARIABLES
*****************************************************************************************
*/
bool        mIsShutdown = false;
u8          mFifoBufs[18];
u32         mCurTS;
u32         mLastTS;
u32         mLastBattTS;
u32         mLastSonarTS;

// class default I2C address is 0x68 for MPU6050
MPU6050     mMPU;

// Angle of the robot (used for stability control)
float       mAngleAdjusted;
float       mAngleAdjustedOld;

// Default control values from constant definitions
float       mP = KP;
float       mD = KD;
float       mThrP = KP_THROTTLE;
float       mThrI = KI_THROTTLE;
float       mUserP = KP;
float       mUserD = KD;
float       mUserThrP = KP_THROTTLE;
float       mUserThrI = KI_THROTTLE;

float       mPIDErrSum;
float       mPIDErrOld = 0;
float       mPIDErrOld2 = 0;
float       mSetPointOld = 0;
float       mTargetAngle;
float       mThrottle = 0;
float       mSteering = 0;
float       mMaxThrottle = MAX_THROTTLE;
float       mMaxSteering = MAX_STEERING;
float       mMaxTargetAngle = MAX_TARGET_ANGLE;
float       mControlOutput = 0;

u16         mPeriod[2];
s16         mMotors[2];
s16         mSpeeds[2];         // Actual speed of motors
volatile s8 mDirs[2];           // Actual direction of steppers motors
s16         mActSpeed;          // overall robot speed (measured from steppers speed)
s16         mActSpeedOld;
float       mEstSpeedFiltered;  // Estimated robot speed
SerialProtocol mSerial;

RobotAux    mRobotAux;
u8          mLastBatt;

u8          mLastAuxBtn = 0;
u8          mRaiseUp = 0;
u8          mHeartBeat = 0;



/*
*****************************************************************************************
* DMP functions
*****************************************************************************************
*/
// DMP FUNCTIONS
// This function defines the weight of the accel on the sensor fusion
// default value is 0x80
// The official invensense name is inv_key_0_96 (??)
void setSensorFusionAccelGain(u8 gain)
{
    // INV_KEY_0_96
    mMPU.setMemoryBank(0);
    mMPU.setMemoryStartAddress(0x60);
    mMPU.writeMemoryByte(0);
    mMPU.writeMemoryByte(gain);
    mMPU.writeMemoryByte(0);
    mMPU.writeMemoryByte(0);
}

// Quick calculation to obtein Phi angle from quaternion solution (from DMP internal quaternion solution)
float getPhiAngle()
{
    Quaternion  q;

    mMPU.getFIFOBytes(mFifoBufs, 16); // We only read the quaternion
    mMPU.dmpGetQuaternion(&q, mFifoBufs);
    mMPU.resetFIFO();  // We always reset FIFO

    //return( asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI); //roll
    //return Phi angle (robot orientation) from quaternion DMP output
    return (atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * RAD2GRAD);
}

// PD controller implementation(Proportional, derivative). DT is in miliseconds
float getStabilityPDControlOutput(float DT, float input, float setPoint,  float p, float d)
{
    float error;
    float output;

    error = setPoint - input;
    // Kd is implemented in two parts
    //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
    //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
    output = p * error + (d * (setPoint - mSetPointOld) - d * (input - mPIDErrOld2)) / DT;
    mPIDErrOld2 = mPIDErrOld;
    mPIDErrOld = input;  // error for Kd is only the input component
    mSetPointOld = setPoint;
    return (output);
}

// PI controller implementation (Proportional, integral). DT is in miliseconds
float getSpeedPIControlOutput(float DT, float input, float setPoint,  float p, float i)
{
    float error;
    float output;

    error = setPoint - input;
    mPIDErrSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
    mPIDErrSum = constrain(mPIDErrSum, -ITERM_MAX, ITERM_MAX);

    output = p * error + i * mPIDErrSum * DT * 0.001; // DT is in miliseconds...
    return (output);
}

// 16 single cycle instructions = 1us at 16Mhz
void delay1uS()
{
    __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
}


/*
*****************************************************************************************
* ISR for TIMER1
*****************************************************************************************
*/
ISR(TIMER1_COMPA_vect)
{
    if (mDirs[MOT_1] == 0) {
        return;
    }

    OCR1A = TCNT1 + mPeriod[MOT_1];
    SET(PORT_MOT_STEP, BIT_MOT_1_STEP);
    delay1uS();
    CLR(PORT_MOT_STEP, BIT_MOT_1_STEP);
}

ISR(TIMER1_COMPB_vect)
{
    if (mDirs[MOT_2] == 0) {
        return;
    }

    OCR1B = TCNT1 + mPeriod[MOT_2];
    SET(PORT_MOT_STEP, BIT_MOT_2_STEP);
    delay1uS();
    CLR(PORT_MOT_STEP, BIT_MOT_2_STEP);
}


/*
*****************************************************************************************
* setMotorSpeed
*****************************************************************************************
*/
void setMotorSpeed(s16 *motors)
{
    long period;
    s16  speed;

    for (u8 i = 0; i < 2; i++) {
        if ((mSpeeds[i] - motors[i]) > MAX_ACCEL)
            mSpeeds[i] -= MAX_ACCEL;
        else if ((mSpeeds[i] - motors[i]) < -MAX_ACCEL)
            mSpeeds[i] += MAX_ACCEL;
        else
            mSpeeds[i] = motors[i];

#if MICROSTEPPING==16
        speed = mSpeeds[i] * 46;                  // Adjust factor from control output speed to real motor speed in steps/second
#else
        speed = mSpeeds[i] * 23;                  // 1/8 Microstepping
#endif

        if (motors[i] == 0) {
            period = ZERO_SPEED;
            mDirs[i] = 0;
        } else if (motors[i] > 0) {
            period = CLK_PERIOD / speed;

            // forward
            mDirs[i] = 1;
            if (i == MOT_1) {
                SET(PORT_MOT_DIR, BIT_MOT_1_DIR);
            } else {
                CLR(PORT_MOT_DIR, BIT_MOT_2_DIR);
            }
        } else {
            period = CLK_PERIOD / -speed;

            // backward
            mDirs[i] = -1;
            if (i == MOT_1) {
                CLR(PORT_MOT_DIR, BIT_MOT_1_DIR);
            } else {
                SET(PORT_MOT_DIR, BIT_MOT_2_DIR);
            }
        }

        if (period > 65536)
            period = ZERO_SPEED;

        mPeriod[i] = period;
        if (i == MOT_1) {
            OCR1A = TCNT1 + period;
        } else {
            OCR1B = TCNT1 + period;
        }
    }
}


/*
*****************************************************************************************
* mspCallback
*****************************************************************************************
*/
s8 mspCallback(u8 cmd, u8 *data, u8 size, u8 *res)
{
    u16 *rc;
    u32 *ptr;
    u16 val;
    s8  ret = -1;
    static u16 wmCycleTime = 0;

    switch (cmd) {

        case SerialProtocol::MSP_ANALOG:
            res[0] = mLastBatt;
            ret = 7;
            break;

        case SerialProtocol::MSP_STATUS:
            *((u16*)&res[0]) = wmCycleTime++;
            *((u32*)&res[6]) = mIsShutdown ? 0 : 1;
            ret = 11;
            break;

        case SerialProtocol::MSP_ALTITUDE:
            ptr = (u32*)res;
            *ptr = mRobotAux.getDist(0);
            ret = 6;
            break;

        case SerialProtocol::MSP_SET_RAW_RC:
            rc = (u16*)data;

            // roll
            val = (*rc++ - 1000);
            mSteering = (val / 1000.0) - 0.5;
            mSteering = -mSteering;
            if (mSteering > 0)
                mSteering = (mSteering * mSteering + 0.5 * mSteering) * mMaxSteering;
            else
                mSteering = (-mSteering * mSteering + 0.5 * mSteering) * mMaxSteering;

            // pitch
            val = (*rc++ - 1000);
            mThrottle = -((val / 1000.0) - 0.5) * mMaxThrottle;

            // yaw
            val = (*rc++ - 1000);

            // throttle
            val = (*rc++ - 1000);
            if (-45 < mAngleAdjusted && mAngleAdjusted < 45) {
                u16 pwm = map(val, 0, 1000, SERVO_MIN_PWM, SERVO_MAX_PWM);
                mRobotAux.moveServo(0, pwm);
            }

            // AUX1 - AUX4
            val = 0;
            for (u8 i = 0; i < 4; i++) {
                if (*rc++ > 1700)
                    val |= BV(i);
            }
            if (val != mLastAuxBtn) {
                if (val & BV(0)) {
                    mMaxThrottle = MAX_THROTTLE_PRO;
                    mMaxSteering = MAX_STEERING_PRO;
                    mMaxTargetAngle = MAX_TARGET_ANGLE_PRO;
                } else {
                    mMaxThrottle = MAX_THROTTLE;
                    mMaxSteering = MAX_STEERING;
                    mMaxTargetAngle = MAX_TARGET_ANGLE;
                }

                mRaiseUp    = val & BV(1);
                mIsShutdown = val & BV(3);

                if (mIsShutdown) {
                    s16 motors[2] = { 0, 0 };

                    setMotorSpeed(motors);
                    DISABLE_MOTOR();
                    mRobotAux.moveServo(0, SERVO_MID_PWM);
                }
                mLastAuxBtn  = val;
            }
            break;
    }

    return ret;
}


/*
*****************************************************************************************
* setup
*****************************************************************************************
*/
void setup()
{
    pinMode(PIN_MOT_ENABLE, OUTPUT);
    pinMode(PIN_MOT_1_STEP, OUTPUT);
    pinMode(PIN_MOT_1_DIR, OUTPUT);
    pinMode(PIN_MOT_2_DIR, OUTPUT);
    pinMode(PIN_MOT_2_STEP, OUTPUT);
    pinMode(PIN_LED, OUTPUT);

    DISABLE_MOTOR();
    digitalWrite(PIN_LED, LOW);
    mRobotAux.begin();
    mLastBatt = mRobotAux.getBattVolt();

    mSerial.begin(57600);
    mSerial.registerMSPCallback(mspCallback);

    LOG(F("---- BROBOT ---- \n"));

#if !MOTOR_TEST
    Wire.begin();
    TWSR = 0;
    TWBR = ((16000000L / I2C_SPEED) - 16) / 2;
    TWCR = 1 << TWEN;
    delay(200);

    mMPU.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    mMPU.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mMPU.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mMPU.setDLPFMode(MPU6050_DLPF_BW_10);       // 10,20,42,98,188  // Default factor for BROBOT:10
    mMPU.setRate(4);                            // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
    mMPU.setSleepEnabled(false);
    delay(500);

    LOG(F("Initializing DMP...\n"));
    u8 status = mMPU.dmpInitialize();
    if (status == 0) {
        LOG(F("Enabling DMP...\n"));
        mMPU.setDMPEnabled(true);
        status = mMPU.getIntStatus();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        LOG(F("DMP Initialization failed (code : %d)\n"), status);
    }

    // Gyro calibration
    // The robot must be steady during initialization
    // Time to settle things... the bias_from_no_motion algorithm needs some time to take effect and reset gyro bias.
    LOG(F("Gyro calibration!!  Dont move the robot in 10 seconds...\n"));
    mLastTS = millis();
    do {
        mHeartBeat = !mHeartBeat;
        digitalWrite(PIN_LED, mHeartBeat);
        delay(50);
    } while (millis() - mLastTS < 10000);

    LOG(F("Testing device connections...\n"));
    if (mMPU.testConnection()) {
        LOG(F("MPU6050 connection successful\n"));
    } else {
        LOG(F("MPU6050 connection failed\n"));
    }

    LOG(F("Adjusting DMP sensor fusion gain...\n"));
    setSensorFusionAccelGain(0x20);
#endif

    LOG(F("Steper motors initialization...\n"));
    // TIMER1 for motor step
    TCCR1A = 0;                                 // Timer1 normal mode 0, OCxA,B outputs disconnected
    TCCR1B = BV(CS11);                          // Prescaler=8, => 2Mhz
    OCR1A  = ZERO_SPEED;
    OCR1B  = ZERO_SPEED;
    mDirs[MOT_1] = 0;
    mDirs[MOT_2] = 0;
    TCNT1  = 0;
    TIMSK1 |= (BV(OCIE1A) | BV(OCIE1B));        // Enable Timer1 interrupt
    ENABLE_MOTOR();

    // Little motor vibration and servo move to indicate that robot is ready
    s16 speeds[2];
    for (u8 k = 0; k < 5; k++) {
        speeds[0] = 5;
        speeds[1] = -5;
        setMotorSpeed(speeds);
        delay(200);
        speeds[0] = -5;
        speeds[1] = 5;
        setMotorSpeed(speeds);
        delay(200);
    }
    speeds[0] = 0;
    speeds[1] = 0;
    setMotorSpeed(speeds);

    LOG(F("Let's start...\n"));
#if !MOTOR_TEST
    mMPU.resetFIFO();
#endif

    mLastTS = millis() - 1;
    mIsShutdown = false;
}


/*
*****************************************************************************************
* test routines
*****************************************************************************************
*/
#if MOTOR_TEST
void testMotors(void)
{
    static s16  motorSpeeds[2];
    static u16  servoPwm[2] = { 1000, 1000 };
    static bool isBattMon  = false;
    static bool isSonarMon = false;

    if (mSerial.available()) {
        u8  ch = mSerial.read();

        switch (ch) {
            case 'a':
                motorSpeeds[MOT_1] = constrain(motorSpeeds[MOT_1] + 5, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
                setMotorSpeed(motorSpeeds);
                break;

            case 'z':
                motorSpeeds[MOT_1] = constrain(motorSpeeds[MOT_1] - 5, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
                setMotorSpeed(motorSpeeds);
                break;

            case 's':
                motorSpeeds[MOT_2] = constrain(motorSpeeds[MOT_2] + 5, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
                setMotorSpeed(motorSpeeds);
                break;

            case 'x':
                motorSpeeds[MOT_2] = constrain(motorSpeeds[MOT_2] - 5, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
                setMotorSpeed(motorSpeeds);
                break;

            case '1':
                servoPwm[0] = constrain(servoPwm[0] + 10, SERVO_MIN_PWM, SERVO_MAX_PWM);
                mRobotAux.moveServo(0, servoPwm[0]);
                break;

            case 'q':
                servoPwm[0] = constrain(servoPwm[0] - 10, SERVO_MIN_PWM, SERVO_MAX_PWM);
                mRobotAux.moveServo(0, servoPwm[0]);
                break;

            case '[':
                for (u16 i = SERVO_MIN_PWM; i<= SERVO_MAX_PWM; i += 10) {
                    mRobotAux.moveServo(0, i);
                    delay(20);
                }
                break;

            case '2':
                servoPwm[1] = constrain(servoPwm[1] + 10, SERVO_MIN_PWM, SERVO_MAX_PWM);
                mRobotAux.moveServo(1, servoPwm[1]);
                break;

            case 'w':
                servoPwm[1] = constrain(servoPwm[1] - 10, SERVO_MIN_PWM, SERVO_MAX_PWM);
                mRobotAux.moveServo(1, servoPwm[1]);
                break;

            case ']':
                for (u16 i = SERVO_MIN_PWM; i<= SERVO_MAX_PWM; i += 10) {
                    mRobotAux.moveServo(1, i);
                    delay(20);
                }
                break;

            case 'b':
                isBattMon = !isBattMon;
                break;

            case 'u':
                isSonarMon = !isSonarMon;
                break;
        }
        LOG(F("MOTOR1:%4d, MOTOR2:%4d, SERVO1:%4d, SERVO2:%4d, BATTMON:%1d, SONARMON:%1d\n"),
            motorSpeeds[MOT_1], motorSpeeds[MOT_2], servoPwm[0], servoPwm[1], isBattMon, isSonarMon);
    }

    mCurTS = millis();
    if (mCurTS - mLastBattTS > 50) {
        if (isBattMon) {
            mLastBatt = mRobotAux.getBattVolt();
            LOG(F("VOLT:%4d\n"), mLastBatt);
        }

        if (isSonarMon) {
            s16 dist;
            for (u8 i = 0; i < 2; i++) {
                dist = mRobotAux.getDist(i);
                if (dist > 0) {
                    LOG(F("SONAR%d => DIST:%3d Cm\n"), i, dist);
                }
            }
            mRobotAux.updateSonar();
        }
        mLastBattTS = mCurTS;
    }
}
#endif


/*
*****************************************************************************************
* loop
*****************************************************************************************
*/
void loop()
{
    float       fDelta;

#if MOTOR_TEST
    testMotors();
    return;
#endif

    mSerial.handleMSP();
    mCurTS = millis();

    // every 5sec
    if (mCurTS - mLastBattTS > 5000) {
        mLastBatt = mRobotAux.getBattVolt();
        LOG(F("VOLT:%4d\n"), mLastBatt);
        mLastBattTS = mCurTS;
    }

    // every 50ms
    if (mCurTS - mLastSonarTS > 50) {
        s16 dist = mRobotAux.getDist(0);
        if (dist > 0) {
            LOG(F("%8ld DIST:%3d Cm\n"), micros(), dist);
        }
        mRobotAux.updateSonar();
        mHeartBeat = !mHeartBeat;
        digitalWrite(PIN_LED, mHeartBeat);
        mLastSonarTS = mCurTS;
    }

    if (mIsShutdown) {
        return;
    }

    u16 nFifoCtr = mMPU.getFIFOCount();
    if (nFifoCtr >= 18) {
        // If we have more than one packet we take the easy path: discard the buffer and wait for the next one
        if (nFifoCtr > 18) {
            LOG(F("FIFO RESET!!\n"));
            mMPU.resetFIFO();
            return;
        }
        fDelta = (mCurTS - mLastTS);
        mLastTS = mCurTS;

        mAngleAdjustedOld = mAngleAdjusted;
        // Get new orientation angle from IMU (MPU6050)
        mAngleAdjusted = getPhiAngle();

        mMPU.resetFIFO();  // We always reset FIFO

        // We calculate the estimated robot speed:
        // Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
        mActSpeedOld = mActSpeed;
        mActSpeed = (mSpeeds[MOT_1] + mSpeeds[MOT_2]) / 2; // Positive: forward

        s16 angular_velocity = (mAngleAdjusted - mAngleAdjustedOld) * 90.0; // 90 is an empirical extracted factor to adjust for real units
        s16 estimated_speed = -mActSpeedOld - angular_velocity;     // We use robot_speed(t-1) or (t-2) to compensate the delay
        mEstSpeedFiltered = mEstSpeedFiltered * 0.95 + (float)estimated_speed * 0.05;  // low pass filter on estimated speed

        // SPEED CONTROL: This is a PI controller.
        //    input:user throttle, variable: estimated robot speed, output: target robot angle to get the desired speed
        mTargetAngle = getSpeedPIControlOutput(fDelta, mEstSpeedFiltered, mThrottle, mThrP, mThrI);
        mTargetAngle = constrain(mTargetAngle, -mMaxTargetAngle, mMaxTargetAngle); // limited output
        //LOG(F("AngleAdjusted:%f, EstSpeedFiltered:%f, TargetAngle:%f\n"), mAngleAdjusted, mEstSpeedFiltered, mTargetAngle);

#if 1
        // check sonar sensor when normal condition
        if (-45 < mAngleAdjusted && mAngleAdjusted < 45) {
            s16 dist = mRobotAux.getDist(0);
            if ((0 < dist && dist < 20) && mTargetAngle < 0) {
                mTargetAngle = 0;
            }
        }
#endif

        // Stability control: This is a PD controller.
        //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
        //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
        mControlOutput += getStabilityPDControlOutput(fDelta, mAngleAdjusted, mTargetAngle, mP, mD);
        mControlOutput = constrain(mControlOutput, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

        // The steering part from the user is injected directly on the output
        mMotors[MOT_1] = mControlOutput + mSteering;
        mMotors[MOT_2] = mControlOutput - mSteering;

        // Limit max speed (control output)
        mMotors[MOT_1] = constrain(mMotors[MOT_1], -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
        mMotors[MOT_2] = constrain(mMotors[MOT_2], -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

#if 0
        Serial.print(mCurTS);
        Serial.print("  AA:");
        Serial.print(mAngleAdjusted);
        Serial.print(", TA:");
        Serial.print(mTargetAngle);
        Serial.print(", ES:");
        Serial.print(mEstSpeedFiltered);
        Serial.print(", CO:");
        Serial.print(mControlOutput);
        Serial.print(", DT:");
        Serial.print(fDelta);
        Serial.print(", M1:");
        Serial.print(mMotors[MOT_1]);
        Serial.print(", M2:");
        Serial.println(mMotors[MOT_2]);
#endif

        // NOW we send the commands to the motors
        if (-76 < mAngleAdjusted && mAngleAdjusted < 76) { // Is robot ready (upright?)
            // NORMAL MODE
            ENABLE_MOTOR();
            setMotorSpeed(mMotors);

            // Normal condition?
            if (-45 < mAngleAdjusted && mAngleAdjusted < 45) {
                mP = mUserP;            // Default user control gains
                mD = mUserD;
                mThrP = mUserThrP;
                mThrI = mUserThrI;
            } else {   // We are in the raise up procedure => we use special control parameters
                mP = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
                mD = KD_RAISEUP;
                mThrP = KP_THROTTLE_RAISEUP;
                mThrI = KI_THROTTLE_RAISEUP;
            }
        } else {   // Robot not ready (flat)
            DISABLE_MOTOR();
            s16 motors[2] = { 0, 0 };
            setMotorSpeed(motors);
            mPIDErrSum = 0;  // Reset PID I term
            mP = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
            mD = KD_RAISEUP;
            mThrP = KP_THROTTLE_RAISEUP;
            mThrI = KI_THROTTLE_RAISEUP;

            if (mRaiseUp) {
                if (mAngleAdjusted > 0) {
                    mRobotAux.moveServo(0, SERVO_MIN_PWM);
                } else {
                    mRobotAux.moveServo(0, SERVO_MAX_PWM);
                }
            } else {
                mRobotAux.moveServo(0, SERVO_MID_PWM);
            }
        }
    }
}

