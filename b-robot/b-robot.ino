#include <Arduino.h>
#include <stdarg.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <JJ_MPU6050_DMP_6Axis.h>  // Modified version of the MPU6050 library to work with DMP (see comments inside)

#include "common.h"
#include "utils.h"
#include "SerialProtocol.h"

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/

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
#define DEBUG                   0       // 0 = No debug info (default)

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

#define MOTOR_TEST              1

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
bool        mIsShutdown = false; // Robot shutdown flag => Out of

// MPU control/status vars
u8          mFifoBufs[18];          // FIFO storage buffer

u32         mLastBattTS;
u32         mLastTS;
u32         mCurTS;

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

volatile u16 mTimerVals[2];
volatile u16 mTimerValOrgs[2];
s16         mMotors[2];
s16         mSpeeds[2];         // Actual speed of motors
volatile s8 mDirs[2];           // Actual direction of steppers motors
s16         mActSpeed;          // overall robot speed (measured from steppers speed)
s16         mActSpeedOld;
float       mEstSpeedFiltered;  // Estimated robot speed

#if !__STD_SERIAL__
SerialProtocol mSerial;
#endif

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
void delay_1us()
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

//
// 8bit mode for matching with timer2
//
ISR(TIMER1_COMPA_vect)
{
    if (mDirs[MOT_1] == 0) {
        return;
    }

    u16 tick = mTimerVals[MOT_1];
    if (tick > 255) {
        tick -= 255;
        if (tick < 20) {
            goto trigger;
        } else if (tick <= 255) {
            OCR1A = tick;
            TCNT1 = 0;
        }
        mTimerVals[MOT_1] = tick;
        return;
    }

trigger:
    SET(PORT_MOT_STEP, BIT_MOT_1_STEP);
    u16 tickOrg = mTimerValOrgs[MOT_1];
    if (tickOrg > 255) {
        OCR1A = 255;
        TCNT1 = 0;
        mTimerVals[MOT_1] = tickOrg;
    } else {
        OCR1A = tickOrg;
        TCNT1 = 0;
    }
    CLR(PORT_MOT_STEP, BIT_MOT_1_STEP);
}


ISR(TIMER2_COMPA_vect)
{
    if (mDirs[MOT_2] == 0) {
        return;
    }

    u16 tick = mTimerVals[MOT_2];
    if (tick > 255) {
        tick -= 255;
        if (tick < 20) {
            goto trigger;
        } else if (tick <= 255) {
            OCR2A = tick;
            TCNT2 = 0;
        }
        mTimerVals[MOT_2] = tick;
        return;
    }

trigger:
    SET(PORT_MOT_STEP, BIT_MOT_2_STEP);
    u16 tickOrg = mTimerValOrgs[MOT_2];
    if (tickOrg > 255) {
        OCR2A = 255;
        TCNT2 = 0;
        mTimerVals[MOT_2] = tickOrg;
    } else {
        OCR2A = tickOrg;
        TCNT2 = 0;
    }
    CLR(PORT_MOT_STEP, BIT_MOT_2_STEP);
}


// tspeed could be positive or negative (reverse)
void setMotorSpeed(s8 mot, s16 tspeed)
{
    long timer_period;
    s16  speed;

    // WE LIMIT MAX ACCELERATION of the motors
    if ((mSpeeds[mot] - tspeed) > MAX_ACCEL)
        mSpeeds[mot] -= MAX_ACCEL;
    else if ((mSpeeds[mot] - tspeed) < -MAX_ACCEL)
        mSpeeds[mot] += MAX_ACCEL;
    else
        mSpeeds[mot] = tspeed;

#if MICROSTEPPING==16
    speed = mSpeeds[mot] * 46;                  // Adjust factor from control output speed to real motor speed in steps/second
#else
    speed = mSpeeds[mot] * 23;                  // 1/8 Microstepping
#endif

    if (speed == 0) {
        timer_period = ZERO_SPEED;
        mDirs[mot] = 0;
    } else if (speed > 0) {
        timer_period = CLK_PERIOD / speed;
        mDirs[mot] = 1;
        if (mot == MOT_1)
            SET(PORT_MOT_DIR, BIT_MOT_1_DIR);   // DIR Motor 1 (Forward)
        else
            CLR(PORT_MOT_DIR, BIT_MOT_2_DIR);   // DIR Motor 2 (Forward)
    } else {
        timer_period = CLK_PERIOD / -speed;
        mDirs[mot] = -1;
        if (mot == MOT_1)
            CLR(PORT_MOT_DIR, BIT_MOT_1_DIR);   // DIR Motor 1 (backward)
        else
            SET(PORT_MOT_DIR, BIT_MOT_2_DIR);   // DIR Motor 2 (backward)
    }

    if (timer_period > 65536)                   // Check for minimun speed (maximun period without overflow)
        timer_period = ZERO_SPEED;

    if (mot == MOT_1) {
        TIMSK1 &= ~BV(OCIE1A);
        mTimerVals[mot] = timer_period;
        mTimerValOrgs[mot] = timer_period;
        if (timer_period <= 255) {
            OCR1A = (u8)timer_period;
            if (TCNT1 > OCR1A)
                TCNT1 = 0;
        } else {
            OCR1A = 255;
            TCNT1 = 0;
        }
        TIMSK1 |= BV(OCIE1A);
    } else {
        TIMSK2 &= ~BV(OCIE2A);
        mTimerVals[mot] = timer_period;
        mTimerValOrgs[mot] = timer_period;
        if (timer_period <= 255) {
            OCR2A = (u8)timer_period;
            if (TCNT2 > OCR2A)
                TCNT2 = 0;
        } else {
            OCR2A = 255;
            TCNT2 = 0;
        }
        TIMSK2 |= BV(OCIE2A);
    }

    //LOG(F("MOT:%2d, REQ_SPEED:%3d, SPEED=%5d, PERIOD:%8ld\n"), mot, tspeed, speed, mTimerVals[mot]);
}


u8 mAuxBtn = 0;

s8 inputCallback(u8 cmd, u8 *data, u8 size, u8 *res)
{
    u16 *rc;
    u16 val;
    s8  ret = -1;

    switch (cmd) {

        case SerialProtocol::MSP_ANALOG:
            res[0] = 120;
            ret = 7;
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

            // AUX1 - AUX4
            val = 0;
            for (u8 i = 0; i < 4; i++) {
                if (*rc++ > 1700)
                    val |= BV(i);
            }
            if (val != mAuxBtn) {
                if (val & BV(0)) {
                    mMaxThrottle = MAX_THROTTLE_PRO;
                    mMaxSteering = MAX_STEERING_PRO;
                    mMaxTargetAngle = MAX_TARGET_ANGLE_PRO;
                } else {
                    mMaxThrottle = MAX_THROTTLE;
                    mMaxSteering = MAX_STEERING;
                    mMaxTargetAngle = MAX_TARGET_ANGLE;
                }
                mAuxBtn = val;
            }
            break;
    }

    return ret;
}


#define CONFIG_VBAT_SMOOTH      16

u16 mVoltBuf[CONFIG_VBAT_SMOOTH];
u16 mVoltSum;
u8  mVoltIdx;

u8 getBattVolt(void)
{
    u16 v;

    v = analogRead(PIN_ANALOG_VOLT);

    mVoltSum += v;
    mVoltSum -= mVoltBuf[mVoltIdx];
    mVoltBuf[mVoltIdx++] = v;
    mVoltIdx %= CONFIG_VBAT_SMOOTH;

    u8 t = map(mVoltSum / CONFIG_VBAT_SMOOTH, 0, 1023, 0, 130);
    LOG(F("ADC:%4d ==> VOLT:%4d\n"), mVoltSum, t);

    return t;
}

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

    for (u8 i = 0; i < CONFIG_VBAT_SMOOTH; i++)
        getBattVolt();

#if __STD_SERIAL__
    Serial.begin(115200);
#else
    mSerial.begin(57600);
    mSerial.registerMSPCallback(inputCallback);
#endif

    LOG(F("---- BROBOT ---- \n"));

#if !MOTOR_TEST
    LOG(F("Initializing I2C devices...\n"));
    Wire.begin();
    // I2C 400Khz fast mode
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
        // turn on the DMP, now that it's ready
        LOG(F("Enabling DMP...\n"));
        mMPU.setDMPEnabled(true);
        status = mMPU.getIntStatus();
    } else { // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        LOG(F("DMP Initialization failed (code : %d)\n"), status);
    }

    // Gyro calibration
    // The robot must be steady during initialization
    LOG(F("Gyro calibration!!  Dont move the robot in 10 seconds...\n"));
    // Time to settle things... the bias_from_no_motion algorithm needs some time to take effect and reset gyro bias.
    delay(10000);

    // Verify connection
    LOG(F("Testing device connections...\n"));
    if (mMPU.testConnection())
        LOG(F("MPU6050 connection successful\n"));
    else
        LOG(F("MPU6050 connection failed\n"));

    //Adjust sensor fusion gain
    LOG(F("Adjusting DMP sensor fusion gain...\n"));
    setSensorFusionAccelGain(0x20);
    delay(200);

    // Init servos
    LOG(F("Servo initialization...\n"));
#endif

    // STEPPER MOTORS INITIALIZATION
    LOG(F("Steper motors initialization...\n"));

    // MOTOR1 => TIMER1
    TCCR1A = 0;                                 // Timer1 CTC mode 4, OCxA,B outputs disconnected
    TCCR1B = BV(WGM12) | BV(CS11);              // Prescaler=8, => 2Mhz
    OCR1A  = ZERO_SPEED;
    mDirs[MOT_1] = 0;
    TCNT1  = 0;

    // MOTOR2 => TIMER2
    TCCR2A = 0;                                 // Timer2 CTC mode 2, OCxA,B outputs disconnected
    TCCR2B = BV(WGM21) | BV(CS21);              // Prescaler=8, => 2MHz
    OCR2A  = ZERO_SPEED;                        // Motor stopped
    mDirs[MOT_2] = 0;
    TCNT2 = 0;

    // Enable TIMERs interrupts
    TIMSK1 |= BV(OCIE1A); // Enable Timer1 interrupt
    TIMSK2 |= BV(OCIE2A); // Enable Timer2 interrupt

    ENABLE_MOTOR();
    // Little motor vibration and servo move to indicate that robot is ready
    for (u8 k = 0; k < 5; k++) {
        setMotorSpeed(MOT_1, 5);
        setMotorSpeed(MOT_2, -5);
        delay(200);
        setMotorSpeed(MOT_1, -5);
        setMotorSpeed(MOT_2, 5);
        delay(200);
    }
    setMotorSpeed(MOT_1, 0);
    setMotorSpeed(MOT_2, 0);

    LOG(F("Let's start...\n"));
#if !MOTOR_TEST
    mMPU.resetFIFO();
#endif

    mLastTS = millis() - 1;
    mIsShutdown = false;
}

void loop()
{
    float       fDelta;

#if MOTOR_TEST
    s16         motorSpeeds[2];


    if (mSerial.available()) {
        u8  ch = mSerial.read();

        switch (ch) {
            case '+':
                motorSpeeds[MOT_1] = constrain(motorSpeeds[MOT_1] + 5, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
                setMotorSpeed(MOT_1, motorSpeeds[MOT_1]);
                break;

            case '-':
                motorSpeeds[MOT_1] = constrain(motorSpeeds[MOT_1] - 5, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
                setMotorSpeed(MOT_1, motorSpeeds[MOT_1]);
                break;

            case 'a':
                motorSpeeds[MOT_2] = constrain(motorSpeeds[MOT_2] + 5, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
                setMotorSpeed(MOT_2, motorSpeeds[MOT_2]);
                break;

            case 'z':
                motorSpeeds[MOT_2] = constrain(motorSpeeds[MOT_2] - 5, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
                setMotorSpeed(MOT_2, motorSpeeds[MOT_2]);
                break;
        }
        LOG(F("MOTOR1:%4d, MOTOR2:%4d\n"), motorSpeeds[MOT_1], motorSpeeds[MOT_2]);
    }

    getBattVolt();

    return;
#endif


#if !__STD_SERIAL__
    mSerial.handleMSP();
#endif

    mCurTS = millis();
    // New DMP Orientation solution?
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
        if ((mAngleAdjusted < 76) && (mAngleAdjusted > -76)) { // Is robot ready (upright?)
            // NORMAL MODE
            ENABLE_MOTOR();
            setMotorSpeed(MOT_1, mMotors[MOT_1]);
            setMotorSpeed(MOT_2, mMotors[MOT_2]);

            // Normal condition?
            if ((mAngleAdjusted < 45) && (mAngleAdjusted > -45)) {
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
        } else {   // Robot not ready (flat), angle > 70º => ROBOT OFF
            DISABLE_MOTOR();
            setMotorSpeed(MOT_1, 0);
            setMotorSpeed(MOT_2, 0);
            mPIDErrSum = 0;  // Reset PID I term
            mP = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
            mD = KD_RAISEUP;
            mThrP = KP_THROTTLE_RAISEUP;
            mThrI = KI_THROTTLE_RAISEUP;

#if 0
            // if we pulse push1 button we raise up the robot with the servo arm
            if (OSC.push1) {
                // Because we know the robot orientation (face down of face up), we move the servo in the appropiate direction for raise up
                if (angle_adjusted > 0)
                    BROBOT.moveServo1(SERVO_MIN_PULSEWIDTH);
                else
                    BROBOT.moveServo1(SERVO_MAX_PULSEWIDTH);
            } else
                BROBOT.moveServo1(SERVO_AUX_NEUTRO);
#endif
        }
        //readControlParameters();
    } // End of new IMU data

#if BATTERY_CHECK
    if (mCurTS - mLastBattTS > 500) {
        mLastBattTS = mCurTS;
    }
#endif
}

