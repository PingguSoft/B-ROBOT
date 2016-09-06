#include <Arduino.h>
#include <stdarg.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <JJ_MPU6050_DMP_6Axis.h>  // Modified version of the MPU6050 library to work with DMP (see comments inside)

#include "common.h"
#include "utils.h"

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/

// PINS
#define PORT_MOT_STEP           PORTC
#define PORT_MOT_DIR            PORTC

#define PIN_MOT_ENABLE          10

#define PIN_MOT_L_STEP          A0
#define BIT_MOT_L_STEP          0

#define PIN_MOT_L_DIR           A1
#define BIT_MOT_L_DIR           1

#define PIN_MOT_R_DIR           A2
#define BIT_MOT_R_DIR           2

#define PIN_MOT_R_STEP          A3
#define BIT_MOT_R_STEP          3

#define PIN_LED                 13


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
volatile u32 mMillis = 0;

bool        mIsShutdown = false; // Robot shutdown flag => Out of

// MPU control/status vars
bool        mIsDMPReady = false;    // set true if DMP init was successful
u8          mMpuIntStatus;          // holds actual interrupt status byte from MPU
u8          mDevStatus;             // return status after each device operation (0 = success, !0 = error)
u16         mPacketSize;            // expected DMP packet size (for us 18 bytes)
u16         mFifoCount;             // count of all bytes currently in FIFO
u8          mFifoBufs[18];          // FIFO storage buffer
Quaternion  mQ;

u8          mLoop40Hz;              // To generate a medium loop 40Hz
u8          mLoop2Hz;               // slow loop 2Hz
u8          mBattCtr;               // To send battery status

u32         mOldTime;
u32         mCurTime;
int         mDebugCtr;
float       mDebugVar;


// class default I2C address is 0x68 for MPU6050
MPU6050     mMPU;

// Angle of the robot (used for stability control)
float       mAngleAdjusted;
float       mAngleAdjustedOld;

// Default control values from constant definitions
float       Kp = KP;
float       Kd = KD;
float       KpThr = KP_THROTTLE;
float       KiThr = KI_THROTTLE;
float       KpUser = KP;
float       KdUser = KD;
float       KpThrUser = KP_THROTTLE;
float       KiThrUser = KI_THROTTLE;
bool        mIsNewParam = false;
bool        mIsModifyingParam = false;

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

u8          mMode = 0;  // mMode = 0 Normal mMode, mMode = 1 Pro mMode (More agressive)

volatile u16 mTicks[2];
volatile u16 mTicksOrg[2];
s16         mMotors[2];
s16         mSpeeds[2];         // Actual speed of motors
volatile s8 mDirs[2];           // Actual direction of steppers motors
s16         mActSpeed;          // overall robot speed (measured from steppers speed)
s16         mActSpeedOld;
float       mEstSpeedFiltered;  // Estimated robot speed


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
    mMPU.getFIFOBytes(mFifoBufs, 16); // We only read the quaternion
    mMPU.dmpGetQuaternion(&mQ, mFifoBufs);
    mMPU.resetFIFO();  // We always reset FIFO

    //return( asin(-2*(mQ.x * mQ.z - mQ.w * mQ.y)) * 180/M_PI); //roll
    //return Phi angle (robot orientation) from quaternion DMP output
    return (atan2(2 * (mQ.y * mQ.z + mQ.w * mQ.x), mQ.w * mQ.w - mQ.x * mQ.x - mQ.y * mQ.y + mQ.z * mQ.z) * RAD2GRAD);
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
    //Serial.print(d*(error-mPIDErrOld));Serial.print("\t");
    mPIDErrOld2 = mPIDErrOld;
    mPIDErrOld = input;  // error for Kd is only the input component
    mSetPointOld = setPoint;
    return (output);
}


// PI controller implementation (Proportional, integral). DT is in miliseconds
float getSpeedPIControlOutput(float DT, float input, float setPoint,  float Kp, float Ki)
{
    float error;
    float output;

    error = setPoint - input;
    mPIDErrSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
    mPIDErrSum = constrain(mPIDErrSum, -ITERM_MAX, ITERM_MAX);

    //Serial.println(mPIDErrSum);

    output = Kp * error + Ki * mPIDErrSum * DT * 0.001; // DT is in miliseconds...
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

ISR(TIMER0_COMPA_vect)
{
    if (mDirs[MOT_1] == 0) {
        return;
    }

    u16 tick = mTicks[MOT_1];
    if (tick > 255) {
        tick -= 255;
        if (tick < 20) {
            goto trigger;
        } else if (tick <= 255) {
            OCR0A = tick;
            TCNT0 = 0;
        }
        mTicks[MOT_1] = tick;
        return;
    }

trigger:
    SET(PORT_MOT_STEP, BIT_MOT_L_STEP);
    delay_1us();
    CLR(PORT_MOT_STEP, BIT_MOT_L_STEP);

    u16 tickOrg = mTicksOrg[MOT_1];
    if (tickOrg > 255) {
        OCR0A = 255;
        TCNT0 = 0;
        mTicks[MOT_1] = tickOrg;
    } else {
        OCR0A = tickOrg;
        TCNT0 = 0;
    }
}

ISR(TIMER2_COMPA_vect)
{
    if (mDirs[MOT_2] == 0) {
        return;
    }

    u16 tick = mTicks[MOT_2];
    if (tick > 255) {
        tick -= 255;
        if (tick < 20) {
            goto trigger;
        } else if (tick <= 255) {
            OCR2A = tick;
            TCNT2 = 0;
        }
        mTicks[MOT_2] = tick;
        return;
    }

trigger:
    SET(PORT_MOT_STEP, BIT_MOT_R_STEP);
    delay_1us();
    CLR(PORT_MOT_STEP, BIT_MOT_R_STEP);

    u16 tickOrg = mTicksOrg[MOT_2];
    if (tickOrg > 255) {
        OCR2A = 255;
        TCNT2 = 0;
        mTicks[MOT_2] = tickOrg;
    } else {
        OCR2A = tickOrg;
        TCNT2 = 0;
    }
}

ISR(TIMER1_COMPA_vect)
{
    mMillis++;
}

u32 getMillis()
{
    return mMillis;
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
            SET(PORT_MOT_DIR, BIT_MOT_L_DIR);   // DIR Motor 1 (Forward)
        else
            CLR(PORT_MOT_DIR, BIT_MOT_R_DIR);   // DIR Motor 2 (Forward)
    } else {
        timer_period = CLK_PERIOD / -speed;
        mDirs[mot] = -1;
        if (mot == MOT_1)
            CLR(PORT_MOT_DIR, BIT_MOT_L_DIR);   // DIR Motor 1 (backward)
        else
            SET(PORT_MOT_DIR, BIT_MOT_R_DIR);   // DIR Motor 2 (backward)
    }

    if (timer_period > 65536)                   // Check for minimun speed (maximun period without overflow)
        timer_period = ZERO_SPEED;

    if (mot == MOT_1) {
        TIMSK0 &= ~BV(OCIE0A);
        mTicks[mot] = timer_period;
        mTicksOrg[mot] = timer_period;
        if (timer_period <= 255) {
            OCR0A = (u8)timer_period;
            if (TCNT0 > OCR0A)
                TCNT0 = 0;
        } else {
            OCR0A = 255;
            TCNT0 = 0;
        }
        TIMSK0 |= BV(OCIE0A);
    } else {
        TIMSK2 &= ~BV(OCIE2A);
        mTicks[mot] = timer_period;
        mTicksOrg[mot] = timer_period;
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

    //LOG(F("MOT:%2d, REQ_SPEED:%3d, SPEED=%5d, PERIOD:%8ld\n"), mot, tspeed, speed, mTicks[mot]);
}

void setup()
{
    // 1ms counter => TIMER1
    TCCR1A = 0;                                 // Timer1 CTC mode 4, OCxA,B outputs disconnected
    TCCR1B = BV(WGM12) | BV(CS11) | BV(CS10);   // Prescaler=64, => 250Khz
    OCR1A  = 250;
    TCNT1  = 0;
    TIMSK1 |= BV(OCIE1A);                       // Enable Timer1 interrupt

    pinMode(PIN_MOT_ENABLE, OUTPUT);
    pinMode(PIN_MOT_L_STEP, OUTPUT);
    pinMode(PIN_MOT_L_DIR, OUTPUT);
    pinMode(PIN_MOT_R_STEP, OUTPUT);
    pinMode(PIN_MOT_R_STEP, OUTPUT);
    pinMode(PIN_LED, OUTPUT);

    DISABLE_MOTOR();
    digitalWrite(PIN_LED, HIGH);

    Serial.begin(115200);

    Wire.begin();
    // I2C 400Khz fast mMode
    TWSR = 0;
    TWBR = ((16000000L / I2C_SPEED) - 16) / 2;
    TWCR = 1 << TWEN;
    delay(200);

#if 1
    LOG(F("---- BROBOT ---- \n"));
    LOG(F("Initializing I2C devices...\n"));

    mMPU.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    mMPU.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mMPU.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mMPU.setDLPFMode(MPU6050_DLPF_BW_10);       // 10,20,42,98,188  // Default factor for BROBOT:10
    mMPU.setRate(4);                            // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
    mMPU.setSleepEnabled(false);
    delay(500);

    LOG(F("Initializing DMP...\n"));
    mDevStatus = mMPU.dmpInitialize();
    if (mDevStatus == 0) {
        // turn on the DMP, now that it's ready
        LOG(F("Enabling DMP...\n"));
        mMPU.setDMPEnabled(true);
        mMpuIntStatus = mMPU.getIntStatus();
        mIsDMPReady = true;
    } else { // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        LOG(F("DMP Initialization failed (code : %d)\n"), mDevStatus);
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

    // MOTOR1 => TIMER0
    TCCR0A = 0;                                 // Timer0 CTC mMode 2, OCxA,B outputs disconnected
    TCCR0B = BV(WGM01) | BV(CS01);              // Prescaler=8, => 2MHz
    OCR0A = ZERO_SPEED;                         // Motor stopped
    mDirs[MOT_1] = 0;
    TCNT0 = 0;

    // MOTOR2 => TIMER2
    TCCR2A = 0;                                 // Timer2 CTC mMode 2, OCxA,B outputs disconnected
    TCCR2B = BV(WGM21) | BV(CS21);              // Prescaler=8, => 2MHz
    OCR2A = ZERO_SPEED;                         // Motor stopped
    mDirs[MOT_2] = 0;
    TCNT2 = 0;

    ENABLE_MOTOR();
    // Enable TIMERs interrupts
    TIMSK0 |= BV(OCIE0A); // Enable Timer0 interrupt
    TIMSK2 |= BV(OCIE2A); // Enable Timer2 interrupt

    // Little motor vibration and servo move to indicate that robot is ready
    for (u8 k = 0; k < 5; k++) {
        setMotorSpeed(MOT_1, 5);
        setMotorSpeed(MOT_2, 5);
        delay(200);
        setMotorSpeed(MOT_1, -5);
        setMotorSpeed(MOT_2, -5);
        delay(200);
    }

    LOG(F("Let's start...\n"));
    mMPU.resetFIFO();
    mOldTime = getMillis() - 1;
    mIsShutdown = false;
    mMode = 0;
}


s16 mMotSpd[2];

void loop()
{
    float       delta;

#if SHUTDOWN_WHEN__OFF==1
    if (mIsShutdown)
        return;
#endif

    mDebugCtr++;


#if MOTOR_TEST
    if (Serial.available()) {
        u8  ch = Serial.read();

        switch (ch) {
            case '+':
                mMotSpd[MOT_1] = constrain(mMotSpd[MOT_1] + 5, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
                setMotorSpeed(MOT_1, mMotSpd[MOT_1]);
                break;

            case '-':
                mMotSpd[MOT_1] = constrain(mMotSpd[MOT_1] - 5, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
                setMotorSpeed(MOT_1, mMotSpd[MOT_1]);
                break;

            case 'a':
                mMotSpd[MOT_2] = constrain(mMotSpd[MOT_2] + 5, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
                setMotorSpeed(MOT_2, mMotSpd[MOT_2]);
                break;

            case 'z':
                mMotSpd[MOT_2] = constrain(mMotSpd[MOT_2] - 5, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
                setMotorSpeed(MOT_2, mMotSpd[MOT_2]);
                break;
        }
        LOG(F("MOTOR1:%4d, MOTOR2:%4d\n"), mMotSpd[MOT_1], mMotSpd[MOT_2]);
    }
    return;
#endif


#if 0
    OSC.MsgRead();  // Read UDP OSC messages
    if (OSC.newMessage) {
        OSC.newMessage = 0;
        if (OSC.page == 1) {  // Get commands from user (PAGE1 are user commands: throttle, steering...)
            OSC.newMessage = 0;
            throttle = (OSC.fadder1 - 0.5) * max_throttle;
            // We add some exponential on steering to smooth the center band
            steering = OSC.fadder2 - 0.5;
            if (steering > 0)
                steering = (steering * steering + 0.5 * steering) * max_steering;
            else
                steering = (-steering * steering + 0.5 * steering) * max_steering;

            modifing_control_parameters = false;
            if ((mode == 0) && (OSC.toggle1)) {
                // Change to PRO mode
                max_throttle = MAX_THROTTLE_PRO;
                max_steering = MAX_STEERING_PRO;
                max_target_angle = MAX_TARGET_ANGLE_PRO;
                mode = 1;
            }
            if ((mode == 1) && (OSC.toggle1 == 0)) {
                // Change to NORMAL mode
                max_throttle = MAX_THROTTLE;
                max_steering = MAX_STEERING;
                max_target_angle = MAX_TARGET_ANGLE;
                mode = 0;
            }
        }
    } // End new OSC message
#endif


#if 1
    KpUser = KP * 2 * 0.5;
    KdUser = KD * 2 * 0.5;
    KpThrUser = KP_THROTTLE * 2 * 0.5;
    KiThrUser = (KI_THROTTLE + 0.1) * 2 * 0.0;
#endif


#if 1
    mCurTime = getMillis();

    // New DMP Orientation solution?
    mFifoCount = mMPU.getFIFOCount();
    if (mFifoCount >= 18) {
        // If we have more than one packet we take the easy path: discard the buffer and wait for the next one
        if (mFifoCount > 18) {
            LOG(F("FIFO RESET!!\n"));
            mMPU.resetFIFO();
            return;
        }
        mLoop40Hz++;
        mLoop2Hz++;
        delta = (mCurTime - mOldTime);
        mOldTime = mCurTime;

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
        mTargetAngle = getSpeedPIControlOutput(delta, mEstSpeedFiltered, mThrottle, KpThr, KiThr);
        mTargetAngle = constrain(mTargetAngle, -mMaxTargetAngle, mMaxTargetAngle); // limited output
        //LOG(F("AngleAdjusted:%f, EstSpeedFiltered:%f, TargetAngle:%f\n"), mAngleAdjusted, mEstSpeedFiltered, mTargetAngle);

        // Stability control: This is a PD controller.
        //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
        //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
        mControlOutput += getStabilityPDControlOutput(delta, mAngleAdjusted, mTargetAngle, Kp, Kd);
        mControlOutput = constrain(mControlOutput, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

        // The steering part from the user is injected directly on the output
        mMotors[MOT_1] = mControlOutput + mSteering;
        mMotors[MOT_2] = mControlOutput - mSteering;

        // Limit max speed (control output)
        mMotors[MOT_1] = constrain(mMotors[MOT_1], -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
        mMotors[MOT_2] = constrain(mMotors[MOT_2], -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

#if 1
        Serial.print(mCurTime);
        Serial.print("  AA:");
        Serial.print(mAngleAdjusted);
        Serial.print(", TA:");
        Serial.print(mTargetAngle);
        Serial.print(", ES:");
        Serial.print(mEstSpeedFiltered);
        Serial.print(", CO:");
        Serial.print(mControlOutput);
        Serial.print(", DT:");
        Serial.print(delta);
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

#if 0
            // Push1 Move servo arm
            if (OSC.push1) {  // Move arm
                // Update to correct bug when the robot is lying backward
                if (angle_adjusted > -40)
                    BROBOT.moveServo1(SERVO_MIN_PULSEWIDTH + 100);
                else
                    BROBOT.moveServo1(SERVO_MAX_PULSEWIDTH + 100);
                }
            else
                BROBOT.moveServo1(SERVO_AUX_NEUTRO);

            // Push2 reset controls to neutral position
            if (OSC.push2) {
                OSC.fadder1 = 0.5;
                OSC.fadder2 = 0.5;
            }
#endif

            // Normal condition?
            if ((mAngleAdjusted < 45) && (mAngleAdjusted > -45)) {
                Kp = KpUser;            // Default user control gains
                Kd = KdUser;
                KpThr = KpThrUser;
                KiThr = KiThrUser;
            } else {   // We are in the raise up procedure => we use special control parameters
                Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
                Kd = KD_RAISEUP;
                KpThr = KP_THROTTLE_RAISEUP;
                KiThr = KI_THROTTLE_RAISEUP;
            }
        } else {   // Robot not ready (flat), angle > 70º => ROBOT OFF
            DISABLE_MOTOR();
            setMotorSpeed(MOT_1, 0);
            setMotorSpeed(MOT_2, 0);
            mPIDErrSum = 0;  // Reset PID I term
            Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
            Kd = KD_RAISEUP;
            KpThr = KP_THROTTLE_RAISEUP;
            KiThr = KI_THROTTLE_RAISEUP;

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

    // Medium loop 40Hz
    if (mLoop40Hz >= 5) {
        mLoop40Hz = 0;
        // We do nothing here now...

    } // End of medium loop


    if (mLoop2Hz >= 99) { // 2Hz
        mLoop2Hz = 0;

#if BATTERY_CHECK==1
        int BatteryValue = BROBOT.readBattery();
        mBattCtr++;

        if (mBattCtr >= 10) { //Every 5 seconds we send a message
            mBattCtr=0;
#if LIPOBATT==0
            // From >10.6 volts (100%) to 9.2 volts (0%) (aprox)
            float value = constrain((BatteryValue-92)/14.0,0.0,1.0);
#else
            // For Lipo battery use better this config: (From >11.5v (100%) to 9.5v (0%)
            float value = constrain((BatteryValue-95)/20.0,0.0,1.0);
#endif
            //Serial.println(value);
            OSC.MsgSend("/1/rotary1\0\0,f\0\0\0\0\0\0",20,value);
        }
#endif


#if DEBUG==6
        Serial.print("B:");
        Serial.println(BatteryValue);
#endif

#if SHUTDOWN_WHEN_BATTERY_OFF==1
        if (BROBOT.readBattery(); < BATTERY_SHUTDOWN) {
            // Robot shutdown !!!
            Serial.println("LOW BAT!! SHUTDOWN");
            mIsShutdown = true;
            // Disable steppers
            DISABLE_MOTOR();
        }
#endif
    }  // End of slow loop
#endif

}

