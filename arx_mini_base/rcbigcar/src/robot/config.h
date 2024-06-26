#ifndef __ONEBOT_CONFIG__
#define __ONEBOT_CONFIG__

#include <cmath>

/* 
 * Preset Types
 */
enum MotorCloseloopType
{
    CLOSELOOP_VELOCITY,
    CLOSELOOP_POSITION
};

struct MotorPreset
{
    int PWMMaxValue = 0;
    double RPMToRad = 0;
    double TickToRad = 0;
};

struct MotorParamter
{
    MotorCloseloopType CloseloopType = CLOSELOOP_VELOCITY;

    bool DoCalibration = false;
    double CalibrateCurrent = 0.0;
};

struct MotionMotor
{
    int ID = 0;

    const MotorPreset *Preset;
    MotorParamter Paramter;
};

/* 
 * Robot Hardware Config
 */

#define ROBOT_SAMPLING_RATE 200

#define HW_MOTOR_COUNT 2
#define HW_CAN_MOTOR_ID_1 0x200
#define HW_CAN_MOTOR_ID_2 0x1FF
#define HW_CAN0_ID "can0"

const double MOTOR_CALIBRATION_THRESHOLD = 0.628;
const double MOTOR_CALIBRATION_DURATION  = 1.0;

/*
 * Different Motor Presets
 */
const MotorPreset MOTOR_M2006 = {
    10000,                                           //PWM Max Value
    (1.0 / 60.0) * 2.0 * M_PI / 36.0,                //RPM To rad/s
    (1.0 / 8192.0) * 2.0 * M_PI / 36.0               //TICK to rad
};

const MotorPreset MOTOR_M3508 = {
    16384,
    (1.0 / 60.0) * 2.0 * M_PI / (3591.0 / 187.0),
    (1.0 / 8192.0) * 2.0 * M_PI / (3591.0 / 187.0)
};

const MotorPreset MOTOR_GM3510 = {
    29000,
    (1.0 / 60.0) * 2.0 * M_PI,
    (1.0 / 8192.0) * 2.0 * M_PI
};

const MotorPreset MOTOR_M3508_NO_REDUCER = {
    16384,
    (1.0 / 60.0) * 2.0 * M_PI,
    (1.0 / 8192.0) * 2.0 * M_PI
};

/*
 * Default Motor Paramter
 */
const MotorParamter MOTOR_PARAMTER_DEFAULT = {
    CLOSELOOP_VELOCITY,
    false,
    0.0
};

/*
 * Chassis Paramters (SI Unit)
 */

#define MOTOR_CHASSIS MOTOR_M3508_NO_REDUCER
#define MOTOR_CHASSIS_ID_START 0
#define MOTOR_CHASSIS_PARAMTER MOTOR_PARAMTER_DEFAULT

const double CHASSIS_WATCHDOG_TIMEOUT = 1.0;

// Differential Kinematics
// Ref: https://globaljournals.org/GJRE_Volume14/1-Kinematics-Localization-and-Control.pdf
const double CHASSIS_D = 0.244;
const double CHASSIS_WHEEL_RADIUS = 0.03;

/*
 * Motion Paramters
 */
#define MOTION_MOTOR_COUNT 0
const double MOTION_WATCHDOG_TIMEOUT = 1.0;

const MotionMotor MOTION_MOTOR_PRESET[MOTION_MOTOR_COUNT] = {
};

#endif
