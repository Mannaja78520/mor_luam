#pragma once
// A copy of everything the robot knows, taken once per control tick.
// The web page and the ROS bridge read this copy, never the live objects.
#include <stdint.h>
#include "control/DriveCommand.h"

struct RobotState {
    uint32_t stampMs = 0;

    // sensors
    int32_t ticks = 0;
    float rpm = 0.0f;              // wheel rpm (signed)
    float steerDeg = 0.0f;         // wheel angle vs the body, 0..360 (AS5600)
    bool steerOk = false;
    bool steerAimed = false;       // controller's own 'inside the steering tolerance' (with hysteresis)
    uint32_t steerGlitches = 0;    // AS5600 readings ignored as impossible jumps (SteerSensor)
    float imuYawDeg = 0.0f;        // relative to the reference taken at the first command
    bool imuOk = false;
    bool imuHeadingFresh = false; // finite orientation report received within 100 ms
    float imuGyroDps[3] = {}, imuAccelMps2[3] = {}; // sensor axes; acceleration excludes gravity
    uint16_t imuGyroSeq = 0, imuAccelSeq = 0;
    uint8_t imuMotionFlags = 0;    // bit 1 gyro fresh, bit 2 linear acceleration fresh (<=100 ms)
    float headingDeg = 0.0f;       // body heading used by the controller
    float wheelHeadingDeg = 0.0f;  // where the wheel points in the world

    // odometry (odom frame, metres)
    float x = 0.0f, y = 0.0f, thetaRad = 0.0f;
    float vx = 0.0f, wz = 0.0f;

    // controller
    bool driving = false;          // false = STEER_TO_HEADING, true = DRIVE
    bool halted = true;            // motor held at 0 until the next command
    bool overshot = false;         // halted because the wheel turned past its angle
    float overshootDeg = 0.0f;     // by how much (known only once it happened)
    float targetRpm = 0.0f;
    float targetHeadingDeg = 0.0f;
    float steerTargetDeg = 0.0f;   // wanted wheel angle vs the body
    float steerErrDeg = 0.0f;      // signed
    float steerRateDps = 0.0f;     // how fast the wheel is steering
    bool coasting = false;         // power cut early, wheel coasting into its angle
    float coastS = 0.0f;           // learned coast time (SteerStopPredictor)
    unsigned coastSamples = 0;
    float driveGain = 1.0f;        // learned multiplier for KS + KF * rpm (no battery ADC)
    float driveLearnedS = 0.0f;    // steady driving used to learn, since boot/model change
    uint8_t motionFault = 0;      // 0 none, 1 steering no response, 2 drive no response
    int pwm = 0;
    bool goalActive = false;
    float goalX = 0.0f, goalY = 0.0f;
    CommandSource source = CommandSource::None;
};
