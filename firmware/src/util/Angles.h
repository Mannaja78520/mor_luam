#pragma once
// Angle helpers shared by the controller, the odometry and the planners.
// Pure functions: no Arduino, no state.
#include <math.h>

namespace angles {

inline float wrap360(float a) { float x = fmodf(a, 360.0f); return x < 0 ? x + 360.0f : x; }

// Signed error in [-180, 180): used to see if the wheel drifted while driving.
inline float errDeg(float target, float current) {
    return fmodf((wrap360(target) - wrap360(current) + 540.0f), 360.0f) - 180.0f;
}

// How far the wheel must still turn to reach target, 0..360, in the ONE
// direction it steers. mor_luam steers clockwise seen from above, which
// DEcreases the angle in the CCW frame used everywhere (IMU, odometry, web).
// Found on the robot 2026-10-08: with (target - current) a +90 deg steer went
// 268 deg the long way, every time (the original firmware did the same).
inline float cwErrorDeg(float target, float current) { return fmodf(current - target + 360.0f, 360.0f); }

// Close enough on either side. Just past the target reads as ~360.
inline bool cwInTolerance(float e, float tol) { return (e <= tol) || (e >= (360.0f - tol)); }

inline float wrapPi(float a) {
    float x = fmodf(a + (float)M_PI, 2.0f * (float)M_PI);
    return x < 0.0f ? x + 2.0f * (float)M_PI : x - (float)M_PI;
}

inline float deg2rad(float d) { return d * ((float)M_PI / 180.0f); }
inline float rad2deg(float r) { return r * (180.0f / (float)M_PI); }

}  // namespace angles
