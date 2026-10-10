#include "control/Odometry.h"
#include <math.h>
#include <stdlib.h>
#include <config.h>
#include "util/Angles.h"
#include "app_config.h"

#ifndef ODOM_TICKS_SIGN
#define ODOM_TICKS_SIGN (+1.0f)
#endif

static const float TICKS_TO_WHEEL_REV = 1.0f / ((float)COUNTS_PER_REV * MOTOR_ENCODER_RATIO);
static const float TICKS_TO_METERS = ((float)M_PI * WHEEL_DIAMETER * TICKS_TO_WHEEL_REV * (float)ODOM_TICKS_SIGN);
// A jump bigger than 20 wheel turns in one tick is an encoder reset, not motion.
static const int64_t TICK_RESET_THRESHOLD = (int64_t)((float)COUNTS_PER_REV * MOTOR_ENCODER_RATIO * 20.0f);

float Odometry::update(bool driving, int64_t ticks, float bodyHeadingDeg, float steerDeg) {
    const float heading = angles::wrapPi(angles::deg2rad(bodyHeadingDeg));

    if (!driving) {                       // steering: hold position, follow the heading
        theta_ = heading;
        prevTheta_ = heading;
        lastTicks_ = ticks;
        initialized_ = false;
        vx_ = wz_ = 0.0f;
        return 0.0f;
    }
    if (!initialized_) {                  // first tick of a drive
        initialized_ = true;
        lastTicks_ = ticks;
        theta_ = heading;
        prevTheta_ = heading;
        vx_ = wz_ = 0.0f;
        return 0.0f;
    }

    const int64_t delta = ticks - lastTicks_;
    if (llabs(delta) > TICK_RESET_THRESHOLD) {   // encoder jumped: resync, no motion
        lastTicks_ = ticks;
        theta_ = heading;
        prevTheta_ = heading;
        vx_ = wz_ = 0.0f;
        return 0.0f;
    }

    lastTicks_ = ticks;
    const float dm = (float)delta * TICKS_TO_METERS;
    const float steer = angles::wrapPi(angles::deg2rad(steerDeg));
    theta_ = heading;
    const float bx = dm * cosf(steer);           // displacement in the robot frame
    const float by = dm * sinf(steer);
    const float c = cosf(heading), s = sinf(heading);
    x_ += bx * c - by * s;                       // rotate into the odom frame
    y_ += bx * s + by * c;

    const float dTheta = angles::wrapPi(theta_ - prevTheta_);
    prevTheta_ = theta_;
    vx_ = dm / CTRL_PERIOD_S;
    wz_ = dTheta / CTRL_PERIOD_S;
    return dm;
}

void Odometry::resetPosition() { setPosition(0.0f, 0.0f); }

void Odometry::setPosition(float x, float y) {
    x_ = x;
    y_ = y;
}
