#pragma once
// Wheel odometry in the odom frame. Same arithmetic as before the refactor:
// distance from encoder ticks, direction = wheel angle rotated by the body heading.
#include <stdint.h>

class Odometry {
public:
    // One control tick. Returns the distance driven this tick (m).
    // While steering nothing moves, so the integrator only re-syncs.
    float update(bool driving, int64_t ticks, float bodyHeadingDeg, float steerDeg);
    void resetPosition();          // the current spot becomes (0, 0)

    float x() const { return x_; }
    float y() const { return y_; }
    float theta() const { return theta_; }
    float vx() const { return vx_; }
    float wz() const { return wz_; }

private:
    bool initialized_ = false;
    float x_ = 0.0f, y_ = 0.0f, theta_ = 0.0f, prevTheta_ = 0.0f;
    float vx_ = 0.0f, wz_ = 0.0f;
    int64_t lastTicks_ = 0;
};
