#pragma once
// Smooth the FINAL steering PWM including its base power. A zero request cuts
// immediately, so E-STOP/coast/goal stops never wait for a ramp.
#include <math.h>
class SteerPowerRamp {
public:
    float step(float requested, float dt) {
        if (requested <= 0) { reset(); return 0; }
        const float limit = (requested > power_ ? 1500.0f : 4000.0f) * dt;
        power_ += fmaxf(-limit, fminf(limit, requested - power_));
        return power_;
    }
    void reset() { power_ = 0; }
private:
    float power_ = 0;
};
