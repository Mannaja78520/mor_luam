#pragma once
// Missing feedback is a fault, not proof of an emergency switch or ground contact.
// A lifted drive wheel can still produce encoder ticks. No motor probes run here:
// this watches only motion that the operator has already commanded.
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

class MotorResponseWatch {
public:
    enum Fault { None, SteerNoResponse, DriveNoResponse };
    void reset() { fault_ = None; mode_ = 0; elapsed_ = 0; }
    Fault fault() const { return fault_; }
    bool observe(int mode, float pwm, int32_t ticks, float angle, float dt) {
        if (fault_ != None) return true;
        if (mode != mode_) { mode_ = mode; elapsed_ = 0; ticks_ = ticks; angle_ = angle; }
        if (mode == 0 || fabsf(pwm) < 300.0f) {
            elapsed_ = 0; ticks_ = ticks; angle_ = angle; return false;
        }
        const float turn = fabsf(fmodf(angle - angle_ + 540.0f, 360.0f) - 180.0f);
        const bool responded = mode == 2 ? llabs((long long)ticks - ticks_) >= 2 : turn >= 1.0f;
        if (responded) { elapsed_ = 0; ticks_ = ticks; angle_ = angle; }
        else elapsed_ += dt;
        if (elapsed_ >= 1.5f) fault_ = mode == 2 ? DriveNoResponse : SteerNoResponse;
        return fault_ != None;
    }
private:
    Fault fault_ = None;
    int mode_ = 0;
    float elapsed_ = 0, angle_ = 0;
    int32_t ticks_ = 0;
};
