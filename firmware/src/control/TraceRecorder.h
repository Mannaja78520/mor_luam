#pragma once
// The last 15 s of the control loop at 100 Hz, for tuning the PIDF on the real
// robot (WiFi polling is far too slow to see a steer step). GET /api/trace
// returns the window as CSV; web/js or a PC script plots it.
//
// record() runs in the control task under ControlLoop's lock; the web task
// freezes the buffer under the same lock, reads it, then unfreezes. A reader
// that vanishes mid-way is forgiven after 5 s.
#include <Arduino.h>
#include "control/RobotState.h"

class TraceRecorder {
public:
    struct Sample {
        uint32_t ms;
        int16_t steer10, target10;   // wheel angle and its target vs the body, 0.1 deg
        int16_t rateDps;             // steering speed
        int16_t pwm;
        int16_t rpm10, targetRpm10;  // 0.1 rpm
        uint16_t driveGain1000;     // learned drive multiplier, 0.001
        int16_t xMm, yMm;            // odometry
        uint8_t flags;               // 1 steer, 2 drive (0 = halted), 4 coasting, 8 overshot
        uint16_t imuYaw100, gyroSeq, accelSeq;
        int16_t gyro10[3], accel100[3]; // 0.1 deg/s and 0.01 m/s^2
        uint8_t imuFlags;
    };
    static const uint16_t N = 1500;

    bool begin() {
        buf_ = static_cast<Sample*>(malloc(sizeof(Sample) * N));
        return buf_ != nullptr;
    }

    void record(const RobotState& s) {
        if (!buf_) return;
        if (frozen_) {
            if (millis() - frozenAtMs_ < 5000) return;
            frozen_ = false;
        }
        Sample& d = buf_[head_];
        d.ms = s.stampMs;
        d.steer10 = (int16_t)lroundf(s.steerDeg * 10.0f);
        d.target10 = (int16_t)lroundf(s.steerTargetDeg * 10.0f);
        d.rateDps = (int16_t)lroundf(s.steerRateDps);
        d.pwm = (int16_t)s.pwm;
        d.rpm10 = (int16_t)lroundf(s.rpm * 10.0f);
        d.targetRpm10 = (int16_t)lroundf(s.targetRpm * 10.0f);
        d.driveGain1000 = (uint16_t)lroundf(s.driveGain * 1000.0f);
        d.xMm = (int16_t)lroundf(s.x * 1000.0f);
        d.yMm = (int16_t)lroundf(s.y * 1000.0f);
        d.flags = (s.halted ? 0 : (s.driving ? 2 : 1)) | (s.coasting ? 4 : 0) | (s.overshot ? 8 : 0);
        d.imuYaw100 = (uint16_t)lroundf(s.imuYawDeg * 100.0f);
        d.gyroSeq = s.imuGyroSeq; d.accelSeq = s.imuAccelSeq; d.imuFlags = s.imuMotionFlags;
        for (unsigned axis = 0; axis < 3; ++axis) {
            d.gyro10[axis] = (int16_t)lroundf(constrain(s.imuGyroDps[axis] * 10.0f, -32767.0f, 32767.0f));
            d.accel100[axis] = (int16_t)lroundf(constrain(s.imuAccelMps2[axis] * 100.0f, -32767.0f, 32767.0f));
        }
        head_ = (head_ + 1) % N;
        if (count_ < N) ++count_;
    }

    // Stop recording and select the last lastMs; returns how many samples.
    uint16_t freeze(uint32_t lastMs) {
        frozen_ = true;
        frozenAtMs_ = millis();
        if (!buf_ || !count_) return sel_ = 0;
        const uint32_t newest = buf_[(head_ + N - 1) % N].ms;
        sel_ = 0;
        while (sel_ < count_ && newest - buf_[(head_ + N - 1 - sel_) % N].ms <= lastMs) ++sel_;
        return sel_;
    }
    // i = 0 is the oldest of the selected window
    const Sample& at(uint16_t i) const { return buf_[(head_ + N - sel_ + i) % N]; }
    void unfreeze() { frozen_ = false; }

private:
    Sample* buf_ = nullptr;
    uint16_t head_ = 0, count_ = 0, sel_ = 0;
    volatile bool frozen_ = false;
    uint32_t frozenAtMs_ = 0;
};
