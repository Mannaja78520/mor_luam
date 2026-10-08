#pragma once
// Auto feedforward for the drive wheel: follows the battery without measuring it.
//
// The drive needs  PWM ~= gain * (KS + KF * rpm)  (curve measured 2026-10-08, see
// PIDF_config.h). As the battery runs down the same PWM gives fewer rpm, so the
// PID's I term has to make up the difference at every new drive - slowly, and
// from zero. Instead, while the wheel runs steadily at its target, compare the
// PWM really needed with the model and move `gain` a little towards the ratio.
// The next drive then starts with the right power. The robot has no battery
// voltage input, so this is the battery compensation.
//
// Plain C++ so test_host/tests.cpp checks it on a PC.
#include <math.h>

class DriveGainLearner {
public:
    explicit DriveGainLearner(float gain = 1.0f) { setGain(gain); }

    // feedforward PWM for a target rpm (> 0)
    float feedforward(float rpmSet, float ks, float kf) const {
        return rpmSet > 0.0f ? gain_ * (ks + kf * rpmSet) : 0.0f;
    }

    // Every control tick while driving. pwmOut: the PWM really applied (magnitude).
    // Learns only when steady: near the target, not saturated, after the start-up.
    void observe(float rpmSet, float rpmMeas, float pwmOut, float ks, float kf, float pwmMax, float dtS,
                 float sinceDriveS) {
        if (!isfinite(rpmSet) || !isfinite(rpmMeas) || !isfinite(pwmOut) ||
            !isfinite(ks) || !isfinite(kf) || !isfinite(pwmMax) ||
            !isfinite(dtS) || !isfinite(sinceDriveS) || dtS <= 0.0f) return;
        if (rpmSet <= 0.5f || sinceDriveS < SETTLE_S) return;
        if (fabsf(rpmMeas - rpmSet) > 0.1f * rpmSet + 0.2f) return;   // not at the target yet
        if (pwmOut <= 0.0f || pwmOut >= pwmMax - 1.0f) return;          // saturated: says nothing
        const float model = ks + kf * rpmSet;
        if (model <= 1.0f) return;
        float ratio = pwmOut / model;
        if (ratio < MIN_GAIN) ratio = MIN_GAIN;
        if (ratio > MAX_GAIN) ratio = MAX_GAIN;
        const float alpha = fminf(dtS / TAU_S, 1.0f);
        gain_ += (ratio - gain_) * alpha;                            // slow first-order follow
        steadyS_ += dtS;
    }

    float gain() const { return gain_; }
    void setGain(float g) {
        gain_ = (g >= MIN_GAIN && g <= MAX_GAIN) ? g : 1.0f;
        steadyS_ = 0.0f;
    }
    float learnedS() const { return steadyS_; }   // seconds of steady driving learned from

    static constexpr float SETTLE_S = 1.0f;   // ignore the start of every drive
    static constexpr float TAU_S = 4.0f;      // ~4 s of steady driving to follow a change
    static constexpr float MIN_GAIN = 0.6f;
    static constexpr float MAX_GAIN = 1.8f;

private:
    float gain_ = 1.0f;
    float steadyS_ = 0.0f;
};
