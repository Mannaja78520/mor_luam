#pragma once
// Drops single wrong readings from an angle sensor (used for the AS5600).
//
// While the motor runs, the AS5600 on mor_luam returns bad single readings: on
// the robot (2026-10-08, tools/test_out/steer_*.csv) 30-55 % of samples jumped
// 8-53 deg and came straight back; at rest there were none. The wheel cannot
// turn more than ~2 deg in one 10 ms tick, so a reading further than maxStep
// from the last good one is ignored - unless `confirm` readings in a row agree
// with each other, which means the wheel really is there now.
//
// Plain C++ so test_host/tests.cpp checks it on a PC.
#include <math.h>
#include <stdint.h>

class AngleSpikeFilter {
public:
    AngleSpikeFilter(float maxStepDeg, uint8_t confirm) : maxStep_(maxStepDeg), confirm_(confirm) {}

    // deg: new reading 0..360; returns the filtered angle
    float update(float deg) {
        if (!have_) {
            have_ = true;
            good_ = deg;
            return good_;
        }
        if (fabsf(diff(deg, good_)) <= maxStep_) {
            good_ = deg;
            streak_ = 0;
            return good_;
        }
        ++rejected_;
        if (streak_ > 0 && fabsf(diff(deg, cand_)) <= maxStep_) {
            if (++streak_ >= confirm_) {
                good_ = deg;
                streak_ = 0;
            }
        } else {
            streak_ = 1;
        }
        cand_ = deg;
        return good_;
    }

    uint32_t rejected() const { return rejected_; }

private:
    static float diff(float a, float b) { return fmodf(a - b + 540.0f, 360.0f) - 180.0f; }   // -180..180

    float maxStep_;
    uint8_t confirm_;
    bool have_ = false;
    float good_ = 0.0f, cand_ = 0.0f;
    uint8_t streak_ = 0;
    uint32_t rejected_ = 0;
};
