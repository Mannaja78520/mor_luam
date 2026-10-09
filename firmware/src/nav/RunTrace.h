#pragma once
// The path of one run, kept small for the web picture: a point every step metres.
// When full, every second point is dropped and the step doubles, so a long run
// still fits from start to end. Plain C++ (checked in test_host/tests.cpp).
#include <math.h>
#include <stdint.h>

template <uint8_t N>
class RunTrace {
public:
    void clear(float stepM) {
        n_ = 0;
        step_ = stepM;
    }
    // force: keep this point even if it is closer than the step (the end of a run)
    void add(float x, float y, bool force = false) {
        if (n_ && !force && hypotf(x - xs_[n_ - 1], y - ys_[n_ - 1]) < step_) return;
        if (n_ && force && x == xs_[n_ - 1] && y == ys_[n_ - 1]) return;
        if (n_ == N) thin();
        xs_[n_] = x;
        ys_[n_] = y;
        ++n_;
    }
    uint8_t size() const { return n_; }
    float x(uint8_t i) const { return xs_[i]; }
    float y(uint8_t i) const { return ys_[i]; }
    float step() const { return step_; }

private:
    void thin() {                      // keep points 0, 2, 4, ...: the start stays
        uint8_t k = 0;
        for (uint8_t i = 0; i < n_; i += 2, ++k) {
            xs_[k] = xs_[i];
            ys_[k] = ys_[i];
        }
        n_ = k;
        step_ *= 2.0f;
    }

    float xs_[N] = {}, ys_[N] = {};
    uint8_t n_ = 0;
    float step_ = 0.005f;
};
