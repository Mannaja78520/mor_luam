#pragma once
// What the robot did during a timed run, as segments on a time line: turning the
// wheel in place, driving, or standing still. A change counts once it has lasted
// holdMs and is dated from when it began, so sensor flicker does not split a
// segment. Plain C++ (checked in test_host/tests.cpp).
#include <stdint.h>

template <uint8_t N>
class ActivityLog {
public:
    enum Act : uint8_t { Still = 0, Turn = 1, Drive = 2 };

    void clear(uint32_t holdMs) {
        n_ = 0;
        holdMs_ = holdMs;
        pending_ = false;
    }
    void update(uint32_t tMs, Act a) {
        if (!n_) { push(tMs, a); return; }
        if (a == act_[n_ - 1]) { pending_ = false; return; }      // flicker back: no change
        if (!pending_ || a != cand_) { pending_ = true; cand_ = a; candMs_ = tMs; }
        if (tMs - candMs_ >= holdMs_) {
            push(candMs_, cand_);
            pending_ = false;
        }
    }
    uint8_t size() const { return n_; }
    uint32_t t(uint8_t i) const { return t_[i]; }    // ms since the run started
    Act act(uint8_t i) const { return act_[i]; }

private:
    void push(uint32_t tMs, Act a) {
        if (n_ == N) return;                          // full: the last segment runs to the end
        t_[n_] = tMs;
        act_[n_] = a;
        ++n_;
    }

    uint32_t t_[N] = {};
    Act act_[N] = {};
    uint8_t n_ = 0;
    uint32_t holdMs_ = 150;
    bool pending_ = false;
    Act cand_ = Still;
    uint32_t candMs_ = 0;
};
