#pragma once
// Counts button clicks: 1, 2, 3 ... short presses in a row. The gesture ends
// when no new press comes for gapMs after the last release.
//
//   update() returns Press  on every debounced press (used to STOP a moving robot)
//                    Clicks when a gesture ended; clicks() tells how many
//
// A press held longer than maxPressMs is not a click (and cancels the gesture),
// so leaning on the button does not start anything. Plain C++: test_host/
// tests.cpp checks it on a PC.
#include <stdint.h>

class ClickCounter {
public:
    enum class Event : uint8_t { None, Press, Clicks };

    ClickCounter(uint32_t debounceMs = 30, uint32_t gapMs = 1000, uint32_t maxPressMs = 1500)
        : debounceMs_(debounceMs), gapMs_(gapMs), maxPressMs_(maxPressMs) {}

    // rawPressed: the pin right now (true = pressed); nowMs: millis()
    Event update(bool rawPressed, uint32_t nowMs) {
        if (rawPressed != lastRaw_) {                 // contact bounce: wait until it settles
            lastRaw_ = rawPressed;
            rawSinceMs_ = nowMs;
        }
        if (rawPressed != stable_ && nowMs - rawSinceMs_ >= debounceMs_) {
            stable_ = rawPressed;
            if (stable_) {                             // pressed
                pressMs_ = nowMs;
                return Event::Press;
            }
            // released
            if (suppress_) {
                suppress_ = false;                     // this press was used for something else
            } else if (nowMs - pressMs_ <= maxPressMs_) {
                if (count_ < 9) ++count_;
                releaseMs_ = nowMs;
            } else {
                count_ = 0;                            // a long hold is not a click
            }
        }
        if (!stable_ && count_ > 0 && nowMs - releaseMs_ >= gapMs_) {
            done_ = count_;
            count_ = 0;
            return Event::Clicks;
        }
        return Event::None;
    }

    uint8_t clicks() const { return done_; }
    bool pressed() const { return stable_; }

    // Forget the clicks so far, and do not count the press now held down.
    void cancel() {
        count_ = 0;
        suppress_ = stable_;
    }

private:
    uint32_t debounceMs_, gapMs_, maxPressMs_;
    bool lastRaw_ = false, stable_ = false, suppress_ = false;
    uint32_t rawSinceMs_ = 0, pressMs_ = 0, releaseMs_ = 0;
    uint8_t count_ = 0, done_ = 0;
};
