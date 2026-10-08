#pragma once
// When to cut the steering motor so the wheel COASTS into its angle.
//
// The wheel steers one way only, so stopping a little past the angle costs a
// whole extra turn - and a geared motor keeps turning after its power is cut.
// A simulation of the steering (test_host/tests.cpp) showed the PID terms were
// not the problem: with a motor that coasts, both the old and the reworked PID
// went ~740 deg past, because the wheel arrives at full speed. What helps is
// cutting the power EARLY, by the angle the wheel will still coast:
//
//   coast_deg = rate_dps * coast_s          cut when  e_cw <= tol + coast_deg
//
// coast_s is learned on the robot: at each cut the speed is remembered, and
// when the wheel has stopped the angle it really coasted gives a new sample
// (coasted_deg / rate_at_cut), averaged in. A different motor, battery or
// load re-tunes itself after a few steers.
//
// Plain C++ (no Arduino) so test_host/ can run it on a PC.

class SteerStopPredictor {
public:
    explicit SteerStopPredictor(float initialCoastS = 0.10f) : coastS_(initialCoastS) {}

    // While the motor drives: should the power be cut now?
    // eCwDeg: error in the steering direction 0..360; rateDps: steering speed (+ = steering)
    bool shouldCut(float eCwDeg, float rateDps, float tolDeg) const {
        if (eCwDeg > 180.0f || rateDps <= 0.0f) return false;   // not approaching yet, or not moving
        return eCwDeg <= tolDeg + rateDps * coastS_;
    }

    void onCut(float rateDps) { rateAtCut_ = rateDps > 0.0f ? rateDps : 0.0f; }

    // The wheel stopped after a cut: learn from how far it really went.
    void onStopped(float coastedDeg) {
        if (rateAtCut_ < 5.0f || coastedDeg < 0.0f || coastedDeg > 180.0f) return;   // nothing to learn
        float sample = coastedDeg / rateAtCut_;
        if (sample > MAX_COAST_S) sample = MAX_COAST_S;
        coastS_ = (1.0f - LEARN) * coastS_ + LEARN * sample;
        ++samples_;
    }

    float coastS() const { return coastS_; }
    unsigned samples() const { return samples_; }

    static constexpr float LEARN = 0.3f;        // weight of a new sample
    static constexpr float MAX_COAST_S = 1.0f;

private:
    float coastS_;
    float rateAtCut_ = 0.0f;
    unsigned samples_ = 0;
};
