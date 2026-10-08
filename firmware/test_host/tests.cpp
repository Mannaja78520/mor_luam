// PC tests for the plain-C++ parts of the firmware: src/algorithm/, lib/PIDF, net/WifiPolicy.h.
// Build and run (Docker):  docker\mor_luam.bat fw-test
//
//  1. Detour Steer on the robot gives the SAME numbers as the homework's
//     detour_steer.py (goal 1 m at 350 deg: a 1.034 m, beta* 254.2 deg, b 0.180 m, 9.34 s).
//  2. Theorems T1, T3, T4 hold for the C++ version too (brute force).
//  3. PIDF: the original against the reworked one - the derivative kick when
//     the wheel leaves the deadband, and simulated 300 deg steers on several
//     motor models, with and without SteerStopPredictor.
//  4. Wi-Fi priority rules (net/WifiPolicy.h).
//  5. Steering direction (angles::cwErrorDeg) and the AS5600 spike filter.
//  6. Learned drive feedforward, with the real drive gains and a weaker motor.
//
// The motor models are made up (no measurement of the real steering exists):
// they show how each version behaves, not how far the real robot will go.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "PIDF.h"
#include "PIDF_config.h"
#include "PIDF_old.h"
#include "algorithm/DetourSteer.h"
#include "algorithm/DirectPlanner.h"
#include "algorithm/DriveGainLearner.h"
#include "algorithm/MotorResponseWatch.h"
#include "algorithm/SteerPowerRamp.h"
#include "algorithm/SteerStopPredictor.h"
#include "net/WifiPolicy.h"
#include "util/AngleSpikeFilter.h"
#include "util/Angles.h"
#include "esp32_hardware.h"   // PWM_STEER_Max

unsigned long g_fake_us = 1;
static int failures = 0;

#define CHECK(cond, ...)                                    \
    do {                                                    \
        if (cond) printf("  PASS  ");                       \
        else { printf("  FAIL  "); ++failures; }            \
        printf(__VA_ARGS__);                                \
        printf("\n");                                       \
    } while (0)

static RobotParams homeworkParams() {
    RobotParams p;
    p.driveMps = 0.25f;
    p.steerDps = 60.0f;
    p.settleS = 0.05f;
    p.stopS = 0.20f;
    return p;
}

// ---- 1, 2: Detour Steer ----------------------------------------------------------

static void testDetourMatchesHomework() {
    printf("1. Detour Steer vs the homework (detour_steer.py)\n");
    const RobotParams p = homeworkParams();
    const DetourSteer ds;
    const LegPlan l = ds.plan(350.0f, 1.0f, p);
    CHECK(l.kind == LegPlan::Detour, "goal 1 m at 350 deg -> detour");
    CHECK(fabsf(l.a - 1.034f) < 0.001f, "a = %.3f m (homework 1.034)", l.a);
    CHECK(fabsf(l.betaDeg - 254.2f) < 0.1f, "beta* = %.1f deg (homework 254.2)", l.betaDeg);
    CHECK(fabsf(l.b - 0.180f) < 0.001f, "b = %.3f m (homework 0.180)", l.b);
    CHECK(fabsf(l.timeS - 9.34f) < 0.01f, "time = %.2f s (homework 9.34)", l.timeS);
    const float direct = DirectPlanner::directTime(350.0f, 1.0f, p);
    CHECK(fabsf(direct - 9.88f) < 0.01f, "steer straight = %.2f s (homework 9.88)", direct);
    CHECK(ds.plan(60.0f, 1.0f, p).kind == LegPlan::Direct, "goal at 60 deg -> straight (theorem 1)");
    RobotParams slow = p;
    slow.driveMps = 0.03f;
    const LegPlan slowLeg = ds.plan(354.0f, 0.4f, slow);
    CHECK(slowLeg.kind == LegPlan::Direct && slowLeg.k < 2.0f,
          "0.4 m at 354 deg, v 0.03 m/s: k %.3f < 2, but extra stop makes direct faster", slowLeg.k);
    const LegPlan shorter = ds.plan(354.0f, 0.3f, slow);
    CHECK(shorter.kind == LegPlan::Detour && shorter.a > 0.0f && shorter.b > 0.0f &&
          shorter.timeS < DirectPlanner::directTime(354.0f, 0.3f, slow),
          "0.3 m at 354 deg, v 0.03 m/s, steer 60 deg/s -> detour (k %.3f)", shorter.k);
}

static float twoLegTime(float phi, float d, float beta, const RobotParams& p) {
    const float r = 3.14159265f / 180.0f;
    const float s = sinf(beta * r);
    const float a = d * sinf((beta - phi) * r) / s, b = d * sinf(phi * r) / s;
    if (a < 0 || b < 0) return 1e9f;
    return a / p.driveMps + p.stopS + beta / p.steerDps + p.settleS + b / p.driveMps;
}

static void testTheorems() {
    printf("2. Theorems, brute force (2000 random goals)\n");
    srand(7);
    float worst = 0.0f;
    int t1bad = 0, t4bad = 0;
    for (int i = 0; i < 2000; ++i) {
        RobotParams p = homeworkParams();
        p.steerDps = 15.0f + rand() % 166;
        const float phi = (rand() % 36000) / 100.0f;
        const float d = 0.05f + (rand() % 2950) / 1000.0f;
        const LegPlan l = DetourSteer().plan(phi, d, p);
        float best = DirectPlanner::directTime(phi, d, p);
        if (phi > 180.0f)
            for (int j = 1; j < 2000; ++j) {
                const float t = twoLegTime(phi, d, 180.0f + (phi - 180.0f) * j / 2000.0f, p);
                if (t < best) best = t;
            }
        if (l.timeS - best > worst) worst = l.timeS - best;
        if (phi <= 180.0f && l.kind != LegPlan::Direct) ++t1bad;
        if (l.kind == LegPlan::Detour) {
            const LegPlan again = DetourSteer().plan(l.betaDeg, l.b, p);
            if (again.kind != LegPlan::Direct || fabsf(again.betaDeg - l.betaDeg) > 1e-3f) ++t4bad;
        }
    }
    CHECK(worst < 2e-3f, "T3 closed form vs 2000-point grid: worst %.4f s slower", worst);
    CHECK(t1bad == 0, "T1 phi <= 180 is always straight: %d wrong", t1bad);
    CHECK(t4bad == 0, "T4 plan again after leg 1 gives 'steer beta*, drive b': %d changed", t4bad);
}

// ---- 3: PIDF ----------------------------------------------------------------------

template <class P>
static P makeSteerPid() {
    P pid(0, PWM_STEER_Max, Wheel_STEER_KP, Wheel_STEER_KI, Wheel_STEER_I_Min, Wheel_STEER_I_Max,
          Wheel_STEER_KD, Wheel_STEER_KF, Wheel_STEER_ERROR_TOLERANCE);
    pid.setDFilterCutoffHz(Wheel_STEER_D_FILTER_HZ);
    return pid;
}

// The wheel sits inside the tolerance, then is knocked 20 deg off. Each
// version is used the way its controller uses it inside the band.
template <class P>
static float kickLeavingBand(bool newController) {
    P pid = makeSteerPid<P>();
    pid.reset();
    for (int i = 0; i < 5; ++i) {
        g_fake_us += 10000;
        if (newController) pid.reset();
        else pid.compute_with_error(0.0f);
    }
    g_fake_us += 10000;
    return pid.compute_with_error(20.0f);
}

static void applyZone(PIDF& p) { p.setIZone(Wheel_STEER_I_ZONE); }
static void applyZone(PIDF_old&) {}

struct SteerRun {
    float worstPastDeg = 0.0f;    // furthest past the target over all steers
    float lastPastDeg = 0.0f;     // in the last steer (after learning)
    float totalS = 0.0f;
};

// N steers of 300 deg in a row, driven the way the firmware drives them:
// clockwise only, base speed + PID, motor off inside the tolerance.
// Plant: geared DC motor, first-order lag K/tau, braked by tauBrake when off.
template <class P>
static SteerRun steerRuns(bool newController, bool predict, float K, float tau, float tauBrake, int n = 5) {
    P pid = makeSteerPid<P>();
    if (newController) applyZone(pid);
    SteerStopPredictor pred;
    const float DT = 0.01f, tol = Wheel_STEER_ERROR_TOLERANCE;
    float angle = 0.0f, w = 0.0f, rate = 0.0f, prevAngle = 0.0f;
    SteerRun out;
    for (int run = 0; run < n; ++run) {
        const float target = angle + 300.0f;
        pid.reset();
        bool coasting = false;
        float cutAngle = 0.0f, past = 0.0f;
        int settled = 0, i = 0;
        for (; i < 3000; ++i) {
            g_fake_us += 10000;
            const float e = fmodf(target - angle + 7200.0f, 360.0f);
            const bool inTol = e <= tol || e >= 360.0f - tol;
            int pwm = 0;
            if (!inTol) {
                settled = 0;
                if (coasting) {
                    if (fabsf(rate) < 2.0f) { coasting = false; pred.onStopped(angle - cutAngle); }
                } else if (predict && pred.shouldCut(e, rate, tol)) {
                    coasting = true;
                    cutAngle = angle;
                    pred.onCut(rate);
                } else {
                    float mag = pid.compute_with_error(e) + Wheel_STEER_BASE_SPEED;
                    if (mag > PWM_STEER_Max) mag = PWM_STEER_Max;
                    pwm = (int)lroundf(mag);
                }
            } else {
                if (newController) pid.reset(); else pid.compute_with_error(0.0f);
                if (coasting && fabsf(rate) < 2.0f) { coasting = false; pred.onStopped(angle - cutAngle); }
                if (++settled > 30 && fabsf(w) < 0.5f) break;
            }
            w += (K * pwm - w) / (pwm ? tau : tauBrake) * DT;
            prevAngle = angle;
            angle += w * DT;
            rate = 0.5f * rate + 0.5f * (angle - prevAngle) / DT;
            if (angle - target > past) past = angle - target;
        }
        out.totalS += i * DT;
        out.lastPastDeg = past;
        if (past > out.worstPastDeg) out.worstPastDeg = past;
        angle = target + fmodf(angle - target + 7200.0f, 360.0f) - (fmodf(angle - target + 7200.0f, 360.0f) > 180 ? 360.0f : 0.0f);
    }
    return out;
}

static void testPidf() {
    printf("3. PIDF: original vs reworked (made-up motor models, see the top of this file)\n");
    const float kickOld = kickLeavingBand<PIDF_old>(false);
    const float kickNew = kickLeavingBand<PIDF>(true);
    printf("        wheel knocked 20 deg out of the deadband: output old %.0f, new %.0f (P alone = %.0f)\n",
           kickOld, kickNew, Wheel_STEER_KP * 20.0f);
    CHECK(kickNew < kickOld, "no derivative kick when the wheel leaves the deadband");

    struct NewPid : PIDF {
        NewPid(float a, float b, float c, float d, float e, float f, float g, float h, float i)
            : PIDF(a, b, c, d, e, f, g, h, i) {}
    };
    // K deg/s per PWM, motor lag tau s, brake tau s (how fast it stops with the drive off)
    const float plants[][3] = {{0.20f, 0.10f, 0.04f}, {0.20f, 0.15f, 0.15f}, {0.30f, 0.20f, 0.30f},
                               {0.40f, 0.25f, 0.50f}, {0.15f, 0.30f, 0.60f}};
    printf("        5 steers of 300 deg in a row, worst degrees past the target (tolerance %.1f):\n",
           Wheel_STEER_ERROR_TOLERANCE);
    printf("          %-27s %10s %10s %16s %12s\n", "motor model", "old PID", "new PID", "new + predictor",
           "(last steer)");
    bool predictorHelps = true;
    for (auto& pl : plants) {
        const SteerRun o = steerRuns<PIDF_old>(false, false, pl[0], pl[1], pl[2]);
        const SteerRun nw = steerRuns<NewPid>(true, false, pl[0], pl[1], pl[2]);
        const SteerRun np = steerRuns<NewPid>(true, true, pl[0], pl[1], pl[2]);
        printf("          K %.2f tau %.2f brake %.2f  %10.1f %10.1f %16.1f %12.1f\n", pl[0], pl[1], pl[2],
               o.worstPastDeg, nw.worstPastDeg, np.worstPastDeg, np.lastPastDeg);
        if (np.lastPastDeg > o.lastPastDeg + 0.5f) predictorHelps = false;
    }
    CHECK(predictorHelps, "with the predictor, the last steer is never worse than the original");
}

// ---- 4: Wi-Fi priority (net/WifiPolicy.h) -------------------------------------------

static bool same(const std::vector<int>& a, std::initializer_list<int> b) { return a == std::vector<int>(b); }

static void testWifiPolicy() {
    using namespace wifipolicy;
    printf("4. Wi-Fi priority (saved list order = priority)\n");
    CHECK(same(joinOrder({-60, -50, -70}, -1), {0, 1, 2}), "fresh start: priority 1, 2, 3 (not the strongest first)");
    CHECK(same(joinOrder({-60, -50, -70}, 1), {1, 0, 2}), "after a drop: the network it was on first, then 1, 3");
    CHECK(same(joinOrder({-60, NOT_SEEN, -70}, 1), {0, 2}), "the old network is gone: priority 1, then 3");
    CHECK(same(joinOrder({NOT_SEEN, NOT_SEEN}, 0), {}), "nothing on the air: nothing to try");
    CHECK(moveUpTo({-60, -50, -70}, 2, -75) == 0, "on priority 3, priority 1 back -> move to 1");
    CHECK(moveUpTo({NOT_SEEN, -50, -70}, 2, -75) == 1, "on priority 3, only 2 back -> move to 2");
    CHECK(moveUpTo({-80, NOT_SEEN, -70}, 2, -75) == -1, "priority 1 too weak (-80 dBm) -> stay");
    CHECK(moveUpTo({-60, -50}, 0, -75) == -1, "already on priority 1 -> stay");
    CHECK(moveUpTo({-60, -50}, -1, -75) == 0, "on a network no longer saved -> move to priority 1");
}

// ---- 5: steering direction + AS5600 spike filter ----------------------------------------

static void testSteerSensorMath() {
    printf("5. Steering direction and AS5600 spike filter\n");
    CHECK(fabsf(angles::cwErrorDeg(100.0f, 90.0f) - 10.0f) < 1e-3f, "wheel at 90, target 100: 10 deg to turn (angle goes up)");
    CHECK(fabsf(angles::cwErrorDeg(80.0f, 90.0f) - 350.0f) < 1e-3f, "wheel at 90, target 80: 350 deg (one way only)");
    CHECK(fabsf(angles::cwErrorDeg(5.0f, 355.0f) - 10.0f) < 1e-3f, "across 0: wheel at 355, target 5 -> 10 deg");
    AngleSpikeFilter f(6.0f, 3);
    float out = 0;
    // steering 1 deg per tick from 100 down, with single bad readings like the robot's
    const float in[] = {100, 99, 98, 150, 97, 96, 40, 95, 94, 93.5f};
    bool okSpikes = true;
    for (float v : in) {
        out = f.update(v);
        if (v == 150 || v == 40) okSpikes = okSpikes && out < 100.5f && out > 90.0f;
    }
    CHECK(okSpikes && fabsf(out - 93.5f) < 1e-3f && f.rejected() == 2, "single spikes ignored (%u), angle follows the wheel", (unsigned)f.rejected());
    AngleSpikeFilter g(6.0f, 3);
    g.update(10);
    g.update(200);
    g.update(201);
    out = g.update(202);
    CHECK(fabsf(out - 202.0f) < 1e-3f, "a real jump is accepted after 3 agreeing readings");
    out = g.update(1);
    CHECK(fabsf(out - 202.0f) < 1e-3f, "wrap-around: 1 deg is a spike from 202, ignored");
    AngleSpikeFilter h(6.0f, 3);
    h.update(358);
    out = h.update(2);
    CHECK(fabsf(out - 2.0f) < 1e-3f, "358 -> 2 deg is a 4 deg step across 0, accepted");
}

// ---- 6: drive gain learning + external PID feedforward ---------------------------------

static PIDF makeDrivePid() {
    PIDF pid(0.0f, 1023.0f, Wheel_SPIN_KP, Wheel_SPIN_KI, Wheel_SPIN_I_Min, Wheel_SPIN_I_Max,
             Wheel_SPIN_KD, Wheel_SPIN_KF, Wheel_SPIN_ERROR_TOLERANCE);
    pid.setDFilterCutoffHz(Wheel_SPIN_D_FILTER_HZ);
    pid.setOutputRamp(Wheel_SPIN_RAMP);
    return pid;
}

struct DriveRun {
    float earlyMeanError = 0.0f;
    float finalRpm = 0.0f;
};

// Synthetic weaker battery/motor: PWM = 1.18 * (KS + KF * rpm), lag 0.25 s.
// Use the firmware's gains, deadband, ramp and rounded PWM. Each run starts
// stopped with a fresh PID; only the learner's saved gain survives the stop.
static DriveRun driveRun(DriveGainLearner& learner, bool learn, float seconds) {
    const float dt = 0.01f, target = 7.5f, weakening = 1.18f;
    PIDF pid = makeDrivePid();
    DriveRun out;
    float rpm = 0.0f;
    int earlySamples = 0;
    for (int tick = 0; tick < (int)lroundf(seconds / dt); ++tick) {
        g_fake_us += 10000;
        const float ff = learner.feedforward(target, Wheel_SPIN_KS, Wheel_SPIN_KF);
        const float pwm = roundf(pid.compute_with_feedforward(target, rpm, ff));
        if (learn) learner.observe(target, rpm, pwm, Wheel_SPIN_KS, Wheel_SPIN_KF, 1023.0f,
                                   dt, tick * dt);
        if (tick < 300) {
            out.earlyMeanError += fabsf(target - rpm);
            ++earlySamples;
        }
        const float steadyRpm = fmaxf(0.0f, (pwm / weakening - Wheel_SPIN_KS) / Wheel_SPIN_KF);
        rpm += (steadyRpm - rpm) * dt / 0.25f;
    }
    out.earlyMeanError /= earlySamples;
    out.finalRpm = rpm;
    return out;
}

static void testDriveLearning() {
    printf("6. Drive gain learner + PID external feedforward (synthetic weaker motor)\n");
    DriveGainLearner learner;
    CHECK(learner.feedforward(0.0f, 160.0f, 85.0f) == 0.0f &&
          learner.feedforward(-7.5f, 160.0f, 85.0f) == 0.0f,
          "zero/reverse target has no positive drive feedforward");

    struct RejectedSample {
        float target, measured, pwm, dt, since;
        const char* why;
    };
    const RejectedSample rejected[] = {
        {7.5f, 7.5f, 940.0f, 0.01f, 0.99f, "startup"},
        {7.5f, 5.0f, 940.0f, 0.01f, 2.0f, "off-target speed"},
        {7.5f, 7.5f, 1022.0f, 0.01f, 2.0f, "upper saturation"},
        {7.5f, 7.5f, 0.0f, 0.01f, 2.0f, "zero output"},
        {0.5f, 0.5f, 300.0f, 0.01f, 2.0f, "near-zero target"},
        {7.5f, 7.5f, 940.0f, 0.0f, 2.0f, "zero dt"},
        {7.5f, 7.5f, 940.0f, -0.01f, 2.0f, "negative dt"},
        {7.5f, NAN, 940.0f, 0.01f, 2.0f, "invalid measurement"},
    };
    for (const auto& s : rejected) {
        learner.observe(s.target, s.measured, s.pwm, 160.0f, 85.0f, 1023.0f, s.dt, s.since);
        CHECK(learner.gain() == 1.0f && learner.learnedS() == 0.0f, "no learning during %s", s.why);
    }

    const float invalidSaved[] = {-1.0f, 0.0f, 2.0f, NAN, INFINITY, -INFINITY};
    bool invalidReset = true;
    for (float g : invalidSaved) {
        DriveGainLearner saved(g);
        invalidReset = invalidReset && saved.gain() == 1.0f && saved.learnedS() == 0.0f;
    }
    CHECK(invalidReset, "invalid saved gain (including NaN/infinity) resets to 1.0");
    DriveGainLearner low(DriveGainLearner::MIN_GAIN), high(DriveGainLearner::MAX_GAIN);
    CHECK(low.gain() == DriveGainLearner::MIN_GAIN && high.gain() == DriveGainLearner::MAX_GAIN,
          "valid saved gain includes both bounds");
    low.setGain(1.0f);
    high.setGain(1.0f);
    bool bounded = true;
    for (int tick = 0; tick < 5000; ++tick) {
        low.observe(7.5f, 7.5f, 20.0f, 160.0f, 85.0f, 1023.0f, 0.01f, 2.0f);
        high.observe(2.0f, 2.0f, 900.0f, 160.0f, 85.0f, 1023.0f, 0.01f, 2.0f);
        bounded = bounded && low.gain() >= DriveGainLearner::MIN_GAIN && high.gain() <= DriveGainLearner::MAX_GAIN;
    }
    CHECK(bounded && fabsf(low.gain() - DriveGainLearner::MIN_GAIN) < 0.001f &&
          fabsf(high.gain() - DriveGainLearner::MAX_GAIN) < 0.001f,
          "learning remains bounded for extreme valid PWM ratios (%.3f..%.3f)", low.gain(), high.gain());

    const DriveRun training = driveRun(learner, true, 30.0f);
    CHECK(fabsf(learner.gain() - 1.18f) < 0.055f && learner.learnedS() > 20.0f &&
          fabsf(training.finalRpm - 7.5f) <= 0.35f,
          "weaker motor: gain learns %.3f (plant 1.18), steady rpm %.2f, learned %.1f s",
          learner.gain(), training.finalRpm, learner.learnedS());

    DriveGainLearner saved(learner.gain()), fixed;
    const DriveRun next = driveRun(saved, false, 3.0f);
    const DriveRun withoutLearning = driveRun(fixed, false, 3.0f);
    CHECK(next.earlyMeanError < 0.75f * withoutLearning.earlyMeanError,
          "next run after PID reset: mean first-3-s error %.3f rpm vs %.3f without learning",
          next.earlyMeanError, withoutLearning.earlyMeanError);

    PIDF ramped = makeDrivePid();
    g_fake_us += 10000;
    float previous = ramped.compute_with_feedforward(7.5f, 0.0f, 900.0f);
    bool rampOk = previous == 0.0f;
    for (int tick = 0; tick < 20; ++tick) {
        g_fake_us += 10000;
        const float pwm = ramped.compute_with_feedforward(7.5f, 0.0f, 900.0f);
        rampOk = rampOk && pwm > previous && pwm - previous <= Wheel_SPIN_RAMP * 0.01f + 0.001f;
        previous = pwm;
    }
    CHECK(rampOk && fabsf(previous - 800.0f) < 0.01f,
          "drive ramp starts at zero after reset, then rises by at most 40 PWM per 10 ms");

    PIDF external = makeDrivePid();
    external.setOutputRamp(0.0f);
    g_fake_us += 10000;
    const float atTarget = external.compute_with_feedforward(7.5f, 7.5f, 900.0f);
    CHECK(fabsf(atTarget - 900.0f) < 0.001f && external.lastF() == 900.0f,
          "external feedforward replaces configured Kf and appears in tuning read-back");
    g_fake_us += 10000;
    const float tooFast = external.compute_with_feedforward(7.5f, 9.0f, 900.0f);
    CHECK(tooFast < 900.0f && external.lastP() < 0.0f && external.lastI() < 0.0f,
          "overspeed reduces PWM below external feedforward (%.1f < 900)", tooFast);

    external.reset();
    bool limited = true;
    for (int tick = 0; tick < 500; ++tick) {
        g_fake_us += 10000;
        const float pwm = external.compute_with_feedforward(7.5f, 0.0f, 1200.0f);
        limited = limited && pwm == 1023.0f && external.lastI() == 0.0f;
    }
    CHECK(limited, "external feedforward saturation clamps the total PWM without I windup");
    g_fake_us += 10000;
    const float recovered = external.compute_with_feedforward(7.5f, 7.5f, 700.0f);
    CHECK(fabsf(recovered - 700.0f) < 0.001f && external.lastI() == 0.0f,
          "after saturation, output immediately returns to requested feedforward");
}

static void testMotorFeedbackAndSmoothing() {
    printf("7. Missing motor response and steering power smoothing\n");
    MotorResponseWatch drive, steer, moving;
    bool early = false;
    for (int i = 0; i < 100; ++i) early |= drive.observe(2, 800, 0, 100, 0.01f);
    CHECK(!early, "drive startup is allowed before the 1.5 s missing-response timeout");
    for (int i = 0; i < 60; ++i) drive.observe(2, 800, 0, 100, 0.01f);
    CHECK(drive.fault() == MotorResponseWatch::DriveNoResponse, "no encoder response stops a powered drive");
    for (int i = 0; i < 160; ++i) steer.observe(1, -600, 0, 100, 0.01f);
    CHECK(steer.fault() == MotorResponseWatch::SteerNoResponse, "no angle response stops powered steering in either PWM direction");
    bool falseFault = false;
    for (int i = 0; i < 1000; ++i) falseFault |= moving.observe(2, 800, i / 10, 100, 0.01f);
    CHECK(!falseFault, "regular drive encoder feedback does not cause a false missing-response fault");
    moving.reset();
    for (int i = 0; i < 1000; ++i) falseFault |= moving.observe(1, -600, 0, fmodf(7200 - i * 0.3f, 360), 0.01f);
    CHECK(!falseFault, "clockwise steering through angle zero is accepted as feedback");
    drive.reset();
    CHECK(drive.fault() == MotorResponseWatch::None, "explicit new-command reset clears the response fault");
    for (int i = 0; i < 1000; ++i) drive.observe(2, 0, 0, 100, 0.01f);
    CHECK(drive.fault() == MotorResponseWatch::None, "an unpowered motor is not probed or faulted");
    SteerPowerRamp ramp;
    float prev = 0;
    bool bounded = true;
    for (int i = 0; i < 1000; ++i) {
        const float pwm = ramp.step(i % 20 < 10 ? 1000.0f : 210.0f, 0.01f);
        bounded &= pwm - prev <= 15.001f && prev - pwm <= 40.001f;
        prev = pwm;
    }
    CHECK(bounded, "final steering power including base is limited to +15/-40 PWM per tick");
    CHECK(ramp.step(0, 0.01f) == 0 && ramp.step(600, 0.01f) <= 15.001f,
          "zero power cuts immediately and a restart ramps from zero");
}

int main() {
    testDetourMatchesHomework();
    testTheorems();
    testPidf();
    testWifiPolicy();
    testSteerSensorMath();
    testDriveLearning();
    testMotorFeedbackAndSmoothing();
    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASS", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
