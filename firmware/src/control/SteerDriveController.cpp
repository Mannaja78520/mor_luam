#include "control/SteerDriveController.h"
#include <math.h>
#include <config.h>
#include "app_config.h"
#include "algorithm/FinalApproach.h"
#include "algorithm/SteerApproach.h"
#include "util/Angles.h"

using namespace angles;

// Re-steer mid-drive when the wheel drifts this far from where it should point.
static const float DRIVE_RELOCK_ERROR_DEG = Wheel_STEER_ERROR_TOLERANCE * 2.0f;
static const int STEER_EFFECTIVE_MAX = PWM_STEER_Max < STEER_POWER_LIMIT_PWM ? PWM_STEER_Max : STEER_POWER_LIMIT_PWM;
static const int STEER_PID_MAX = STEER_EFFECTIVE_MAX - Wheel_STEER_BASE_SPEED;
static_assert(STEER_PID_MAX > 0, "steering power limit must exceed base power");

SteerDriveController::SteerDriveController(Controller& motor, esp32_Encoder& encoder,
                                           SteerSensor& steer, ImuHeading& imu)
    : motor_(motor),
      encoder_(encoder),
      steerSensor_(steer),
      imu_(imu),
      spin_(0, PWM_SPIN_Max, Wheel_SPIN_KP, Wheel_SPIN_KI, Wheel_SPIN_I_Min, Wheel_SPIN_I_Max,
            Wheel_SPIN_KD, Wheel_SPIN_KF, Wheel_SPIN_ERROR_TOLERANCE),
      steer_(0, STEER_PID_MAX, Wheel_STEER_KP, Wheel_STEER_KI, Wheel_STEER_I_Min, Wheel_STEER_I_Max,
             Wheel_STEER_KD, Wheel_STEER_KF, Wheel_STEER_ERROR_TOLERANCE),
      spinTol_(Wheel_SPIN_ERROR_TOLERANCE),
      steerTol_(Wheel_STEER_ERROR_TOLERANCE),
      pidSpin_{Wheel_SPIN_KP, Wheel_SPIN_KI, Wheel_SPIN_KD, Wheel_SPIN_KF, Wheel_SPIN_ERROR_TOLERANCE},
      pidSteer_{Wheel_STEER_KP, Wheel_STEER_KI, Wheel_STEER_KD, Wheel_STEER_KF, Wheel_STEER_ERROR_TOLERANCE},
      targetTolM_(DEFAULT_GOAL_TOL_M),
      predictor_(STEER_COAST_INIT_S) {}

void SteerDriveController::begin() {
    steer_.setDFilterCutoffHz(Wheel_STEER_D_FILTER_HZ);
    spin_.setDFilterCutoffHz(Wheel_SPIN_D_FILTER_HZ);
    steer_.setIZone(Wheel_STEER_I_ZONE);
    spin_.setOutputRamp(Wheel_SPIN_RAMP);
    steer_.reset();
    spin_.reset();
    spin_.setPIDF(Wheel_SPIN_KP, Wheel_SPIN_KI, Wheel_SPIN_KD, Wheel_SPIN_KF, spinTol_);
    spin_.setIClamp(Wheel_SPIN_I_Min, Wheel_SPIN_I_Max);
    steer_.setPIDF(Wheel_STEER_KP, Wheel_STEER_KI, Wheel_STEER_KD, Wheel_STEER_KF, steerTol_);
    steer_.setIClamp(Wheel_STEER_I_Min, Wheel_STEER_I_Max);
    mode_ = Mode::Steer;
    // Halted until the first command. (Before the refactor the wheel steered
    // to heading 0 as soon as the agent connected, with nobody asking.)
    halted_ = true;
    motor_.spin(0);
}

// ---- commands -------------------------------------------------------------

void SteerDriveController::apply(const DriveCommand& m, CommandSource src) {
    const float prevRpm = targetRpm_;
    const float prevHeading = targetHeadingDeg_;
    const float prevDist = targetDistM_;
    const bool prevGoal = goalActive_;

    float rpm = m.rpm;
    if (rpm > MOTOR_MAX_RPM) rpm = MOTOR_MAX_RPM;
    if (rpm < -MOTOR_MAX_RPM) rpm = -MOTOR_MAX_RPM;
    const float heading = wrap360(m.headingDeg);
    const float tolCmd = fabsf(m.tolM);
    const float tol = tolCmd > 0.0f ? tolCmd : targetTolM_;

    // A re-send of nearly the same goal only updates the tolerance, so a planner
    // that repeats itself does not keep resetting the controllers mid-drive.
    const bool smallAdjust = !halted_ && prevGoal &&
                             (fabsf(prevRpm) > 1e-3f || fabsf(rpm) > 1e-3f) &&
                             fabsf(rpm - prevRpm) < CMD_SMALL_RPM_EPS &&
                             fabsf(errDeg(heading, prevHeading)) < CMD_SMALL_HEADING_EPS &&
                             fabsf(m.distM - prevDist) < CMD_SMALL_DIST_EPS;
    if (smallAdjust) {
        targetTolM_ = tol;
        return;
    }

    source_ = src;
    responseWatch_.reset();
    steerPower_.reset();
    halted_ = false;
    stopOnOvershoot_ = m.stopOnOvershoot;
    overshot_ = false;
    havePrevE_ = false;
    coasting_ = false;
    approaching_ = false;
    targetRpm_ = rpm;
    targetHeadingDeg_ = heading;
    targetDistM_ = m.distM;
    targetTolM_ = tol;
    hasTarget_ = fabsf(m.distM) > 1e-4f && fabsf(rpm) > 1e-4f;
    if (hasTarget_) {
        startX_ = odom_.x();
        startY_ = odom_.y();
        const float h = deg2rad(targetHeadingDeg_);
        goalX_ = startX_ + cosf(h) * m.distM;
        goalY_ = startY_ + sinf(h) * m.distM;
        goalVecX_ = goalX_ - startX_;
        goalVecY_ = goalY_ - startY_;
        goalLenSq_ = goalVecX_ * goalVecX_ + goalVecY_ * goalVecY_;
        goalActive_ = true;
    } else {
        goalActive_ = false;
        goalLenSq_ = 0.0f;
    }

    // every new command starts from STEER, so the ESP aims the wheel itself
    steer_.reset();
    spin_.reset();
    mode_ = Mode::Steer;
    steerOkSinceMs_ = millis();
    steerAimed_ = false;
    pwm_ = 0;
    imu_.requestReference();
}

void SteerDriveController::halt() {
    steerPower_.reset();
    approaching_ = false;
    overshot_ = false;
    motor_.spin(0);
    pwm_ = 0;
    halted_ = true;
    targetRpm_ = 0.0f;
    stopGoal();
    steer_.reset();
    spin_.reset();
    mode_ = Mode::Steer;
    steerAimed_ = false;
}

void SteerDriveController::stopGoal() {
    hasTarget_ = false;
    goalActive_ = false;
    goalLenSq_ = 0.0f;
}

bool SteerDriveController::setPid(bool steerLoop, const float* v, size_t n) {
    if (!v || n < 4 || n > 9 || n == 6 || n == 8) return false;
    for (size_t i = 0; i < n; ++i) if (!isfinite(v[i])) return false;
    for (size_t i = 0; i < n && i < 5; ++i) if (v[i] < 0.0f) return false;
    if (n >= 7 && v[5] > v[6]) return false;
    const float maxOutput = steerLoop ? STEER_PID_MAX : PWM_SPIN_Max;
    if (n >= 9 && (v[7] < 0.0f || v[7] > v[8] || v[8] > maxOutput)) return false;
    PIDF& pid = steerLoop ? steer_ : spin_;
    float& tol = steerLoop ? steerTol_ : spinTol_;
    float* keep = steerLoop ? pidSteer_ : pidSpin_;
    if (n >= 5) tol = v[4];
    // A different feedforward curve must not inherit the old curve's correction.
    if (!steerLoop && v[3] != keep[3]) gainLearner_.setGain(1.0f);
    pid.setPIDF(v[0], v[1], v[2], v[3], tol);
    if (n >= 7) pid.setIClamp(v[5], v[6]);
    if (n >= 9) pid.setOutputLimits(v[7], v[8]);
    pid.reset();
    for (int i = 0; i < 4; i++) keep[i] = v[i];
    keep[4] = tol;
    return true;
}

void SteerDriveController::getPid(bool steerLoop, float out[5]) const {
    const float* src = steerLoop ? pidSteer_ : pidSpin_;
    for (int i = 0; i < 5; i++) out[i] = src[i];
}

void SteerDriveController::resetPose() { setPose(0.0f, 0.0f, 0.0f); }

void SteerDriveController::setPose(float x, float y, float headingDeg) {
    odom_.setPosition(x, y);
    imu_.resetReference(headingDeg);                 // the body heading reads headingDeg from the next tick
}

// ---- the 10 ms tick ---------------------------------------------------------

void SteerDriveController::step() {
    // feedback
    ticks_ = encoder_.read();
    rpm_ = encoder_.getRPM();
    steerDeg_ = wrap360(steerSensor_.readDeg());
    imu_.update();
    // Steering speed over the last RATE_WINDOW ticks (50 ms), + while steering
    // (angle goes up). Per tick, one AS5600 count of flicker at rest (0.09 deg)
    // read as +-4 deg/s and made "is the wheel still?" fail at random
    // (seen 2026-10-08); over 50 ms it is 1.8 deg/s, and real steering
    // (30-100 deg/s) is measured just as well.
    if (havePrevSteer_ && rateFill_ == RATE_WINDOW)
        rateDps_ = errDeg(steerDeg_, steerHist_[rateIdx_]) / (RATE_WINDOW * CTRL_PERIOD_S);
    steerHist_[rateIdx_] = steerDeg_;
    rateIdx_ = (rateIdx_ + 1) % RATE_WINDOW;
    if (rateFill_ < RATE_WINDOW) ++rateFill_;
    prevSteerDeg_ = steerDeg_;
    havePrevSteer_ = true;

    // odometry: only integrates while driving
    lastDeltaM_ = odom_.update(mode_ == Mode::Drive, ticks_, imu_.bodyHeadingDeg(), steerDeg_);

    headingMeasDeg_ = imu_.available() ? wrap360(imu_.yawDeg()) : imu_.bodyHeadingDeg();
    if (headFill_ == RATE_WINDOW)
        bodyRateDps_ = errDeg(headingMeasDeg_, headHist_[headIdx_]) / (RATE_WINDOW * CTRL_PERIOD_S);
    headHist_[headIdx_] = headingMeasDeg_;
    headIdx_ = (headIdx_ + 1) % RATE_WINDOW;
    if (headFill_ < RATE_WINDOW) ++headFill_;
    steerTargetDeg_ = wrap360((targetHeadingDeg_ - headingMeasDeg_) + STEER_CMD_ZERO_DEG);
    steerErrDeg_ = errDeg(steerTargetDeg_, steerDeg_);
    const float eCw = cwErrorDeg(steerTargetDeg_, steerDeg_);       // 0..360

    if (halted_) {
        if (pwm_ != 0) motor_.spin(0);
        pwm_ = 0;
        learnIfStopped();
        fillState();
        return;
    }

    // The wheel went past its angle: the error (steering direction) jumped from "almost
    // there" (< 90) to "almost a full turn" (> 270). Steering on would cost a
    // whole extra turn; when asked, stop here and let the planner aim again.
    if (stopOnOvershoot_ && havePrevE_ && prevECw_ < 90.0f && eCw > 270.0f &&
        !cwInTolerance(eCw, steerTol_)) {
        motor_.spin(0);
        pwm_ = 0;
        overshootDeg_ = 360.0f - eCw;
        overshot_ = true;
        halted_ = true;
        targetRpm_ = 0.0f;
        stopGoal();
        steer_.reset();
        spin_.reset();
        mode_ = Mode::Steer;
        fillState();
        return;
    }
    prevECw_ = eCw;
    havePrevE_ = true;

    // Aimed = inside the tolerance, with hysteresis: once in, the wheel stays
    // "aimed" until it is STEER_TOL_HYST_DEG further out. Without it, a wheel
    // stopped right at the edge flickered in/out with the sensor's last digit,
    // the power ramp restarted every tick and it stood still for 8 s (2026-10-08).
    steerAimed_ = cwInTolerance(eCw, steerTol_ + (steerAimed_ ? STEER_TOL_HYST_DEG : 0.0f));

    switch (mode_) {
        case Mode::Steer: {
            // A powered approach goes on INSIDE the tolerance band, until the cut that lets the
            // wheel coast to STEER_LAND_DEG short of its angle. (Until 2026-10-10 the motor
            // stopped at the band edge, so the wheel always ended 3-5 deg short.)
            const float land = landDeg_ < steerTol_ ? landDeg_ : steerTol_;
            if (steerapproach::keepPushing(steerAimed_, approaching_, coasting_, eCw, land, rateDps_,
                                           STEER_APPROACH_MIN_DPS)) {
                steerOkSinceMs_ = millis();
                aimedSinceMs_ = millis();
                if (coasting_) {                                    // power is off: let it run out
                    motor_.spin(0);
                    pwm_ = 0;
                    learnIfStopped();                               // stopped short: drive again next tick
                } else if (STEER_STOP_PREDICT && predictor_.shouldCut(eCw, rateDps_, land)) {
                    motor_.spin(0);                                 // cut now: the wheel coasts the rest
                    pwm_ = 0;
                    coasting_ = true;
                    approaching_ = false;
                    cutAngleDeg_ = steerDeg_;
                    predictor_.onCut(rateDps_);
                    steer_.reset();                                 // resume later without a stale D
                } else {
                    approaching_ = true;
                    // one way only (counter-clockwise): the error is always positive, the motor one way
                    float mag = steer_.compute_with_error(eCw) + Wheel_STEER_BASE_SPEED;
                    if (mag > STEER_EFFECTIVE_MAX) mag = STEER_EFFECTIVE_MAX;
                    mag = steerPower_.step(mag, CTRL_PERIOD_S);
                    pwm_ = STEER_MOTOR_DIR * (int)lroundf(mag);
                    motor_.spin(pwm_);
                }
            } else {
                if (approaching_ && !coasting_) {                   // reached the landing point under power:
                    coasting_ = true;                               // cut late, and learn from this coast too
                    cutAngleDeg_ = steerDeg_;
                    predictor_.onCut(rateDps_);
                }
                approaching_ = false;
                motor_.spin(0);
                pwm_ = 0;
                steer_.reset();                                     // clear I and D
                learnIfStopped();
                if (coasting_) steerOkSinceMs_ = millis();          // still rolling: not settled yet
                // the body still swinging after the steer moves where the wheel really points:
                // wait for the IMU to say it is still, but not forever (a vibrating robot)
                if (fabsf(bodyRateDps_) > STEER_SETTLE_BODY_DPS && millis() - aimedSinceMs_ < STEER_SETTLE_MAX_MS)
                    steerOkSinceMs_ = millis();
                if (millis() - steerOkSinceMs_ >= STEER_SETTLE_MS) {
                    spin_.reset();
                    if (fabsf(targetRpm_) <= 1e-3f) {
                        steerOkSinceMs_ = millis();                 // aimed, nothing to drive
                    } else {
                        mode_ = Mode::Drive;
                        driveSinceMs_ = millis();
                        progressM_ = 0.0f;
                        lastDeltaM_ = 0.0f;
                    }
                }
            }
            break;
        }
        case Mode::Drive: {
            if (fabsf(steerErrDeg_) > DRIVE_RELOCK_ERROR_DEG) {     // wheel drifted: aim again
                motor_.spin(0);
                spin_.reset();
                steer_.reset();
                pwm_ = 0;
                mode_ = Mode::Steer;
                steerAimed_ = false;
                steerOkSinceMs_ = millis();
                break;
            }

            const float rpmMeas = fabsf(rpm_);
            float rpmSet = fabsf(targetRpm_);
            bool reached = false;

            if (hasTarget_) {                                       // distance along the heading
                progressM_ += lastDeltaM_;
                const float remaining = targetDistM_ - progressM_;
                const float tol = fabsf(targetTolM_);
                reached = targetDistM_ >= 0.0f ? remaining <= tol : remaining >= -tol;
            }
            if (goalActive_) {                                      // the goal point itself
                const float dxg = goalX_ - odom_.x();
                const float dyg = goalY_ - odom_.y();
                const float along = (odom_.x() - startX_) * goalVecX_ + (odom_.y() - startY_) * goalVecY_;
                const bool atGoal = sqrtf(dxg * dxg + dyg * dyg) <= targetTolM_;
                const bool overshot = goalLenSq_ > 1e-6f && along >= goalLenSq_;
                reached = reached || atGoal || overshot;
            }
            // It cannot steer while it drives (one motor). If the line it drives on
            // passes the goal more than the goal radius to the left, stop while a
            // small left turn still fixes it (algorithm/FinalApproach.h).
            if (!reached && reaimOn_ && goalActive_) {
                float ahead = 0.0f, left = 0.0f;
                finalapproach::goalInWheelFrame(goalX_ - odom_.x(), goalY_ - odom_.y(),
                                                deg2rad(wrap360(steerDeg_ - STEER_CMD_ZERO_DEG + headingMeasDeg_)),
                                                ahead, left);
                if (finalapproach::stopToReaim(ahead, left, targetTolM_, reaimRatio_, DRIVE_REAIM_MIN_M)) {
                    reached = true;                                 // the planner aims again from here
                    ++reaims_;
                }
            }
            if (reached) {
                targetRpm_ = 0.0f;
                rpmSet = 0.0f;
                stopGoal();
                mode_ = Mode::Steer;
                steerOkSinceMs_ = millis();
            }

            if (rpmSet <= 1e-3f) {
                motor_.spin(0);
                pwm_ = 0;
                spin_.reset();
                stopGoal();
                mode_ = Mode::Steer;
                steerOkSinceMs_ = millis();
            } else {
                const float ff = gainLearner_.feedforward(rpmSet, Wheel_SPIN_KS, pidSpin_[3]);
                float mag = spin_.compute_with_feedforward(rpmSet, rpmMeas, ff);
                if (mag < 0.0f) mag = 0.0f;
                if (mag > (float)PWM_SPIN_Max) mag = (float)PWM_SPIN_Max;
                const int dir = targetRpm_ >= 0.0f ? DRIVE_MOTOR_DIR : -DRIVE_MOTOR_DIR;
                pwm_ = dir * (int)lroundf(mag);
                motor_.spin(pwm_);
                if (mag > spin_.outputMin() + 1.0f) {
                    gainLearner_.observe(rpmSet, rpmMeas, fabsf((float)pwm_), Wheel_SPIN_KS, pidSpin_[3],
                                         spin_.outputMax(), CTRL_PERIOD_S, (millis() - driveSinceMs_) * 0.001f);
                }
            }
            break;
        }
    }
    if (pwm_ == 0 || mode_ != Mode::Steer) steerPower_.reset();
    const int responseMode = pwm_ == 0 ? 0 : (mode_ == Mode::Drive ? 2 : 1);
    if (responseWatch_.observe(responseMode, (float)pwm_, (int32_t)ticks_, steerDeg_, CTRL_PERIOD_S)) halt();
    fillState();
}

// After an early cut: once the wheel has stopped, learn how far it coasted.
void SteerDriveController::learnIfStopped() {
    if (!coasting_ || fabsf(rateDps_) >= 2.0f) return;
    coasting_ = false;
    predictor_.onStopped(cwErrorDeg(steerDeg_, cutAngleDeg_));   // distance turned since the cut
}

void SteerDriveController::fillState() {
    RobotState& s = state_;
    s.stampMs = millis();
    s.ticks = (int32_t)ticks_;
    s.rpm = rpm_;
    s.steerDeg = steerDeg_;
    s.steerOk = steerSensor_.ok();
    s.steerAimed = steerAimed_;
    s.steerGlitches = steerSensor_.glitches();
    s.imuYawDeg = imu_.yawDeg();
    s.imuOk = imu_.receiving();      // data is coming; the zero is taken at the first command
    for (unsigned axis = 0; axis < 3; ++axis) {
        s.imuGyroDps[axis] = imu_.gyroDps(axis);
        s.imuAccelMps2[axis] = imu_.accelMps2(axis);
    }
    s.imuGyroSeq = imu_.gyroSeq(); s.imuAccelSeq = imu_.accelSeq();
    s.imuMotionFlags = (imu_.gyroFresh() ? 1 : 0) | (imu_.accelFresh() ? 2 : 0);
    s.headingDeg = headingMeasDeg_;
    s.wheelHeadingDeg = wrap360(steerDeg_ - STEER_CMD_ZERO_DEG + headingMeasDeg_);
    s.imuHeadingFresh = imu_.headingFresh();
    s.x = odom_.x();
    s.y = odom_.y();
    s.thetaRad = odom_.theta();
    s.vx = odom_.vx();
    s.wz = odom_.wz();
    s.driving = mode_ == Mode::Drive;
    s.halted = halted_;
    s.overshot = overshot_;
    s.overshootDeg = overshootDeg_;
    s.targetRpm = targetRpm_;
    s.targetHeadingDeg = targetHeadingDeg_;
    s.steerTargetDeg = steerTargetDeg_;
    s.steerErrDeg = steerErrDeg_;
    s.steerRateDps = rateDps_;
    s.bodyRateDps = bodyRateDps_;
    s.reaims = reaims_;
    s.coasting = coasting_;
    s.coastS = predictor_.coastS();
    s.coastSamples = predictor_.samples();
    s.driveGain = gainLearner_.gain();
    s.driveLearnedS = gainLearner_.learnedS();
    s.motionFault = (uint8_t)responseWatch_.fault();
    s.pwm = pwm_;
    s.goalActive = goalActive_;
    s.goalX = goalX_;
    s.goalY = goalY_;
    s.source = source_;
}
