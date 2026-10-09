#include "nav/WaypointRunner.h"
#include <math.h>
#include <config.h>
#include "algorithm/PlannerFactory.h"
#include "app_config.h"
#include "util/Angles.h"

static const char* statusName(WaypointRunner::Status s) {
    switch (s) {
        case WaypointRunner::Status::Running: return "running";
        case WaypointRunner::Status::Done: return "done";
        case WaypointRunner::Status::Stopped: return "stopped";
        case WaypointRunner::Status::Failed: return "failed";
        default: return "idle";
    }
}

// wheel rpm for a ground speed (same as drive_to_xy.py mps_to_rpm)
static float rpmFor(float mps) { return mps / ((float)M_PI * WHEEL_DIAMETER) * 60.0f; }

void WaypointRunner::begin(ControlLoop* ctrl, Settings* settings) {
    ctrl_ = ctrl;
    settings_ = settings;
    mtx_ = xSemaphoreCreateMutex();
    prefs_.begin("nav", false);
    load();
}

// ---- points -----------------------------------------------------------------

bool WaypointRunner::setPoints(const Waypoint* pts, size_t n, String& err) {
    if (n > NAV_MAX_POINTS) { err = "จุดได้ไม่เกิน " + String(NAV_MAX_POINTS) + " จุด"; return false; }
    for (size_t i = 0; i < n; ++i) {
        if (!isfinite(pts[i].x) || !isfinite(pts[i].y) || fabsf(pts[i].x) > 50.0f || fabsf(pts[i].y) > 50.0f) {
            err = "จุดที่ " + String(i + 1) + " อยู่นอกช่วง ±50 m";
            return false;
        }
    }
    lock();
    if (status_ == Status::Running) { unlock(); err = "หยุดเส้นทางก่อน แล้วค่อยแก้จุด"; return false; }
    for (size_t i = 0; i < n; ++i) pts_[i] = pts[i];
    count_ = n;
    save();
    unlock();
    return true;
}

void WaypointRunner::pointsJson(JsonArray out) {
    lock();
    for (uint8_t i = 0; i < count_; ++i) {
        JsonObject p = out.add<JsonObject>();
        p["x"] = pts_[i].x;
        p["y"] = pts_[i].y;
    }
    unlock();
}

void WaypointRunner::save() {
    prefs_.putUChar("n", count_);
    if (count_) prefs_.putBytes("pts", pts_, sizeof(Waypoint) * count_);
}

void WaypointRunner::load() {
    const uint8_t n = prefs_.getUChar("n", 0);
    if (n == 0 || n > NAV_MAX_POINTS || prefs_.getBytesLength("pts") != sizeof(Waypoint) * n) return;
    prefs_.getBytes("pts", pts_, sizeof(Waypoint) * n);
    count_ = n;
}

// ---- run / stop ---------------------------------------------------------------

void WaypointRunner::prepare(const SettingsData& s) {
    status_ = Status::Running;
    message_ = "กำลังวิ่ง";
    idx_ = 0;
    tries_ = 0;
    overshoots_ = 0;
    legActive_ = false;
    loop_ = s.navLoop;
    speedMps_ = s.navSpeedMps;
    tolM_ = s.navTolM;
    plannerName_ = s.planner;
    params_.driveMps = s.navSpeedMps;
    params_.steerDps = s.steerDps;
    params_.settleS = DETOUR_SETTLE_S;
    params_.stopS = DETOUR_STOP_S;
    heartbeatMs_ = millis();
    localStartMs_ = millis();
    commandPending_ = false;
    // the same 3 s rule, enforced in the control task too (loop() may be blocked);
    // a demo started from the robot's button has its own stop (the button) instead
    if (!local_) ctrl_->armWatchdog(WEB_HEARTBEAT_TIMEOUT_MS, "หน้าเว็บขาดการเชื่อมต่อ - หยุดเพื่อความปลอดภัย");
}

bool WaypointRunner::start(String& err) {
    const SettingsData s = settings_->get();
    lock();
    if (status_ == Status::Running) { unlock(); err = "หยุดเส้นทางก่อนเริ่มใหม่"; return false; }
    if (count_ == 0) { unlock(); err = "ยังไม่มีจุด: คลิกบนระนาบเพื่อวางจุด"; return false; }
    local_ = false;
    prepare(s);
    testPhase_ = TestPhase::Idle;
    testTimed_ = testValid_ = false;
    testElapsedMs_ = 0;
    unlock();
    return true;
}

bool WaypointRunner::testSensorsOk(const RobotState& s, uint32_t now) const {
    if (!s.steerOk || !s.imuOk || !s.imuHeadingFresh || (s.imuMotionFlags & 3) != 3 || now - s.stampMs > 100 ||
        !isfinite(s.wheelHeadingDeg) || !isfinite(s.headingDeg) || !isfinite(s.thetaRad) ||
        !isfinite(s.x) || !isfinite(s.y) || !isfinite(s.rpm) || !isfinite(s.steerRateDps)) return false;
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(s.imuGyroDps[i]) || !isfinite(s.imuAccelMps2[i])) return false;
    }
    return true;
}

bool WaypointRunner::startTest(const String& planner, float heading, bool ready, String& err, bool byButton) {
    if (!ready) { err = "ยืนยันว่าอยู่ข้างหุ่นและวางหุ่นที่จุดเริ่มต้นก่อน"; return false; }
    if (planner != "direct" && planner != "detour") { err = "เลือกแบบที่ 1 direct หรือแบบที่ 2 detour"; return false; }
    if (!isfinite(heading) || heading < 0.0f || heading > 360.0f) { err = "มุมเริ่มต้นต้องอยู่ในช่วง 0-360 องศา"; return false; }
    const SettingsData settings = settings_->get();
    lock();
    if (status_ == Status::Running) { unlock(); err = "หยุดเส้นทางก่อนเริ่มเทส"; return false; }
    if (count_ == 0) { unlock(); err = "บันทึกจุดเส้นทางก่อนเริ่มเทส"; return false; }
    const RobotState s = ctrl_->snapshot();
    if (!s.halted || s.pwm != 0 || fabsf(s.rpm) >= 0.5f || fabsf(s.steerRateDps) >= 2.0f) {
        unlock(); err = "หยุดหุ่นและรอให้ล้อหยุดนิ่งก่อนเริ่มเทส"; return false;
    }
    if (!testSensorsOk(s, millis())) { unlock(); err = "รอเซนเซอร์มุมล้อและ IMU ส่งข้อมูลใหม่ก่อนเริ่มเทส"; return false; }
    bool needsMotion = false;
    for (uint8_t i = 0; i < count_; ++i) {
        if (hypotf(pts_[i].x - s.x, pts_[i].y - s.y) > settings.navTolM) needsMotion = true;
    }
    if (!needsMotion) { unlock(); err = "ทุกจุดอยู่ในระยะถึงแล้ว เลือกจุดที่ต้องเคลื่อนที่ก่อน"; return false; }
    float pid[5];
    const uint32_t pidRevision = ctrl_->pidRevision();
    ctrl_->getPid(true, pid);
    if (ctrl_->pidRevision() != pidRevision) {
        unlock(); err = "PID เปลี่ยนระหว่างเตรียมเทส กรุณาเริ่มใหม่"; return false;
    }
    if (!isfinite(pid[4]) || pid[4] <= 0.0f || pid[4] >= 180.0f) {
        unlock(); err = "ค่าความคลาดเคลื่อนมุมล้อไม่เหมาะกับการเทส"; return false;
    }
    local_ = byButton;
    prepare(settings);
    loop_ = false;
    plannerName_ = planner;
    testPhase_ = TestPhase::Aligning;
    if (++testId_ == 0) ++testId_;                 // boot-local trial identity
    testHeadingDeg_ = angles::wrap360(heading);
    testSteerTolDeg_ = pid[4];
    testPidRevision_ = pidRevision;
    testPrepMs_ = millis();
    testStable_ = testTimed_ = testValid_ = false;
    testElapsedMs_ = testMaxUpdateGapMs_ = 0;
    message_ = "กำลังตั้งมุมล้อร่วมก่อนจับเวลา";
    DriveCommand cmd;
    cmd.headingDeg = testHeadingDeg_;
    cmd.stopOnOvershoot = true;
    sendCommand(cmd, s);
    unlock();
    return true;
}

void WaypointRunner::sendCommand(const DriveCommand& cmd, const RobotState& before) {
    commandStampMs_ = before.stampMs;
    commandPending_ = true;
    ctrl_->command(cmd, CommandSource::Web);
}

void WaypointRunner::alignTest(const RobotState& s, uint32_t now) {
    if (now - testPrepMs_ >= 30000) {
        finish(Status::Failed, "ตั้งมุมล้อไม่สำเร็จภายใน 30 วินาที", true);
        return;
    }
    const float gx = s.imuGyroDps[0], gy = s.imuGyroDps[1], gz = s.imuGyroDps[2];
    const bool still = s.pwm == 0 && !s.coasting && fabsf(s.rpm) < 0.5f &&
                       fabsf(s.steerRateDps) < 2.0f && gx * gx + gy * gy + gz * gz < 25.0f &&
                       s.steerAimed &&   // the controller's own aimed state (tolerance + hysteresis)
                       fabsf(angles::errDeg(testHeadingDeg_, s.wheelHeadingDeg)) <= testSteerTolDeg_ + STEER_TOL_HYST_DEG;
    if (!still) { testStable_ = false; return; }
    if (!testStable_ || now - testStableSampleMs_ > 100) {
        testStable_ = true; testStableMs_ = testStableSampleMs_ = now; return;
    }
    testStableSampleMs_ = now;
    if (now - testStableMs_ < 200) return;
    testActualHeadingDeg_ = s.wheelHeadingDeg;
    testStartX_ = s.x;
    testStartY_ = s.y;
    testStartThetaDeg_ = angles::rad2deg(s.thetaRad);
    testPhase_ = TestPhase::Running;
    testTimed_ = true;
    testStartMs_ = millis();                      // before first route command; preparation excluded
    testMaxUpdateGapMs_ = 0;
    message_ = "กำลังเทสและจับเวลาบนหุ่น";
    planNext(s);
}

void WaypointRunner::finish(Status st, const char* why, bool haltWheel) {
    ctrl_->disarmWatchdog();
    if (testPhase_ == TestPhase::Aligning || testPhase_ == TestPhase::Running) {
        // PID writes come from another task; check again at the result boundary
        // in case a write happened after update() checked its snapshot.
        if (st == Status::Done && ctrl_->pidRevision() != testPidRevision_) {
            st = Status::Failed;
            why = "PID เปลี่ยนระหว่างเทส - ผลใช้เปรียบเทียบไม่ได้";
            haltWheel = true;
        }
        testElapsedMs_ = testTimed_ ? millis() - testStartMs_ : 0;
        testValid_ = st == Status::Done && testTimed_;
        testPhase_ = st == Status::Done ? TestPhase::Done :
                     (st == Status::Failed ? TestPhase::Failed : TestPhase::Stopped);
    }
    status_ = st;
    message_ = why;
    legActive_ = false;
    commandPending_ = false;
    if (restorePts_) {                       // demo 1 borrowed pts_: give the web route back
        for (uint8_t i = 0; i < savedCount_; ++i) pts_[i] = savedPts_[i];
        count_ = savedCount_;
        restorePts_ = false;
    }
    if (haltWheel) ctrl_->halt(why);
}

bool WaypointRunner::startDemoSquare(float side, String& err) {
    const SettingsData s = settings_->get();
    lock();
    if (status_ == Status::Running) { unlock(); err = "หยุดเส้นทางก่อน"; return false; }
    const RobotState st = ctrl_->snapshot();
    if (!st.halted && (st.pwm != 0 || fabsf(st.rpm) >= 0.5f)) { unlock(); err = "หยุดหุ่นก่อน"; return false; }
    for (uint8_t i = 0; i < count_; ++i) savedPts_[i] = pts_[i];
    savedCount_ = count_;
    restorePts_ = true;
    pts_[0] = {side, 0.0f};                  // forward
    pts_[1] = {side, side};                  // then left (+y)
    pts_[2] = {0.0f, 0.0f};                  // back to the start
    count_ = 3;
    ctrl_->resetPose();                      // here becomes (0,0), +x = where the robot faces
    local_ = true;
    prepare(s);
    loop_ = false;
    testPhase_ = TestPhase::Idle;
    testTimed_ = testValid_ = false;
    testElapsedMs_ = 0;
    char msg[96];
    snprintf(msg, sizeof(msg), "เดโม 1: หน้า %.1f ม. -> ซ้าย %.1f ม. -> กลับจุดเริ่ม", side, side);
    message_ = msg;
    unlock();
    return true;
}

void WaypointRunner::noteButton(const String& text, uint8_t clicks) {
    lock();
    buttonText_ = text;
    buttonClicks_ = clicks;
    buttonMs_ = millis();
    unlock();
}

void WaypointRunner::stop(const char* why) {
    lock();
    finish(Status::Stopped, why, true);
    unlock();
}

void WaypointRunner::cancelForRos() {
    lock();
    if (status_ == Status::Running) finish(Status::Stopped, "ROS สั่งงานแทน", false);
    unlock();
}

void WaypointRunner::heartbeat() {
    lock();
    heartbeatMs_ = millis();
    unlock();
    ctrl_->feedWatchdog();
}

bool WaypointRunner::running() {
    lock();
    const bool r = status_ == Status::Running;
    unlock();
    return r;
}

// ---- the loop -------------------------------------------------------------------

void WaypointRunner::update() {
    const uint32_t now = millis();
    if (now - lastUpdateMs_ < 50) return;           // 20 Hz is plenty to follow legs
    const uint32_t updateGapMs = now - lastUpdateMs_;
    lastUpdateMs_ = now;

    lock();
    if (status_ != Status::Running) { unlock(); return; }
    if (testPhase_ == TestPhase::Running && updateGapMs > testMaxUpdateGapMs_) testMaxUpdateGapMs_ = updateGapMs;
    if (local_ && now - localStartMs_ > DEMO_MAX_MS) {
        finish(Status::Stopped, "เดโมเกินเวลาที่กำหนด - หยุด", true);
        unlock();
        return;
    }
    if (!local_ && now - heartbeatMs_ > WEB_HEARTBEAT_TIMEOUT_MS) {
        finish(Status::Stopped, "หน้าเว็บขาดการเชื่อมต่อ - หยุดเพื่อความปลอดภัย", true);
        unlock();
        return;
    }
    const RobotState s = ctrl_->snapshot();
    const uint32_t sampleNow = millis();          // snapshot may be newer than now after waiting for its mutex
    const bool testing = testPhase_ == TestPhase::Aligning || testPhase_ == TestPhase::Running;
    if (testing && ctrl_->pidRevision() != testPidRevision_) {
        finish(Status::Failed, "PID เปลี่ยนระหว่างเทส - ผลใช้เปรียบเทียบไม่ได้", true);
        unlock(); return;
    }
    if (testing && !testSensorsOk(s, sampleNow)) {
        finish(Status::Failed, "เซนเซอร์ขาดข้อมูลระหว่างเทส - หยุด", true);
        unlock(); return;
    }
    // apply() is synchronous but snapshot() is published at the next 10 ms tick.
    // An old halted/goal-free state must never complete or cancel a new command.
    if (commandPending_) {
        if (s.stampMs == commandStampMs_) { unlock(); return; }
        commandPending_ = false;
    }
    if (testPhase_ == TestPhase::Aligning) {
        if (s.source != CommandSource::Web) finish(Status::Stopped, "ROS สั่งงานแทน", false);
        else if (s.halted) finish(s.motionFault || s.overshot ? Status::Failed : Status::Stopped, "ล้อหยุดระหว่างตั้งมุมก่อนเทส", false);
        else alignTest(s, sampleNow);
        unlock(); return;
    }
    if (legActive_) {
        if (s.source != CommandSource::Web) {
            finish(Status::Stopped, "ROS สั่งงานแทน", false);
        } else if (s.overshot) {
            // the wheel passed its angle; plan again from where it stopped
            ++overshoots_;
            legActive_ = false;
            planNext(s);
        } else if (s.halted) {
            finish(Status::Stopped, "หยุดฉุกเฉิน", false);
        } else if (!s.goalActive && fabsf(s.targetRpm) < 1e-3f) {
            legActive_ = false;                     // leg finished
            planNext(s);
        }
    } else {
        planNext(s);
    }
    unlock();
}

void WaypointRunner::planNext(const RobotState& s) {
    while (true) {
        const Waypoint& wp = pts_[idx_];
        const float dx = wp.x - s.x, dy = wp.y - s.y;
        const float dist = sqrtf(dx * dx + dy * dy);
        if (dist > tolM_) break;
        tries_ = 0;                                 // reached this point
        if (++idx_ >= count_) {
            if (!loop_) { finish(Status::Done, "ถึงจุดสุดท้ายแล้ว", true); return; }
            idx_ = 0;
        }
    }
    if (tries_ >= NAV_MAX_RETRIES) {
        finish(Status::Failed, "เล็งจุดนี้หลายครั้งแล้วยังไม่ถึง - หยุด", true);
        return;
    }

    const Waypoint& wp = pts_[idx_];
    const float dx = wp.x - s.x, dy = wp.y - s.y;
    const float dist = sqrtf(dx * dx + dy * dy);
    const float bearing = angles::wrap360(angles::rad2deg(atan2f(dy, dx)));
    // how far the wheel must still turn (it steers one way only). Already inside
    // the steering tolerance = aimed: the controller will not steer, so no plan
    // may count (or detour around) a near-full turn that will never happen.
    float phi = angles::cwErrorDeg(bearing, s.wheelHeadingDeg);
    if (angles::cwInTolerance(phi, Wheel_STEER_ERROR_TOLERANCE)) phi = 0.0f;

    const LegPlanner& planner = PlannerFactory::get(plannerName_.c_str());
    const LegPlan plan = planner.plan(phi, dist, params_);

    DriveCommand cmd;
    cmd.rpm = rpmFor(speedMps_);
    cmd.tolM = tolM_;
    cmd.stopOnOvershoot = plannerName_ == "detour";
    if (plan.kind == LegPlan::Detour) {
        cmd.headingDeg = s.wheelHeadingDeg;         // leg 1: straight on, no steer
        cmd.distM = plan.a;
    } else {
        cmd.headingDeg = bearing;
        cmd.distM = dist;
    }
    sendCommand(cmd, s);
    legActive_ = true;
    ++tries_;
    lastPlan_ = plan;
    lastPhiDeg_ = phi;
    lastDistM_ = dist;
}

void WaypointRunner::statusJson(JsonObject o) {
    lock();
    o["status"] = statusName(status_);
    o["message"] = message_;
    o["byButton"] = local_;
    JsonObject btn = o["button"].to<JsonObject>();
    btn["text"] = buttonText_;
    btn["clicks"] = buttonClicks_;
    btn["ageMs"] = buttonMs_ ? millis() - buttonMs_ : 0;
    btn["pressed"] = (bool)buttonPressed_;
    o["index"] = idx_;
    o["count"] = count_;
    o["loop"] = loop_;
    o["planner"] = plannerName_;
    o["tries"] = tries_;
    o["overshoots"] = overshoots_;
    o["heartbeatAgeMs"] = status_ == Status::Running ? millis() - heartbeatMs_ : 0;
    JsonObject t = o["test"].to<JsonObject>();
    static const char* const testPhases[] = {"idle", "aligning", "running", "done", "stopped", "failed"};
    t["id"] = testId_;
    t["pidRevision"] = testPidRevision_;
    t["active"] = testPhase_ == TestPhase::Aligning || testPhase_ == TestPhase::Running;
    t["phase"] = testPhases[static_cast<uint8_t>(testPhase_)];
    t["planner"] = plannerName_;
    t["startHeadingDeg"] = testHeadingDeg_;
    if (testTimed_) {
        t["actualStartHeadingDeg"] = testActualHeadingDeg_;
        t["startX"] = testStartX_;
        t["startY"] = testStartY_;
        t["startThetaDeg"] = testStartThetaDeg_;
    } else {
        t["actualStartHeadingDeg"] = nullptr;
        t["startX"] = nullptr; t["startY"] = nullptr; t["startThetaDeg"] = nullptr;
    }
    t["speedMps"] = speedMps_;
    t["tolM"] = tolM_;
    t["steerDps"] = params_.steerDps;
    t["elapsedMs"] = testPhase_ == TestPhase::Running ? millis() - testStartMs_ : testElapsedMs_;
    t["valid"] = testValid_;
    t["observationMaxGapMs"] = testMaxUpdateGapMs_;
    JsonObject p = o["plan"].to<JsonObject>();
    p["kind"] = lastPlan_.kind == LegPlan::Detour ? "detour" : "direct";
    p["phiDeg"] = lastPhiDeg_;
    p["distM"] = lastDistM_;
    p["k"] = lastPlan_.k;
    p["a"] = lastPlan_.a;
    p["betaDeg"] = lastPlan_.betaDeg;
    p["b"] = lastPlan_.b;
    p["timeS"] = lastPlan_.timeS;
    unlock();
}
