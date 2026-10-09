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

// demo 3/4 time line: what the robot is visibly doing now. The controller's own
// mode decides drive vs turn: the drive encoder also counts while the wheel steers
// (measured 2026-10-09), so rpm alone would call a turn in place "driving".
static ActivityLog<32>::Act activityOf(const RobotState& s) {
    if (s.halted) return ActivityLog<32>::Still;
    if (s.driving) return fabsf(s.rpm) >= DEMO_DRIVE_RPM ? ActivityLog<32>::Drive : ActivityLog<32>::Still;
    return fabsf(s.steerRateDps) >= DEMO_TURN_DPS ? ActivityLog<32>::Turn : ActivityLog<32>::Still;
}

void WaypointRunner::begin(ControlLoop* ctrl, Settings* settings) {
    ctrl_ = ctrl;
    settings_ = settings;
    mtx_ = xSemaphoreCreateMutex();
    prefs_.begin("nav", false);
    load();
}

// ---- points -----------------------------------------------------------------

bool WaypointRunner::setPoints(const Waypoint* pts, size_t n, String& err, uint8_t slot) {
    if (slot > DEMO_ROUTES) { err = "ไม่มีเส้นทางนี้"; return false; }
    if (n > NAV_MAX_POINTS) { err = "จุดได้ไม่เกิน " + String(NAV_MAX_POINTS) + " จุด"; return false; }
    for (size_t i = 0; i < n; ++i) {
        if (!isfinite(pts[i].x) || !isfinite(pts[i].y) || fabsf(pts[i].x) > 50.0f || fabsf(pts[i].y) > 50.0f) {
            err = "จุดที่ " + String(i + 1) + " อยู่นอกช่วง ±50 m";
            return false;
        }
        if (!isfinite(pts[i].waitS) || pts[i].waitS < 0.0f || pts[i].waitS > NAV_MAX_WAIT_S) {
            err = "จุดที่ " + String(i + 1) + ": เวลารอต้องอยู่ระหว่าง 0-" + String((int)NAV_MAX_WAIT_S) + " วินาที";
            return false;
        }
    }
    lock();
    if (slot) {                                     // a button route: a running demo keeps its own copy
        for (size_t i = 0; i < n; ++i) routes_[slot - 1][i] = pts[i];
        routeCount_[slot - 1] = n;
        saveRoute(slot);
        unlock();
        return true;
    }
    if (status_ == Status::Running) { unlock(); err = "หยุดเส้นทางก่อน แล้วค่อยแก้จุด"; return false; }
    for (size_t i = 0; i < n; ++i) pts_[i] = pts[i];
    count_ = n;
    save();
    unlock();
    return true;
}

void WaypointRunner::pointsJson(JsonArray out, uint8_t slot) {
    if (slot > DEMO_ROUTES) return;
    lock();
    // while a button demo runs, pts_ is borrowed and the web route waits in savedPts_
    const Waypoint* src = slot ? routes_[slot - 1] : (restorePts_ ? savedPts_ : pts_);
    const uint8_t n = slot ? routeCount_[slot - 1] : (restorePts_ ? savedCount_ : count_);
    for (uint8_t i = 0; i < n; ++i) {
        JsonObject p = out.add<JsonObject>();
        p["x"] = src[i].x;
        p["y"] = src[i].y;
        p["waitS"] = src[i].waitS;
    }
    unlock();
}

void WaypointRunner::save() {
    prefs_.putUChar("n", count_);
    if (count_) prefs_.putBytes("pts", pts_, sizeof(Waypoint) * count_);
}

void WaypointRunner::saveRoute(uint8_t slot) {
    char kn[6], kp[6];
    snprintf(kn, sizeof(kn), "r%un", (unsigned)slot);
    snprintf(kp, sizeof(kp), "r%up", (unsigned)slot);
    prefs_.putUChar(kn, routeCount_[slot - 1]);
    if (routeCount_[slot - 1]) prefs_.putBytes(kp, routes_[slot - 1], sizeof(Waypoint) * routeCount_[slot - 1]);
}

// Saved points are x, y, waitS (12 bytes each). Firmware before 2026-10-09 saved
// x, y only (8 bytes): read those with no stops, so an update keeps the routes.
bool WaypointRunner::readPoints(const char* key, uint8_t n, Waypoint* out) {
    const size_t len = prefs_.getBytesLength(key);
    if (len == sizeof(Waypoint) * n) {
        prefs_.getBytes(key, out, len);
    } else if (len == sizeof(float) * 2 * n) {
        float xy[NAV_MAX_POINTS * 2];
        prefs_.getBytes(key, xy, len);
        for (uint8_t i = 0; i < n; ++i) out[i] = {xy[2 * i], xy[2 * i + 1], 0.0f};
    } else {
        return false;
    }
    for (uint8_t i = 0; i < n; ++i) {
        if (!isfinite(out[i].waitS) || out[i].waitS < 0.0f || out[i].waitS > NAV_MAX_WAIT_S) out[i].waitS = 0.0f;
    }
    return true;
}

void WaypointRunner::load() {
    // button routes; never saved = forward, left, back to the start
    static_assert(DEMO_ROUTES == 2, "one default route per button slot");
    const float sides[DEMO_ROUTES] = {DEMO_SQUARE_M, DEMO_TRIANGLE_M};
    for (uint8_t slot = 1; slot <= DEMO_ROUTES; ++slot) {
        Waypoint* r = routes_[slot - 1];
        char kn[6], kp[6];
        snprintf(kn, sizeof(kn), "r%un", (unsigned)slot);
        snprintf(kp, sizeof(kp), "r%up", (unsigned)slot);
        const uint8_t n = prefs_.getUChar(kn, 0xFF);   // 0xFF = never saved (0 = cleared on purpose)
        if (n <= NAV_MAX_POINTS && (n == 0 || readPoints(kp, n, r))) {
            routeCount_[slot - 1] = n;
        } else {
            const float side = sides[slot - 1];
            r[0] = {side, 0.0f, 0.0f};
            r[1] = {side, side, 0.0f};
            r[2] = {0.0f, 0.0f, 0.0f};
            routeCount_[slot - 1] = 3;
        }
    }

    const uint8_t n = prefs_.getUChar("n", 0);
    if (n == 0 || n > NAV_MAX_POINTS || !readPoints("pts", n, pts_)) return;
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
    waiting_ = false;
    // the same 3 s rule, enforced in the control task too (loop() may be blocked);
    // a demo started from the robot's button has its own stop (the button) instead
    if (!local_) ctrl_->armWatchdog(WEB_HEARTBEAT_TIMEOUT_MS, "หน้าเว็บขาดการเชื่อมต่อ - หยุดเพื่อความปลอดภัย");
}

bool WaypointRunner::start(String& err) {
    const SettingsData s = settings_->get();
    lock();
    if (status_ == Status::Running) { unlock(); err = "หยุดเส้นทางก่อนเริ่มใหม่"; return false; }
    if (count_ == 0) { unlock(); err = "ยังไม่มีจุด: คลิกบนระนาบเพื่อวางจุด"; return false; }
    local_ = compare_ = false;
    returnHome_ = homing_ = false;
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

bool WaypointRunner::startTest(const String& planner, float heading, bool ready, String& err) {
    return beginTest(planner, heading, ready, err, false, false);
}

bool WaypointRunner::startCompare(const String& planner, String& err, bool byButton, bool ready) {
    return beginTest(planner, DEMO_START_HEADING_DEG, ready, err, true, byButton);
}

bool WaypointRunner::beginTest(const String& planner, float heading, bool ready, String& err, bool compare, bool local) {
    if (!ready) { err = "ยืนยันว่าอยู่ข้างหุ่นและวางหุ่นที่จุดเริ่มต้นก่อน"; return false; }
    if (planner != "direct" && planner != "detour") { err = "เลือกแบบที่ 1 direct หรือแบบที่ 2 detour"; return false; }
    if (!isfinite(heading) || heading < 0.0f || heading > 360.0f) { err = "มุมเริ่มต้นต้องอยู่ในช่วง 0-360 องศา"; return false; }
    const SettingsData settings = settings_->get();
    lock();
    if (status_ == Status::Running) { unlock(); err = "หยุดเส้นทางก่อนเริ่มเทส"; return false; }
    if (!compare && count_ == 0) { unlock(); err = "บันทึกจุดเส้นทางก่อนเริ่มเทส"; return false; }
    const RobotState s = ctrl_->snapshot();
    if (!s.halted || s.pwm != 0 || fabsf(s.rpm) >= 0.5f || fabsf(s.steerRateDps) >= 2.0f) {
        unlock(); err = "หยุดหุ่นและรอให้ล้อหยุดนิ่งก่อนเริ่มเทส"; return false;
    }
    if (!testSensorsOk(s, millis())) { unlock(); err = "รอเซนเซอร์มุมล้อและ IMU ส่งข้อมูลใหม่ก่อนเริ่มเทส"; return false; }
    bool needsMotion = compare;                  // the demo goal is DEMO_COMPARE_DIST_M away
    for (uint8_t i = 0; i < count_ && !compare; ++i) {
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
    local_ = local;                          // from the button: no web heartbeat
    compare_ = compare;
    testDemo_ = compare;
    returnHome_ = compare;                   // so the next demo can start from the same place
    homing_ = false;
    homeX_ = s.x;
    homeY_ = s.y;
    if (compare) {                           // goal placed in alignTest(); hold here until then
        borrowPts();
        pts_[0] = {s.x, s.y, 0.0f};
        count_ = 1;
    }
    prepare(settings);
    localLimitMs_ = DEMO_MAX_MS;             // demos 3/4: one short goal
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
    if (compare_) {   // demo 3/4: same goal for both, measured from where the wheel really points
        const float b = angles::deg2rad(s.wheelHeadingDeg - DEMO_COMPARE_RIGHT_DEG);
        pts_[0] = {s.x + DEMO_COMPARE_DIST_M * cosf(b), s.y + DEMO_COMPARE_DIST_M * sinf(b), 0.0f};
        count_ = 1;
        runIdx_ = plannerName_ == "detour" ? 1 : 0;
        CompareRun& r = runs_[runIdx_];
        r.id = testId_;
        r.open = true;
        r.valid = false;
        r.elapsedMs = 0;
        r.startX = s.x;
        r.startY = s.y;
        r.headingDeg = s.wheelHeadingDeg;
        r.goalX = pts_[0].x;
        r.goalY = pts_[0].y;
        r.speedMps = speedMps_;
        r.steerDps = params_.steerDps;
        r.tolM = tolM_;
        r.path.clear(DEMO_TRACE_STEP_M);
        r.path.add(s.x, s.y, true);
        r.acts.clear(DEMO_ACT_HOLD_MS);
        r.acts.update(0, activityOf(s));
        trackX_ = s.x;
        trackY_ = s.y;
    }
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
    closeRun();
    status_ = st;
    message_ = why;
    legActive_ = false;
    commandPending_ = false;
    waiting_ = false;
    returnHome_ = homing_ = compare_ = false;
    if (restorePts_) {                       // a button demo borrowed pts_: give the web route back
        for (uint8_t i = 0; i < savedCount_; ++i) pts_[i] = savedPts_[i];
        count_ = savedCount_;
        restorePts_ = false;
    }
    if (haltWheel) ctrl_->halt(why);
}

// Button demos: the route is done - record a timed result now, then drive back
// to where the demo started (not timed), so the next demo can run at once.
void WaypointRunner::beginReturnHome() {
    if (testPhase_ == TestPhase::Running) {
        const bool pidSame = ctrl_->pidRevision() == testPidRevision_;
        testElapsedMs_ = testTimed_ ? millis() - testStartMs_ : 0;
        testValid_ = testTimed_ && pidSame;
        testPhase_ = pidSame ? TestPhase::Done : TestPhase::Failed;
    }
    closeRun();
    borrowPts();
    pts_[0] = {homeX_, homeY_, 0.0f};
    count_ = 1;
    idx_ = 0;
    tries_ = 0;
    legActive_ = false;
    plannerName_ = "direct";
    homing_ = true;
    message_ = testTimed_ ? "กลับจุดเริ่ม (ไม่นับเวลา)" : "กลับจุดเริ่ม";
    if (compare_ && DEMO_COMPARE_HOLD_S > 0.0f) {   // demo 3/4: stay at the goal so people can see it
        waiting_ = true;
        waitAdvances_ = false;                      // afterwards plan towards home
        waitStartMs_ = millis();
        waitUntilMs_ = waitStartMs_ + (uint32_t)(DEMO_COMPARE_HOLD_S * 1000.0f);
        char msg[128];
        snprintf(msg, sizeof(msg), "ถึงเป้าแล้ว: หยุด %.0f วินาที แล้วกลับจุดเริ่ม", DEMO_COMPARE_HOLD_S);
        message_ = msg;
    }
}

void WaypointRunner::closeRun() {
    CompareRun& r = runs_[runIdx_];
    if (!compare_ || !r.open) return;
    r.path.add(trackX_, trackY_, true);       // where the timed part ended
    r.open = false;
    r.elapsedMs = testElapsedMs_;
    r.valid = testValid_;
}

void WaypointRunner::borrowPts() {
    if (restorePts_) return;                 // already borrowed: savedPts_ holds the web route
    for (uint8_t i = 0; i < count_; ++i) savedPts_[i] = pts_[i];
    savedCount_ = count_;
    restorePts_ = true;
}

bool WaypointRunner::startButtonRoute(uint8_t slot, String& err) {
    if (slot < 1 || slot > DEMO_ROUTES) { err = "ไม่มีเส้นทางนี้"; return false; }
    const SettingsData s = settings_->get();
    lock();
    if (status_ == Status::Running) { unlock(); err = "หยุดเส้นทางก่อน"; return false; }
    const uint8_t n = routeCount_[slot - 1];
    if (n == 0) {
        unlock();
        err = "เส้นทาง " + String((int)slot) + " ยังไม่มีจุด: ตั้งในเว็บ แท็บเส้นทาง";
        return false;
    }
    const RobotState st = ctrl_->snapshot();
    if (!st.halted && (st.pwm != 0 || fabsf(st.rpm) >= 0.5f)) { unlock(); err = "หยุดหุ่นก่อน"; return false; }
    borrowPts();
    for (uint8_t i = 0; i < n; ++i) pts_[i] = routes_[slot - 1][i];
    count_ = n;
    ctrl_->resetPose();                      // here becomes (0,0), +x = where the robot faces
    local_ = true;
    compare_ = false;
    returnHome_ = true;                      // end here, so the next demo can start at once
    homing_ = false;
    homeX_ = homeY_ = 0.0f;
    prepare(s);
    localLimitMs_ = demoLimitMs();
    loop_ = false;
    // until the next control tick the snapshot still holds the pose from before the reset
    commandStampMs_ = st.stampMs;
    commandPending_ = true;
    testPhase_ = TestPhase::Idle;
    testTimed_ = testValid_ = false;
    testElapsedMs_ = 0;
    char msg[96];
    snprintf(msg, sizeof(msg), "เส้นทาง %u: %u จุด เริ่มจากตรงนี้", (unsigned)slot, (unsigned)n);
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
    if (local_ && !waiting_ && now - localStartMs_ > localLimitMs_) {   // a stop is not moving time
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
    if (compare_ && testPhase_ == TestPhase::Running && runs_[runIdx_].open) {   // the path, for the web picture
        runs_[runIdx_].path.add(s.x, s.y);
        runs_[runIdx_].acts.update(sampleNow - testStartMs_, activityOf(s));
        trackX_ = s.x;
        trackY_ = s.y;
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
    if (waiting_) {                                 // stopped at a reached point for its waitS
        if (s.source == CommandSource::Ros) {
            finish(Status::Stopped, "ROS สั่งงานแทน", false);
        } else if ((int32_t)(sampleNow - waitUntilMs_) >= 0) {
            waiting_ = false;
            localStartMs_ += sampleNow - waitStartMs_;  // the stop does not count towards a button demo's limit
            message_ = homing_ ? (testTimed_ ? "กลับจุดเริ่ม (ไม่นับเวลา)" : "กลับจุดเริ่ม")
                               : (testTimed_ ? "กำลังเทสและจับเวลาบนหุ่น" : "กำลังวิ่ง");
            if (!waitAdvances_ || advance()) planNext(s);
        }
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
        if (!homing_ && wp.waitS > 0.0f) {          // stop here first; update() moves on afterwards
            waiting_ = true;
            waitAdvances_ = true;
            waitStartMs_ = millis();
            waitUntilMs_ = waitStartMs_ + (uint32_t)(wp.waitS * 1000.0f);
            char msg[128];
            snprintf(msg, sizeof(msg), "ถึงจุดที่ %u: หยุดรอ %.1f วินาที", (unsigned)idx_ + 1, wp.waitS);
            message_ = msg;
            return;
        }
        if (!advance()) return;
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

// A generous moving-time limit for a button route (see DEMO_MAX_MS in app_config.h):
// starts at (0,0) after the pose reset and drives back there at the end.
uint32_t WaypointRunner::demoLimitMs() const {
    if (!(speedMps_ > 0.001f)) return DEMO_MAX_MS;
    const float turnS = 360.0f / DEMO_SLOW_STEER_DPS;
    float x = 0.0f, y = 0.0f, s = 0.0f;
    for (uint8_t i = 0; i < count_; ++i) {
        s += hypotf(pts_[i].x - x, pts_[i].y - y) / speedMps_ + turnS;
        x = pts_[i].x;
        y = pts_[i].y;
    }
    s += hypotf(x, y) / speedMps_ + turnS;     // the drive back
    const float ms = DEMO_TIME_FACTOR * s * 1000.0f;
    if (ms <= (float)DEMO_MAX_MS) return DEMO_MAX_MS;
    if (ms >= (float)DEMO_LIMIT_CEIL_MS) return DEMO_LIMIT_CEIL_MS;
    return (uint32_t)ms;
}

void WaypointRunner::compareJson(JsonObject o) {
    lock();
    o["distM"] = DEMO_COMPARE_DIST_M;
    o["rightDeg"] = DEMO_COMPARE_RIGHT_DEG;
    static const char* const names[2] = {"direct", "detour"};
    for (uint8_t k = 0; k < 2; ++k) {
        const CompareRun& r = runs_[k];
        if (!r.id) { o[names[k]] = nullptr; continue; }
        JsonObject j = o[names[k]].to<JsonObject>();
        j["id"] = r.id;
        j["open"] = r.open;
        j["valid"] = r.valid;
        j["elapsedMs"] = r.open ? millis() - testStartMs_ : r.elapsedMs;
        j["startX"] = r.startX;
        j["startY"] = r.startY;
        j["headingDeg"] = r.headingDeg;
        j["goalX"] = r.goalX;
        j["goalY"] = r.goalY;
        j["speedMps"] = r.speedMps;
        j["steerDps"] = r.steerDps;
        j["tolM"] = r.tolM;
        static const char* const actNames[3] = {"still", "turn", "drive"};
        JsonArray acts = j["acts"].to<JsonArray>();     // [{t: ms since the start, a: what}]
        for (uint8_t i = 0; i < r.acts.size(); ++i) {
            JsonObject a = acts.add<JsonObject>();
            a["t"] = r.acts.t(i);
            a["a"] = actNames[r.acts.act(i)];
        }
        JsonArray path = j["path"].to<JsonArray>();
        for (uint8_t i = 0; i < r.path.size(); ++i) {
            JsonObject p = path.add<JsonObject>();
            p["x"] = roundf(r.path.x(i) * 10000.0f) / 10000.0f;   // 0.1 mm
            p["y"] = roundf(r.path.y(i) * 10000.0f) / 10000.0f;
        }
    }
    unlock();
}

bool WaypointRunner::advance() {
    if (++idx_ >= count_) {
        if (returnHome_ && !homing_) { beginReturnHome(); return !waiting_; }   // then (after a hold) home
        if (!loop_) { finish(Status::Done, homing_ ? "กลับถึงจุดเริ่มแล้ว" : "ถึงจุดสุดท้ายแล้ว", true); return false; }
        idx_ = 0;
    }
    return true;
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
    const int32_t waitLeft = (int32_t)(waitUntilMs_ - millis());
    o["waitLeftMs"] = waiting_ && waitLeft > 0 ? waitLeft : 0;
    o["heartbeatAgeMs"] = status_ == Status::Running ? millis() - heartbeatMs_ : 0;
    // button demo: moving time so far and the limit (seconds); 0 when not running from the button
    const bool demo = local_ && status_ == Status::Running;
    o["demoLimitS"] = demo ? localLimitMs_ / 1000 : 0;
    o["demoMovingS"] = demo ? ((waiting_ ? waitStartMs_ : millis()) - localStartMs_) / 1000 : 0;
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
    t["demo"] = testDemo_;                         // demo 3/4 from the button
    if (compare_ && testPhase_ == TestPhase::Running) {
        t["goalX"] = pts_[0].x;
        t["goalY"] = pts_[0].y;
    } else {
        t["goalX"] = nullptr; t["goalY"] = nullptr;
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
