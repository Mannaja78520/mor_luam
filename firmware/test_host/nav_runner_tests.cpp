#include "app_config.h"
#include <cmath>
// Tests the production WaypointRunner against a delayed control snapshot.
// See test_host/nav_runner_stubs: hardware/network/NVS are not involved.
#include "nav/WaypointRunner.h"
#include <cassert>
#include <iostream>
#include <limits>
uint32_t g_nav_ms = 1000;
struct Trial {
    ControlLoop ctrl;
    Settings settings;
    WaypointRunner runner;
    String err;
    Trial() {
        g_nav_ms = 1000;
        ctrl.state.stampMs = g_nav_ms;
        ctrl.state.steerOk = ctrl.state.imuOk = true;
        ctrl.state.steerAimed = true;          // the controller reports the wheel as aimed
        ctrl.state.imuHeadingFresh = true;
        ctrl.state.imuMotionFlags = 3;
        ctrl.applied = ctrl.state;
        runner.begin(&ctrl, &settings);
        const Waypoint point{0.3f, 0.03f};
        assert(runner.setPoints(&point, 1, err));
    }
    JsonNode status() { JsonNode out; runner.statusJson(JsonObject(&out)); return out; }
    std::string phase() { return status().children["test"].children["phase"].text; }
    double field(const char* key) { return status().children["test"].children[key].number; }
    void tick(uint32_t ms = 50, bool publish = true, bool heartbeat = true) {
        g_nav_ms += ms;
        if (publish) ctrl.publish();
        if (heartbeat) runner.heartbeat();
        runner.update();
    }
    void align(float actual = 0) {
        ctrl.applied.wheelHeadingDeg = actual;
        for (int i = 0; i < 5; ++i) tick();
        assert(phase() == "running");
    }
    void complete() {
        ctrl.applied.goalActive = false;
        ctrl.applied.targetRpm = 0;
        ctrl.applied.x = 0.3f; ctrl.applied.y = 0.03f;
        tick();
    }
};
int main() {
    {
        Trial t;
        assert(!t.runner.startTest("direct", 0, false, t.err));
        assert(!t.runner.startTest("unknown", 0, true, t.err));
        assert(!t.runner.startTest("direct", -1, true, t.err));
        assert(!t.runner.startTest("direct", std::numeric_limits<float>::quiet_NaN(), true, t.err));
        t.ctrl.state.imuMotionFlags = 0;
        assert(!t.runner.startTest("direct", 0, true, t.err));
        t.ctrl.state.imuMotionFlags = 3; t.ctrl.state.halted = false;
        assert(!t.runner.startTest("direct", 0, true, t.err));
        assert(t.ctrl.commands.empty());
    }
    {
        Trial t;
        t.settings.data.navLoop = true;
        assert(t.runner.startTest("direct", 360, true, t.err));
        assert(t.field("id") == 1 && t.field("startHeadingDeg") == 0);
        assert(!t.runner.startTest("detour", 0, true, t.err));
        assert(!t.runner.start(t.err));
        t.tick(50, false);                        // old halted snapshot is not a cancellation
        assert(t.phase() == "aligning" && t.ctrl.commands.size() == 1);
        t.align(359);                            // circular heading tolerance and stable hold
        assert(t.field("elapsedMs") == 0 && t.field("actualStartHeadingDeg") == 359);
        assert(t.settings.data.navLoop && t.settings.data.planner == "detour");
        assert(t.status().children["loop"].number == 0);
        assert(t.ctrl.commands.size() == 2);
        t.tick(50, false);                        // old goal-free prep state cannot complete leg
        assert(t.phase() == "running" && t.ctrl.commands.size() == 2);
        t.complete();
        assert(t.phase() == "done" && t.field("valid") == 1 && t.field("elapsedMs") == 100);
        assert(t.field("observationMaxGapMs") == 50 && t.ctrl.halts == 1);
        const double elapsed = t.field("elapsedMs");
        t.tick(500); assert(t.field("elapsedMs") == elapsed);
    }
    {
        Trial t;
        assert(t.runner.startTest("detour", 0, true, t.err));
        t.tick(3050, true, false);                 // heartbeat applies during preparation too
        assert(t.phase() == "stopped" && t.field("valid") == 0 && t.field("elapsedMs") == 0);
        assert(t.ctrl.halts == 1);
    }
    {
        Trial t;
        assert(t.runner.startTest("direct", 0, true, t.err));
        t.ctrl.applied.source = CommandSource::Ros;
        t.tick();
        assert(t.phase() == "stopped" && t.ctrl.halts == 0); // retain ROS replacement command
    }
    {
        Trial t;
        assert(t.runner.startTest("direct", 0, true, t.err));
        t.ctrl.applied.imuMotionFlags = 1;
        t.tick(); assert(t.phase() == "failed" && t.ctrl.halts == 1);
    }
    {
        Trial t;
        assert(t.runner.startTest("direct", 0, true, t.err));
        t.ctrl.applied.wheelHeadingDeg = 90;
        t.tick(30000);                            // heartbeat alive, alignment bounded
        assert(t.phase() == "failed" && t.field("valid") == 0 && t.ctrl.halts == 1);
    }
    {
        Trial t;
        assert(t.runner.startTest("direct", 0, true, t.err));
        t.align();
        t.tick(250);
        t.runner.stop("operator stop");
        assert(t.phase() == "stopped" && t.field("valid") == 0 && t.field("elapsedMs") == 250);
        assert(t.field("observationMaxGapMs") == 250);
        t.tick();
        assert(t.runner.startTest("detour", 0, true, t.err));
        assert(t.field("id") == 2);
        t.runner.cancelForRos();
        assert(t.phase() == "stopped");
    }
    {
        Trial t;
        assert(t.runner.startTest("direct", 0, true, t.err));
        t.align();
        t.ctrl.applied.imuGyroDps[0] = std::numeric_limits<float>::infinity();
        t.tick(); assert(t.phase() == "failed" && t.field("valid") == 0);
    }
    {
        Trial t;
        assert(t.runner.startTest("direct", 0, true, t.err));
        t.tick(150, false);                       // stale control tick invalidates trial
        assert(t.phase() == "failed" && t.ctrl.halts == 1);
    }
    {
        Trial t;
        assert(t.runner.startTest("direct", 0, true, t.err));
        t.ctrl.applied.imuGyroDps[0] = 6;          // body moving: angle alone is insufficient
        for (int i = 0; i < 6; ++i) t.tick();
        assert(t.phase() == "aligning");
        t.ctrl.applied.imuGyroDps[0] = 0;
        t.tick();
        t.tick(150);                             // cannot claim stable hold over unobserved gap
        t.tick(); t.tick(); t.tick();
        assert(t.phase() == "aligning");
        t.tick(); assert(t.phase() == "running" && t.field("elapsedMs") == 0);
    }
    {
        Trial t;
        assert(t.runner.startTest("direct", 0, true, t.err));
        t.ctrl.snapshotDelayMs = 1;              // snapshot may be newer than update()'s first millis()
        t.align();
        assert(t.phase() == "running");
    }
    {
        Trial t;
        assert(t.runner.start(t.err));
        t.tick();
        assert(t.ctrl.commands.size() == 1);
        t.tick(50, false);                       // normal routes also wait for command publication
        assert(t.ctrl.commands.size() == 1 && t.runner.running());
        t.runner.stop("stop");
        assert(t.phase() == "idle");
    }
    {
        Trial t;
        assert(t.runner.startTest("direct", 0, true, t.err));
        ++t.ctrl.revision;                       // PID or advanced limits changed during preparation
        t.tick();
        assert(t.phase() == "failed" && t.field("valid") == 0 && t.ctrl.halts == 1);
    }
    {
        Trial t;
        assert(t.runner.startTest("direct", 0, true, t.err));
        t.align();
        ++t.ctrl.revision;                       // changed during timed route, even if final goal reached
        t.complete();
        assert(t.phase() == "failed" && t.field("valid") == 0 && t.ctrl.halts == 1);
    }
    {
        Trial t;
        t.ctrl.state.imuHeadingFresh = false;
        assert(!t.runner.startTest("direct", 0, true, t.err));
        t.ctrl.state.imuHeadingFresh = true;
        assert(t.runner.startTest("direct", 0, true, t.err));
        t.align();
        t.ctrl.applied.imuHeadingFresh = false;   // gyro/accel still fresh; quaternion report stalled
        t.tick();
        assert(t.phase() == "failed" && t.field("valid") == 0 && t.ctrl.halts == 1);
    }
    {   // demo 1 from the button: own points, no web heartbeat, web route given back
        Trial t;
        const int arms = t.ctrl.wdArms;
        assert(t.runner.startDemoSquare(1.0f, t.err));
        assert(t.ctrl.poseResets == 1 && t.ctrl.wdArms == arms);          // no web watchdog
        assert(t.status().children["count"].number == 3 && t.status().children["byButton"].number == 1);
        for (int i = 0; i < 100; ++i) t.tick(50, true, false);           // 5 s, no heartbeat
        assert(t.status().children["status"].text == "running");
        t.runner.stop("button");
        JsonNode pts; t.runner.pointsJson(JsonArray(&pts));
        assert(pts.items.size() == 1 && std::fabs(pts.items[0].children["x"].number - 0.3) < 1e-6);  // saved route back
    }
    {   // demo 2/3 from the button: timed test, no heartbeat, stops at the time cap
        Trial t;
        assert(t.runner.startTest("detour", 0, true, t.err, true));
        t.align();
        for (int i = 0; i < 100; ++i) t.tick(50, true, false);           // 5 s, no heartbeat
        assert(t.phase() == "running");
        t.tick(DEMO_MAX_MS, true, false);
        assert(t.status().children["status"].text == "stopped" && t.ctrl.halts == 1);
    }
    {   // a web route still stops after 3 s without heartbeat
        Trial t;
        assert(t.runner.start(t.err));
        for (int i = 0; i < 70; ++i) t.tick(50, true, false);            // 3.5 s
        assert(t.status().children["status"].text == "stopped");
    }
    std::cout << "WaypointRunner comparison tests PASS (18 scenarios)\n";
}
