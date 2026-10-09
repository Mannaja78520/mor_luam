#include "app_config.h"
#include <cmath>
// Tests the production WaypointRunner against a delayed control snapshot.
// See test_host/nav_runner_stubs: hardware/network/NVS are not involved.
#include "nav/WaypointRunner.h"
#include <cassert>
#include <iostream>
#include <initializer_list>
#include <limits>
#include <string>
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
        const Waypoint point{0.3f, 0.03f, 0.0f};
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
    // one demo 3/4 run: align, reach the goal, hold, drive home
    void compareRun() {
        align();
        tick(1000);
        const float b = -DEMO_COMPARE_RIGHT_DEG * 3.14159265f / 180.0f;
        ctrl.applied.goalActive = false; ctrl.applied.targetRpm = 0;
        ctrl.applied.x = DEMO_COMPARE_DIST_M * std::cos(b); ctrl.applied.y = DEMO_COMPARE_DIST_M * std::sin(b);
        tick(); tick();                                   // at the goal: the hold starts
        for (int i = 0; i < 110; ++i) tick();            // 5.5 s: hold over, the drive home is sent
        ctrl.applied.goalActive = false; ctrl.applied.targetRpm = 0;
        ctrl.applied.x = 0; ctrl.applied.y = 0;
        tick(); tick();                                   // home: done
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
    {   // button route 1 (default square): own points, no web heartbeat, web route given back
        Trial t;
        const int arms = t.ctrl.wdArms;
        t.ctrl.state.x = t.ctrl.applied.x = 5; t.ctrl.state.y = t.ctrl.applied.y = 5;   // far from (0,0)
        assert(t.runner.startButtonRoute(1, t.err));
        assert(t.ctrl.poseResets == 1 && t.ctrl.wdArms == arms);          // no web watchdog
        assert(t.status().children["count"].number == 3 && t.status().children["byButton"].number == 1);
        JsonNode web; t.runner.pointsJson(JsonArray(&web));               // a page loading now sees the web route
        assert(web.items.size() == 1 && std::fabs(web.items[0].children["x"].number - 0.3) < 1e-6);
        t.tick(50, false);                                                // pose before the reset: no leg yet
        assert(t.ctrl.commands.empty());
        t.tick();                                                         // reset pose published
        assert(t.ctrl.commands.size() == 1 && std::fabs(t.ctrl.commands[0].headingDeg) < 1e-3);  // towards (1,0)
        for (int i = 0; i < 100; ++i) t.tick(50, true, false);           // 5 s, no heartbeat
        assert(t.status().children["status"].text == "running");
        t.runner.stop("button");
        JsonNode pts; t.runner.pointsJson(JsonArray(&pts));
        assert(pts.items.size() == 1 && std::fabs(pts.items[0].children["x"].number - 0.3) < 1e-6);  // saved route back
    }
    {   // button routes are saved per slot, apart from the web route
        Trial t;
        const Waypoint two[] = {{0.4f, 0.0f, 0.0f}, {0.4f, -0.2f, 0.0f}};
        assert(t.runner.setPoints(two, 2, t.err, 2));
        assert(!t.runner.setPoints(two, 2, t.err, DEMO_ROUTES + 1));
        JsonNode r2; t.runner.pointsJson(JsonArray(&r2), 2);
        assert(r2.items.size() == 2 && std::fabs(r2.items[1].children["y"].number + 0.2) < 1e-6);
        JsonNode r1; t.runner.pointsJson(JsonArray(&r1), 1);
        assert(r1.items.size() == 3 && std::fabs(r1.items[1].children["y"].number - DEMO_SQUARE_M) < 1e-6);
        JsonNode web; t.runner.pointsJson(JsonArray(&web));
        assert(web.items.size() == 1);
        assert(t.runner.startButtonRoute(2, t.err));
        assert(t.runner.setPoints(two, 1, t.err, 1));                     // a button route can change while one runs
        assert(!t.runner.setPoints(two, 1, t.err));                       // the web route cannot
        t.runner.stop("button");
        assert(t.runner.setPoints(two, 0, t.err, 2));                     // cleared on purpose
        assert(!t.runner.startButtonRoute(2, t.err) && std::string(t.err.c_str()).find("ยังไม่มีจุด") != std::string::npos);
        assert(!t.runner.startButtonRoute(0, t.err) && !t.runner.startButtonRoute(DEMO_ROUTES + 1, t.err));
    }
    {   // button route: after the last point, drive back to where it started
        Trial t;
        const Waypoint one{0.5f, 0.0f, 0.0f};
        assert(t.runner.setPoints(&one, 1, t.err, 1));
        assert(t.runner.startButtonRoute(1, t.err));
        t.tick(); t.tick();
        t.ctrl.applied.goalActive = false; t.ctrl.applied.targetRpm = 0;
        t.ctrl.applied.x = 0.5f; t.ctrl.applied.y = 0;                     // at the point
        t.tick(); t.tick();
        assert(t.status().children["status"].text == "running");           // going home
        assert(std::fabs(t.ctrl.commands.back().headingDeg - 180.0f) < 1e-3);
        t.ctrl.applied.goalActive = false; t.ctrl.applied.targetRpm = 0;
        t.ctrl.applied.x = 0; t.ctrl.applied.y = 0;
        t.tick(); t.tick();
        assert(t.status().children["status"].text == "done" && t.status().children["message"].text == "กลับถึงจุดเริ่มแล้ว");
    }
    {   // demo 3/4 from the button: timed test, no heartbeat, stops at the time cap
        Trial t;
        assert(t.runner.startCompare("detour", t.err));
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
    {   // demo 3/4: goal 10 deg clockwise of the aligned wheel - Direct turns ~350, Detour drives first
        for (const char* planner : {"direct", "detour"}) {
            Trial t;
            t.ctrl.state.x = t.ctrl.applied.x = 1; t.ctrl.state.y = t.ctrl.applied.y = 2;
            assert(t.runner.startCompare(planner, t.err));
            t.align(357);                                                 // wheel stopped 3 deg short
            const DriveCommand& c = t.ctrl.commands.back();
            assert(std::fabs(t.status().children["plan"].children["phiDeg"].number - (360.0 - DEMO_COMPARE_RIGHT_DEG)) < 0.05);
            if (std::string(planner) == "direct") {
                assert(t.status().children["plan"].children["kind"].text == "direct");
                assert(std::fabs(c.headingDeg - (357.0f - DEMO_COMPARE_RIGHT_DEG)) < 0.05f && std::fabs(c.distM - DEMO_COMPARE_DIST_M) < 1e-4);
            } else {
                assert(t.status().children["plan"].children["kind"].text == "detour");
                assert(std::fabs(c.headingDeg - 357.0f) < 0.05f && c.distM > 0.5f * DEMO_COMPARE_DIST_M && c.distM < 1.2f * DEMO_COMPARE_DIST_M);
            }
        }
    }
    {   // demo 3/4 from the button: after the timed goal, drive back home (untimed)
        Trial t;
        assert(t.runner.startCompare("direct", t.err));
        t.align();
        t.tick(4000);
        const float b = -DEMO_COMPARE_RIGHT_DEG * 3.14159265f / 180.0f;
        t.ctrl.applied.goalActive = false; t.ctrl.applied.targetRpm = 0;
        t.ctrl.applied.x = DEMO_COMPARE_DIST_M * std::cos(b);           // reached the demo goal
        t.ctrl.applied.y = DEMO_COMPARE_DIST_M * std::sin(b);
        t.tick();
        t.tick();
        assert(t.phase() == "done" && t.field("valid") == 1 && t.field("elapsedMs") >= 4000);
        assert(t.status().children["status"].text == "running");          // holding at the goal
        const double elapsed = t.field("elapsedMs");
        const size_t sent = t.ctrl.commands.size();
        assert(t.status().children["waitLeftMs"].number > DEMO_COMPARE_HOLD_S * 1000 - 200);
        for (int i = 0; i < 90; ++i) t.tick();                           // 4.5 s: still at the goal
        assert(t.ctrl.commands.size() == sent);
        for (int i = 0; i < 12; ++i) t.tick();                           // hold over: drive home
        assert(t.ctrl.commands.size() == sent + 1 && t.field("elapsedMs") == elapsed);
        assert(std::fabs(t.ctrl.commands.back().distM - DEMO_COMPARE_DIST_M) < 1e-3);
        {   // the robot's record of the run, for the web picture
            JsonNode o; t.runner.compareJson(JsonObject(&o));
            JsonNode& d = o.children["direct"];
            assert(o.children["detour"].null && !d.null);
            assert(d.children["valid"].number == 1 && d.children["open"].number == 0);
            assert(d.children["elapsedMs"].number == elapsed);
            assert(std::fabs(d.children["goalX"].number - DEMO_COMPARE_DIST_M * std::cos(b)) < 1e-4);
            auto& path = d.children["path"].items;
            assert(path.size() == 2 && path[0].children["x"].number == 0);  // start, then where it ended
            auto& acts = d.children["acts"].items;
            assert(!acts.empty() && acts[0].children["t"].number == 0 && acts[0].children["a"].text == "still");
            assert(std::fabs(path[1].children["x"].number - DEMO_COMPARE_DIST_M * std::cos(b)) < 1e-3);
        }
        t.ctrl.applied.goalActive = false; t.ctrl.applied.targetRpm = 0;
        t.ctrl.applied.x = 0; t.ctrl.applied.y = 0;                       // back at the start
        t.tick(); t.tick();
        assert(t.status().children["status"].text == "done" && t.field("elapsedMs") == elapsed);
        JsonNode pts; t.runner.pointsJson(JsonArray(&pts));
        assert(pts.items.size() == 1 && std::fabs(pts.items[0].children["x"].number - 0.3) < 1e-6);
    }
    {   // a web-started test does NOT drive home
        Trial t;
        assert(t.runner.startTest("direct", 0, true, t.err));
        t.align();
        t.tick(1000);
        t.complete();
        t.tick();
        assert(t.status().children["status"].text == "done");
    }
    {   // a point with a wait: stop there for waitS, then go on to the next point
        Trial t;
        const Waypoint two[] = {{0.3f, 0.0f, 2.0f}, {0.6f, 0.0f, 0.0f}};
        assert(t.runner.setPoints(two, 2, t.err));
        assert(t.runner.start(t.err));
        t.tick();                                                         // leg 1 sent
        assert(t.ctrl.commands.size() == 1);
        t.ctrl.applied.goalActive = false; t.ctrl.applied.targetRpm = 0;
        t.ctrl.applied.x = 0.3f;
        t.tick(); t.tick();                                               // reached point 1: waiting
        assert(t.status().children["waitLeftMs"].number > 1500 && t.ctrl.commands.size() == 1);
        for (int i = 0; i < 30; ++i) t.tick();                           // 1.5 s later: still waiting
        assert(t.ctrl.commands.size() == 1 && t.status().children["status"].text == "running");
        for (int i = 0; i < 12; ++i) t.tick();                           // past 2 s: on to point 2
        assert(t.ctrl.commands.size() == 2 && t.status().children["waitLeftMs"].number == 0);
        assert(t.status().children["index"].number == 1);
        t.runner.stop("test");
        Waypoint bad{0.1f, 0.0f, NAV_MAX_WAIT_S + 1.0f};
        assert(!t.runner.setPoints(&bad, 1, t.err));
        bad.waitS = NAN;
        assert(!t.runner.setPoints(&bad, 1, t.err));
        bad.waitS = -1.0f;
        assert(!t.runner.setPoints(&bad, 1, t.err, 1));
        JsonNode pts; t.runner.pointsJson(JsonArray(&pts));
        assert(pts.items.size() == 2 && std::fabs(pts.items[0].children["waitS"].number - 2.0) < 1e-6);
    }
    {   // button route limit grows with the route; stops at points are not moving time
        Trial t;
        const Waypoint far[] = {{3.0f, 0.0f, 30.0f}, {3.0f, 3.0f, 0.0f}};      // 3 m + 3 m + 4.24 m back
        assert(t.runner.setPoints(far, 2, t.err, 1));
        assert(t.runner.startButtonRoute(1, t.err));
        // 2 x ((3 + 3 + 4.243) / 0.03 + 3 x 18) s = 790.8 s
        const double limit = t.status().children["demoLimitS"].number;
        assert(limit > 780 && limit < 800);
        t.tick(); t.tick();
        t.ctrl.applied.goalActive = false; t.ctrl.applied.targetRpm = 0;
        t.ctrl.applied.x = 3.0f;                                          // at point 1: 30 s stop
        t.tick(); t.tick();
        const double moving = t.status().children["demoMovingS"].number;
        for (int i = 0; i < 400; ++i) t.tick(50, true, false);           // 20 s of the stop
        assert(t.status().children["demoMovingS"].number == moving);
        for (int i = 0; i < 300; ++i) t.tick(50, true, false);           // stop over, on to point 2
        assert(t.status().children["index"].number == 1);
        t.tick(DEMO_MAX_MS, true, false);                                // past 5 min of moving
        assert(t.status().children["status"].text == "running");
        t.tick(500000, true, false);                                     // past the route's own limit
        assert(t.status().children["status"].text == "stopped");
    }
    {   // a short button route keeps the 5 min floor; the 30 min ceiling holds for a huge one
        Trial t;
        assert(t.runner.startButtonRoute(2, t.err));                      // default 0.5 m triangle
        assert(t.status().children["demoLimitS"].number == DEMO_MAX_MS / 1000);
        t.runner.stop("test");
        Waypoint huge[NAV_MAX_POINTS];
        for (int i = 0; i < NAV_MAX_POINTS; ++i) huge[i] = {i % 2 ? 40.0f : -40.0f, 40.0f, 0.0f};
        assert(t.runner.setPoints(huge, NAV_MAX_POINTS, t.err, 2));
        assert(t.runner.startButtonRoute(2, t.err));
        assert(t.status().children["demoLimitS"].number == DEMO_LIMIT_CEIL_MS / 1000);
    }
    {   // demo 4 stopped on the way: the record keeps the path, not valid
        Trial t;
        assert(t.runner.startCompare("detour", t.err));
        t.align();
        assert(t.status().children["test"].children["demo"].number == 1);
        assert(!t.status().children["test"].children["goalX"].null);
        for (int i = 1; i <= 10; ++i) { t.ctrl.applied.x = 0.01f * i; t.tick(); }   // 10 cm
        t.runner.stop("button");
        JsonNode o; t.runner.compareJson(JsonObject(&o));
        JsonNode& d = o.children["detour"];
        assert(!d.null && d.children["valid"].number == 0 && d.children["open"].number == 0);
        assert(d.children["path"].items.size() >= 10);
        assert(o.children["direct"].null);
        assert(t.status().children["test"].children["goalX"].null);
    }
    {   // demo 3/4 from the web: ready needed, the heartbeat rule applies, then it drives home
        Trial t;
        assert(!t.runner.startCompare("direct", t.err, false, false));        // not ready
        const int arms = t.ctrl.wdArms;
        assert(t.runner.startCompare("direct", t.err, false, true));
        assert(t.ctrl.wdArms == arms + 1 && t.status().children["byButton"].number == 0);
        t.align();
        for (int i = 0; i < 70; ++i) t.tick(50, true, false);                // 3.5 s without heartbeat
        assert(t.status().children["status"].text == "stopped");
    }
    {   // demo 3 time line: steering in place is "turn" even though the drive encoder moves
        Trial t;
        assert(t.runner.startCompare("direct", t.err));
        t.align();
        t.ctrl.applied.driving = false; t.ctrl.applied.steerRateDps = 35; t.ctrl.applied.rpm = 2;   // coupling
        for (int i = 0; i < 20; ++i) t.tick();                                                     // 1 s turning
        t.ctrl.applied.driving = true; t.ctrl.applied.steerRateDps = 0; t.ctrl.applied.rpm = 7.5;
        for (int i = 0; i < 20; ++i) t.tick();                                                     // 1 s driving
        JsonNode o; t.runner.compareJson(JsonObject(&o));
        auto& acts = o.children["direct"].children["acts"].items;
        assert(acts.size() >= 3 && acts[1].children["a"].text == "turn" && acts[2].children["a"].text == "drive");
        t.runner.stop("test");
    }
    {   // "test N rounds": runs one after another, rounds alternate who goes first, results kept
        Trial t;
        assert(!t.runner.startSeries(2, t.err, false, false));                       // not ready
        assert(!t.runner.startSeries(DEMO_SERIES_MAX_ROUNDS + 1, t.err, false, true));
        assert(t.runner.startSeries(2, t.err, false, true));
        assert(!t.runner.start(t.err) && !t.runner.startCompare("direct", t.err));     // nothing else meanwhile
        assert(!t.runner.startButtonRoute(1, t.err) && t.runner.seriesActive());
        for (int run = 0; run < 4; ++run) {
            if (run) {
                for (int i = 0; i < 30; ++i) t.tick();                                // 1.5 s: still the pause
                assert(t.status().children["status"].text == "done");
                for (int i = 0; i < 11; ++i) t.tick();                                // just past 2 s: the next run aligns
            }
            assert(t.phase() == "aligning");
            t.compareRun();
            assert(t.status().children["series"].children["count"].number == run + 1);
        }
        for (int i = 0; i < 60; ++i) t.tick();                                        // no fifth run
        JsonNode o; t.runner.seriesJson(JsonObject(&o));
        assert(o.children["count"].number == 4 && o.children["total"].number == 4);
        assert(o.children["active"].number == 0 && o.children["why"].text.empty());
        auto& runs = o.children["runs"].items;
        const char* order[4] = {"direct", "detour", "detour", "direct"};
        for (int i = 0; i < 4; ++i) {
            assert(runs[i].children["planner"].text == order[i] && runs[i].children["valid"].number == 1);
            assert(runs[i].children["xy"].items.size() >= 4 && !runs[i].children["acts"].items.empty());
        }
        assert(t.status().children["status"].text == "done" && !t.runner.seriesActive());
    }
    {   // a stop between runs ends the series
        Trial t;
        assert(t.runner.startSeries(3, t.err, false, true));
        t.compareRun();
        t.runner.stop("E-STOP");
        for (int i = 0; i < 60; ++i) t.tick();
        JsonNode o; t.runner.seriesJson(JsonObject(&o));
        assert(o.children["count"].number == 1 && o.children["active"].number == 0 && o.children["why"].text == "E-STOP");
        assert(t.phase() != "aligning");
    }
    {   // the web page goes away: the next run stops within 3 s and the series ends
        Trial t;
        assert(t.runner.startSeries(3, t.err, false, true));
        t.compareRun();
        for (int i = 0; i < 120; ++i) t.tick(50, true, false);                       // 6 s, no heartbeat
        JsonNode o; t.runner.seriesJson(JsonObject(&o));
        // run 2 started (page seen 2 s ago), then stopped by the 3 s rule: kept, not valid
        assert(o.children["active"].number == 0 && o.children["count"].number == 2);
        assert(o.children["runs"].items[1].children["valid"].number == 0 && !o.children["why"].text.empty());
        assert(t.status().children["status"].text != "running");
    }
    std::cout << "WaypointRunner comparison tests PASS (33 scenarios)\n";
}
