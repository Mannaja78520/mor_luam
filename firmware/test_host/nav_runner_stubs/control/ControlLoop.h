#pragma once
#include "control/RobotState.h"
#include <vector>
class ControlLoop {
public:
    // Commands deliberately leave snapshot() stale until the test publishes a tick.
    void command(const DriveCommand& cmd, CommandSource source) {
        commands.push_back(cmd); applied = state;
        if (resetPending) applied.x = applied.y = 0;   // the controller already has the reset pose
        applied.halted = false; applied.source = source;
        applied.targetRpm = cmd.rpm; applied.targetHeadingDeg = cmd.headingDeg;
        applied.goalActive = cmd.rpm != 0 && cmd.distM != 0;
        applied.overshot = false; applied.motionFault = 0;
    }
    void halt(const char*) { ++halts; applied.halted = true; applied.pwm = 0; applied.targetRpm = 0; applied.goalActive = false; }
    // motion watchdog (the real one lives in the control task)
    int wdArms = 0, wdFeeds = 0, wdDisarms = 0;
    int poseResets = 0;
    // like the robot: the controller's pose is reset at once, the snapshot shows it after the next publish()
    void resetPose() { ++poseResets; applied.x = applied.y = 0; resetPending = true; }
    bool resetPending = false;
    void armWatchdog(uint32_t, const char*) { ++wdArms; }
    void feedWatchdog() { ++wdFeeds; }
    void disarmWatchdog() { ++wdDisarms; }
    RobotState snapshot() {
        if (snapshotDelayMs) { g_nav_ms += snapshotDelayMs; state.stampMs = millis(); }
        return state;
    }
    void getPid(bool, float out[5]) { for (int i = 0; i < 5; ++i) out[i] = 0; out[4] = 3.5f; }
    uint32_t pidRevision() { return revision; }
    void publish() { state = applied; state.stampMs = millis(); resetPending = false; }
    RobotState state, applied;
    std::vector<DriveCommand> commands;
    unsigned halts = 0;
    uint32_t snapshotDelayMs = 0;
    uint32_t revision = 0;
};
