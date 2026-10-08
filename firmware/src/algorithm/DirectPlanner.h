#pragma once
// The method the firmware always used: steer straight at the goal, then drive.
// Kept as an algorithm of its own so the two can be compared on the robot.
#include "algorithm/LegPlanner.h"

class DirectPlanner : public LegPlanner {
public:
    const char* name() const override { return "direct"; }

    static float directTime(float phiDeg, float d, const RobotParams& p) {
        const float steer = phiDeg > 1e-6f ? phiDeg / p.steerDps + p.settleS : 0.0f;
        return steer + d / p.driveMps;
    }

    LegPlan plan(float phiDeg, float d, const RobotParams& p) const override {
        LegPlan l;
        l.kind = LegPlan::Direct;
        l.betaDeg = phiDeg;
        l.b = d;
        l.timeS = directTime(phiDeg, d, p);
        return l;
    }
};
