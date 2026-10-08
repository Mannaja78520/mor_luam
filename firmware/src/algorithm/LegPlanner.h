#pragma once
// How the robot gets from where it is to ONE waypoint: the plug-in point for
// steering algorithms. Everything in src/algorithm/ is plain C++ (no Arduino),
// so it can be compiled and tested on a PC (test_host/tests.cpp).
//
// To add an algorithm: implement LegPlanner in a new file here, then add it to
// PlannerFactory.h. Nothing outside this folder needs to change.
//
// Angles: the robot steers one way only - clockwise seen from above, so the
// wheel's heading only DEcreases (CCW frame). phiDeg is "how far the wheel must
// still turn", 0..360 - exactly angles::cwErrorDeg, as the controller uses.

struct RobotParams {
    float driveMps = 0.25f;   // v: drive speed
    float steerDps = 60.0f;   // omega: steering speed of the wheel module
    float settleS = 0.05f;    // t_s: hold inside tolerance before driving
    float stopS = 0.20f;      // t_b: stop + switch the motor from drive to steer
};

struct LegPlan {
    enum Kind { Direct, Detour };
    Kind kind = Direct;
    float a = 0.0f;           // Detour: leg 1 along the CURRENT wheel heading (no steer), m
    float betaDeg = 0.0f;     // the one steer: Direct = phi, Detour = beta*
    float b = 0.0f;           // leg to the goal after the steer, m
    float timeS = 0.0f;       // predicted time of this plan
    float k = 0.0f;           // Detour Steer's k (0 when not computed)
};

class LegPlanner {
public:
    virtual ~LegPlanner() {}
    virtual const char* name() const = 0;
    // phiDeg: how far the wheel must turn to point at the goal (0..360); d: distance (m)
    virtual LegPlan plan(float phiDeg, float d, const RobotParams& p) const = 0;
};
