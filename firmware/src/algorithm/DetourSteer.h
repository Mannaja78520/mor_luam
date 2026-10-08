#pragma once
// Detour Steer - the algorithm of the 271401 homework
// (E:\271401\271401_pee_aut_homework\algorithm\DETOUR_STEERING\algorithm.pdf,
//  reference implementation: aut_HW_04_detour_steering\detour_steer.py).
//
// A wheel that steers one way only pays almost a full turn for a goal just
// "behind" its rotation. Two straight legs whose headings differ by more than
// 180 deg reach any direction, so instead of steering phi the robot may:
//   1. drive a metres along its current heading (no steer),
//   2. steer only beta* (180 < beta* < phi),
//   3. drive b metres to the goal.
//
//   k     = (d * omega / v) * |sin phi|
//   beta* = 360 - acos(k - 1)            only if phi > 180, k < 2, beta* < phi
//   a     = d * sin(beta* - phi) / sin(beta*)
//   b     = d * sin(phi) / sin(beta*)
//
// and the detour is taken only when it is really faster than steering
// straight (it costs one extra stop t_b). Theorems T1-T4 in the PDF: optimal,
// at most two legs, the closed form is the minimum, and planning again after
// leg 1 gives "steer beta*, drive b" - so the caller simply plans again after
// every move, and a wheel that stops PAST its angle is recovered the same way.
#include "algorithm/LegPlanner.h"

class DetourSteer : public LegPlanner {
public:
    const char* name() const override { return "detour"; }
    LegPlan plan(float phiDeg, float d, const RobotParams& p) const override;

    // Algorithm 1.2: beta* in degrees, or a negative value when no detour can
    // beat steering straight. Exposed for tests.
    static float optimalDetourAngleDeg(float phiDeg, float d, const RobotParams& p, float* kOut = nullptr);
};
