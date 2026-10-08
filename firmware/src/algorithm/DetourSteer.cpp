#include "algorithm/DetourSteer.h"
#include <math.h>
#include "algorithm/DirectPlanner.h"

namespace {
const float PI_F = 3.14159265358979f;
const float TAU_F = 2.0f * PI_F;
inline float rad(float deg) { return deg * PI_F / 180.0f; }
inline float deg(float r) { return r * 180.0f / PI_F; }
}  // namespace

float DetourSteer::optimalDetourAngleDeg(float phiDeg, float d, const RobotParams& p, float* kOut) {
    const float phi = rad(phiDeg);
    if (kOut) *kOut = 0.0f;
    if (phi <= PI_F) return -1.0f;                         // T1: straight at the goal is optimal
    const float w = rad(p.steerDps);
    const float k = (d * w / p.driveMps) * fabsf(sinf(phi));
    if (kOut) *kOut = k;
    if (k >= 2.0f) return -1.0f;                           // T3: time keeps falling up to beta = phi
    const float beta = TAU_F - acosf(k - 1.0f);            // T3: the one root of dT/dbeta = 0
    return beta < phi ? deg(beta) : -1.0f;
}

// Algorithm 1.3 (PlanLeg): the faster of "steer phi, drive d" and the best detour.
LegPlan DetourSteer::plan(float phiDeg, float d, const RobotParams& p) const {
    LegPlan direct = DirectPlanner().plan(phiDeg, d, p);
    float k = 0.0f;
    const float betaDeg = optimalDetourAngleDeg(phiDeg, d, p, &k);
    direct.k = k;
    if (betaDeg < 0.0f) return direct;

    const float phi = rad(phiDeg), beta = rad(betaDeg);
    const float s = sinf(beta);
    const float a = d * sinf(beta - phi) / s;              // leg 1: heading unchanged
    const float b = d * sinf(phi) / s;                     // leg 2: heading beta*, ends at the goal
    const float t = a / p.driveMps + p.stopS + betaDeg / p.steerDps + p.settleS + b / p.driveMps;
    if (!(t < direct.timeS)) return direct;                // the extra stop must pay for itself

    LegPlan l;
    l.kind = LegPlan::Detour;
    l.a = a;
    l.betaDeg = betaDeg;
    l.b = b;
    l.timeS = t;
    l.k = k;
    return l;
}
