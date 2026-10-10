#pragma once
// Stop early and aim again, when the wheel would pass beside the goal.
//
// The robot has ONE motor: one direction drives the wheel, the other direction
// steers it (counter-clockwise only). So it cannot correct its course while it
// drives. The wheel stops a little SHORT of its angle on purpose (never past),
// so a miss always leaves the goal to the LEFT of the line it drives on, and a
// left (counter-clockwise) turn is the cheap one. Found at the goal, the miss
// needs a ~90 deg turn; found early - stop when the goal is `ratio` times its
// side offset ahead - it needs only atan(1 / ratio) (18 deg for 3).
//
// Plain C++ (checked in test_host/tests.cpp).
#include <math.h>

namespace finalapproach {

// The goal seen from the wheel: `ahead` along its heading, `left` beside it (+ = left).
inline void goalInWheelFrame(float dxM, float dyM, float headingRad, float& ahead, float& left) {
    const float c = cosf(headingRad), s = sinf(headingRad);
    ahead = dxM * c + dyM * s;
    left = -dxM * s + dyM * c;
}

// true = stop now so the planner aims again at the goal.
//   tolM:      the goal radius; a pass this close needs no correction
//   ratio:     stop when ahead <= ratio * left (the re-aim is then atan(1/ratio))
//   minAheadM: never closer than this (the last leg must still be drivable)
inline bool stopToReaim(float ahead, float left, float tolM, float ratio, float minAheadM) {
    if (left <= tolM) return false;     // passes within the goal radius, or the goal is to the
                                        // right: a re-aim would be almost a full turn - drive on
    if (ahead <= 0.0f) return false;    // already beside or past it: the normal end handles it
    const float stopAt = ratio * left > minAheadM ? ratio * left : minAheadM;
    return ahead <= stopAt;
}

}  // namespace finalapproach
