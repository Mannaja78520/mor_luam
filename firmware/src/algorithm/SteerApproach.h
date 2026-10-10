#pragma once
// Should the steering motor keep pushing the wheel toward its angle?
//
// Outside the tolerance band: yes (the controller then cuts early, see
// SteerStopPredictor.h). Inside the band, only to finish a powered approach
// that is still MOVING, until the landing point (STEER_LAND_DEG): then the
// wheel coasts to ~2 deg short instead of stopping at the band edge (3-5 deg).
//
// Never push a wheel that stands inside the band: from a standstill the motor
// has to build up power to break loose, the wheel then jumps (9 deg measured on
// 2026-10-10, from 3.4 short to 6 past) and past the angle it costs a full extra
// turn - which then repeated until the 30 s alignment limit.
//
// Plain C++ (checked in test_host/tests.cpp).

namespace steerapproach {

inline bool keepPushing(bool aimed, bool approaching, bool coasting, float eCwDeg, float landDeg, float rateDps,
                        float minMovingDps) {
    if (!aimed) return true;                       // outside the band (the caller handles a coast)
    return approaching && !coasting && eCwDeg < 180.0f && eCwDeg > landDeg && rateDps >= minMovingDps;
}

}  // namespace steerapproach
