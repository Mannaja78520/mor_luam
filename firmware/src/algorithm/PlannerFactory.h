#pragma once
// The one place that knows which steering algorithms exist.
// Add a new LegPlanner here and it appears as a choice (settings "planner").
#include <string.h>
#include "algorithm/DetourSteer.h"
#include "algorithm/DirectPlanner.h"

class PlannerFactory {
public:
    // "detour" (default) or "direct"; anything unknown falls back to detour
    static const LegPlanner& get(const char* name) {
        static const DetourSteer detour;
        static const DirectPlanner direct;
        if (name && strcmp(name, "direct") == 0) return direct;
        return detour;
    }
    static const char* const* names() {
        static const char* const n[] = {"detour", "direct", nullptr};
        return n;
    }
};
