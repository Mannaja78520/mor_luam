#pragma once
#include <Arduino.h>
struct SettingsData {
    float navSpeedMps = 0.03f, navTolM = 0.02f, steerDps = 60.0f;
    bool navLoop = false;
    String planner = "detour";
    bool demoReset = true;
    float demoDistM = 0.30f, demoRightDeg = 6.0f;
};
class Settings {
public:
    SettingsData get() { return data; }
    SettingsData data;
};
