#pragma once
#include <Arduino.h>
struct SettingsData {
    float navSpeedMps = 0.03f, navTolM = 0.02f, steerDps = 60.0f;
    bool navLoop = false;
    String planner = "detour";
};
class Settings {
public:
    SettingsData get() { return data; }
    SettingsData data;
};
