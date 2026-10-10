#pragma once
// Settings that people change in the field, kept in NVS (namespace "morluam")
// and edited from the web page, tab "ตั้งค่า". Defaults: conf_network.h.
//
// To add a setting: one field here, one line in load()/save(), one in
// toJson()/fromJson(), and one input in web/WebPage.h (SettingsView).
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>

struct SettingsData {
    String robotName;
    String hostname;      // mDNS: http://<hostname>.local
    String agentHost;     // "" = automatic (Wi-Fi gateway, then AGENT_IP)
    uint16_t agentPort = 8888;
    String otaPass;
    String apPass;
    float navSpeedMps = 0.03f;           // NAV_DEFAULT_SPEED_MPS
    float navTolM = 0.05f;
    bool navLoop = false;
    String planner;       // steering algorithm for web routes: src/algorithm/PlannerFactory.h
    float steerDps = 60.0f;
    // demo button
    bool demoReset = true;                // a demo press makes HERE (0,0), the front 0 deg
    float demoDistM = 0.30f;              // demos 3/4: goal distance  (DEMO_COMPARE_DIST_M)
    float demoRightDeg = 6.0f;            // ... and angle clockwise of the aligned wheel (DEMO_COMPARE_RIGHT_DEG)
    // steering (SteerDriveController::setTuning)
    float steerLandDeg = 2.0f;            // STEER_LAND_DEG
    bool reaimOn = true;                  // DRIVE_REAIM
    float reaimRatio = 3.0f;              // DRIVE_REAIM_RATIO
};

class Settings {
public:
    void begin();
    SettingsData get();                       // a copy
    // Apply fields present in `in`; returns false with `err` set if a value is bad.
    bool update(JsonObjectConst in, String& err);
    void toJson(JsonObject out, bool withSecrets);
    // learned steering coast time (SteerStopPredictor): saved by App, not edited on the page
    float coastS(float fallback);
    void saveCoastS(float s);
    float driveGain(float fallback, float ks, float kf);
    bool saveDriveGain(float gain, float ks, float kf);

private:
    void load();
    void save();
    Preferences prefs_;
    SettingsData d_;
    SemaphoreHandle_t mtx_ = nullptr;
};
