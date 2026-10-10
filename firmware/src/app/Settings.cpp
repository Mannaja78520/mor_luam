#include "app/Settings.h"
#include "app_config.h"
#include <config.h>
#include <string.h>
#include "algorithm/PlannerFactory.h"

namespace {
struct Lock {
    explicit Lock(SemaphoreHandle_t m) : m_(m) { xSemaphoreTake(m_, portMAX_DELAY); }
    ~Lock() { xSemaphoreGive(m_); }
    SemaphoreHandle_t m_;
};

// mDNS names: letters, digits and '-' only
bool validHostname(const String& h) {
    if (h.length() < 1 || h.length() > 31) return false;
    for (char c : h) if (!isalnum((unsigned char)c) && c != '-') return false;
    return h[0] != '-';
}
}  // namespace

void Settings::begin() {
    mtx_ = xSemaphoreCreateMutex();
    prefs_.begin("morluam", false);
    load();
}

void Settings::load() {
    // isKey() first: Preferences prints an error for every key a new board has not saved yet
    auto str = [this](const char* k, const char* d) { return prefs_.isKey(k) ? prefs_.getString(k, d) : String(d); };
    auto flt = [this](const char* k, float d) { return prefs_.isKey(k) ? prefs_.getFloat(k, d) : d; };
    d_.robotName = str("name", DEFAULT_ROBOT_NAME);
    d_.hostname = str("host", DEFAULT_HOSTNAME);
    d_.agentHost = str("agent", "");
    d_.agentPort = prefs_.getUShort("aport", AGENT_PORT);
    d_.otaPass = str("ota", DEFAULT_OTA_PASS);
    d_.apPass = str("appass", DEFAULT_AP_PASS);
    d_.navSpeedMps = flt("speed", NAV_DEFAULT_SPEED_MPS);
    // saved before the speed limit was measured (e.g. 0.25): the wheel cannot do it
    if (!(d_.navSpeedMps >= 0.005f && d_.navSpeedMps <= NAV_MAX_SPEED_MPS)) d_.navSpeedMps = NAV_DEFAULT_SPEED_MPS;
    d_.navTolM = flt("tol", 0.05f);
    d_.navLoop = prefs_.getBool("loop", false);
    d_.planner = str("planner", "detour");
    d_.steerDps = flt("sdps", 60.0f);
    d_.demoReset = prefs_.isKey("dreset") ? prefs_.getBool("dreset", true) : true;
    d_.demoDistM = flt("ddist", DEMO_COMPARE_DIST_M);
    d_.demoRightDeg = flt("dright", DEMO_COMPARE_RIGHT_DEG);
    d_.steerLandDeg = flt("sland", STEER_LAND_DEG);
    d_.reaimOn = prefs_.isKey("reaim") ? prefs_.getBool("reaim", DRIVE_REAIM) : DRIVE_REAIM;
    d_.reaimRatio = flt("rratio", DRIVE_REAIM_RATIO);
}

void Settings::save() {
    prefs_.putString("name", d_.robotName);
    prefs_.putString("host", d_.hostname);
    prefs_.putString("agent", d_.agentHost);
    prefs_.putUShort("aport", d_.agentPort);
    prefs_.putString("ota", d_.otaPass);
    prefs_.putString("appass", d_.apPass);
    prefs_.putFloat("speed", d_.navSpeedMps);
    prefs_.putFloat("tol", d_.navTolM);
    prefs_.putBool("loop", d_.navLoop);
    prefs_.putString("planner", d_.planner);
    prefs_.putFloat("sdps", d_.steerDps);
    prefs_.putBool("dreset", d_.demoReset);
    prefs_.putFloat("ddist", d_.demoDistM);
    prefs_.putFloat("dright", d_.demoRightDeg);
    prefs_.putFloat("sland", d_.steerLandDeg);
    prefs_.putBool("reaim", d_.reaimOn);
    prefs_.putFloat("rratio", d_.reaimRatio);
}

SettingsData Settings::get() {
    Lock l(mtx_);
    return d_;
}

bool Settings::update(JsonObjectConst in, String& err) {
    Lock l(mtx_);
    SettingsData n = d_;
    if (in["robotName"].is<const char*>()) n.robotName = in["robotName"].as<const char*>();
    if (in["hostname"].is<const char*>()) n.hostname = in["hostname"].as<const char*>();
    if (in["agentHost"].is<const char*>()) n.agentHost = in["agentHost"].as<const char*>();
    if (in["agentPort"].is<int>()) n.agentPort = in["agentPort"].as<int>();
    if (in["otaPass"].is<const char*>()) n.otaPass = in["otaPass"].as<const char*>();
    if (in["apPass"].is<const char*>()) n.apPass = in["apPass"].as<const char*>();
    if (in["navSpeedMps"].is<float>()) n.navSpeedMps = in["navSpeedMps"].as<float>();
    if (in["navTolM"].is<float>()) n.navTolM = in["navTolM"].as<float>();
    if (in["navLoop"].is<bool>()) n.navLoop = in["navLoop"].as<bool>();
    if (in["planner"].is<const char*>()) n.planner = in["planner"].as<const char*>();
    if (in["steerDps"].is<float>()) n.steerDps = in["steerDps"].as<float>();
    if (in["demoReset"].is<bool>()) n.demoReset = in["demoReset"].as<bool>();
    if (in["demoDistM"].is<float>()) n.demoDistM = in["demoDistM"].as<float>();
    if (in["demoRightDeg"].is<float>()) n.demoRightDeg = in["demoRightDeg"].as<float>();
    if (in["steerLandDeg"].is<float>()) n.steerLandDeg = in["steerLandDeg"].as<float>();
    if (in["reaimOn"].is<bool>()) n.reaimOn = in["reaimOn"].as<bool>();
    if (in["reaimRatio"].is<float>()) n.reaimRatio = in["reaimRatio"].as<float>();

    n.robotName.trim();
    n.hostname.trim();
    n.hostname.toLowerCase();
    n.agentHost.trim();
    if (n.robotName.isEmpty() || n.robotName.length() > 31) { err = "ชื่อหุ่นต้องยาว 1-31 ตัวอักษร"; return false; }
    if (!validHostname(n.hostname)) { err = "hostname ใช้ได้แค่ a-z 0-9 และ - (ไม่เกิน 31 ตัว)"; return false; }
    if (n.agentPort == 0) { err = "port ของ agent ต้องไม่เป็น 0"; return false; }
    if (n.otaPass.length() < 4) { err = "รหัส OTA ต้องยาวอย่างน้อย 4 ตัว"; return false; }
    if (n.apPass.length() < 8 || n.apPass.length() > 63) { err = "รหัส hotspot ต้องยาว 8-63 ตัว"; return false; }
    if (n.navSpeedMps < 0.005f || n.navSpeedMps > NAV_MAX_SPEED_MPS) {
        err = "ความเร็วต้องอยู่ระหว่าง 0.005-" + String(NAV_MAX_SPEED_MPS, 3) + " m/s (เต็มกำลังได้ ~0.039 m/s)";
        return false;
    }
    if (n.navTolM < 0.0025f || n.navTolM > 0.5f) { err = "ระยะถึงจุดต้องอยู่ระหว่าง 0.0025-0.5 m (2.5 มม. - 50 ซม.)"; return false; }
    if (n.steerDps < 5.0f || n.steerDps > 720.0f) { err = "ความเร็วเลี้ยวต้องอยู่ระหว่าง 5-720 °/s"; return false; }
    if (!(n.demoDistM >= 0.10f && n.demoDistM <= 1.0f)) { err = "ระยะเป้าเดโม 3/4 ต้องอยู่ระหว่าง 0.10-1.00 m"; return false; }
    if (!(n.demoRightDeg >= 5.0f && n.demoRightDeg <= 30.0f)) {
        err = "มุมเป้าเดโม 3/4 ต้องอยู่ระหว่าง 5-30° (ต่ำกว่า ~5° ล้อถือว่าเล็งตรงอยู่แล้ว สองแบบจะวิ่งเหมือนกัน)"; return false;
    }
    if (!(n.steerLandDeg >= 0.5f && n.steerLandDeg <= 3.5f)) { err = "ระยะหยุดก่อนมุมเป้าต้องอยู่ระหว่าง 0.5-3.5°"; return false; }
    if (!(n.reaimRatio >= 1.5f && n.reaimRatio <= 8.0f)) { err = "หยุดแก้ทางที่ระยะ 1.5-8 เท่าของระยะที่พลาด"; return false; }
    bool known = false;
    for (const char* const* p = PlannerFactory::names(); *p; ++p) known = known || n.planner == *p;
    if (!known) { err = "ไม่รู้จัก algorithm: " + n.planner; return false; }
    d_ = n;
    save();
    return true;
}

float Settings::coastS(float fallback) {
    Lock l(mtx_);
    return prefs_.isKey("coast") ? prefs_.getFloat("coast", fallback) : fallback;
}

void Settings::saveCoastS(float s) {
    Lock l(mtx_);
    prefs_.putFloat("coast", s);
}

float Settings::driveGain(float fallback, float ks, float kf) {
    Lock l(mtx_);
    // A new motor curve must not reuse the correction learned for an older one.
    if (!prefs_.isKey("driveks") || !prefs_.isKey("drivekf") ||
        prefs_.getFloat("driveks") != ks || prefs_.getFloat("drivekf") != kf) return fallback;
    return prefs_.isKey("drivegain") ? prefs_.getFloat("drivegain", fallback) : fallback;
}

bool Settings::saveDriveGain(float gain, float ks, float kf) {
    Lock l(mtx_);
    if (prefs_.putFloat("driveks", ks) != sizeof(float)) return false;
    if (prefs_.putFloat("drivekf", kf) != sizeof(float)) return false;
    return prefs_.putFloat("drivegain", gain) == sizeof(float);
}

void Settings::toJson(JsonObject o, bool withSecrets) {
    Lock l(mtx_);
    o["robotName"] = d_.robotName;
    o["hostname"] = d_.hostname;
    o["agentHost"] = d_.agentHost;
    o["agentPort"] = d_.agentPort;
    o["navSpeedMps"] = d_.navSpeedMps;
    o["navTolM"] = d_.navTolM;
    o["navLoop"] = d_.navLoop;
    o["planner"] = d_.planner;
    JsonArray pl = o["planners"].to<JsonArray>();
    for (const char* const* p = PlannerFactory::names(); *p; ++p) pl.add(*p);
    o["steerDps"] = d_.steerDps;
    o["demoReset"] = d_.demoReset;
    o["demoDistM"] = d_.demoDistM;
    o["demoRightDeg"] = d_.demoRightDeg;
    o["steerLandDeg"] = d_.steerLandDeg;
    o["reaimOn"] = d_.reaimOn;
    o["reaimRatio"] = d_.reaimRatio;
    if (withSecrets) {
        o["otaPass"] = d_.otaPass;
        o["apPass"] = d_.apPass;
    }
}
