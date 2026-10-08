#include "net/OtaService.h"
#include <ArduinoOTA.h>
#include <Update.h>
#include "app/Settings.h"
#include "control/ControlLoop.h"

void OtaService::begin(Settings* settings, ControlLoop* ctrl, const String& hostname) {
    settings_ = settings;
    ctrl_ = ctrl;
    ArduinoOTA.setHostname(hostname.c_str());
    ArduinoOTA.setPassword(settings_->get().otaPass.c_str());
    ArduinoOTA.setMdnsEnabled(false);         // NetworkManager owns mDNS and announces _arduino._tcp
    ArduinoOTA.onStart([this]() {
        ctrl_->halt("กำลังอัปเดต firmware");
        updating_ = true;
        pct_ = 0;
        Serial.println("[OTA] update starting (espota)");
    });
    ArduinoOTA.onProgress([this](unsigned int done, unsigned int total) {
        pct_ = total ? (uint8_t)((done * 100ULL) / total) : 0;
    });
    ArduinoOTA.onEnd([this]() {
        updating_ = false;
        Serial.println("[OTA] image verified, rebooting into it");
    });
    ArduinoOTA.onError([this](ota_error_t e) {
        updating_ = false;
        lastErr_ = e == OTA_AUTH_ERROR ? "รหัส OTA ไม่ถูก" : "อัปเดตไม่สำเร็จ (firmware เดิมยังอยู่)";
        Serial.printf("[OTA] failed (%u) - the running firmware is unchanged\n", (unsigned)e);
    });
    ArduinoOTA.begin();
    Serial.printf("[OTA] ready: espota to %s.local, or the web page\n", hostname.c_str());
}

void OtaService::loop() {
    // Not servicing the port is the clean refusal: the uploader just sees no
    // answer, and a driving robot is not disturbed.
    if (!updating_ && ctrl_->moving()) return;
    ArduinoOTA.handle();
}

bool OtaService::beginWeb(size_t size, const String& pass, String& err) {
    lastErr_ = "";
    if (pass != settings_->get().otaPass) err = "รหัส OTA ไม่ถูก";
    else if (ctrl_->moving()) err = "หุ่นกำลังวิ่ง - หยุดก่อนแล้วค่อยอัปเดต";
    else if (updating_) err = "มีการอัปเดตอื่นอยู่";
    else if (!Update.begin(size ? size : UPDATE_SIZE_UNKNOWN, U_FLASH)) err = "เริ่มอัปเดตไม่ได้ (ไฟล์ใหญ่เกิน?)";
    if (!err.isEmpty()) { lastErr_ = err; return false; }
    ctrl_->halt("กำลังอัปเดต firmware");
    updating_ = true;
    total_ = size;
    done_ = 0;
    pct_ = 0;
    lastErr_ = "";
    Serial.printf("[OTA] web upload, %u bytes\n", (unsigned)size);
    return true;
}

bool OtaService::writeWeb(const uint8_t* data, size_t len, String& err) {
    if (!updating_) { err = "ยังไม่ได้เริ่มอัปเดต"; return false; }
    if (Update.write(const_cast<uint8_t*>(data), len) != len) {
        err = "เขียน flash ไม่ได้";
        lastErr_ = err;
        abortWeb();
        return false;
    }
    done_ += len;
    if (total_) pct_ = (uint8_t)((done_ * 100ULL) / total_);
    return true;
}

bool OtaService::endWeb(String& err) {
    if (!updating_) { err = "ยังไม่ได้เริ่มอัปเดต"; return false; }
    updating_ = false;
    if (!Update.end(true)) {
        err = String("ไฟล์ไม่ผ่านการตรวจ: ") + Update.errorString() + " (firmware เดิมยังอยู่)";
        lastErr_ = err;
        return false;
    }
    pct_ = 100;
    return true;
}

void OtaService::abortWeb() {
    if (updating_) Update.abort();
    updating_ = false;
}

void OtaService::refreshPassword() { ArduinoOTA.setPassword(settings_->get().otaPass.c_str()); }
