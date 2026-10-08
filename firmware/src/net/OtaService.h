#pragma once
// Firmware update over Wi-Fi, two ways:
//   * ArduinoOTA / espota (port 3232):  pio run -e morluam_ota -t upload
//   * the web page (POST /api/ota, handled in WebApp with beginWeb/writeWeb/endWeb)
// Both need the OTA password (Settings::otaPass) and both refuse while the
// robot is moving - rewriting flash while the wheel turns leaves the motor on
// its last duty cycle with nothing running that could stop it. Standing still
// is fine: the wheel is halted the moment an update is accepted.
//
// The new image goes to the inactive OTA partition and is only switched to
// after it arrives whole and verified, so a dropped link costs a retry, not a board.
#include <Arduino.h>

class ControlLoop;
class Settings;

class OtaService {
public:
    void begin(Settings* settings, ControlLoop* ctrl, const String& hostname);
    void loop();                                    // services espota only while standing still
    void refreshPassword();                         // after the password changed on the web page

    // web upload, in chunks (WebApp)
    bool beginWeb(size_t size, const String& pass, String& err);
    bool writeWeb(const uint8_t* data, size_t len, String& err);
    bool endWeb(String& err);
    void abortWeb();

    bool updating() const { return updating_; }
    uint8_t progressPct() const { return pct_; }
    String lastError() const { return lastErr_; }

private:
    Settings* settings_ = nullptr;
    ControlLoop* ctrl_ = nullptr;
    volatile bool updating_ = false;
    volatile uint8_t pct_ = 0;
    size_t total_ = 0, done_ = 0;
    String lastErr_;
};
