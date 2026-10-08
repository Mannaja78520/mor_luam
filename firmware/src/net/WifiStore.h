#pragma once
// The Wi-Fi networks this robot knows, kept in NVS on the robot (namespace
// "wifi"). Adapted from GPS_Localize/firmware/src/net/wifi_store.h.
//
// The compiled-in list (config/network_secrets.h) is only a FIRST-BOOT SEED:
// copied in once when NVS is empty, never read again. After that the web page
// is the only editor.
//
// List order is priority. NetworkManager retries the previous network first
// after a drop, then tries visible saved networks from the top of the list.
// While stopped, it can move back to a higher-priority network in range.
//
// Unlike GPS_Localize, passwords CAN be read back: the owner asked to see
// them on the web page. Anyone who can open the page can therefore read them,
// so keep the robot on networks you trust.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <config.h>   // WifiSeed

class WifiStore {
public:
    static const uint8_t MAX = 6;
    static const uint8_t SSID_MAX = 33;   // 32 + terminator (802.11)
    static const uint8_t PASS_MAX = 65;   // 64 + terminator

    struct Entry {
        char ssid[SSID_MAX];
        char pass[PASS_MAX];
    };

    template <size_t N>
    void begin(const WifiSeed (&seed)[N]);

    uint8_t count();
    bool entry(uint8_t i, Entry& out);
    bool set(const char* ssid, const char* pass);   // add, or replace the password
    bool remove(const char* ssid);
    bool moveTo(const char* ssid, uint8_t to);
    void toJson(JsonArray out);
    bool seeded() const { return seeded_; }

private:
    int16_t indexOf(const char* ssid) const;
    static void copy(Entry& dst, const char* ssid, const char* pass);
    bool load();
    bool save();
    void lock() { xSemaphoreTake(mtx_, portMAX_DELAY); }
    void unlock() { xSemaphoreGive(mtx_); }

    Preferences prefs_;
    Entry entries_[MAX];
    uint8_t count_ = 0;
    bool seeded_ = false;
    SemaphoreHandle_t mtx_ = nullptr;
};

template <size_t N>
void WifiStore::begin(const WifiSeed (&seed)[N]) {
    mtx_ = xSemaphoreCreateMutex();
    prefs_.begin("wifi", false);
    count_ = 0;
    if (!load()) {                          // first boot on this board: copy the seed in
        for (size_t i = 0; i < N && count_ < MAX; ++i) {
            if (!seed[i].ssid || !seed[i].ssid[0]) continue;
            copy(entries_[count_++], seed[i].ssid, seed[i].pass);
        }
        save();
        seeded_ = true;
    }
}
