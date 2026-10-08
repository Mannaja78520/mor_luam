#include "net/NetworkManager.h"
#include <ESPmDNS.h>
#include <esp_wifi.h>
#include <algorithm>
#include <cstring>
#include <config.h>
#include "app_config.h"
#include "net/WifiPolicy.h"

namespace {
struct Lock {
    explicit Lock(SemaphoreHandle_t m) : m_(m) { xSemaphoreTake(m_, portMAX_DELAY); }
    ~Lock() { xSemaphoreGive(m_); }
    SemaphoreHandle_t m_;
};
}  // namespace

void NetworkManager::begin(WifiStore* store, Settings* settings) {
    store_ = store;
    settings_ = settings;
    mtx_ = xSemaphoreCreateMutex();

    uint8_t mac[6];
    WiFi.macAddress(mac);
    char buf[16];
    snprintf(buf, sizeof(buf), "%02X%02X%02X", mac[3], mac[4], mac[5]);
    id_ = buf;
    snprintf(buf, sizeof(buf), "%02X%02X", mac[4], mac[5]);
    apSsid_ = String(SETUP_AP_PREFIX) + buf;

    host_ = settings_->get().hostname;
    WiFi.persistent(false);              // NVS list is ours; do not let the SDK keep its own
    WiFi.setHostname(host_.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                // micro-ROS needs low latency
    startMdns();
    lostSinceMs_ = millis();
    nextAttemptMs_ = 0;
    Serial.printf("[net] id %s, %u known network(s)%s\n", id_.c_str(), store_->count(),
                  store_->seeded() ? " (seeded from network_secrets.h)" : "");
}

void NetworkManager::startMdns() {
    MDNS.end();
    if (!MDNS.begin(host_.c_str())) {
        Serial.println("[mdns] failed to start");
        return;
    }
    const SettingsData s = settings_->get();
    MDNS.addService("http", "tcp", 80);
    MDNS.enableArduino(3232, true);      // espota / Arduino IDE see the board (OtaService)
    // The same announcement as the mice modules (firmware/src/core/PeerDiscovery.cpp)
    MDNS.addService("module", "tcp", 80);
    MDNS.addServiceTxt("module", "tcp", "id", id_.c_str());
    MDNS.addServiceTxt("module", "tcp", "name", s.robotName.c_str());
    MDNS.addServiceTxt("module", "tcp", "type", "morluam");
    MDNS.addServiceTxt("module", "tcp", "fw", FW_VERSION);
    Serial.printf("[mdns] http://%s.local\n", host_.c_str());
}

// ---- joining a network ------------------------------------------------------

void NetworkManager::loadSaved(std::vector<WifiStore::Entry>& out) {
    out.clear();
    const uint8_t n = store_->count();
    for (uint8_t k = 0; k < n; ++k) {
        WifiStore::Entry e;
        if (store_->entry(k, e)) out.push_back(e);
    }
}

// signal of each saved network in the last scan, wifipolicy::NOT_SEEN if absent
std::vector<int> NetworkManager::rssiOf(const std::vector<WifiStore::Entry>& saved) {
    std::vector<int> rssi(saved.size(), wifipolicy::NOT_SEEN);
    Lock l(mtx_);
    for (size_t k = 0; k < saved.size(); ++k)
        for (auto& s : scan_)
            if (s.ssid == saved[k].ssid && s.rssi > rssi[k]) rssi[k] = s.rssi;
    return rssi;
}

int NetworkManager::indexIn(const std::vector<WifiStore::Entry>& saved, const String& ssid) {
    for (size_t k = 0; k < saved.size(); ++k)
        if (ssid == saved[k].ssid) return (int)k;
    return -1;
}

void NetworkManager::startConnect() {
    // What is on the air, then the order from WifiPolicy: the network it was
    // on first, then the saved list top to bottom.
    scanAir();
    std::vector<WifiStore::Entry> saved;
    loadSaved(saved);
    candidates_.clear();
    candIdx_ = 0;
    for (int k : wifipolicy::joinOrder(rssiOf(saved), indexIn(saved, joinedSsid_))) candidates_.push_back(saved[k]);

    if (candidates_.empty()) {
        Serial.printf("[wifi] none of the %u known network(s) is in range\n", (unsigned)saved.size());
        nextAttemptMs_ = millis() + WIFI_RETRY_PERIOD_MS;
        return;
    }
    tryCandidate();
}

void NetworkManager::tryCandidate() {
    while (candIdx_ < candidates_.size()) {
        const WifiStore::Entry& e = candidates_[candIdx_];
        Serial.printf("[wifi] joining \"%s\"\n", e.ssid);
        // Arduino 2.0.17 leaves SAE PWE unspecified (Hunt-and-Peck). Prepare
        // without connecting, then allow H2E too for WPA3-only phone hotspots.
        // The final false is tryConnect, after channel and optional BSSID.
        // begin(false) returns the previous association status, so verify
        // the prepared SDK configuration rather than treating that as an error.
        WiFi.begin(e.ssid, e.pass, 0, nullptr, false);
        wifi_config_t cfg{};
        esp_err_t err = esp_wifi_get_config(WIFI_IF_STA, &cfg);
        if (err != ESP_OK ||
            strncmp(reinterpret_cast<const char*>(cfg.sta.ssid), e.ssid, sizeof(cfg.sta.ssid)) != 0 ||
            strncmp(reinterpret_cast<const char*>(cfg.sta.password), e.pass, sizeof(cfg.sta.password)) != 0) {
            Serial.printf("[wifi] station configuration failed: %s; trying next network\n",
                          err == ESP_OK ? "configuration mismatch" : esp_err_to_name(err));
            ++candIdx_;
            continue;
        }
        cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
        err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
        if (err != ESP_OK) {
            // Keep the prepared configuration usable for existing WPA2
            // networks even if the optional SAE configuration is rejected.
            Serial.printf("[wifi] SAE H2E configuration failed: %s; using prepared configuration\n",
                          esp_err_to_name(err));
        }
        err = esp_wifi_connect();
        if (err != ESP_OK) {
            Serial.printf("[wifi] connection start failed: %s; trying next network\n", esp_err_to_name(err));
            ++candIdx_;
            continue;
        }
        lastSsid_ = e.ssid;
        connecting_ = true;
        connectStartMs_ = millis();
        return;
    }
    connecting_ = false;
    // a move up that failed goes straight back to the network it left
    nextAttemptMs_ = millis() + (movingUp_ ? 0 : WIFI_RETRY_PERIOD_MS);
    movingUp_ = false;
}

// ---- setup hotspot ----------------------------------------------------------

void NetworkManager::startAp() {
    const SettingsData s = settings_->get();
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(apSsid_.c_str(), s.apPass.c_str());
    apActive_ = true;
    Serial.printf("[ap] setup hotspot \"%s\" at http://%s\n", apSsid_.c_str(), WiFi.softAPIP().toString().c_str());
}

void NetworkManager::stopAp() {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    apActive_ = false;
    Serial.println("[ap] setup hotspot closed");
}

// ---- the loop ---------------------------------------------------------------

void NetworkManager::loop() {
    const uint32_t now = millis();
    const bool up = connected();

    if (up && !wasConnected_) {
        connecting_ = false;
        movingUp_ = false;
        joinedSsid_ = WiFi.SSID();
        moveUpCheckMs_ = now;
        Serial.printf("[wifi] joined \"%s\" as %s (http://%s.local)\n", WiFi.SSID().c_str(),
                      WiFi.localIP().toString().c_str(), host_.c_str());
        if (apActive_) apCloseAtMs_ = now + AP_KEEP_AFTER_JOIN_MS;
    }
    if (!up && wasConnected_) {
        Serial.println("[wifi] link lost");
        lostSinceMs_ = now;
        nextAttemptMs_ = now + 1000;
    }
    wasConnected_ = up;

    if (reconnectReq_) {                       // the list changed: choose again
        reconnectReq_ = false;
        WiFi.disconnect(false, false);
        connecting_ = false;
        wasConnected_ = false;
        joinedSsid_ = "";                      // choose again from priority 1
        lostSinceMs_ = now;
        nextAttemptMs_ = now + 300;
    }

    if (!up) {
        if (connecting_ && now - connectStartMs_ > WIFI_CONNECT_TIMEOUT_MS) {
            Serial.printf("[wifi] \"%s\" did not answer\n", lastSsid_.c_str());
            WiFi.disconnect(false, false);
            ++candIdx_;
            tryCandidate();
        } else if (!connecting_ && (int32_t)(now - nextAttemptMs_) >= 0) {
            startConnect();
        }
        if (!apActive_ && now - lostSinceMs_ > AP_START_AFTER_MS) startAp();
    } else {
        if (apActive_ && apCloseAtMs_ && (int32_t)(now - apCloseAtMs_) >= 0) {
            // Keep it while a phone is still on it: closing would cut off the person setting up.
            if (WiFi.softAPgetStationNum() == 0) stopAp();
            else apCloseAtMs_ = now + 10000;
        }
        if (!connecting_) checkMoveUp(now);
    }

    if (scanReq_ && !connecting_) { scanReq_ = false; scanAir(); }
    if (peersReq_ && up) { peersReq_ = false; doPeers(); }
    if (hostnameReq_) {
        hostnameReq_ = false;
        host_ = settings_->get().hostname;
        WiFi.setHostname(host_.c_str());
        startMdns();
    }
}

// On priority 2, 3 ...: every WIFI_MOVE_UP_CHECK_MS, while the robot stands
// still, look for a network higher in the list and move to it.
void NetworkManager::checkMoveUp(uint32_t now) {
    if (now - moveUpCheckMs_ < WIFI_MOVE_UP_CHECK_MS) return;
    moveUpCheckMs_ = now;
    std::vector<WifiStore::Entry> saved;
    loadSaved(saved);
    if (saved.empty() || indexIn(saved, WiFi.SSID()) == 0) return;   // already on priority 1
    if (busy_ && busy_()) return;          // never drop the link under a moving robot
    scanAir();                             // ~1.5 s; the link stays up
    const int to = wifipolicy::moveUpTo(rssiOf(saved), indexIn(saved, WiFi.SSID()), WIFI_MOVE_UP_MIN_RSSI);
    if (to < 0 || (busy_ && busy_())) return;
    Serial.printf("[wifi] priority %d \"%s\" is in range: moving up from \"%s\"\n", to + 1, saved[to].ssid,
                  WiFi.SSID().c_str());
    candidates_.assign(1, saved[to]);
    candIdx_ = 0;
    movingUp_ = true;                      // joinedSsid_ stays: if this fails, back to it at once
    WiFi.disconnect(false, false);
    tryCandidate();
}

// Scan once; the result serves both the web page list and the choice of network.
void NetworkManager::scanAir() {
    scanning_ = true;
    const int n = WiFi.scanNetworks(false, false, false, WIFI_SCAN_MS_PER_CHANNEL);
    std::vector<Seen> seen;
    for (int i = 0; i < n; ++i) {
        const String ssid = WiFi.SSID(i);
        if (ssid.isEmpty()) continue;
        bool dup = false;
        for (auto& s : seen) if (s.ssid == ssid) { dup = true; s.rssi = max(s.rssi, WiFi.RSSI(i)); }
        if (!dup) seen.push_back({ssid, WiFi.RSSI(i), WiFi.encryptionType(i) != WIFI_AUTH_OPEN});
    }
    WiFi.scanDelete();
    std::sort(seen.begin(), seen.end(), [](const Seen& a, const Seen& b) { return a.rssi > b.rssi; });
    Lock l(mtx_);
    scan_ = seen;
    scanAtMs_ = millis();
    scanning_ = false;
}

void NetworkManager::doPeers() {
    peering_ = true;
    const int n = MDNS.queryService("module", "tcp");     // blocks ~1-3 s
    std::vector<Peer> found;
    for (int i = 0; i < n; ++i) {
        Peer p;
        p.host = MDNS.hostname(i);
        p.ip = MDNS.IP(i).toString();
        p.name = MDNS.txt(i, "name");
        p.type = MDNS.txt(i, "type");
        p.id = MDNS.txt(i, "id");
        if (p.id == id_) continue;                         // that is us
        found.push_back(p);
    }
    Lock l(mtx_);
    peers_ = found;
    peersAtMs_ = millis();
    peering_ = false;
}

// ---- JSON for the web page --------------------------------------------------

void NetworkManager::statusJson(JsonObject o) {
    o["connected"] = connected();
    o["ssid"] = connected() ? WiFi.SSID() : String("");
    o["rssi"] = connected() ? WiFi.RSSI() : 0;
    o["ip"] = WiFi.localIP().toString();
    o["gateway"] = WiFi.gatewayIP().toString();
    o["host"] = host_ + ".local";
    o["connecting"] = connecting_ ? lastSsid_ : String("");
    std::vector<WifiStore::Entry> saved;
    loadSaved(saved);
    o["priority"] = connected() ? indexIn(saved, WiFi.SSID()) + 1 : 0;   // 1 = top of the list, 0 = not in it
    JsonObject ap = o["ap"].to<JsonObject>();
    ap["active"] = apActive_;
    ap["ssid"] = apSsid_;
    ap["ip"] = WiFi.softAPIP().toString();
    ap["clients"] = apActive_ ? WiFi.softAPgetStationNum() : 0;
}

void NetworkManager::scanJson(JsonObject o) {
    Lock l(mtx_);
    o["scanning"] = scanning_ || scanReq_;
    o["ageMs"] = scanAtMs_ ? millis() - scanAtMs_ : 0;
    JsonArray a = o["nets"].to<JsonArray>();
    for (auto& s : scan_) {
        JsonObject n = a.add<JsonObject>();
        n["ssid"] = s.ssid;
        n["rssi"] = s.rssi;
        n["secure"] = s.secure;
    }
}

void NetworkManager::peersJson(JsonObject o) {
    Lock l(mtx_);
    o["searching"] = peering_ || peersReq_;
    o["ageMs"] = peersAtMs_ ? millis() - peersAtMs_ : 0;
    JsonArray a = o["peers"].to<JsonArray>();
    for (auto& p : peers_) {
        JsonObject n = a.add<JsonObject>();
        n["name"] = p.name;
        n["host"] = p.host;
        n["ip"] = p.ip;
        n["type"] = p.type;
        n["id"] = p.id;
    }
}
