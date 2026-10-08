#pragma once
// Wi-Fi, the setup hotspot, mDNS and finding other robots.
//
//  * joins by PRIORITY = the order of the saved list (net/WifiPolicy.h):
//    after a drop the network it was on first, then priority 1, 2, 3 ...;
//    on a lower one it moves up when a higher one is back (robot standing still)
//  * no known network for AP_START_AFTER_MS -> opens "mor-luam-XXXX"
//    (192.168.4.1) so the robot can be set up from a phone; closes it
//    AP_KEEP_AFTER_JOIN_MS after a real network is joined
//  * mDNS: http://<hostname>.local, and the same "_module._tcp" announcement
//    the mice modules use (TXT id/name/type=morluam/fw), so a hub or the PC
//    tool tools/find_robots.py can find it
//
// loop() runs in Arduino loop(). The query methods are safe from any task.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <functional>
#include <vector>
#include "app/Settings.h"
#include "net/WifiStore.h"

class NetworkManager {
public:
    void begin(WifiStore* store, Settings* settings);
    void loop();

    // requests from the web page (handled in loop())
    void requestReconnect() { reconnectReq_ = true; }
    void requestScan() { scanReq_ = true; }
    void requestPeers() { peersReq_ = true; }
    void requestHostnameApply() { hostnameReq_ = true; }
    // true while the robot moves: then the link is never dropped to move up a priority
    void setBusyCheck(std::function<bool()> busy) { busy_ = busy; }

    bool connected() const { return WiFi.status() == WL_CONNECTED; }
    bool apActive() const { return apActive_; }
    IPAddress ip() const { return WiFi.localIP(); }
    IPAddress gateway() const { return WiFi.gatewayIP(); }
    String id() const { return id_; }
    String apSsid() const { return apSsid_; }
    void statusJson(JsonObject out);
    void scanJson(JsonObject out);
    void peersJson(JsonObject out);

private:
    void startConnect();
    void tryCandidate();
    void startAp();
    void stopAp();
    void startMdns();
    void scanAir();
    void loadSaved(std::vector<WifiStore::Entry>& out);
    std::vector<int> rssiOf(const std::vector<WifiStore::Entry>& saved);
    static int indexIn(const std::vector<WifiStore::Entry>& saved, const String& ssid);
    void checkMoveUp(uint32_t now);
    void doPeers();

    WifiStore* store_ = nullptr;
    Settings* settings_ = nullptr;
    String id_, apSsid_, host_;

    // connecting
    std::vector<WifiStore::Entry> candidates_;
    size_t candIdx_ = 0;
    bool connecting_ = false;
    bool wasConnected_ = false;
    uint32_t connectStartMs_ = 0;
    uint32_t nextAttemptMs_ = 0;
    uint32_t lostSinceMs_ = 0;
    String lastSsid_;                    // the one being tried
    String joinedSsid_;                  // the one joined last (tried first after a drop)
    bool movingUp_ = false;              // this attempt is a move to a higher priority
    uint32_t moveUpCheckMs_ = 0;
    std::function<bool()> busy_;

    // hotspot
    bool apActive_ = false;
    uint32_t apCloseAtMs_ = 0;

    // web requests
    volatile bool reconnectReq_ = false, scanReq_ = false, peersReq_ = false, hostnameReq_ = false;

    struct Seen { String ssid; int32_t rssi; bool secure; };
    struct Peer { String name, host, ip, type, id; };
    std::vector<Seen> scan_;
    std::vector<Peer> peers_;
    uint32_t scanAtMs_ = 0, peersAtMs_ = 0;
    bool scanning_ = false, peering_ = false;
    SemaphoreHandle_t mtx_ = nullptr;
};
