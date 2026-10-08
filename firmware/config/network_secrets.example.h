// Copy to network_secrets.h (gitignored) and put your networks here.
// Only used on a board whose stored Wi-Fi list is empty; after that, edit the
// list from the robot's web page (tab "WiFi").
#pragma once
static const WifiSeed WIFI_SEED[] = {
    {"manny", "your-hotspot-password"},     // this PC's Windows Mobile Hotspot
    {"Teelek_IoT", "router-password"},
};

// Default passwords until changed on the web page (Settings).
#define MORLUAM_DEFAULT_AP_PASS "change-me-ap"     // 8-63 characters
#define MORLUAM_DEFAULT_OTA_PASS "change-me-ota"   // 4+ characters
