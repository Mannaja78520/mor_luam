/**
 * @file conf_network.h
 * @brief Network defaults. Everything here can be changed later from the
 *        robot's web page without reflashing; these are only the first-boot values.
 */
#ifndef CONF_NETWORK_H
#define CONF_NETWORK_H

#include <Arduino.h>

//---- Wi-Fi: first-boot seed ----
// The real list lives in NVS on the board and is edited from the web page
// (tab "WiFi"). This seed is copied in once, on a board whose list is empty.
// The passwords are in network_secrets.h, which is gitignored.
struct WifiSeed { const char* ssid; const char* pass; };

#if defined(__has_include)
#  if __has_include("network_secrets.h")
#    include "network_secrets.h"           // defines WIFI_SEED[]
#  else
#    warning "config/network_secrets.h missing: the board starts with no known Wi-Fi. Copy network_secrets.example.h, or add networks from the setup hotspot."
static const WifiSeed WIFI_SEED[] = { {"", ""} };
#  endif
#endif
static const size_t WIFI_SEED_COUNT = sizeof(WIFI_SEED) / sizeof(WIFI_SEED[0]);

//---- names ----
// mDNS name: the web page is http://mor-luam.local and OTA targets the same name.
static const char* DEFAULT_HOSTNAME    = "mor-luam";
static const char* DEFAULT_ROBOT_NAME  = "mor_luam";

// Setup hotspot, opened when no known Wi-Fi can be joined: "mor-luam-XXXX".
static const char* SETUP_AP_PREFIX     = "mor-luam-";
#ifndef MORLUAM_DEFAULT_AP_PASS
#define MORLUAM_DEFAULT_AP_PASS "change-me-ap"   // placeholder: the real default is in network_secrets.h (gitignored)
#endif
static const char* DEFAULT_AP_PASS = MORLUAM_DEFAULT_AP_PASS;   // WPA2 needs 8+ characters
#ifndef MORLUAM_DEFAULT_OTA_PASS
#define MORLUAM_DEFAULT_OTA_PASS "change-me-ota"   // placeholder: the real default is in network_secrets.h (gitignored)
#endif
static const char* DEFAULT_OTA_PASS = MORLUAM_DEFAULT_OTA_PASS;   // OTA over Wi-Fi (web and espota)

//---- micro-ROS agent ----
// Where the agent is looked for, in order (MicroRosBridge::candidates()):
//   1. the address set on the web page (an IP, or a name such as "my-pc.local")
//   2. the Wi-Fi gateway - on a PC hotspot (Windows "manny") the PC IS the gateway
//   3. this compiled-in address (Windows Mobile Hotspot uses 192.168.137.1)
static const IPAddress AGENT_IP(192, 168, 137, 1);
static const uint16_t  AGENT_PORT = 8888;
static const size_t    ROS_DOMAIN_ID = 10;      // ros2 side: ROS_DOMAIN_ID=10

#endif
