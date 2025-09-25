/**
 * @file conf_network.h
 * @brief Network parameters for micro-ROS with mDNS discovery
 */
#ifndef CONF_NETWORK_H
#define CONF_NETWORK_H

#include <Arduino.h>

//---- Wi-Fi ----
static const char* WIFI_SSID = "teelek_Iot";
static const char* WIFI_PASS = "12345678";

//---- micro-ROS Agent ----
// ใช้ mDNS แทนการ fix IP (เช่น 'ros2-agent.local')
// ให้ตั้ง hostname ฝั่ง agent เป็น "ros2-agent"
static const char* AGENT_HOSTNAME = "ros2-agent";
static const uint16_t AGENT_PORT   = 8888;

#endif
