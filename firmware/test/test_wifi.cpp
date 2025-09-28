#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "config.h"

static bool wifi_connect(uint32_t timeout_ms = 20000) {
  Serial.println("\n[WiFi] begin");
  WiFi.persistent(false);   // ไม่เขียนแฟลชบ่อย
  WiFi.setSleep(false);     // กันค้าง/แกว่งบางรุ่น
  WiFi.mode(WIFI_STA);

#ifdef WIFI_USE_STATIC_IP
  if (!WiFi.config(WIFI_LOCAL_IP, WIFI_GATEWAY, WIFI_SUBNET, WIFI_DNS)) {
    Serial.println("[WiFi] config STATIC IP failed");
  } else {
    Serial.printf("[WiFi] static IP: %s\n", WIFI_LOCAL_IP.toString().c_str());
  }
#endif

  // สแกนเผื่อเช็คว่าเจอ SSID มั้ย (ช่วยดีบัก)
  int n = WiFi.scanNetworks(false, true);
  bool seen = false;
  Serial.printf("[WiFi] scan: found %d APs\n", n);
  for (int i = 0; i < n; ++i) {
    String ssid = WiFi.SSID(i);
    if (ssid == WIFI_SSID) seen = true;
    Serial.printf("  - %s  RSSI=%d  ENC=%d\n", ssid.c_str(), (int)WiFi.RSSI(i), (int)WiFi.encryptionType(i));
  }
  if (!seen) {
    Serial.printf("[WiFi] SSID '%s' NOT FOUND (2.4G เท่านั้น)\n", WIFI_SSID);
  }

  // ตั้ง hostname (บางบอร์ดต้องทำก่อน begin)
#if ARDUINO_USB_MODE
  // no-op
#endif
  WiFi.setHostname(WIFI_HOSTNAME);

  Serial.printf("[WiFi] connecting to '%s' ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeout_ms) {
    delay(200);
    Serial.printf("[WiFi] status=%d\n", (int)WiFi.status());
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] FAILED to connect (เช็ค 2.4G/WPA2/รหัสผ่าน/PMF)");
    return false;
  }

  Serial.printf("[WiFi] Connected  IP=%s  MAC=%s  RSSI=%d dBm\n",
                WiFi.localIP().toString().c_str(),
                WiFi.macAddress().c_str(),
                WiFi.RSSI());

  if (!MDNS.begin(WIFI_HOSTNAME)) {
    Serial.println("[mDNS] start failed (not fatal)");
  } else {
    Serial.printf("[mDNS] You can ping %s.local\n", WIFI_HOSTNAME);
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(LED_BUILTIN, OUTPUT);

  bool ok = wifi_connect();
  if (!ok) {
    // กระพริบเร็ว = ต่อไม่ติด
    for (int i = 0; i < 10; ++i) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(150); }
  } else {
    // กระพริบช้า 3 ที = ต่อสำเร็จ
    for (int i = 0; i < 6; ++i) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(300); }
  }
}

unsigned long last_check = 0;

void loop() {
  // เฝ้าสถานะ ถ้าหลุดจะพยายามต่อใหม่อัตโนมัติ
  if (millis() - last_check > 3000) {
    last_check = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] lost connection, reconnecting...");
      WiFi.disconnect(true, true);
      delay(200);
      wifi_connect(15000);
    } else {
      // heartbeat log
      Serial.printf("[WiFi] OK  IP=%s  RSSI=%d\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
    }
  }

  // ไฟสถานะ หายใจช้าเมื่อออนไลน์
  static uint8_t duty = 0; 
  duty = (duty + 1) % 100;
  digitalWrite(LED_BUILTIN, duty < 2 ? HIGH : LOW);
  delay(20);
}
