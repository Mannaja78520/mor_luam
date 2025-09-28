#include <WiFi.h>
#include <vector>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

// ===== Wi-Fi & micro-ROS agent (UDP) =====
static const char* WIFI_SSID  = "Teelek_IoT";
static const char* WIFI_PASS  = "12345678";
static const uint16_t AGENT_PORT = 8888;     // micro-ROS agent UDP port (ปรับได้)

// ===== TCP probe ใช้กรอง host ที่มีชีวิตก่อนจะลอง ping agent =====
static const uint16_t PROBE_PORTS[] = {80, 53, 22, 8888};
static const size_t   PROBE_N = sizeof(PROBE_PORTS)/sizeof(PROBE_PORTS[0]);

// สแกนหนึ่งรอบ /24: ถ้าไม่เจอ agent ให้รีบูท
static const uint32_t REBOOT_AFTER_ONE_SCAN = 1;  // 1 = เปิดใช้งาน

// ---------------- micro-ROS app (ของจริงคุณค่อยยัด createEntities()/executor ลงไป) -------------
extern "C" {
  #include <rcl/rcl.h>
  #include <rcl/error_handling.h>
  #include <rclc/rclc.h>
  #include <rclc/executor.h>
}

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

#define RCCHECK(fn) do{ rcl_ret_t rc = fn; if(rc!=RCL_RET_OK){ Serial.println("[ROS] RCCHECK fail, reboot"); delay(1000); ESP.restart(); } }while(0)

bool ros_create_entities() {
  allocator = rcl_get_default_allocator();
  rcl_init_options_t opts = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&opts, allocator));
  // ถ้ามี domain id ก็ใส่
  // rcl_init_options_set_domain_id(&opts, 10);

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &opts, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  return true;
}

void ros_spin_some() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
}

void ros_destroy_entities() {
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
}

// ---------------- TCP probe ----------------
bool tcpProbeHost(const IPAddress& ip) {
  for (size_t i=0; i<PROBE_N; ++i) {
    WiFiClient c;
    c.setTimeout(150); // ช่วยให้ไว
    if (c.connect(ip, PROBE_PORTS[i])) {
      c.stop();
      return true;
    }
  }
  return false;
}

// ---------------- สแกน /24 แล้วไล่ ping agent ----------------
bool find_and_bind_agent_udp(IPAddress &found_ip) {
  IPAddress me = WiFi.localIP();
  IPAddress base(me[0], me[1], me[2], 0);

  Serial.printf("[Scan] base=%s/24 me=%s\n",
                base.toString().c_str(), me.toString().c_str());

  std::vector<IPAddress> candidates;

  // (1) ลอง gateway ก่อน (มักเป็นเราท์เตอร์/บางทีรัน agent)
  candidates.push_back(WiFi.gatewayIP());

  // (2) สแกนทั้ง /24
  for (int last=1; last<=254; ++last) {
    IPAddress ip(base[0], base[1], base[2], last);
    if (ip == me) continue;
    // กรองคร่าวๆ ว่ามีชีวิตด้วย TCP probe เพื่อไม่ต้องตั้ง transport มั่วทุก IP
    if (tcpProbeHost(ip)) {
      Serial.printf("[+] %s alive\n", ip.toString().c_str());
      candidates.push_back(ip);
    }
    delay(1);
  }

  // (3) ไล่ IP ที่เจอ: ตั้ง UDP transport แล้ว ping agent
  for (auto &ip : candidates) {
    Serial.printf("[Agent] try %s:%u ...\n", ip.toString().c_str(), AGENT_PORT);
    // ตั้ง WiFi UDP transport (ฟังก์ชันนี้จะ ensure Wi-Fi พร้อมใช้เอง)
    set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, ip, AGENT_PORT);

    // ping agent สั้นๆ
    rmw_ret_t ok = rmw_uros_ping_agent(150, 2);
    if (ok == RMW_RET_OK) {
      Serial.printf("[Agent] FOUND at %s:%u\n", ip.toString().c_str(), AGENT_PORT);
      found_ip = ip;
      return true;
    }
  }
  return false;
}

// ---------------- setup/loop ----------------
void setup() {
  Serial.begin(115200);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);

  Serial.printf("[WiFi] Connecting to %s ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print("."); }
  Serial.printf("\n[WiFi] Connected, IP=%s, GW=%s, MASK=%s\n",
    WiFi.localIP().toString().c_str(),
    WiFi.gatewayIP().toString().c_str(),
    WiFi.subnetMask().toString().c_str());

  IPAddress agent_ip;
  bool found = find_and_bind_agent_udp(agent_ip);
  if (!found) {
    Serial.println("[Agent] NOT found in this /24 scan.");
    if (REBOOT_AFTER_ONE_SCAN) {
      Serial.println("[Agent] Reboot to retry...");
      delay(1000);
      ESP.restart();
    } else {
      // อยู่เฉยๆ แล้วค่อยลองใหม่ใน loop()
    }
  } else {
    // เจอแล้ว: สร้าง entity แล้วเข้า runtime loop
    if (ros_create_entities()) {
      Serial.println("[ROS] Entities created, entering runtime.");
    }
  }
}

unsigned long last_retry = 0;

void loop() {
  // ถ้าเข้า runtime แล้วให้ spin + คอยเช็ค agent หลุด
  static bool running = (WiFi.status()==WL_CONNECTED);
  // เช็คว่าเรามี node อยู่มั้ย (หยาบๆ จาก executor)
  if (running) {
    // ping สั้นๆ ไว้กัน connection ตาย
    if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
      ros_spin_some();
    } else {
      Serial.println("[Agent] disconnected -> destroy & reboot");
      ros_destroy_entities();
      delay(500);
      ESP.restart();
    }
    return;
  }

  // ถ้าอยากให้สแกนซ้ำเป็นช่วงๆ แบบยังไม่รีบูท (กรณีปิด REBOOT_AFTER_ONE_SCAN)
  if (millis() - last_retry > 30000) {
    last_retry = millis();
    IPAddress ip;
    if (find_and_bind_agent_udp(ip)) {
      if (ros_create_entities()) {
        running = true;
      }
    }
  }
}
