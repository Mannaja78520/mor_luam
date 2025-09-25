#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <vector>
#include <cmath>
#include <utility>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>             
#include <std_msgs/msg/int32.h>               

#if defined(MICROROS_TRANSPORT_WIFI)
  #include <ESPmDNS.h>
#endif

#include <config.h>
#include <motor.h>
#include <PIDF.h>
#include <Utilize.h>
#include <Encoder.h>

//================ Util/macros ===================//
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { rclErrorLoop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t t0=-1; if(t0==-1) t0=uxr_millis(); if(uxr_millis()-t0>MS){ X; t0=uxr_millis(); } } while(0)

//================ Parameters (tune here) ========//
static const float   STEER_TOL_DEG          = 2.0f;   // ยอมให้ต่างจากเป้าหมายได้เท่านี้
static const uint32_t STEER_SETTLE_MS       = 100;    // ต้องอยู่ในกรอบกี่ ms ก่อนปล่อยวิ่ง
static const int     STEER_LEAK_TICKS_THR   = 5;      // ticks รั่วตอนเลี้ยวที่ยอมให้ทิ้ง

// ทิศการใช้งานมอเตอร์เดี่ยว
static const int     STEER_MOTOR_DIR        = +1;     // RPM>0 = โหมดสเตียร์ (หมุนขวา)
static const int     DRIVE_MOTOR_DIR        = -1;     // RPM<0 = โหมดวิ่งหน้า (หมุนซ้าย)

static const float   MAX_RPM                 = 120.0f;
static const int     PWM_MAX_ABS             = PWM_Max;  // จาก motor.h
static const float   RPM_TO_PWM_K            = (PWM_MAX_ABS / MAX_RPM); // สเกลง่ายๆ

//================ ROS entities ==================//
rcl_publisher_t pub_debug_cmd;
rcl_subscription_t sub_cmd_move;

geometry_msgs__msg__Twist cmd_move_msg;
geometry_msgs__msg__Twist dbg_cmd_msg;

// feedback publishers
rcl_publisher_t pub_ticks;            
rcl_publisher_t pub_steer_deg;        
std_msgs__msg__Int32 msg_ticks;       
std_msgs__msg__Float32 msg_steer_deg; 

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_ctrl;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;

//================ App state =====================//
enum Phase { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;
enum Mode  { STEER_TO_HEADING, DRIVE } mode;

unsigned long steer_enter_time_ms = 0;

Controller motor(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR_INV, MOTOR_BRAKE, MOTOR_PWM, MOTOR_IN_A, MOTOR_IN_B);

Encoder encoder(MOTOR_ENCODER_PIN_A, MOTOR_ENCODER_PIN_B, MOTOR_ENCODER_INV, MOTOR_ENCODER_RATIO);


// targets from ROS
volatile float target_rpm = 0.0f;
volatile float target_steer_deg = 0.0f;

// feedback
volatile long ticks_acc = 0;
volatile float steer_deg = 0.0f;

// helpers
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
struct timespec getTime();

void controlCallback(rcl_timer_t *timer, int64_t last_call_time);
void cmd_move_callback(const void * msgin);

// // hardware functions you must implement or map
// static inline float readSteerDeg() {
//   // TODO: อ่าน AS5600 -> องศา 0..360
//   // สมมติ Utilize.h มีฟังก์ชันที่คุณใช้อยู่ เช่น Utilize::getAS5600Deg();
//   return Utilize::getAS5600Deg();  // แก้ให้ตรงของจริง
// }

//================ WIFI/mDNS =====================//
#if defined(MICROROS_TRANSPORT_WIFI)
  static bool connectWifiAndResolveAgent(IPAddress &out_ip) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis()-t0 < 15000) delay(100);
    if (WiFi.status() != WL_CONNECTED) return false;

    if (!MDNS.begin("esp32")) { /* not fatal */ }
    IPAddress ip = MDNS.queryHost(AGENT_HOSTNAME, 2000);
    if (ip == (uint32_t)0) return false;
    out_ip = ip;
    return true;
  }
#endif

//================ Setup/Loop ====================//
void setup() {
  Serial.begin(115200);
  delay(50);

#if defined(MICROROS_TRANSPORT_WIFI)
  IPAddress agent_ip;
  if (!connectWifiAndResolveAgent(agent_ip)) { delay(3000); ESP.restart(); }
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, agent_ip, AGENT_PORT);
#else
  set_microros_serial_transports(Serial);
#endif

  state = WAITING_AGENT;
  mode = STEER_TO_HEADING;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1500, state = (RMW_RET_OK == rmw_uros_ping_agent(1000,10)) ? AGENT_AVAILABLE : WAITING_AGENT; );
      break;
    case AGENT_AVAILABLE:
      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroyEntities();
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(700, state = (RMW_RET_OK == rmw_uros_ping_agent(600,5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED; );
      if (state == AGENT_CONNECTED) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
      break;
    case AGENT_DISCONNECTED:
      motor.spin(0);
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }
}

//================ Control =======================//
static inline int rpmToPwm(float rpm_signed) {
  rpm_signed = constrain(rpm_signed, -MAX_RPM, MAX_RPM);
  int pwm = (int) roundf(rpm_signed * RPM_TO_PWM_K);
  pwm = constrain(pwm, -PWM_MAX_ABS, PWM_MAX_ABS);
  return pwm;
}

void cmd_move_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;
  target_rpm       = msg->linear.x;       // RPM (signed)
  target_steer_deg = msg->angular.z;      // deg (0..360)
}

void controlCallback(rcl_timer_t *timer, int64_t) {
  if (!timer) return;

  // --- read feedback ---
  long ticks_now = encoder.read();
  // steer_deg = readSteerDeg();

  // publish feedback
  msg_ticks.data = (int32_t)ticks_now;
  msg_steer_deg.data = steer_deg;
  RCSOFTCHECK(rcl_publish(&pub_ticks, &msg_ticks, NULL));
  RCSOFTCHECK(rcl_publish(&pub_steer_deg, &msg_steer_deg, NULL));

  // --- FSM: decide motor action ---
  float err = target_steer_deg - steer_deg;
  // wrap to [-180,180]
  err = fmodf(err + 540.0f, 360.0f) - 180.0f;

  static Mode last_mode = STEER_TO_HEADING;
  static unsigned long enter_ms = millis();

  switch (mode) {
    case STEER_TO_HEADING: {
      // อยู่โหมดสเตียร์: สั่งหมุนทิศ "สเตียร์", ตัดระยะที่รั่ว
      int steer_pwm = STEER_MOTOR_DIR * (PWM_MAX_ABS / 2); // 50% duty
      if (fabsf(err) > STEER_TOL_DEG) {
        motor.spin(steer_pwm);
        steer_enter_time_ms = millis(); // reset settle timer
        // ทิ้ง ticks รั่ว: รีเซ็ต baselineในโอโดเมตรีฝั่งบน (เราส่งแต่ ticks สะสม ให้ฝั่งบนตัดเอง)
      } else {
        // ภายในกรอบ → ต้องอยู่ครบเวลาหน่วงก่อนปล่อยวิ่ง
        if (millis() - steer_enter_time_ms >= STEER_SETTLE_MS) {
          mode = DRIVE;
          enter_ms = millis();
        } else {
          motor.spin(0);
        }
      }
    } break;

    case DRIVE: {
      // ถ้าเลี้ยวหลุดกรอบ → กลับไปสเตียร์
      if (fabsf(err) > STEER_TOL_DEG) {
        mode = STEER_TO_HEADING;
        motor.spin(0);
        break;
      }
      // โหมดวิ่งหน้า: ใช้ทิศ “วิ่งหน้า” เท่านั้น
      float rpm_cmd_drive = DRIVE_MOTOR_DIR * fabsf(target_rpm);
      int pwm = rpmToPwm(rpm_cmd_drive);
      motor.spin(pwm);
    } break;
  }

  // debug out
  dbg_cmd_msg.linear.x  = target_rpm;
  dbg_cmd_msg.angular.z = target_steer_deg;
  RCSOFTCHECK(rcl_publish(&pub_debug_cmd, &dbg_cmd_msg, NULL));
}

//================ ROS entities ==================//
bool createEntities() {
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 10);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  RCCHECK(rclc_node_init_default(&node, "mor_luam_firmware", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_default(
      &pub_debug_cmd, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/mor_luam/debug/wheel/cmd_vel"));

  RCCHECK(rclc_publisher_init_default(               
      &pub_ticks, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/mor_luam/feedback/drive_ticks"));

  RCCHECK(rclc_publisher_init_default(               
      &pub_steer_deg, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/mor_luam/feedback/steer_deg"));

  // Subscriber (รับคำสั่งเดียวตามสัญญา)
  RCCHECK(rclc_subscription_init_default(
      &sub_cmd_move, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/mor_luam/cmd_vel/move"));                      // UPDATED topic ชัดเจน

  // Timer @ 100 Hz (10 ms)
  RCCHECK(rclc_timer_init_default(&timer_ctrl, &support, RCL_MS_TO_NS(10), controlCallback));

  // Executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_ctrl));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_move, &cmd_move_msg, &cmd_move_callback, ON_NEW_DATA));

  syncTime();
  return true;
}

bool destroyEntities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_subscription_fini(&sub_cmd_move, &node);
  rcl_publisher_fini(&pub_debug_cmd, &node);
  rcl_publisher_fini(&pub_ticks, &node);         
  rcl_publisher_fini(&pub_steer_deg, &node);

  rcl_node_fini(&node);
  rcl_timer_fini(&timer_ctrl);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
  return true;
}

//================ Time sync =====================//
void syncTime() {
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_ms = rmw_uros_epoch_millis();
  time_offset = ros_ms - now;
}
struct timespec getTime() {
  struct timespec tp{0};
  unsigned long long now = millis() + time_offset;
  tp.tv_sec  = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000ULL;
  return tp;
}
void rclErrorLoop() { ESP.restart(); }
