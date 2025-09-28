#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>

#include <config.h>
#include <motor.h>
#include <PIDF.h>
#include <esp32_Encoder.h>        // ENCODER
#include <Wire.h>
#include <Adafruit_AS5600.h>      // AS5600

//========================= Utils =========================//
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { rclErrorLoop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do{ static volatile int64_t t0=-1; if(t0==-1) t0=uxr_millis(); if(uxr_millis()-t0>(MS)){ X; t0=uxr_millis(); } }while(0)

static inline float wrap360(float a){ float x=fmodf(a,360.0f); return x<0?x+360.0f:x; }
// error [-180,180] (ใช้เช็คหลุด tolerance ใน DRIVE)
static inline float ang_err_deg(float target, float current){
  return fmodf((wrap360(target)-wrap360(current)+540.0f),360.0f)-180.0f;
}
// error 0..360 ตามเข็ม (ใช้สั่งเลี้ยวทิศเดียว)
static inline float cw_error_deg(float target, float current){
  return fmodf(target - current + 360.0f, 360.0f);
}
static inline bool cw_in_tolerance(float e, float tol){
  return (e <= tol) || (e >= (360.0f - tol));
}

//===================== Parameters (tune) ===================//
static const int STEER_MOTOR_DIR = +1; // ทิศหมุนสำหรับโหมดเลี้ยว
static const int DRIVE_MOTOR_DIR = -1; // ทิศหมุนสำหรับโหมดวิ่งหน้า

static const uint32_t STEER_SETTLE_MS = 100; // ต้องอยู่ในกรอบต่อเนื่องก่อนปล่อยวิ่ง
static const uint32_t CTRL_PERIOD_MS  = 10;  // 100 Hz

// static const float RPM_TO_PWM_K = ((float)PWM_SPIN_Max * MAX_RPM_RATIO) / (float)MOTOR_MAX_RPM;

//==================== micro-ROS entities ===================//
rclc_support_t support;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t ctrl_timer;
rclc_executor_t executor;

rcl_subscription_t sub_cmd_move;
geometry_msgs__msg__Twist cmd_move_msg;

rcl_publisher_t pub_debug_cmd;   // Twist (debug)
rcl_publisher_t pub_ticks;       // Int32
rcl_publisher_t pub_steer_deg;   // Float32
rcl_publisher_t pub_imu_yaw;     // Float32
rcl_publisher_t pub_imu_ok;      // Bool
rcl_publisher_t pub_state;       // String

std_msgs__msg__Int32   msg_ticks;
std_msgs__msg__Float32 msg_steer_deg, msg_imu_yaw;
std_msgs__msg__Bool    msg_imu_ok;
std_msgs__msg__String  msg_state;
geometry_msgs__msg__Twist dbg_cmd_msg;

unsigned long long time_offset = 0;

enum Conn { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } conn_state;
enum Mode { STEER_TO_HEADING, DRIVE } mode;

//==================== Hardware instances ===================//
Controller motor(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS,
                 MOTOR_INV, MOTOR_BRAKE, MOTOR_PWM, MOTOR_IN_A, MOTOR_IN_B);

esp32_Encoder encoder(MOTOR_ENCODER_PIN_A, MOTOR_ENCODER_PIN_B,
                      COUNTS_PER_REV, MOTOR_ENCODER_INV, MOTOR_ENCODER_RATIO, WHEEL_DIAMETER);

PIDF spin (0, PWM_SPIN_Max,
           Wheel_SPIN_KP,  Wheel_SPIN_KI,  Wheel_SPIN_I_Min,  Wheel_SPIN_I_Max,
           Wheel_SPIN_KD,  Wheel_SPIN_KF,  Wheel_SPIN_ERROR_TOLERANCE);

PIDF steer(0, PWM_STEER_Max,
           Wheel_STEER_KP, Wheel_STEER_KI, Wheel_STEER_I_Min, Wheel_STEER_I_Max,
           Wheel_STEER_KD, Wheel_STEER_KF, Wheel_STEER_ERROR_TOLERANCE);

Adafruit_AS5600 as5600;

//======================= App state =========================//
volatile float target_rpm = 0.0f;       // RPM (signed) จาก Python
volatile float target_steer_deg = 0.0f; // 0..360 จาก Python
volatile long  ticks_acc = 0;
volatile float steer_deg = 0.0f;

bool imu_available = false;             // true ถ้าอ่าน yaw ได้จริง
float imu_yaw_deg  = 0.0f;

unsigned long steer_enter_ok_ms = 0;
int last_pwm_cmd = 0;                   // ไว้ debug

//===================== Forward decls =======================//
void rclErrorLoop();
bool createEntities();
bool destroyEntities();
void controlCallback(rcl_timer_t *timer, int64_t);

//======================== Setup/Loop =======================//
#if defined(MICROROS_TRANSPORT_WIFI)
static bool wifi_connect(uint32_t tmo_ms=20000){
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0=millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t0<tmo_ms) delay(200);
  return WiFi.status()==WL_CONNECTED;
}
#endif

void setup(){
  Serial.begin(115200);
  delay(50);

  Wire.begin(SDA_PIN, SCL_PIN);
  imu_available = false; // ตั้ง false ไว้ก่อน (ถ้ามี IMU จริงให้เซ็ต true หลัง init สำเร็จ)

  if(!as5600.begin()){
    Serial.println("[AS5600] NOT FOUND!");
  }else{
    Serial.println("[AS5600] OK");
    as5600.setSlowFilter(AS5600_SLOW_FILTER_16X);
    as5600.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);
    as5600.setPowerMode(AS5600_POWER_MODE_NOM);
  }

#if defined(MICROROS_TRANSPORT_WIFI)
  if(!wifi_connect()){ Serial.println("[WiFi] FAIL"); delay(3000); ESP.restart(); }
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, AGENT_IP, AGENT_PORT);
#else
  set_microros_serial_transports(Serial);
#endif

  // derivative filter (ถ้าใน PIDF ของคุณรองรับ)
  steer.setDFilterCutoffHz(8.0f);
  spin .setDFilterCutoffHz(12.0f);
  steer.reset();
  spin.reset();
  mode = STEER_TO_HEADING;
  conn_state = WAITING_AGENT;
}

void loop(){
  switch (conn_state){
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1500, conn_state = (RMW_RET_OK==rmw_uros_ping_agent(1000,10)) ? AGENT_AVAILABLE : WAITING_AGENT; );
      break;
    case AGENT_AVAILABLE:
      conn_state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if(conn_state==WAITING_AGENT) destroyEntities();
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(700, conn_state = (RMW_RET_OK==rmw_uros_ping_agent(600,5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED; );
      if(conn_state==AGENT_CONNECTED) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
      break;
    case AGENT_DISCONNECTED:
      motor.spin(0);
      destroyEntities();
      conn_state = WAITING_AGENT;
      break;
  }
}

//======================= Helpers ==========================//
static inline float readSteerDegRaw(){
  uint16_t raw = as5600.getAngle();
  return (raw * 360.0f) / 4096.0f; // 0..360 จากชิป
}

static inline float readSteerDeg(){
  float raw = readSteerDegRaw();                     // 0..360
  float mech = STEER_SENSE * raw + STEER_ZERO_OFFSET_DEG;
  return wrap360(mech);                              // มุมกลไก 0..360 ที่ “ทิศถูกต้อง”
}

// TODO: ถ้ามี IMU จริง ให้เปลี่ยนฟังก์ชันนี้ให้คืน true/false ตามสถานะอ่านได้
static inline bool readImuYawDeg(float &yaw_out){
  // ตัวอย่าง fallback: ไม่มี IMU → false
  // ถ้ามี IMU จริง เช่น Utilize::getYawDeg() ให้เช็คสถานะและคืน true
  yaw_out = 0.0f;
  return false;
}

//=================== ROS Callbacks/Control =================//
static void cmd_move_cb(const void* msgin){
  const geometry_msgs__msg__Twist* m = (const geometry_msgs__msg__Twist*) msgin;
  float rpm_cmd = m->linear.x;
  if (rpm_cmd >  MOTOR_MAX_RPM) rpm_cmd =  MOTOR_MAX_RPM;
  if (rpm_cmd < -MOTOR_MAX_RPM) rpm_cmd = -MOTOR_MAX_RPM;
  target_rpm       = rpm_cmd;       // คงเครื่องหมายไว้ (เดินหน้า/ถอยตามที่คุณกำหนด)
  target_steer_deg = wrap360(m->angular.z); // 0..360
}

void controlCallback(rcl_timer_t *timer, int64_t){
  if(!timer) return;

  // ----- Feedback -----
  ticks_acc = encoder.read();
  float rpm_raw = encoder.getRPM();
  steer_deg = wrap360(readSteerDeg());

  float yaw_tmp;
  imu_available = readImuYawDeg(yaw_tmp);
  if (imu_available) imu_yaw_deg = wrap360(yaw_tmp);

  msg_ticks.data      = (int32_t)ticks_acc;
  msg_steer_deg.data  = steer_deg;
  msg_imu_yaw.data    = imu_yaw_deg;
  msg_imu_ok.data     = imu_available;

  RCSOFTCHECK(rcl_publish(&pub_ticks,      &msg_ticks,     NULL));
  RCSOFTCHECK(rcl_publish(&pub_steer_deg,  &msg_steer_deg, NULL));
  RCSOFTCHECK(rcl_publish(&pub_imu_yaw,    &msg_imu_yaw,   NULL));
  RCSOFTCHECK(rcl_publish(&pub_imu_ok,     &msg_imu_ok,    NULL));

  // ----- FSM: STEER -> DRIVE -----
  float e_signed = ang_err_deg(target_steer_deg, steer_deg); // [-180,180]
  float e_cw     = cw_error_deg(target_steer_deg, steer_deg); // 0..360

  switch (mode){
    case STEER_TO_HEADING: {
      if (!cw_in_tolerance(e_cw, Wheel_STEER_ERROR_TOLERANCE)) {
        // magnitude ตาม e_cw (บวกเสมอ), ทิศ = ตามเข็มเท่านั้น
        float pwm_mag = steer.compute_with_error(/*ใช้ error แบบบวก*/ e_cw) + Wheel_STEER_BASE_SPEED;
        
        // กันไม่ให้เล็กจนไม่ขยับ และไม่ใหญ่เกินไป
        const float PWM_MIN = Wheel_STEER_BASE_SPEED; // ปรับตามแรงเสียดทาน/สังเคราะห์จริง
        if (pwm_mag < PWM_MIN) pwm_mag = PWM_MIN;
        if (pwm_mag > PWM_STEER_Max) pwm_mag = PWM_STEER_Max;
        
        int pwm_cmd = STEER_MOTOR_DIR * (int)lroundf(pwm_mag); // STEER_MOTOR_DIR = +1 (ทางขวา)
        motor.spin(pwm_cmd);
        if (fabsf(rpm_raw) < 0.001f && e_cw > 100){
          pwm_cmd = PWM_Max * 0.8f * STEER_MOTOR_DIR;
          motor.spin(pwm_cmd);
        }
        last_pwm_cmd = pwm_cmd;
        steer_enter_ok_ms = millis();
      } else {
        motor.spin(0);
        last_pwm_cmd = 0;
        (void)steer.compute_with_error(0.0f); // ล้าง I/D
        if (millis() - steer_enter_ok_ms >= STEER_SETTLE_MS) {
          spin.reset();
          mode = DRIVE;
        }
      }
    } break;

    case DRIVE: {
      // ถ้าล้อเพี้ยนเกิน tol → กลับไปเลี้ยว
      if (fabsf(e_signed) > Wheel_STEER_ERROR_TOLERANCE) {
        motor.spin(0);
        spin.reset();
        steer.reset();
        last_pwm_cmd = 0;
        mode = STEER_TO_HEADING;
        break;
      }

      float rpm_meas = fabsf(rpm_raw);
      float rpm_set  = fabsf(target_rpm);

      if (rpm_set <= 1e-3f) {
        motor.spin(0);
        last_pwm_cmd = 0;
        spin.reset();
      } else {
        // แปลงรอบเป้าหมายเป็น PWM แบบ feed-forward แล้วให้ PIDF คอยเติมแก้ไขเล็กน้อย
        // float pwm_ff   = rpm_set * RPM_TO_PWM_K;
        float pwm_corr = spin.compute(rpm_set, rpm_meas);  // PIDF คุมส่วนต่างรอบจริง vs คำสั่ง
        // float pwm_mag  = pwm_ff + pwm_corr;
        float pwm_mag  = pwm_corr;
        if (pwm_mag < 0.0f) pwm_mag = 0.0f;
        if (pwm_mag > (float)PWM_SPIN_Max) pwm_mag = (float)PWM_SPIN_Max;

        int pwm_cmd = DRIVE_MOTOR_DIR * (int) lroundf(pwm_mag);
        motor.spin(pwm_cmd);
        last_pwm_cmd = pwm_cmd;
      }
    } break;
  }

  // ---- DEBUG (Twist + String) ----
  dbg_cmd_msg.linear.x  = target_rpm;          // คำสั่งจาก Python (rpm)
  dbg_cmd_msg.linear.y  = rpm_raw;             // รอบจริง (signed)
  dbg_cmd_msg.angular.x = last_pwm_cmd;        // pwm ล่าสุด
  dbg_cmd_msg.angular.z = target_steer_deg;    // มุมเป้าหมาย
  RCSOFTCHECK(rcl_publish(&pub_debug_cmd, &dbg_cmd_msg, NULL));

  char buf[128];
  snprintf(buf, sizeof(buf),
           "mode=%s, e=%.1f/%.1f, steer=%.1f->%.1f, imu_ok=%d, pwm=%d",
           (mode==STEER_TO_HEADING?"STEER":"DRIVE"),
           e_signed, e_cw, steer_deg, target_steer_deg, imu_available ? 1:0, last_pwm_cmd);
  msg_state.data.data = (char*)buf;
  msg_state.data.size = strlen(buf);
  msg_state.data.capacity = msg_state.data.size + 1;
  RCSOFTCHECK(rcl_publish(&pub_state, &msg_state, NULL));
}

//================== Entities create/destroy =================//
bool createEntities(){
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 10);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  RCCHECK(rclc_node_init_default(&node, "mor_luam_firmware", "", &support));

  // pubs
  RCCHECK(rclc_publisher_init_default(&pub_debug_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/mor_luam/debug/wheel/cmd_vel"));
  RCCHECK(rclc_publisher_init_default(&pub_ticks, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/mor_luam/feedback/drive_ticks"));
  RCCHECK(rclc_publisher_init_default(&pub_steer_deg, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/mor_luam/feedback/steer_deg"));
  RCCHECK(rclc_publisher_init_default(&pub_imu_yaw, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/mor_luam/feedback/imu_yaw_deg"));
  RCCHECK(rclc_publisher_init_default(&pub_imu_ok, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/mor_luam/feedback/imu_ok"));
  RCCHECK(rclc_publisher_init_default(&pub_state, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/mor_luam/debug/esp_state"));

  // sub (รับ goal ที่ถูกคำนวณแล้วจาก Python: rpm signed + steer deg)
  RCCHECK(rclc_subscription_init_default(&sub_cmd_move, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/mor_luam/cmd_vel/move"));

  // timer 100Hz
  RCCHECK(rclc_timer_init_default(&ctrl_timer, &support, RCL_MS_TO_NS(CTRL_PERIOD_MS), controlCallback));

  // executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &ctrl_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_move, &cmd_move_msg, &cmd_move_cb, ON_NEW_DATA));

  return true;
}

bool destroyEntities(){
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_subscription_fini(&sub_cmd_move, &node);
  rcl_publisher_fini(&pub_debug_cmd, &node);
  rcl_publisher_fini(&pub_ticks, &node);
  rcl_publisher_fini(&pub_steer_deg, &node);
  rcl_publisher_fini(&pub_imu_yaw, &node);
  rcl_publisher_fini(&pub_imu_ok, &node);
  rcl_publisher_fini(&pub_state, &node);

  rcl_node_fini(&node);
  rcl_timer_fini(&ctrl_timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
  return true;
}

//==================== Fatal handler ======================//
void rclErrorLoop(){ ESP.restart(); }
