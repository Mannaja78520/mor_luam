#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <nav_msgs/msg/odometry.h>

#include <config.h>
#include <motor.h>
#include <PIDF.h>
#include <esp32_Encoder.h>        // ENCODER
#include <Wire.h>
#include <Adafruit_AS5600.h>      // AS5600
#include <Adafruit_BNO08x.h>

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

static inline float wrap_pi(float a){
  float x = fmodf(a + (float)M_PI, 2.0f * (float)M_PI);
  return x < 0.0f ? x + 2.0f * (float)M_PI : x - (float)M_PI;
}
static inline float deg2rad(float deg){ return deg * ((float)M_PI / 180.0f); }

//===================== Parameters (tune) ===================//
static const int STEER_MOTOR_DIR = +1; // ทิศหมุนสำหรับโหมดเลี้ยว
static const int DRIVE_MOTOR_DIR = -1; // ทิศหมุนสำหรับโหมดวิ่งหน้า

static const uint32_t STEER_SETTLE_MS = 50; // ต้องอยู่ในกรอบต่อเนื่องก่อนปล่อยวิ่ง
static const uint32_t CTRL_PERIOD_MS  = 10;  // 100 Hz
#ifndef ODOM_TICKS_SIGN
#define ODOM_TICKS_SIGN (+1.0f)
#endif
static const float    CTRL_PERIOD_S   = CTRL_PERIOD_MS / 1000.0f;
static const float    TICKS_TO_WHEEL_REV = 1.0f / ((float)COUNTS_PER_REV * MOTOR_ENCODER_RATIO);
static const float    TICKS_TO_METERS    = ((float)M_PI * WHEEL_DIAMETER * TICKS_TO_WHEEL_REV * (float)ODOM_TICKS_SIGN);
static const int64_t  ODOM_TICK_RESET_THRESHOLD = (int64_t)((float)COUNTS_PER_REV * MOTOR_ENCODER_RATIO * 20.0f);
static const float    CMD_SMALL_HEADING_EPS = 2.5f;   // deg change to ignore
static const float    CMD_SMALL_DIST_EPS    = 0.02f;  // metres difference to ignore
static const float    CMD_SMALL_RPM_EPS     = 1.0f;   // rpm diff to ignore
static const float    DRIVE_RELOCK_ERROR_DEG = Wheel_STEER_ERROR_TOLERANCE * 2.0f; // threshold to re-steer mid-drive
static const float    DRIVE_IMU_RELOCK_DEG   = 35.0f; // deg difference between IMU and target to re-steer
static const float    STEER_CMD_ZERO_DEG     = 0.0f;  // offset applied to commanded steering angle

// static const float RPM_TO_PWM_K = ((float)PWM_SPIN_Max * MAX_RPM_RATIO) / (float)MOTOR_MAX_RPM;

//==================== micro-ROS entities ===================//
rclc_support_t support;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t ctrl_timer;
rclc_executor_t executor;

rcl_subscription_t sub_cmd_move;
rcl_subscription_t sub_spin_pid;
rcl_subscription_t sub_steer_pid;
geometry_msgs__msg__Twist cmd_move_msg;
std_msgs__msg__Float32MultiArray spin_pid_msg;
std_msgs__msg__Float32MultiArray steer_pid_msg;

rcl_publisher_t pub_debug_cmd;   // Twist (debug)
rcl_publisher_t pub_ticks;       // Int32
rcl_publisher_t pub_steer_deg;   // Float32
rcl_publisher_t pub_imu_yaw;     // Float32
rcl_publisher_t pub_imu_ok;      // Bool
rcl_publisher_t pub_state;       // String
rcl_publisher_t pub_odom;        // nav_msgs/Odometry

std_msgs__msg__Int32   msg_ticks;
std_msgs__msg__Float32 msg_steer_deg, msg_imu_yaw;
std_msgs__msg__Bool    msg_imu_ok;
std_msgs__msg__String  msg_state;
geometry_msgs__msg__Twist dbg_cmd_msg;
nav_msgs__msg__Odometry msg_odom;

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

#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}


//======================= App state =========================//
volatile float target_rpm = 0.0f;       // RPM (signed) จาก Python
volatile float target_steer_deg = 0.0f; // target heading (global) 0..360 จาก Python
volatile long  ticks_acc = 0;
volatile float steer_deg = 0.0f;
static float steer_target_mech_deg = 0.0f; // มุมล้อที่ต้องการ (สัมพัทธ์ตัวรถ)
static float body_heading_deg = 0.0f;      // หัวหุ่น (deg) ค้างไว้จาก IMU หรือ fallback
static float drive_heading_ref_deg = 0.0f; // หัวหุ่นขณะเข้าโหมด DRIVE
static bool  drive_heading_ref_valid = false;

bool imu_available = false;             // true เมื่ออ้างอิง IMU พร้อมใช้งาน
float imu_yaw_deg  = 0.0f;
bool imu_calibrated = false;
float imu_base_yaw_deg = 0.0f;
bool imu_have_last = false;
float imu_last_raw_yaw_deg = 0.0f;
bool imu_reference_ready = false;
bool imu_reference_pending = false;

unsigned long steer_enter_ok_ms = 0;
int last_pwm_cmd = 0;                   // ไว้ debug

static float spin_pid_buf[9];
static float drive_target_dist_m = 0.0f;
static float drive_target_tol_m  = 0.06f;
static float drive_progress_m    = 0.0f;
static bool  drive_has_target    = false;
static float drive_last_delta_m  = 0.0f;
static bool  drive_goal_active   = false;
static float drive_start_x_m     = 0.0f;
static float drive_start_y_m     = 0.0f;
static float drive_goal_x_m      = 0.0f;
static float drive_goal_y_m      = 0.0f;
static float drive_goal_vec_x    = 0.0f;
static float drive_goal_vec_y    = 0.0f;
static float drive_goal_len_sq   = 0.0f;
static float drive_goal_tol_m    = 0.06f;

static float steer_pid_buf[9];

static float spin_error_tolerance  = Wheel_SPIN_ERROR_TOLERANCE;
static float steer_error_tolerance = Wheel_STEER_ERROR_TOLERANCE;

static bool  odom_initialized = false;
static float odom_x_m = 0.0f;
static float odom_y_m = 0.0f;
static float odom_theta_rad = 0.0f;
static float odom_prev_theta_rad = 0.0f;
static int64_t odom_last_ticks = 0;

//===================== Forward decls =======================//
void rclErrorLoop();
bool createEntities();
bool destroyEntities();
void controlCallback(rcl_timer_t *timer, int64_t);
static void spin_pid_cb(const void* msgin);
static void steer_pid_cb(const void* msgin);

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
  imu_calibrated = false;
  imu_have_last = false;
  imu_last_raw_yaw_deg = 0.0f;
  imu_reference_ready = false;
  imu_reference_pending = false;
  body_heading_deg = 0.0f;

  // Try to initialize!
  if (bno08x.begin_I2C(0x4A)) {
    setReports(reportType, reportIntervalUs);
    imu_available = true;
  }

  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);

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
  spin.setPIDF(Wheel_SPIN_KP,  Wheel_SPIN_KI,  Wheel_SPIN_KD,  Wheel_SPIN_KF,  spin_error_tolerance);
  spin.setIClamp(Wheel_SPIN_I_Min, Wheel_SPIN_I_Max);
  steer.setPIDF(Wheel_STEER_KP, Wheel_STEER_KI, Wheel_STEER_KD, Wheel_STEER_KF, steer_error_tolerance);
  steer.setIClamp(Wheel_STEER_I_Min, Wheel_STEER_I_Max);
  mode = STEER_TO_HEADING;
  conn_state = WAITING_AGENT;

  memset(&spin_pid_msg, 0, sizeof(spin_pid_msg));
  spin_pid_msg.data.data = spin_pid_buf;
  spin_pid_msg.data.size = 0;
  spin_pid_msg.data.capacity = sizeof(spin_pid_buf) / sizeof(float);

  memset(&steer_pid_msg, 0, sizeof(steer_pid_msg));
  steer_pid_msg.data.data = steer_pid_buf;
  steer_pid_msg.data.size = 0;
  steer_pid_msg.data.capacity = sizeof(steer_pid_buf) / sizeof(float);

  memset(&msg_odom, 0, sizeof(msg_odom));
  static char odom_frame_id[] = "odom";
  static char base_frame_id[] = "base_link";
  msg_odom.header.frame_id.data = odom_frame_id;
  msg_odom.header.frame_id.size = strlen(odom_frame_id);
  msg_odom.header.frame_id.capacity = sizeof(odom_frame_id);
  msg_odom.child_frame_id.data = base_frame_id;
  msg_odom.child_frame_id.size = strlen(base_frame_id);
  msg_odom.child_frame_id.capacity = sizeof(base_frame_id);

  odom_initialized = false;
  odom_x_m = 0.0f;
  odom_y_m = 0.0f;
  odom_theta_rad = 0.0f;
  odom_prev_theta_rad = 0.0f;
  odom_last_ticks = 0;
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
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    yaw_out = ypr.yaw;
    return true;
  }
  // ถ้ามี IMU จริง เช่น Utilize::getYawDeg() ให้เช็คสถานะและคืน true
  yaw_out = 0.0f;
  return false;
}

//=================== ROS Callbacks/Control =================//
static void cmd_move_cb(const void* msgin){
  const geometry_msgs__msg__Twist* m = (const geometry_msgs__msg__Twist*) msgin;

  float prev_rpm = target_rpm;
  float prev_heading_deg = target_steer_deg;
  float prev_dist_m = drive_target_dist_m;
  bool prev_goal_active = drive_goal_active;

  float rpm_cmd = m->linear.x;
  if (rpm_cmd >  MOTOR_MAX_RPM) rpm_cmd =  MOTOR_MAX_RPM;
  if (rpm_cmd < -MOTOR_MAX_RPM) rpm_cmd = -MOTOR_MAX_RPM;
  float steer_heading_cmd = wrap360(m->angular.z);         // 0..360

  float dist_cmd = m->linear.y;
  float tol_cmd  = fabsf(m->linear.z);

  float rpm_delta = fabsf(rpm_cmd - prev_rpm);
  float steer_heading_delta = fabsf(ang_err_deg(steer_heading_cmd, prev_heading_deg));
  float dist_delta = fabsf(dist_cmd - prev_dist_m);
  float tol_value = tol_cmd > 0.0f ? tol_cmd : drive_target_tol_m;

  bool small_adjust = prev_goal_active &&
                      (fabsf(prev_rpm) > 1e-3f || fabsf(rpm_cmd) > 1e-3f) &&
                      (rpm_delta < CMD_SMALL_RPM_EPS) &&
                      (steer_heading_delta < CMD_SMALL_HEADING_EPS) &&
                      (dist_delta < CMD_SMALL_DIST_EPS);

  if (small_adjust) {
    drive_target_tol_m = tol_value;
    drive_goal_tol_m   = drive_target_tol_m;
    return; // ignore tiny corrections to avoid resetting controllers
  }

  target_rpm       = rpm_cmd;                       // คำสั่งรอบ (signed)
  target_steer_deg = steer_heading_cmd;

  drive_target_dist_m = dist_cmd;
  drive_target_tol_m  = tol_value;
  drive_goal_tol_m    = drive_target_tol_m;
  drive_has_target    = fabsf(dist_cmd) > 1e-4f && fabsf(rpm_cmd) > 1e-4f;
  if (drive_has_target) {
    drive_start_x_m = odom_x_m;
    drive_start_y_m = odom_y_m;
    float steer_heading_rad = deg2rad(target_steer_deg);
    drive_goal_x_m = odom_x_m + cosf(steer_heading_rad) * dist_cmd;
    drive_goal_y_m = odom_y_m + sinf(steer_heading_rad) * dist_cmd;
    drive_goal_vec_x = drive_goal_x_m - drive_start_x_m;
    drive_goal_vec_y = drive_goal_y_m - drive_start_y_m;
    drive_goal_len_sq = drive_goal_vec_x * drive_goal_vec_x + drive_goal_vec_y * drive_goal_vec_y;
    drive_goal_active = true;
  } else {
    drive_goal_active = false;
    drive_goal_len_sq = 0.0f;
  }

  // reset state machine so ESP handles steering locally on every new command
  steer.reset();
  spin.reset();
  mode = STEER_TO_HEADING;
  steer_enter_ok_ms = millis();
  last_pwm_cmd = 0;
  drive_heading_ref_valid = false;

  if (!imu_reference_ready) {
    imu_reference_pending = true;
  }
}

void controlCallback(rcl_timer_t *timer, int64_t){
  if(!timer) return;

  // ----- Feedback -----
  ticks_acc = encoder.read();
  float rpm_raw = encoder.getRPM();
  steer_deg = wrap360(readSteerDeg());

  float yaw_tmp;
  bool imu_raw_ok = readImuYawDeg(yaw_tmp);
  if (imu_raw_ok) {
    imu_last_raw_yaw_deg = wrap360(yaw_tmp);
    imu_have_last = true;
  }

  if (imu_reference_pending && imu_have_last) {
    imu_base_yaw_deg = wrap360(imu_last_raw_yaw_deg);
    imu_reference_ready = true;
    imu_reference_pending = false;
    imu_calibrated = true;
  }

  if (imu_reference_ready && imu_have_last) {
    float yaw_raw = imu_raw_ok ? wrap360(yaw_tmp) : imu_last_raw_yaw_deg;
    imu_yaw_deg = wrap360(yaw_raw - imu_base_yaw_deg);
    body_heading_deg = imu_yaw_deg;
  } else if (!imu_reference_ready && !imu_have_last) {
    body_heading_deg = 0.0f;
  }

  imu_available = imu_reference_ready && imu_have_last;

  msg_ticks.data      = (int32_t)ticks_acc;
  msg_steer_deg.data  = steer_deg;
  msg_imu_yaw.data    = imu_yaw_deg;
  msg_imu_ok.data     = imu_available;

  RCSOFTCHECK(rcl_publish(&pub_ticks,      &msg_ticks,     NULL));
  RCSOFTCHECK(rcl_publish(&pub_steer_deg,  &msg_steer_deg, NULL));
  RCSOFTCHECK(rcl_publish(&pub_imu_yaw,    &msg_imu_yaw,   NULL));
  RCSOFTCHECK(rcl_publish(&pub_imu_ok,     &msg_imu_ok,    NULL));

  // ----- Odometry integration -----
  if (mode == STEER_TO_HEADING) {
    (void)encoder.getRPM();
    float heading = wrap_pi(deg2rad(body_heading_deg));
    odom_theta_rad = heading;
    odom_prev_theta_rad = heading;
    odom_last_ticks = ticks_acc;
    odom_initialized = false;
    drive_last_delta_m = 0.0f;

    uint32_t ms_now = millis();
    msg_odom.header.stamp.sec = (int32_t)(ms_now / 1000UL);
    msg_odom.header.stamp.nanosec = (uint32_t)((ms_now % 1000UL) * 1000000UL);
    msg_odom.pose.pose.position.x = (double)odom_x_m;
    msg_odom.pose.pose.position.y = (double)odom_y_m;
    msg_odom.pose.pose.position.z = 0.0;
    double half_yaw = (double)(heading * 0.5f);
    msg_odom.pose.pose.orientation.x = 0.0;
    msg_odom.pose.pose.orientation.y = 0.0;
    msg_odom.pose.pose.orientation.z = sin(half_yaw);
    msg_odom.pose.pose.orientation.w = cos(half_yaw);
    msg_odom.twist.twist.linear.x  = 0.0;
    msg_odom.twist.twist.linear.y  = 0.0;
    msg_odom.twist.twist.linear.z  = 0.0;
    msg_odom.twist.twist.angular.x = 0.0;
    msg_odom.twist.twist.angular.y = 0.0;
    msg_odom.twist.twist.angular.z = 0.0;
    RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));
  } else if (!odom_initialized) {
    odom_initialized = true;
    odom_last_ticks = ticks_acc;
    odom_theta_rad = wrap_pi(deg2rad(body_heading_deg));
    odom_prev_theta_rad = odom_theta_rad;
    drive_last_delta_m = 0.0f;

    uint32_t ms_now = millis();
    msg_odom.header.stamp.sec = (int32_t)(ms_now / 1000UL);
    msg_odom.header.stamp.nanosec = (uint32_t)((ms_now % 1000UL) * 1000000UL);
    msg_odom.pose.pose.position.x = (double)odom_x_m;
    msg_odom.pose.pose.position.y = (double)odom_y_m;
    msg_odom.pose.pose.position.z = 0.0;
    double half_yaw = (double)(odom_theta_rad * 0.5f);
    msg_odom.pose.pose.orientation.x = 0.0;
    msg_odom.pose.pose.orientation.y = 0.0;
    msg_odom.pose.pose.orientation.z = sin(half_yaw);
    msg_odom.pose.pose.orientation.w = cos(half_yaw);
    msg_odom.twist.twist.linear.x  = 0.0;
    msg_odom.twist.twist.angular.z = 0.0;
    msg_odom.twist.twist.linear.y  = 0.0;
    msg_odom.twist.twist.linear.z  = 0.0;
    msg_odom.twist.twist.angular.x = 0.0;
    msg_odom.twist.twist.angular.y = 0.0;
    RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));
  } else {
    int64_t delta_ticks = ticks_acc - odom_last_ticks;
    float new_heading = wrap_pi(deg2rad(body_heading_deg));
    float steer_body_rad = wrap_pi(deg2rad(steer_deg));
    if (llabs(delta_ticks) > ODOM_TICK_RESET_THRESHOLD) {
      // encoder jumped (likely reset); resync without applying motion update
      odom_last_ticks = ticks_acc;
      odom_theta_rad = new_heading;
      odom_prev_theta_rad = new_heading;
      drive_last_delta_m = 0.0f;

      uint32_t ms_now = millis();
      msg_odom.header.stamp.sec = (int32_t)(ms_now / 1000UL);
      msg_odom.header.stamp.nanosec = (uint32_t)((ms_now % 1000UL) * 1000000UL);
      msg_odom.pose.pose.position.x = (double)odom_x_m;
      msg_odom.pose.pose.position.y = (double)odom_y_m;
      msg_odom.pose.pose.position.z = 0.0;
      double half_yaw = (double)(odom_theta_rad * 0.5f);
      msg_odom.pose.pose.orientation.x = 0.0;
      msg_odom.pose.pose.orientation.y = 0.0;
      msg_odom.pose.pose.orientation.z = sin(half_yaw);
      msg_odom.pose.pose.orientation.w = cos(half_yaw);
      msg_odom.twist.twist.linear.x  = 0.0;
      msg_odom.twist.twist.angular.z = 0.0;
      msg_odom.twist.twist.linear.y  = 0.0;
      msg_odom.twist.twist.linear.z  = 0.0;
      msg_odom.twist.twist.angular.x = 0.0;
      msg_odom.twist.twist.angular.y = 0.0;
      RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));
    } else {
      odom_last_ticks = ticks_acc;
      float delta_m = (float)delta_ticks * TICKS_TO_METERS;
      drive_last_delta_m = delta_m;
      odom_theta_rad = new_heading;
      float delta_body_x = delta_m * cosf(steer_body_rad);  // displacement in robot frame
      float delta_body_y = delta_m * sinf(steer_body_rad);
      float cos_heading = cosf(new_heading);
      float sin_heading = sinf(new_heading);
      odom_x_m += delta_body_x * cos_heading - delta_body_y * sin_heading;  // rotate into odom frame
      odom_y_m += delta_body_x * sin_heading + delta_body_y * cos_heading;

      float delta_theta = wrap_pi(odom_theta_rad - odom_prev_theta_rad);
      odom_prev_theta_rad = odom_theta_rad;

      double vx = (double)(delta_m / CTRL_PERIOD_S);
      double wz = (double)(delta_theta / CTRL_PERIOD_S);

      uint32_t ms_now = millis();
      msg_odom.header.stamp.sec = (int32_t)(ms_now / 1000UL);
      msg_odom.header.stamp.nanosec = (uint32_t)((ms_now % 1000UL) * 1000000UL);
      msg_odom.pose.pose.position.x = (double)odom_x_m;
      msg_odom.pose.pose.position.y = (double)odom_y_m;
      msg_odom.pose.pose.position.z = 0.0;

      double half_yaw = (double)(odom_theta_rad * 0.5f);
      msg_odom.pose.pose.orientation.x = 0.0;
      msg_odom.pose.pose.orientation.y = 0.0;
      msg_odom.pose.pose.orientation.z = sin(half_yaw);
      msg_odom.pose.pose.orientation.w = cos(half_yaw);

      msg_odom.twist.twist.linear.x  = vx;
      msg_odom.twist.twist.linear.y  = 0.0;
      msg_odom.twist.twist.linear.z  = 0.0;
      msg_odom.twist.twist.angular.x = 0.0;
      msg_odom.twist.twist.angular.y = 0.0;
      msg_odom.twist.twist.angular.z = wz;

      RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));
    }
  }

  // ----- FSM: STEER -> DRIVE -----
  float heading_meas_deg = imu_available ? wrap360(imu_yaw_deg) : body_heading_deg;
  // float heading_error_deg = ang_err_deg(target_steer_deg, heading_meas_deg);
  steer_target_mech_deg = wrap360((target_steer_deg - heading_meas_deg) + STEER_CMD_ZERO_DEG);
  float e_signed_steer   = ang_err_deg(steer_target_mech_deg, steer_deg);
  float e_cw_steer       = cw_error_deg(steer_target_mech_deg, steer_deg);        // 0..360

  switch (mode){
    case STEER_TO_HEADING: {
      if (!cw_in_tolerance(e_cw_steer, steer_error_tolerance)) {
        // magnitude ตาม e_cw (บวกเสมอ), ทิศ = ตามเข็มเท่านั้น
        float pwm_mag = steer.compute_with_error(/*ใช้ error แบบบวก*/ e_cw_steer) + Wheel_STEER_BASE_SPEED;
        
        // กันไม่ให้เล็กจนไม่ขยับ และไม่ใหญ่เกินไป
        // const float PWM_MIN = Wheel_STEER_BASE_SPEED; // ปรับตามแรงเสียดทาน/สังเคราะห์จริง
        // if (pwm_mag < PWM_MIN) pwm_mag = PWM_MIN;
        if (pwm_mag > PWM_STEER_Max) pwm_mag = PWM_STEER_Max;
        
        int pwm_cmd = STEER_MOTOR_DIR * (int)lroundf(pwm_mag); // STEER_MOTOR_DIR = +1 (ทางขวา)
        motor.spin(pwm_cmd);
        // if (fabsf(rpm_raw) < 0.001f && e_cw > 50){
        //   pwm_cmd = PWM_Max * 0.785f * STEER_MOTOR_DIR;
        //   motor.spin(pwm_cmd);
        // }
        last_pwm_cmd = pwm_cmd;
        steer_enter_ok_ms = millis();
      } else {
        motor.spin(0);
        last_pwm_cmd = 0;
        (void)steer.compute_with_error(0.0f); // ล้าง I/D
        if (millis() - steer_enter_ok_ms >= STEER_SETTLE_MS) {
          spin.reset();
          if (fabsf(target_rpm) <= 1e-3f) {
            // target speed is zero; stay in steer mode but consider centered
            steer_enter_ok_ms = millis();
          } else {
            mode = DRIVE;
            drive_progress_m = 0.0f;
            drive_last_delta_m = 0.0f;
          }
        }
      }
    } break;

    case DRIVE: {
      // ถ้ามุมเพี้ยนมากเกิน threshold → กลับไปเลี้ยว แม้จะกำลังวิ่ง
      bool steer_out_of_tol = fabsf(e_signed_steer) > DRIVE_RELOCK_ERROR_DEG;
      // bool imu_out_of_tol   = imu_available &&
      //                         fabsf(heading_error_deg) > DRIVE_IMU_RELOCK_DEG &&
      //                         fabsf(drive_progress_m) > 0.02f;
      // if (steer_out_of_tol || imu_out_of_tol) {
      if (steer_out_of_tol) {
        motor.spin(0);
        spin.reset();
        steer.reset();
        last_pwm_cmd = 0;
        mode = STEER_TO_HEADING;
        steer_enter_ok_ms = millis();
        break;
      }

      float rpm_meas = fabsf(rpm_raw);
      float rpm_set  = fabsf(target_rpm);

      if (drive_has_target) {
        drive_progress_m += drive_last_delta_m;
        float target_m = drive_target_dist_m;
        float tol_m = fabsf(drive_target_tol_m);
        float remaining = target_m - drive_progress_m;

        bool reached = false;
        if (target_m >= 0.0f) {
          if (remaining <= tol_m) {
            reached = true;
          }
        } else {
          if (remaining >= -tol_m) {
            reached = true;
          }
        }

        if (reached) {
          target_rpm = 0.0f;
          rpm_set = 0.0f;
          drive_has_target = false;
          drive_goal_active = false;
          mode = STEER_TO_HEADING;
          steer_enter_ok_ms = millis();
        }
      }

      if (drive_goal_active) {
        float dx_goal = drive_goal_x_m - odom_x_m;
        float dy_goal = drive_goal_y_m - odom_y_m;
        float dist_goal = sqrtf(dx_goal * dx_goal + dy_goal * dy_goal);

        float dx_progress = odom_x_m - drive_start_x_m;
        float dy_progress = odom_y_m - drive_start_y_m;
        float along_dot = dx_progress * drive_goal_vec_x + dy_progress * drive_goal_vec_y;

        bool reached_goal = dist_goal <= drive_goal_tol_m;
        bool overshoot_goal = (drive_goal_len_sq > 1e-6f) && (along_dot >= drive_goal_len_sq);

        if (reached_goal || overshoot_goal) {
          target_rpm = 0.0f;
          rpm_set = 0.0f;
          drive_has_target = false;
          drive_goal_active = false;
          drive_goal_len_sq = 0.0f;
          mode = STEER_TO_HEADING;
          steer_enter_ok_ms = millis();
        }
      }

      if (rpm_set <= 1e-3f) {
        motor.spin(0);
        last_pwm_cmd = 0;
        spin.reset();
        drive_has_target = false;
        drive_goal_active = false;
        drive_goal_len_sq = 0.0f;
        mode = STEER_TO_HEADING;
        steer_enter_ok_ms = millis();
      } else {
        // แปลงรอบเป้าหมายเป็น PWM แบบ feed-forward แล้วให้ PIDF คอยเติมแก้ไขเล็กน้อย
        // float pwm_ff   = rpm_set * RPM_TO_PWM_K;
        float pwm_corr = spin.compute(rpm_set, rpm_meas);  // PIDF คุมส่วนต่างรอบจ<ริง vs คำสั่ง
        // float pwm_mag  = pwm_ff + pwm_corr;
        float pwm_mag  = pwm_corr;
        if (pwm_mag < 0.0f) pwm_mag = 0.0f;
        if (pwm_mag > (float)PWM_SPIN_Max) pwm_mag = (float)PWM_SPIN_Max;

        const int drive_dir = (target_rpm >= 0.0f) ? DRIVE_MOTOR_DIR : -DRIVE_MOTOR_DIR;
        int pwm_cmd = drive_dir * (int) lroundf(pwm_mag);
        motor.spin(pwm_cmd);
        last_pwm_cmd = pwm_cmd;
      }
    } break;
  }

  // ---- DEBUG (Twist + String) ----
  dbg_cmd_msg.linear.x  = target_rpm;          // คำสั่งจาก Python (rpm)
  dbg_cmd_msg.linear.y  = rpm_raw;             // รอบจริง (signed)
  dbg_cmd_msg.linear.z  = heading_meas_deg;    // มุมหัว (IMU หรือ fallback)
  dbg_cmd_msg.angular.x = last_pwm_cmd;        // pwm ล่าสุด
  dbg_cmd_msg.angular.y = steer_deg;           // มุมล้อจริง
  dbg_cmd_msg.angular.z = steer_target_mech_deg;    // มุมล้อเป้าหมาย (สัมพัทธ์ตัวรถ)
  RCSOFTCHECK(rcl_publish(&pub_debug_cmd, &dbg_cmd_msg, NULL));

  char buf[128];
  snprintf(buf, sizeof(buf),
           "mode=%s, tgt=%.1f, heading=%.1f, e_heading=%.1f, e_steer=%.1f, steer=%.1f->%.1f, imu_ok=%d, pwm=%d",
           (mode==STEER_TO_HEADING?"STEER":"DRIVE"),
           target_steer_deg,
           heading_meas_deg,
          //  heading_error_deg,
           e_signed_steer,
           steer_deg, steer_target_mech_deg,
           imu_available ? 1:0, last_pwm_cmd);
  msg_state.data.data = (char*)buf;
  msg_state.data.size = strlen(buf);
  msg_state.data.capacity = msg_state.data.size + 1;
  RCSOFTCHECK(rcl_publish(&pub_state, &msg_state, NULL));
}

static void apply_pid_from_msg(PIDF &pid, float &err_tol, const std_msgs__msg__Float32MultiArray *msg){
  if (!msg) return;
  size_t n = msg->data.size;
  if (n < 4) return; // need at least Kp, Ki, Kd, Kf

  const float kp = msg->data.data[0];
  const float ki = msg->data.data[1];
  const float kd = msg->data.data[2];
  const float kf = msg->data.data[3];
  if (n >= 5) {
    err_tol = msg->data.data[4];
  }
  pid.setPIDF(kp, ki, kd, kf, err_tol);

  if (n >= 7) {
    pid.setIClamp(msg->data.data[5], msg->data.data[6]);
  }
  if (n >= 9) {
    pid.setOutputLimits(msg->data.data[7], msg->data.data[8]);
  }
  pid.reset();
}

static void spin_pid_cb(const void* msgin){
  const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray*) msgin;
  apply_pid_from_msg(spin, spin_error_tolerance, msg);
}

static void steer_pid_cb(const void* msgin){
  const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray*) msgin;
  apply_pid_from_msg(steer, steer_error_tolerance, msg);
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
  RCCHECK(rclc_publisher_init_default(&pub_odom, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/mor_luam/odom/esp"));

  // sub (รับ goal ที่ถูกคำนวณแล้วจาก Python: rpm signed + steer deg)
  RCCHECK(rclc_subscription_init_default(&sub_cmd_move, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/mor_luam/cmd_vel/move"));
  RCCHECK(rclc_subscription_init_default(&sub_spin_pid, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/mor_luam/config/spin_pid"));
  RCCHECK(rclc_subscription_init_default(&sub_steer_pid, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/mor_luam/config/steer_pid"));

  // timer 100Hz
  RCCHECK(rclc_timer_init_default(&ctrl_timer, &support, RCL_MS_TO_NS(CTRL_PERIOD_MS), controlCallback));

  // executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 8, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &ctrl_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_move, &cmd_move_msg, &cmd_move_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_spin_pid, &spin_pid_msg, &spin_pid_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_steer_pid, &steer_pid_msg, &steer_pid_cb, ON_NEW_DATA));

  return true;
}

bool destroyEntities(){
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_subscription_fini(&sub_cmd_move, &node);
  rcl_subscription_fini(&sub_spin_pid, &node);
  rcl_subscription_fini(&sub_steer_pid, &node);
  rcl_publisher_fini(&pub_debug_cmd, &node);
  rcl_publisher_fini(&pub_ticks, &node);
  rcl_publisher_fini(&pub_steer_deg, &node);
  rcl_publisher_fini(&pub_imu_yaw, &node);
  rcl_publisher_fini(&pub_imu_ok, &node);
  rcl_publisher_fini(&pub_state, &node);
  rcl_publisher_fini(&pub_odom, &node);

  rcl_node_fini(&node);
  rcl_timer_fini(&ctrl_timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
  return true;
}

//==================== Fatal handler ======================//
void rclErrorLoop(){ ESP.restart(); }
