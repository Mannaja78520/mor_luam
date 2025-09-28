#ifndef PIDF_H
#define PIDF_H

#include <Arduino.h>

class PIDF {
public:
  // ctor: min,max, Kp,Ki, i_min,i_max, Kd,Kf, tol
  PIDF(float min_val, float max_val,
       float Kp = 0.0f, float Ki = 0.0f,
       float i_min = 0.0f, float i_max = 0.0f,
       float Kd = 0.0f, float Kf = 0.0f,
       float error_tolerance = 0.0f);

  void  setPIDF(float Kp, float Ki, float Kd, float Kf, float error_tolerance);
  void  setOutputLimits(float min_val, float max_val);
  void  setIClamp(float i_min, float i_max);
  void  reset();

  // ตั้งความถี่ตัดของ D-term filter (Hz). 0 = ปิดฟิลเตอร์ (ใช้ D ดิบ)
  void  setDFilterCutoffHz(float fc_hz);

  // ใช้ dt ภายในจาก micros()
  float compute(float setpoint, float measure);
  float compute_with_error(float error);

private:
  // gains
  float Kp, Ki, Kd, Kf;
  float error_tolerance;

  // output & integral clamp
  float out_min, out_max;
  float i_min, i_max;

  // states
  float Setpoint;
  float LastError;
  float Integral;

  // derivative filter
  float Dfilt;       // ค่าอนุพันธ์หลังกรอง
  float d_fc_hz;     // ความถี่ตัดของฟิลเตอร์ (Hz). 0 = ปิด
  bool  d_init;      // ยังไม่เคย init ค่า Dfilt

  // timing
  unsigned long last_us;

  static inline float clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  }
  float step_dt(); // คืนค่า dt (วินาที) และอัปเดต last_us
};

#endif
