#include "PIDF.h"
#include <math.h>  // expf, M_PI

PIDF::PIDF(float min_val, float max_val,
           float Kp_, float Ki_,
           float i_min_, float i_max_,
           float Kd_, float Kf_,
           float tol_)
: Kp(0), Ki(0), Kd(0), Kf(0),
  error_tolerance(0),
  out_min(min_val), out_max(max_val),
  i_min(i_min_), i_max(i_max_),
  Setpoint(0.0f), LastError(0.0f), Integral(0.0f),
  Dfilt(0.0f), d_fc_hz(0.0f), d_init(true),
  last_us(0)
{
  setPIDF(Kp_, Ki_, Kd_, Kf_, tol_);
}

void PIDF::setPIDF(float Kp_, float Ki_, float Kd_, float Kf_, float tol_) {
  Kp = Kp_; Ki = Ki_; Kd = Kd_; Kf = Kf_;
  error_tolerance = tol_;
}

void PIDF::setOutputLimits(float min_val, float max_val) {
  out_min = min_val; out_max = max_val;
}

void PIDF::setIClamp(float i_min_, float i_max_) {
  i_min = i_min_; i_max = i_max_;
}

void PIDF::setDFilterCutoffHz(float fc_hz) {
  d_fc_hz = (fc_hz < 0.0f) ? 0.0f : fc_hz;
  d_init  = true;     // ให้ init ใหม่ในรอบถัดไป
}

void PIDF::reset() {
  Integral = 0.0f;
  LastError = 0.0f;
  Dfilt = 0.0f;
  d_init = true;
  last_us  = 0;       // ให้คำนวณ dt ใหม่ในรอบถัดไป
}

float PIDF::step_dt() {
  unsigned long now = micros();
  if (last_us == 0) { last_us = now; return 0.0f; }
  unsigned long du = now - last_us;
  last_us = now;
  const float dt_min = 1e-4f;  // 0.1 ms
  float dt = du * 1e-6f;
  if (dt < dt_min) dt = dt_min;
  return dt;
}

float PIDF::compute(float setpoint, float measure) {
  Setpoint = setpoint;
  float error = setpoint - measure;
  return compute_with_error(error);
}

float PIDF::compute_with_error(float error) {
  float dt = step_dt();

  if (Kf == 0){
    // deadband
    if (fabsf(error) <= error_tolerance) {
      Integral = 0.0f;
      LastError = error;
      // อย่าลืมรีเซ็ต D ให้ตาม error ด้วยเพื่อลด kick ตอนออกจาก deadband
      if (d_init) { Dfilt = 0.0f; } else { Dfilt = 0.9f*Dfilt; }
      return 0.0f;
    }
  }

  // I-term
  Integral += error * dt;
  if (i_max == -1 && i_min == -1) {
  } else {
    Integral = clamp(Integral, i_min, i_max);
  }

  // D-term (raw)
  float D_raw = (dt > 0.0f) ? (error - LastError) / dt : 0.0f;

  // Low-pass derivative: alpha = exp(-2*pi*fc*dt)
  float D_use = D_raw;
  if (Kd != 0.0f && d_fc_hz > 0.0f) {
    float alpha = expf(-2.0f * (float)M_PI * d_fc_hz * dt);
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    if (d_init) { Dfilt = D_raw; d_init = false; }
    else        { Dfilt = alpha * Dfilt + (1.0f - alpha) * D_raw; }
    D_use = Dfilt;
  }

  float out = Kp*error + Ki*Integral + Kd*D_use + Kf*Setpoint;

  LastError = error;
  return clamp(out, out_min, out_max);
}
