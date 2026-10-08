#ifndef PIDF_H
#define PIDF_H
// PID + feed-forward for one loop (the wheel spin, or the steering).
//
//   out = clamp( P + I + D + F )
//   P = Kp * e                      e = 0 inside the deadband (error_tolerance)
//   I = Ki * integral(e dt)         clamped to [i_min, i_max] (in error*s, as before)
//   D = Kd * d/dt, low-pass filtered (cut-off in Hz)
//   F = Kf * setpoint               compute() only
//
// What changed from the first version, and why (each one moved the robot):
//  * No derivative kick when the wheel leaves the deadband. Inside the
//    tolerance the old code kept LastError = 0, so the first sample outside
//    gave (e - 0)/dt: a 20 deg knock became 2000 deg/s of D (test_host/tests.cpp
//    shows the numbers). Now the controller resets inside the band and the
//    first sample after a reset only seeds the history.
//  * compute() differentiates the MEASUREMENT, not the error, so changing the
//    setpoint does not kick D either.
//  * Anti-windup. The integral stops growing while the output is saturated in
//    the same direction, and optionally only grows near the target (I-zone).
//    A long steer used to build up an I term that was still pushing when the
//    wheel reached its angle - the overshoot.
//  * dt is capped (MAX_DT_S): a loop that was not called for a while cannot
//    integrate seconds of error in one step.
//  * compute_with_error() no longer reuses a setpoint left over from an older
//    compute() call; it has no feed-forward.
//  * "no I clamp" is setIClampEnabled(false), not the magic value -1.
//  * Optional output ramp (units per second) to soften current spikes.
//  * last*() expose P, I, D, F and the output, for tuning from the web page.
//
// The public API of the first version is unchanged, so callers did not move.
#include <Arduino.h>

class PIDF {
public:
  // ctor: min,max, Kp,Ki, i_min,i_max, Kd,Kf, tol   (same order as before)
  PIDF(float min_val, float max_val,
       float Kp = 0.0f, float Ki = 0.0f,
       float i_min = 0.0f, float i_max = 0.0f,
       float Kd = 0.0f, float Kf = 0.0f,
       float error_tolerance = 0.0f);

  void  setPIDF(float Kp, float Ki, float Kd, float Kf, float error_tolerance);
  void  setOutputLimits(float min_val, float max_val);
  void  setIClamp(float i_min, float i_max);
  void  setIClampEnabled(bool on) { i_clamp_on_ = on; }
  void  setIZone(float zone) { i_zone_ = zone < 0.0f ? 0.0f : zone; }        // 0 = integrate always
  void  setDFilterCutoffHz(float fc_hz);                                     // 0 = raw derivative
  void  setOutputRamp(float units_per_s) { ramp_ = units_per_s < 0.0f ? 0.0f : units_per_s; }  // 0 = off
  void  reset();

  // setpoint/measurement: D on measurement, F = Kf * setpoint
  float compute(float setpoint, float measure);
  // Replace Kf * setpoint with a learned model, inside limits/anti-windup/ramp.
  float compute_with_feedforward(float setpoint, float measure, float feedforward);
  // error only (e.g. the clockwise steering error): D on error, no F
  float compute_with_error(float error);

  // tuning read-back
  float kp() const { return Kp; }
  float ki() const { return Ki; }
  float kd() const { return Kd; }
  float kf() const { return Kf; }
  float tolerance() const { return error_tolerance; }
  float outputMin() const { return out_min; }
  float outputMax() const { return out_max; }
  float lastP() const { return last_p_; }
  float lastI() const { return last_i_; }
  float lastD() const { return last_d_; }
  float lastF() const { return last_f_; }
  float lastOut() const { return last_out_; }

  static constexpr float MAX_DT_S = 0.1f;

private:
  float update(float error, float d_source, float ff);
  float stepDt();
  static inline float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  // gains
  float Kp, Ki, Kd, Kf;
  float error_tolerance;
  // limits
  float out_min, out_max;
  float i_min, i_max;
  bool  i_clamp_on_ = true;
  float i_zone_ = 0.0f;
  float ramp_ = 0.0f;
  // state
  float integral_ = 0.0f;
  float prev_src_ = 0.0f;
  float d_filt_ = 0.0f;
  float d_fc_hz_ = 0.0f;
  bool  first_ = true;
  unsigned long last_us_ = 0;
  // last terms
  float last_p_ = 0, last_i_ = 0, last_d_ = 0, last_f_ = 0, last_out_ = 0;
};

#endif
