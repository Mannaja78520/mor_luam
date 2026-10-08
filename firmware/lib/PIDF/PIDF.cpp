#include "PIDF.h"
#include <math.h>

PIDF::PIDF(float min_val, float max_val, float Kp_, float Ki_, float i_min_, float i_max_,
           float Kd_, float Kf_, float tol_)
    : Kp(0), Ki(0), Kd(0), Kf(0), error_tolerance(0),
      out_min(min_val), out_max(max_val), i_min(i_min_), i_max(i_max_) {
  setPIDF(Kp_, Ki_, Kd_, Kf_, tol_);
}

void PIDF::setPIDF(float Kp_, float Ki_, float Kd_, float Kf_, float tol_) {
  Kp = Kp_; Ki = Ki_; Kd = Kd_; Kf = Kf_;
  error_tolerance = tol_ < 0.0f ? 0.0f : tol_;
}

void PIDF::setOutputLimits(float min_val, float max_val) {
  out_min = min_val;
  out_max = max_val;
}

void PIDF::setIClamp(float i_min_, float i_max_) {
  i_min = i_min_;
  i_max = i_max_;
  i_clamp_on_ = true;
}

void PIDF::setDFilterCutoffHz(float fc_hz) {
  d_fc_hz_ = fc_hz < 0.0f ? 0.0f : fc_hz;
}

void PIDF::reset() {
  integral_ = 0.0f;
  d_filt_ = 0.0f;
  first_ = true;          // next sample seeds the history instead of differentiating
  last_us_ = 0;
  last_p_ = last_i_ = last_d_ = last_f_ = last_out_ = 0.0f;
}

float PIDF::stepDt() {
  const unsigned long now = micros();
  if (last_us_ == 0) { last_us_ = now; return 0.0f; }
  float dt = (now - last_us_) * 1e-6f;
  last_us_ = now;
  if (dt < 1e-4f) dt = 1e-4f;
  if (dt > MAX_DT_S) dt = MAX_DT_S;
  return dt;
}

float PIDF::compute(float setpoint, float measure) {
  // differentiate -measure: same sign as the error's derivative while the
  // setpoint holds, and no spike when it changes
  return compute_with_feedforward(setpoint, measure, Kf * setpoint);
}

float PIDF::compute_with_feedforward(float setpoint, float measure, float feedforward) {
  return update(setpoint - measure, -measure, feedforward);
}

float PIDF::compute_with_error(float error) {
  return update(error, error, 0.0f);
}

float PIDF::update(float error, float d_source, float ff) {
  const float dt = stepDt();
  const float e = fabsf(error) <= error_tolerance ? 0.0f : error;

  // ---- D: filtered, never from a missing history -------------------------
  float d_raw = 0.0f;
  if (first_ || dt <= 0.0f) {
    first_ = false;
    d_filt_ = 0.0f;
  } else {
    d_raw = (d_source - prev_src_) / dt;
  }
  prev_src_ = d_source;
  float d_use = d_raw;
  if (d_fc_hz_ > 0.0f && dt > 0.0f) {
    const float alpha = expf(-2.0f * (float)M_PI * d_fc_hz_ * dt);
    d_filt_ = alpha * d_filt_ + (1.0f - alpha) * d_raw;
    d_use = d_filt_;
  }

  // ---- I: conditional integration (anti-windup) ---------------------------
  float candidate = integral_;
  const bool in_zone = i_zone_ <= 0.0f || fabsf(error) < i_zone_;
  if (in_zone && e != 0.0f) candidate += e * dt;
  if (i_clamp_on_) candidate = clamp(candidate, i_min, i_max);

  const float p = Kp * e;
  const float d = Kd * d_use;
  float out = p + Ki * candidate + d + ff;
  // keep the new integral only if it does not push further into saturation
  const bool sat_hi = out > out_max && e > 0.0f;
  const bool sat_lo = out < out_min && e < 0.0f;
  if (!(sat_hi || sat_lo)) integral_ = candidate;
  const float i = Ki * integral_;
  out = clamp(p + i + d + ff, out_min, out_max);

  // ---- optional ramp -------------------------------------------------------
  if (ramp_ > 0.0f) {
    const float step = ramp_ * dt;
    out = clamp(out, last_out_ - step, last_out_ + step);
  }

  last_p_ = p; last_i_ = i; last_d_ = d; last_f_ = ff; last_out_ = out;
  return out;
}
