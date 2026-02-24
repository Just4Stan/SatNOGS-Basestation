#include "satnogs_protocol.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

bool starts_with(const char* s, const char* prefix) { return std::strncmp(s, prefix, std::strlen(prefix)) == 0; }

bool parse_float_after_prefix(const char* token, const char* prefix, float* out_value) {
  if (!starts_with(token, prefix)) return false;
  const char* number = token + std::strlen(prefix);
  if (*number == '\0') return false;
  char* end = nullptr;
  const float value = std::strtof(number, &end);
  if (end == number) return false;
  *out_value = value;
  return true;
}

} // namespace

SatnogsProtocol::SatnogsProtocol(Rotator& rotator, ADXL345* imu)
    : rotator_(rotator), imu_(imu), staged_az_(0.0f), staged_el_(0.0f), staged_az_set_(false), staged_el_set_(false), monitoring_(false) {}

void SatnogsProtocol::handle_line(const char* line, size_t len) {
  // Accept multiple tokens per line, e.g. "AZ145.0 EL45.0" or "AZ EL" for query.
  staged_az_set_ = false;
  staged_el_set_ = false;

  char buffer[128];
  if (len >= sizeof(buffer)) len = sizeof(buffer) - 1;
  std::memcpy(buffer, line, len);
  buffer[len] = '\0';

  // Tokenize in-place.
  const char* delim = " \t";
  char* saveptr = nullptr;
  for (char* tok = strtok_r(buffer, delim, &saveptr); tok != nullptr;
       tok = strtok_r(nullptr, delim, &saveptr)) {
    handle_token(tok);
  }

  // If both were staged in one line, apply atomically.
  if (staged_az_set_ && staged_el_set_) {
    rotator_.set_target(staged_az_, staged_el_);
  }
}

void SatnogsProtocol::handle_token(const char* token) {
  // Normalize token to uppercase for command matching (without allocating).
  char upper[32];
  size_t n = std::min<size_t>(std::strlen(token), sizeof(upper) - 1);
  for (size_t i = 0; i < n; i++) upper[i] = static_cast<char>(std::toupper(static_cast<unsigned char>(token[i])));
  upper[n] = '\0';

  float value = 0.0f;
  if (parse_float_after_prefix(token, "AZ", &value) || parse_float_after_prefix(token, "az", &value)) {
    staged_az_ = value;
    staged_az_set_ = true;
    if (!staged_el_set_) rotator_.set_target_az(value);
    return;
  }
  if (parse_float_after_prefix(token, "EL", &value) || parse_float_after_prefix(token, "el", &value)) {
    staged_el_ = value;
    staged_el_set_ = true;
    if (!staged_az_set_) rotator_.set_target_el(value);
    return;
  }

  if (std::strcmp(upper, "AZ") == 0 || std::strcmp(upper, "EL") == 0 || std::strcmp(upper, "AZEL") == 0) {
    // Some controllers use "AZ EL" as a query. We'll answer on either token.
    reply_position();
    return;
  }
  if (std::strcmp(upper, "GET_AZ") == 0 || std::strcmp(upper, "GET_EL") == 0) {
    reply_position();
    return;
  }

  if (std::strcmp(upper, "RESET") == 0 || std::strcmp(upper, "R") == 0) {
    rotator_.reset_home();
    reply_position();
    return;
  }
  if (std::strcmp(upper, "PARK") == 0) {
    rotator_.park();
    reply_position();
    return;
  }
  if (std::strcmp(upper, "SA") == 0) {
    rotator_.stop_az();
    reply_position();
    return;
  }
  if (std::strcmp(upper, "SE") == 0) {
    rotator_.stop_el();
    reply_position();
    return;
  }
  if (std::strcmp(upper, "S") == 0 || std::strcmp(upper, "STOP") == 0) {
    rotator_.stop_all();
    reply_position();
    return;
  }
  if (std::strcmp(upper, "VE") == 0) {
    std::printf("VESatNOGS-RP2040-v0.2\n");
    return;
  }
  if (std::strcmp(upper, "GS") == 0) {
    const auto st = rotator_.status();
    std::printf("GS%lu\n", static_cast<unsigned long>(st.status_reg));
    return;
  }
  if (std::strcmp(upper, "GE") == 0) {
    const auto st = rotator_.status();
    std::printf("GE%lu\n", static_cast<unsigned long>(st.error_reg));
    return;
  }

  // Bench test commands (raw motor duty, bypasses P-controller).
  if (parse_float_after_prefix(token, "DUTY_AZ", &value) || parse_float_after_prefix(token, "duty_az", &value)) {
    rotator_.set_raw_az(value);
    std::printf("AZ duty=%.2f\n", value);
    return;
  }
  if (parse_float_after_prefix(token, "DUTY_EL", &value) || parse_float_after_prefix(token, "duty_el", &value)) {
    rotator_.set_raw_el(value);
    std::printf("EL duty=%.2f\n", value);
    return;
  }

  if (std::strcmp(upper, "TICKS") == 0) {
    std::printf("AZ_TICKS=%ld EL_TICKS=%ld\n",
                static_cast<long>(rotator_.raw_az_ticks()),
                static_cast<long>(rotator_.raw_el_ticks()));
    return;
  }

  if (std::strcmp(upper, "INFO") == 0) {
    const auto st = rotator_.status();
    std::printf("AZ=%.1f EL=%.1f\n", st.az_deg, st.el_deg);
    std::printf("AZ_TICKS=%ld EL_TICKS=%ld\n",
                static_cast<long>(rotator_.raw_az_ticks()),
                static_cast<long>(rotator_.raw_el_ticks()));
    std::printf("AZ_HOMED=%d EL_HOMED=%d MOVING=%d\n", st.az_homed, st.el_homed, st.moving);
    std::printf("AZ_END=%d EL_END=%d\n", rotator_.endstop_az_raw(), rotator_.endstop_el_raw());
    std::printf("AZ_ALERT=%d EL_ALERT=%d\n", rotator_.alert_az_raw(), rotator_.alert_el_raw());
    std::printf("MANUAL=%d STATUS=%lu ERROR=%lu\n",
                rotator_.manual_mode(),
                static_cast<unsigned long>(st.status_reg),
                static_cast<unsigned long>(st.error_reg));
    return;
  }

  if (std::strcmp(upper, "MONITOR") == 0 || std::strcmp(upper, "MON") == 0) {
    monitoring_ = !monitoring_;
    std::printf("MONITOR %s (send MON again to stop)\n", monitoring_ ? "ON" : "OFF");
    return;
  }

  if (std::strcmp(upper, "ZERO") == 0 || std::strcmp(upper, "Z") == 0) {
    rotator_.zero_encoders();
    std::printf("Encoders zeroed\n");
    return;
  }

  // PID gain tuning commands
  if (std::strcmp(upper, "GAINS") == 0) {
    std::printf("KP=%.3f KI=%.3f KD=%.3f\n", rotator_.kp(), rotator_.ki(), rotator_.kd());
    return;
  }
  if (parse_float_after_prefix(token, "KP", &value) || parse_float_after_prefix(token, "kp", &value)) {
    rotator_.set_gains(value, rotator_.ki(), rotator_.kd());
    std::printf("KP=%.3f KI=%.3f KD=%.3f\n", rotator_.kp(), rotator_.ki(), rotator_.kd());
    return;
  }
  if (parse_float_after_prefix(token, "KI", &value) || parse_float_after_prefix(token, "ki", &value)) {
    rotator_.set_gains(rotator_.kp(), value, rotator_.kd());
    std::printf("KP=%.3f KI=%.3f KD=%.3f\n", rotator_.kp(), rotator_.ki(), rotator_.kd());
    return;
  }
  if (parse_float_after_prefix(token, "KD", &value) || parse_float_after_prefix(token, "kd", &value)) {
    rotator_.set_gains(rotator_.kp(), rotator_.ki(), value);
    std::printf("KP=%.3f KI=%.3f KD=%.3f\n", rotator_.kp(), rotator_.ki(), rotator_.kd());
    return;
  }

  if (std::strcmp(upper, "IMU") == 0) {
    if (!imu_) {
      std::printf("IMU not found\n");
    } else {
      float x, y, z;
      imu_->read_accel(&x, &y, &z);
      std::printf("X=%.3f Y=%.3f Z=%.3f g  EL=%.1f deg\n", x, y, z, imu_->elevation_deg());
    }
    return;
  }

  if (std::strcmp(upper, "RECAL") == 0) {
    if (!imu_) {
      std::printf("IMU not found — cannot recalibrate\n");
    } else {
      const float el = imu_->elevation_deg_averaged(20);
      rotator_.calibrate_el(el);
      std::printf("EL recalibrated from IMU: %.1f deg\n", el);
    }
    return;
  }

  if (std::strcmp(upper, "HELP") == 0 || std::strcmp(upper, "?") == 0) {
    std::printf("=== Bench test commands ===\n");
    std::printf("DUTY_AZ<f>  Raw AZ motor duty (-1..1)\n");
    std::printf("DUTY_EL<f>  Raw EL motor duty (-1..1)\n");
    std::printf("STOP        Stop all motors\n");
    std::printf("TICKS       Print encoder tick counts\n");
    std::printf("ZERO        Reset encoder counters to 0\n");
    std::printf("MONITOR     Toggle live tick stream (10 Hz)\n");
    std::printf("INFO        Full status dump\n");
    std::printf("GAINS       Show PID gains\n");
    std::printf("KP<f> KI<f> KD<f>  Set PID gains at runtime\n");
    std::printf("IMU         Read accelerometer (X/Y/Z g + EL deg)\n");
    std::printf("RECAL       Re-calibrate EL from IMU\n");
    std::printf("=== SatNOGS/EasyComm ===\n");
    std::printf("VE  GS  GE  AZ  EL  AZ<f> EL<f>  RESET  PARK  STOP\n");
    return;
  }

  // Ignore unknown tokens to be tolerant with different hamlib backends.
}

void SatnogsProtocol::reply_position() const {
  const auto st = rotator_.status();
  std::printf("AZ%.1f EL%.1f\n", st.az_deg, st.el_deg);
}

void SatnogsProtocol::print_monitor_line() const {
  std::printf("AZ_T=%ld EL_T=%ld  AZ_END=%d EL_END=%d  AZ_ALR=%d EL_ALR=%d\n",
              static_cast<long>(rotator_.raw_az_ticks()),
              static_cast<long>(rotator_.raw_el_ticks()),
              rotator_.endstop_az_raw(),
              rotator_.endstop_el_raw(),
              rotator_.alert_az_raw(),
              rotator_.alert_el_raw());
}

