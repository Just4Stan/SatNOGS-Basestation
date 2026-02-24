#pragma once

#include <cstddef>
#include <cstdint>

#include "adxl345.h"
#include "rotator.h"

class SatnogsProtocol {
public:
  SatnogsProtocol(Rotator& rotator, ADXL345* imu = nullptr);

  // Feed a single received line (without CR/LF). Responses are written to stdout (USB serial).
  void handle_line(const char* line, size_t len);

  // Call from main loop at ~10Hz when monitoring is active.
  bool monitoring() const { return monitoring_; }
  void print_monitor_line() const;

private:
  void handle_token(const char* token);
  void reply_position() const;

  Rotator& rotator_;
  ADXL345* imu_;

  float staged_az_;
  float staged_el_;
  bool staged_az_set_;
  bool staged_el_set_;
  bool monitoring_;
};
