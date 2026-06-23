// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file status.hpp
/// Status-bit / monitor decoding — mirrors Python status.py.

#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace mdrobot {

// --- bit name tables ---
extern const std::map<int, std::string> STATUS1_BIT_NAMES;
extern const std::map<int, std::string> STATUS2_BIT_NAMES;
extern const std::map<int, std::string> DI_BIT_NAMES;

/// Return names of set bits (ascending bit order).
std::vector<std::string> active_bits(uint8_t value, const std::map<int, std::string>& names);

/// Decoded status1 byte (8 bits).
struct StatusBits {
  uint8_t raw;
  bool alarm;
  bool ctrl_fail;
  bool over_voltage;
  bool over_temperature;
  bool overload;
  bool hall_or_encoder_fail;
  bool inverse_velocity;
  bool stall;

  static StatusBits from_byte(uint8_t value);
  std::vector<std::string> active() const;
};

/// Decoded single-channel monitor.
struct Monitor {
  int speed_rpm;
  std::optional<double> current_a;
  std::optional<int> output_raw;
  int32_t position;
  StatusBits status;
  uint8_t status2_raw = 0;
};

/// Decoded dual-channel monitor.
struct DualMonitor {
  Monitor motor1;
  Monitor motor2;
};

/// Decode a 6-word PID_MONITOR(196) response.
Monitor decode_monitor(const std::vector<uint16_t>& words);

/// Decode a 7-word PID_PNT_MONITOR(216) response.
DualMonitor decode_pnt_monitor(const std::vector<uint16_t>& words);

/// Decode a 9-word PID_PNT_MAIN_DATA(210) response.
DualMonitor decode_pnt_main_data(const std::vector<uint16_t>& words);

}  // namespace mdrobot
