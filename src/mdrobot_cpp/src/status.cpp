// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
#include "mdrobot_cpp/status.hpp"

#include <stdexcept>

#include "mdrobot_cpp/codec.hpp"

namespace mdrobot {

const std::map<int, std::string> STATUS1_BIT_NAMES = {
    {0, "ALARM"},     {1, "CTRL_FAIL"},        {2, "OVER_VOLT"},
    {3, "OVER_TEMP"}, {4, "OVER_LOAD"},        {5, "HALL_OR_ENC_FAIL"},
    {6, "INV_VEL"},   {7, "STALL"},
};

const std::map<int, std::string> STATUS2_BIT_NAMES = {
    {0, "REGEN_OVER_TEMP"}, {1, "ENC_FAIL"},
};

const std::map<int, std::string> DI_BIT_NAMES = {
    {0, "INT_SPEED"},   {1, "ALARM_RESET"}, {2, "DIR"},
    {3, "RUN_BRAKE"},   {4, "START_STOP"},   {5, "ENC_B"},
    {6, "ENC_A"},
};

std::vector<std::string> active_bits(uint8_t value, const std::map<int, std::string>& names) {
  std::vector<std::string> result;
  for (const auto& [bit, name] : names) {
    if (value & (1 << bit)) {
      result.push_back(name);
    }
  }
  return result;
}

StatusBits StatusBits::from_byte(uint8_t value) {
  uint8_t v = value & 0xFF;
  return StatusBits{
      v,
      static_cast<bool>(v & (1 << 0)),  // alarm
      static_cast<bool>(v & (1 << 1)),  // ctrl_fail
      static_cast<bool>(v & (1 << 2)),  // over_voltage
      static_cast<bool>(v & (1 << 3)),  // over_temperature
      static_cast<bool>(v & (1 << 4)),  // overload
      static_cast<bool>(v & (1 << 5)),  // hall_or_encoder_fail
      static_cast<bool>(v & (1 << 6)),  // inverse_velocity
      static_cast<bool>(v & (1 << 7)),  // stall
  };
}

std::vector<std::string> StatusBits::active() const {
  return active_bits(raw, STATUS1_BIT_NAMES);
}

Monitor decode_monitor(const std::vector<uint16_t>& words) {
  if (words.size() != 6) {
    throw std::invalid_argument("PID_MONITOR expects 6 words, got " +
                                std::to_string(words.size()));
  }
  Monitor m;
  m.speed_rpm = to_int16(words[0]);
  m.current_a = words[1] / 10.0;
  m.output_raw = to_int16(words[2]);
  m.position = join_i32_low_word_first(words[3], words[4]);
  m.status = StatusBits::from_byte(static_cast<uint8_t>(words[5] & 0xFF));
  m.status2_raw = static_cast<uint8_t>((words[5] >> 8) & 0xFF);
  return m;
}

DualMonitor decode_pnt_monitor(const std::vector<uint16_t>& words) {
  if (words.size() != 7) {
    throw std::invalid_argument("PID_PNT_MONITOR expects 7 words, got " +
                                std::to_string(words.size()));
  }
  DualMonitor dm;
  dm.motor1.speed_rpm = to_int16(words[0]);
  dm.motor1.current_a = std::nullopt;
  dm.motor1.output_raw = std::nullopt;
  dm.motor1.position = join_i32_low_word_first(words[1], words[2]);
  dm.motor1.status = StatusBits::from_byte(static_cast<uint8_t>(words[6] & 0xFF));
  dm.motor1.status2_raw = 0;

  dm.motor2.speed_rpm = to_int16(words[3]);
  dm.motor2.current_a = std::nullopt;
  dm.motor2.output_raw = std::nullopt;
  dm.motor2.position = join_i32_low_word_first(words[4], words[5]);
  dm.motor2.status = StatusBits::from_byte(static_cast<uint8_t>((words[6] >> 8) & 0xFF));
  dm.motor2.status2_raw = 0;
  return dm;
}

DualMonitor decode_pnt_main_data(const std::vector<uint16_t>& words) {
  if (words.size() != 9) {
    throw std::invalid_argument("PID_PNT_MAIN_DATA expects 9 words, got " +
                                std::to_string(words.size()));
  }
  DualMonitor dm;
  dm.motor1.speed_rpm = to_int16(words[0]);
  dm.motor1.current_a = words[1] / 10.0;
  dm.motor1.output_raw = std::nullopt;
  dm.motor1.position = join_i32_low_word_first(words[2], words[3]);
  dm.motor1.status = StatusBits::from_byte(static_cast<uint8_t>(words[8] & 0xFF));
  dm.motor1.status2_raw = 0;

  dm.motor2.speed_rpm = to_int16(words[4]);
  dm.motor2.current_a = words[5] / 10.0;
  dm.motor2.output_raw = std::nullopt;
  dm.motor2.position = join_i32_low_word_first(words[6], words[7]);
  dm.motor2.status = StatusBits::from_byte(static_cast<uint8_t>((words[8] >> 8) & 0xFF));
  dm.motor2.status2_raw = 0;
  return dm;
}

}  // namespace mdrobot
