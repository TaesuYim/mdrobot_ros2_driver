// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
#include "mdrobot_cpp/crc.hpp"

namespace mdrobot {

static constexpr uint16_t POLY = 0xA001;

uint16_t crc16_modbus(const uint8_t* data, std::size_t len) {
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ POLY;
      } else {
        crc >>= 1;
      }
      crc &= 0xFFFF;
    }
  }
  return crc;
}

std::vector<uint8_t> append_crc(const uint8_t* data, std::size_t len) {
  uint16_t crc = crc16_modbus(data, len);
  std::vector<uint8_t> result(data, data + len);
  result.push_back(static_cast<uint8_t>(crc & 0xFF));
  result.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return result;
}

bool check_crc(const uint8_t* data, std::size_t len) {
  if (len < 3) return false;
  uint16_t expected = crc16_modbus(data, len - 2);
  return (expected & 0xFF) == data[len - 2] &&
         ((expected >> 8) & 0xFF) == data[len - 1];
}

}  // namespace mdrobot
