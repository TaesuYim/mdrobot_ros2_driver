// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file crc.hpp
/// Modbus CRC16 — initial 0xFFFF, reflected poly 0xA001, wire: low byte first.

#pragma once

#include <cstdint>
#include <vector>

namespace mdrobot {

/// Compute Modbus CRC16 over @p data.
uint16_t crc16_modbus(const uint8_t* data, std::size_t len);

/// Append CRC (low byte, high byte) and return the extended frame.
std::vector<uint8_t> append_crc(const uint8_t* data, std::size_t len);

/// Verify CRC of a complete frame (last 2 bytes = CRC low, CRC high).
bool check_crc(const uint8_t* data, std::size_t len);

}  // namespace mdrobot
