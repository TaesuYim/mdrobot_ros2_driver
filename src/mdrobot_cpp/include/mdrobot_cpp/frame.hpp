// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file frame.hpp
/// Modbus RTU frame builder / parser — mirrors Python frame.py.

#pragma once

#include <cstdint>
#include <vector>

namespace mdrobot {

// --- request builders ---
std::vector<uint8_t> build_read_request(uint8_t slave_id, uint16_t pid, uint16_t count);
std::vector<uint8_t> build_write_single_request(uint8_t slave_id, uint16_t pid, uint16_t word);
std::vector<uint8_t> build_write_multiple_request(uint8_t slave_id, uint16_t pid,
                                                   const std::vector<uint16_t>& words);

// --- response lengths ---
inline std::size_t read_response_length(uint16_t count) {
  return 5 + 2 * static_cast<std::size_t>(count);
}
constexpr std::size_t WRITE_SINGLE_RESPONSE_LENGTH   = 8;
constexpr std::size_t WRITE_MULTIPLE_RESPONSE_LENGTH = 8;

// --- response parsers ---
/// Parse 0x03 read response; returns the 16-bit words. Throws on error.
std::vector<uint16_t> parse_read_response(const std::vector<uint8_t>& frame,
                                           uint8_t slave_id, uint16_t count);

/// Validate 0x06 write-single response (echo must match request). Throws on error.
void parse_write_single_response(const std::vector<uint8_t>& frame,
                                  const std::vector<uint8_t>& request);

/// Validate 0x10 write-multiple response (address/count echo). Throws on error.
void parse_write_multiple_response(const std::vector<uint8_t>& frame,
                                    uint8_t slave_id, uint16_t pid, uint16_t count);

}  // namespace mdrobot
