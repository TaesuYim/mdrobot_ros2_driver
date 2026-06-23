// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file codec.hpp
/// byte / word / long encoding helpers (Modbus wire order).

#pragma once

#include <cstdint>
#include <utility>

namespace mdrobot {

/// Mask to 16-bit unsigned.
inline uint16_t u16(int value) { return static_cast<uint16_t>(value & 0xFFFF); }

/// Interpret a 16-bit word as signed int16 (two's complement).
inline int16_t to_int16(uint16_t word) {
  return static_cast<int16_t>(word);
}

/// Encode a signed value as a 16-bit word (two's complement).
inline uint16_t word_from_int16(int16_t value) {
  return static_cast<uint16_t>(value);
}

/// Split a 32-bit unsigned into (low_word, high_word).
inline std::pair<uint16_t, uint16_t> split_u32_low_word_first(uint32_t value) {
  return {static_cast<uint16_t>(value & 0xFFFF),
          static_cast<uint16_t>((value >> 16) & 0xFFFF)};
}

/// Split a signed 32-bit into (low_word, high_word) — same encoding as unsigned.
inline std::pair<uint16_t, uint16_t> split_i32_low_word_first(int32_t value) {
  return split_u32_low_word_first(static_cast<uint32_t>(value));
}

/// Join low/high words into an unsigned 32-bit value.
inline uint32_t join_u32_low_word_first(uint16_t low, uint16_t high) {
  return (static_cast<uint32_t>(high) << 16) | static_cast<uint32_t>(low);
}

/// Join low/high words into a signed 32-bit value (two's complement).
inline int32_t join_i32_low_word_first(uint16_t low, uint16_t high) {
  return static_cast<int32_t>(join_u32_low_word_first(low, high));
}

}  // namespace mdrobot
