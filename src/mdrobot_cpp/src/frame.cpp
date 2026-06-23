// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
#include "mdrobot_cpp/frame.hpp"

#include <sstream>

#include "mdrobot_cpp/constants.hpp"
#include "mdrobot_cpp/crc.hpp"
#include "mdrobot_cpp/exceptions.hpp"

namespace mdrobot {

static uint8_t hi(uint16_t v) { return static_cast<uint8_t>((v >> 8) & 0xFF); }
static uint8_t lo(uint16_t v) { return static_cast<uint8_t>(v & 0xFF); }

static std::string hex_str(const uint8_t* data, std::size_t len) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < len; ++i) {
    if (i > 0) oss << ' ';
    oss << std::hex << std::uppercase;
    if (data[i] < 0x10) oss << '0';
    oss << static_cast<int>(data[i]);
  }
  return oss.str();
}

static std::string hex_str(const std::vector<uint8_t>& v) {
  return hex_str(v.data(), v.size());
}

// --- request builders ---

std::vector<uint8_t> build_read_request(uint8_t slave_id, uint16_t pid, uint16_t count) {
  if (count < 1) throw std::invalid_argument("count must be >= 1");
  uint8_t body[] = {slave_id, FUNC_READ, hi(pid), lo(pid), hi(count), lo(count)};
  return append_crc(body, sizeof(body));
}

std::vector<uint8_t> build_write_single_request(uint8_t slave_id, uint16_t pid, uint16_t word) {
  word &= 0xFFFF;
  uint8_t body[] = {slave_id, FUNC_WRITE_SINGLE, hi(pid), lo(pid), hi(word), lo(word)};
  return append_crc(body, sizeof(body));
}

std::vector<uint8_t> build_write_multiple_request(uint8_t slave_id, uint16_t pid,
                                                   const std::vector<uint16_t>& words) {
  uint16_t count = static_cast<uint16_t>(words.size());
  if (count < 1) throw std::invalid_argument("words must not be empty");
  std::vector<uint8_t> body;
  body.reserve(7 + 2 * count);
  body.push_back(slave_id);
  body.push_back(FUNC_WRITE_MULTIPLE);
  body.push_back(hi(pid));
  body.push_back(lo(pid));
  body.push_back(hi(count));
  body.push_back(lo(count));
  body.push_back(static_cast<uint8_t>(2 * count));
  for (auto w : words) {
    w &= 0xFFFF;
    body.push_back(hi(w));
    body.push_back(lo(w));
  }
  return append_crc(body.data(), body.size());
}

// --- response parsers ---

/// Check for Modbus exception response and throw if found.
static void raise_for_exception(const std::vector<uint8_t>& frame, uint8_t expected_func) {
  if (frame.size() >= 5 && frame[1] == (expected_func | EXCEPTION_FLAG)) {
    if (!check_crc(frame.data(), 5)) {
      throw CrcError("CRC mismatch on exception frame: " + hex_str(frame));
    }
    throw ProtocolError(
        "modbus exception: func=0x" + hex_str(&expected_func, 1) +
            " code=0x" + hex_str(&frame[2], 1),
        expected_func, frame[2]);
  }
}

std::vector<uint16_t> parse_read_response(const std::vector<uint8_t>& frame,
                                           uint8_t slave_id, uint16_t count) {
  raise_for_exception(frame, FUNC_READ);
  auto expected = read_response_length(count);
  if (frame.size() < expected) {
    throw IncompleteResponseError("short read: got " + std::to_string(frame.size()) +
                                  " want " + std::to_string(expected) + ": " + hex_str(frame));
  }
  if (frame.size() > expected) {
    throw ProtocolError("oversized read: got " + std::to_string(frame.size()) +
                        " want " + std::to_string(expected) + ": " + hex_str(frame));
  }
  if (!check_crc(frame.data(), frame.size())) {
    throw CrcError("CRC mismatch: " + hex_str(frame));
  }
  if (frame[0] != slave_id) {
    throw ProtocolError("id mismatch: got " + std::to_string(frame[0]) +
                        " want " + std::to_string(slave_id));
  }
  if (frame[1] != FUNC_READ) {
    throw ProtocolError("function mismatch: got 0x" + hex_str(&frame[1], 1) +
                        " want 0x" + hex_str(&FUNC_READ, 1));
  }
  uint8_t byte_count = frame[2];
  if (byte_count != 2 * count) {
    throw ProtocolError("byte count mismatch: got " + std::to_string(byte_count) +
                        " want " + std::to_string(2 * count));
  }
  std::vector<uint16_t> words;
  words.reserve(count);
  for (uint16_t i = 0; i < count; ++i) {
    auto idx = 3 + 2 * i;
    words.push_back(static_cast<uint16_t>((frame[idx] << 8) | frame[idx + 1]));
  }
  return words;
}

void parse_write_single_response(const std::vector<uint8_t>& frame,
                                  const std::vector<uint8_t>& request) {
  raise_for_exception(frame, FUNC_WRITE_SINGLE);
  if (frame.size() != WRITE_SINGLE_RESPONSE_LENGTH) {
    throw IncompleteResponseError("write-single response length " +
                                  std::to_string(frame.size()) + " != 8: " + hex_str(frame));
  }
  if (!check_crc(frame.data(), frame.size())) {
    throw CrcError("CRC mismatch: " + hex_str(frame));
  }
  if (!std::equal(frame.begin(), frame.begin() + 6, request.begin())) {
    throw ProtocolError("echo mismatch: req=" + hex_str(request.data(), 6) +
                        " resp=" + hex_str(frame.data(), 6));
  }
}

void parse_write_multiple_response(const std::vector<uint8_t>& frame,
                                    uint8_t slave_id, uint16_t pid, uint16_t count) {
  raise_for_exception(frame, FUNC_WRITE_MULTIPLE);
  if (frame.size() != WRITE_MULTIPLE_RESPONSE_LENGTH) {
    throw IncompleteResponseError("write-multiple response length " +
                                  std::to_string(frame.size()) + " != 8: " + hex_str(frame));
  }
  if (!check_crc(frame.data(), frame.size())) {
    throw CrcError("CRC mismatch: " + hex_str(frame));
  }
  if (frame[0] != slave_id) {
    throw ProtocolError("id mismatch: got " + std::to_string(frame[0]) +
                        " want " + std::to_string(slave_id));
  }
  if (frame[1] != FUNC_WRITE_MULTIPLE) {
    throw ProtocolError("function mismatch: got 0x" + hex_str(&frame[1], 1) +
                        " want 0x" + hex_str(&FUNC_WRITE_MULTIPLE, 1));
  }
  uint16_t echoed_pid = static_cast<uint16_t>((frame[2] << 8) | frame[3]);
  if (echoed_pid != pid) {
    throw ProtocolError("start address echo mismatch: got " + std::to_string(echoed_pid) +
                        " want " + std::to_string(pid));
  }
  uint16_t echoed_count = static_cast<uint16_t>((frame[4] << 8) | frame[5]);
  if (echoed_count != count) {
    throw ProtocolError("register count echo mismatch: got " + std::to_string(echoed_count) +
                        " want " + std::to_string(count));
  }
}

}  // namespace mdrobot
