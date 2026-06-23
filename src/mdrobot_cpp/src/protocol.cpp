// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
#include "mdrobot_cpp/protocol.hpp"

#include "mdrobot_cpp/codec.hpp"
#include "mdrobot_cpp/constants.hpp"
#include "mdrobot_cpp/exceptions.hpp"
#include "mdrobot_cpp/frame.hpp"
#include "mdrobot_cpp/registers.hpp"

namespace mdrobot {

ModbusClient::ModbusClient(Transport& transport, uint8_t slave_id)
    : transport_(transport), slave_id_(slave_id) {}

std::vector<uint8_t> ModbusClient::read_exact(std::size_t size) {
  std::vector<uint8_t> buf;
  buf.reserve(size);
  while (buf.size() < size) {
    auto chunk = transport_.read(size - buf.size());
    if (chunk.empty()) {
      std::string hex;
      for (auto b : buf) {
        char tmp[4];
        std::snprintf(tmp, sizeof(tmp), "%02x", b);
        hex += tmp;
      }
      throw IncompleteResponseError("short read: got " + std::to_string(buf.size()) +
                                    " want " + std::to_string(size) + ": " + hex);
    }
    buf.insert(buf.end(), chunk.begin(), chunk.end());
  }
  return buf;
}

std::vector<uint8_t> ModbusClient::transact(const std::vector<uint8_t>& request,
                                             std::size_t expected_len) {
  transport_.flush_input();
  transport_.write(request.data(), request.size());
  auto header = read_exact(2);
  if (header[1] & EXCEPTION_FLAG) {
    auto rest = read_exact(3);  // CODE + CRC_L + CRC_H
    header.insert(header.end(), rest.begin(), rest.end());
    return header;
  }
  auto rest = read_exact(expected_len - 2);
  header.insert(header.end(), rest.begin(), rest.end());
  return header;
}

std::vector<uint16_t> ModbusClient::read_registers(uint16_t pid, uint16_t count) {
  auto request = build_read_request(slave_id_, pid, count);
  auto response = transact(request, read_response_length(count));
  return parse_read_response(response, slave_id_, count);
}

uint16_t ModbusClient::read_register(uint16_t pid) {
  return read_registers(pid, 1)[0];
}

void ModbusClient::write_register(uint16_t pid, uint16_t word) {
  auto request = build_write_single_request(slave_id_, pid, word);
  auto response = transact(request, WRITE_SINGLE_RESPONSE_LENGTH);
  parse_write_single_response(response, request);
}

void ModbusClient::write_registers(uint16_t pid, const std::vector<uint16_t>& words) {
  auto request = build_write_multiple_request(slave_id_, pid, words);
  auto response = transact(request, WRITE_MULTIPLE_RESPONSE_LENGTH);
  parse_write_multiple_response(response, slave_id_, pid, static_cast<uint16_t>(words.size()));
}

int32_t ModbusClient::read_long(uint16_t pid, bool is_signed) {
  auto words = read_registers(pid, 2);
  if (is_signed) {
    return join_i32_low_word_first(words[0], words[1]);
  }
  return static_cast<int32_t>(join_u32_low_word_first(words[0], words[1]));
}

void ModbusClient::write_long(uint16_t pid, int32_t value) {
  auto [low, high] = split_i32_low_word_first(value);
  write_registers(pid, {low, high});
}

void ModbusClient::command(uint16_t cmd) {
  write_register(PID_COMMAND, cmd);
}

}  // namespace mdrobot
