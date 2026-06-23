// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file protocol.hpp
/// ModbusClient — read_registers / write_register / write_registers.

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "mdrobot_cpp/transport.hpp"

namespace mdrobot {

/// Modbus RTU client for a single MDROBOT controller on an RS485 bus.
class ModbusClient {
 public:
  /// Construct with a transport reference and slave ID.
  explicit ModbusClient(Transport& transport, uint8_t slave_id = 1);

  // --- raw primitives ---
  std::vector<uint16_t> read_registers(uint16_t pid, uint16_t count);
  uint16_t read_register(uint16_t pid);
  void write_register(uint16_t pid, uint16_t word);
  void write_registers(uint16_t pid, const std::vector<uint16_t>& words);

  // --- long helpers ---
  int32_t read_long(uint16_t pid, bool is_signed = true);
  void write_long(uint16_t pid, int32_t value);

  // --- command helper ---
  void command(uint16_t cmd);

  Transport& transport() { return transport_; }
  uint8_t slave_id() const { return slave_id_; }

 private:
  std::vector<uint8_t> read_exact(std::size_t size);
  std::vector<uint8_t> transact(const std::vector<uint8_t>& request, std::size_t expected_len);

  Transport& transport_;
  uint8_t slave_id_;
};

}  // namespace mdrobot
