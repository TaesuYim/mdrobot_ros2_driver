// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file constants.hpp
/// Modbus function codes and shared communication constants.

#pragma once

#include <cstdint>

namespace mdrobot {

// Modbus function codes — only these three are implemented.
constexpr uint8_t FUNC_READ           = 0x03;  // read holding registers
constexpr uint8_t FUNC_WRITE_SINGLE   = 0x06;  // write single register
constexpr uint8_t FUNC_WRITE_MULTIPLE = 0x10;  // write multiple registers

// Exception flag in response function byte.
constexpr uint8_t EXCEPTION_FLAG = 0x80;

// ID / write-check constants.
constexpr uint8_t ID_ALL           = 0xFE;
constexpr uint8_t ID_WRITE_CHK     = 0xAA;
constexpr uint8_t ID_DEFAULT_CHK   = 0x55;
constexpr uint8_t ID_DEVELOPER_CHK = 0x77;

// Communication defaults: 19200 8N1.
constexpr uint8_t  DEFAULT_SLAVE_ID = 1;
constexpr int      DEFAULT_BAUDRATE = 19200;
constexpr double   DEFAULT_TIMEOUT  = 0.3;  // serial read timeout (seconds)

}  // namespace mdrobot
