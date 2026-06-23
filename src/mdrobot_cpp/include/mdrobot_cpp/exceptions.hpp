// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file exceptions.hpp
/// mdrobot communication-layer exceptions.

#pragma once

#include <optional>
#include <stdexcept>
#include <string>

namespace mdrobot {

/// Base class for all mdrobot communication exceptions.
class MdrobotError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// CRC16 of a response frame does not match.
class CrcError : public MdrobotError {
 public:
  using MdrobotError::MdrobotError;
};

/// Frame structure / echo / exception and other protocol-level errors.
class ProtocolError : public MdrobotError {
 public:
  explicit ProtocolError(const std::string& message,
                         std::optional<uint8_t> function = std::nullopt,
                         std::optional<uint8_t> code = std::nullopt)
      : MdrobotError(message), function_(function), code_(code) {}

  std::optional<uint8_t> function() const { return function_; }
  std::optional<uint8_t> code() const { return code_; }

 private:
  std::optional<uint8_t> function_;
  std::optional<uint8_t> code_;
};

/// Fewer bytes arrived than expected (partial response / timeout).
class IncompleteResponseError : public ProtocolError {
 public:
  using ProtocolError::ProtocolError;
};

}  // namespace mdrobot
