// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
#include <gtest/gtest.h>

#include "mdrobot_cpp/crc.hpp"
#include "mdrobot_cpp/exceptions.hpp"
#include "mdrobot_cpp/frame.hpp"

using namespace mdrobot;

// helper: hex string to vector
static std::vector<uint8_t> hex(const char* s) {
  std::vector<uint8_t> v;
  while (*s) {
    while (*s == ' ') ++s;
    if (!*s) break;
    char buf[3] = {s[0], s[1], 0};
    v.push_back(static_cast<uint8_t>(std::strtoul(buf, nullptr, 16)));
    s += 2;
  }
  return v;
}

static std::vector<uint8_t> with_crc(const std::vector<uint8_t>& body) {
  return append_crc(body.data(), body.size());
}

// --- golden vectors (build) ---

TEST(Frame, BuildReadVersion) {
  EXPECT_EQ(build_read_request(1, 1, 1), hex("01 03 00 01 00 01 D5 CA"));
}

TEST(Frame, BuildReadMonitor196) {
  EXPECT_EQ(build_read_request(1, 196, 6), hex("01 03 00 C4 00 06 84 35"));
}

TEST(Frame, BuildReadPntMonitor216) {
  EXPECT_EQ(build_read_request(1, 216, 7), hex("01 03 00 D8 00 07 84 33"));
}

TEST(Frame, BuildReadPntMainData210) {
  EXPECT_EQ(build_read_request(1, 210, 9), hex("01 03 00 D2 00 09 25 F5"));
}

TEST(Frame, BuildWriteVelocityPlus100) {
  EXPECT_EQ(build_write_single_request(1, 130, 100), hex("01 06 00 82 00 64 28 09"));
}

TEST(Frame, BuildWriteVelocityMinus100) {
  EXPECT_EQ(build_write_single_request(1, 130, 0xFF9C), hex("01 06 00 82 FF 9C 68 7B"));
}

TEST(Frame, BuildWriteLongPid197) {
  EXPECT_EQ(build_write_multiple_request(1, 197, {0x5678, 0x1234}),
            hex("01 10 00 C5 00 02 04 56 78 12 34 A3 26"));
}

TEST(Frame, BuildPosiVelCmd219LongPlusWord) {
  EXPECT_EQ(build_write_multiple_request(1, 219, {0x5678, 0x1234, 0x9ABC}),
            hex("01 10 00 DB 00 03 06 56 78 12 34 9A BC 10 57"));
}

TEST(Frame, BuildPntPosiVelCmd206Nword) {
  std::vector<uint16_t> words = {0x5678, 0x1234, 0x9ABC, 0x7890, 0x3456, 0x0123};
  EXPECT_EQ(build_write_multiple_request(1, 206, words),
            hex("01 10 00 CE 00 06 0C 56 78 12 34 9A BC 78 90 34 56 01 23 C0 C0"));
}

TEST(Frame, BuildPntVelCmdIndependentWordsNoSwap) {
  auto built = build_write_multiple_request(1, 207, {0x1234, 0x5678});
  auto expected_body = hex("01 10 00 CF 00 02 04 12 34 56 78");
  // Compare body (without CRC)
  EXPECT_EQ(std::vector<uint8_t>(built.begin(), built.end()-2), expected_body);
  EXPECT_TRUE(check_crc(built.data(), built.size()));
}

// --- read response validation ---

TEST(Frame, ParseReadSingleWord) {
  auto resp = with_crc(hex("01 03 02 00 0D"));
  auto words = parse_read_response(resp, 1, 1);
  ASSERT_EQ(words.size(), 1u);
  EXPECT_EQ(words[0], 0x000D);
}

TEST(Frame, ParseReadMultiWord) {
  auto resp = with_crc(hex("01 03 04 00 0D 12 34"));
  auto words = parse_read_response(resp, 1, 2);
  ASSERT_EQ(words.size(), 2u);
  EXPECT_EQ(words[0], 0x000D);
  EXPECT_EQ(words[1], 0x1234);
}

TEST(Frame, ParseReadExceptionResponse) {
  auto resp = with_crc({1, 0x83, 0x02});
  EXPECT_THROW({
    try { parse_read_response(resp, 1, 1); }
    catch (const ProtocolError& e) {
      EXPECT_EQ(e.function(), 0x03);
      EXPECT_EQ(e.code(), 0x02);
      throw;
    }
  }, ProtocolError);
}

TEST(Frame, ParseReadCrcError) {
  auto resp = with_crc(hex("01 03 02 00 0D"));
  resp.back() ^= 0xFF;
  EXPECT_THROW(parse_read_response(resp, 1, 1), CrcError);
}

TEST(Frame, ParseReadByteCountMismatch) {
  auto resp = with_crc({1, 0x03, 0x03, 0x00, 0x0D, 0x00, 0x00});
  EXPECT_THROW(parse_read_response(resp, 1, 2), ProtocolError);
}

TEST(Frame, ParseReadIdMismatch) {
  auto resp = with_crc(hex("02 03 02 00 0D"));
  EXPECT_THROW(parse_read_response(resp, 1, 1), ProtocolError);
}

TEST(Frame, ParseReadShort) {
  auto resp = with_crc(hex("01 03 02 00"));
  EXPECT_THROW(parse_read_response(resp, 1, 1), IncompleteResponseError);
}

// --- write single validation ---

TEST(Frame, ParseWriteSingleEchoOk) {
  auto request = build_write_single_request(1, 130, 100);
  parse_write_single_response(request, request);  // echo = request
}

TEST(Frame, ParseWriteSingleEchoMismatch) {
  auto request = build_write_single_request(1, 130, 100);
  auto bad = request;
  bad[5] ^= 0x01;
  // recompute CRC for the corrupted body
  auto body = std::vector<uint8_t>(bad.begin(), bad.begin() + 6);
  bad = with_crc(body);
  EXPECT_THROW(parse_write_single_response(bad, request), ProtocolError);
}

// --- write multiple validation ---

TEST(Frame, ParseWriteMultipleOk) {
  auto resp = with_crc(hex("01 10 00 DB 00 03"));
  parse_write_multiple_response(resp, 1, 219, 3);
}

TEST(Frame, ParseWriteMultipleCountMismatch) {
  auto resp = with_crc(hex("01 10 00 DB 00 02"));
  EXPECT_THROW(parse_write_multiple_response(resp, 1, 219, 3), ProtocolError);
}

TEST(Frame, ParseWriteMultipleAddressMismatch) {
  auto resp = with_crc(hex("01 10 00 C5 00 03"));
  EXPECT_THROW(parse_write_multiple_response(resp, 1, 219, 3), ProtocolError);
}
