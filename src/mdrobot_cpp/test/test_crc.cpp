// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
#include <gtest/gtest.h>

#include "mdrobot_cpp/crc.hpp"

using namespace mdrobot;

TEST(Crc, KnownAnswerTest) {
  const uint8_t data[] = {'1','2','3','4','5','6','7','8','9'};
  EXPECT_EQ(crc16_modbus(data, sizeof(data)), 0x4B37);
}

TEST(Crc, WireOrderLowByteFirst) {
  const uint8_t data[] = {'1','2','3','4','5','6','7','8','9'};
  auto framed = append_crc(data, sizeof(data));
  EXPECT_EQ(framed[framed.size()-2], 0x37);
  EXPECT_EQ(framed[framed.size()-1], 0x4B);
}

TEST(Crc, AppendCrcMatchesDocReadVersion) {
  // 01 03 00 01 00 01 -> CRC D5 CA
  const uint8_t body[] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01};
  auto result = append_crc(body, sizeof(body));
  std::vector<uint8_t> expected = {0x01,0x03,0x00,0x01,0x00,0x01,0xD5,0xCA};
  EXPECT_EQ(result, expected);
}

TEST(Crc, CheckCrcRoundtrip) {
  const uint8_t data[] = {'h','e','l','l','o',' ','m','o','d','b','u','s'};
  auto framed = append_crc(data, sizeof(data));
  EXPECT_TRUE(check_crc(framed.data(), framed.size()));
}

TEST(Crc, CheckCrcDetectsCorruption) {
  const uint8_t data[] = {'h','e','l','l','o',' ','m','o','d','b','u','s'};
  auto framed = append_crc(data, sizeof(data));
  framed[0] ^= 0xFF;
  EXPECT_FALSE(check_crc(framed.data(), framed.size()));
}

TEST(Crc, CheckCrcRejectsTooShort) {
  const uint8_t data[] = {0x01, 0x02};
  EXPECT_FALSE(check_crc(data, sizeof(data)));
}
