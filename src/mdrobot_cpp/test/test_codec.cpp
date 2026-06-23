// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
#include <gtest/gtest.h>

#include "mdrobot_cpp/codec.hpp"

using namespace mdrobot;

TEST(Codec, U16Masks) {
  EXPECT_EQ(u16(0x12345), 0x2345);
  EXPECT_EQ(u16(-1), 0xFFFF);
}

TEST(Codec, Int16SignedInterpretation) {
  EXPECT_EQ(to_int16(0x0064), 100);
  EXPECT_EQ(to_int16(0xFF9C), -100);
  EXPECT_EQ(to_int16(0x8000), -32768);
  EXPECT_EQ(to_int16(0x7FFF), 32767);
}

TEST(Codec, WordFromInt16TwosComplement) {
  EXPECT_EQ(word_from_int16(100), 0x0064);
  EXPECT_EQ(word_from_int16(-100), 0xFF9C);
  EXPECT_EQ(word_from_int16(-1), 0xFFFF);
}

TEST(Codec, SplitLowWordFirst) {
  auto [low, high] = split_u32_low_word_first(0x12345678);
  EXPECT_EQ(low, 0x5678);
  EXPECT_EQ(high, 0x1234);
  auto [low2, high2] = split_i32_low_word_first(0x12345678);
  EXPECT_EQ(low2, 0x5678);
  EXPECT_EQ(high2, 0x1234);
}

TEST(Codec, JoinLowWordFirst) {
  EXPECT_EQ(join_u32_low_word_first(0x5678, 0x1234), 0x12345678u);
  EXPECT_EQ(join_i32_low_word_first(0x5678, 0x1234), 0x12345678);
}

TEST(Codec, LongSignedRoundtrip) {
  for (int32_t value : {0, 1, -1, 100, -100, 0x12345678, -2, 2147483647, -2147483647-1}) {
    auto [low, high] = split_i32_low_word_first(value);
    EXPECT_EQ(join_i32_low_word_first(low, high), value);
  }
}

TEST(Codec, JoinI32Negative) {
  auto [low, high] = split_i32_low_word_first(-2);
  EXPECT_EQ(low, 0xFFFE);
  EXPECT_EQ(high, 0xFFFF);
  EXPECT_EQ(join_i32_low_word_first(0xFFFE, 0xFFFF), -2);
}
