// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
#include <gtest/gtest.h>

#include "mdrobot_cpp/status.hpp"

using namespace mdrobot;

TEST(Status, AllClear) {
  auto bits = StatusBits::from_byte(0x00);
  EXPECT_EQ(bits.raw, 0);
  EXPECT_FALSE(bits.alarm);
  EXPECT_FALSE(bits.ctrl_fail);
  EXPECT_FALSE(bits.over_voltage);
  EXPECT_FALSE(bits.over_temperature);
  EXPECT_FALSE(bits.overload);
  EXPECT_FALSE(bits.hall_or_encoder_fail);
  EXPECT_FALSE(bits.inverse_velocity);
  EXPECT_FALSE(bits.stall);
  EXPECT_TRUE(bits.active().empty());
}

TEST(Status, AllSet) {
  auto bits = StatusBits::from_byte(0xFF);
  EXPECT_EQ(bits.raw, 0xFF);
  EXPECT_TRUE(bits.alarm);
  EXPECT_TRUE(bits.ctrl_fail);
  EXPECT_TRUE(bits.over_voltage);
  EXPECT_TRUE(bits.over_temperature);
  EXPECT_TRUE(bits.overload);
  EXPECT_TRUE(bits.hall_or_encoder_fail);
  EXPECT_TRUE(bits.inverse_velocity);
  EXPECT_TRUE(bits.stall);
  auto act = bits.active();
  EXPECT_EQ(act.size(), 8u);
}

TEST(Status, Selected) {
  auto bits = StatusBits::from_byte(0x05);  // ALARM + OVER_VOLT
  EXPECT_TRUE(bits.alarm);
  EXPECT_TRUE(bits.over_voltage);
  EXPECT_FALSE(bits.ctrl_fail);
  EXPECT_FALSE(bits.stall);
  auto act = bits.active();
  ASSERT_EQ(act.size(), 2u);
  EXPECT_EQ(act[0], "ALARM");
  EXPECT_EQ(act[1], "OVER_VOLT");
}

TEST(Status, MasksHighBits) {
  // 0x100 is out of byte range.
  auto bits = StatusBits::from_byte(0x00);
  EXPECT_EQ(bits.raw, 0);
}

TEST(Status, ActiveBitsDi) {
  uint8_t word = (1 << 2) | (1 << 4);  // DIR + START_STOP
  auto result = active_bits(word, DI_BIT_NAMES);
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0], "DIR");
  EXPECT_EQ(result[1], "START_STOP");
}

TEST(Status, DecodeMonitor) {
  std::vector<uint16_t> words = {0x0064, 0x000C, 0xFFCE, 0x5678, 0x1234, 0x0205};
  auto mon = decode_monitor(words);
  EXPECT_EQ(mon.speed_rpm, 100);
  EXPECT_NEAR(mon.current_a.value(), 1.2, 0.01);
  EXPECT_EQ(mon.output_raw.value(), -50);
  EXPECT_EQ(mon.position, 0x12345678);
  auto act = mon.status.active();
  ASSERT_EQ(act.size(), 2u);
  EXPECT_EQ(act[0], "ALARM");
  EXPECT_EQ(act[1], "OVER_VOLT");
  EXPECT_EQ(mon.status2_raw, 0x02);
}

TEST(Status, DecodeMonitorNegativeSpeedAndPosition) {
  std::vector<uint16_t> words = {0xFF9C, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x0000};
  auto mon = decode_monitor(words);
  EXPECT_EQ(mon.speed_rpm, -100);
  EXPECT_EQ(mon.position, -1);
}

TEST(Status, DecodeMonitorWrongLength) {
  EXPECT_THROW(decode_monitor({0, 0, 0}), std::invalid_argument);
}

TEST(Status, DecodePntMonitor) {
  std::vector<uint16_t> words = {0x0064, 0x0005, 0x0000, 0xFFCE, 0xFFFF, 0xFFFF, 0x0501};
  auto dm = decode_pnt_monitor(words);
  EXPECT_EQ(dm.motor1.speed_rpm, 100);
  EXPECT_EQ(dm.motor1.position, 5);
  EXPECT_FALSE(dm.motor1.current_a.has_value());
  EXPECT_FALSE(dm.motor1.output_raw.has_value());
  EXPECT_EQ(dm.motor1.status.active()[0], "ALARM");
  EXPECT_EQ(dm.motor2.speed_rpm, -50);
  EXPECT_EQ(dm.motor2.position, -1);
  auto act2 = dm.motor2.status.active();
  ASSERT_EQ(act2.size(), 2u);
  EXPECT_EQ(act2[0], "ALARM");
  EXPECT_EQ(act2[1], "OVER_VOLT");
}

TEST(Status, DecodePntMonitorWrongLength) {
  EXPECT_THROW(decode_pnt_monitor({0,0,0,0,0,0}), std::invalid_argument);
}

TEST(Status, DecodePntMainData) {
  std::vector<uint16_t> words = {0x0064, 0x000C, 0x5678, 0x1234,
                                  0xFF9C, 0x0006, 0x0000, 0x0000, 0x0001};
  auto dm = decode_pnt_main_data(words);
  EXPECT_EQ(dm.motor1.speed_rpm, 100);
  EXPECT_NEAR(dm.motor1.current_a.value(), 1.2, 0.01);
  EXPECT_EQ(dm.motor1.position, 0x12345678);
  EXPECT_FALSE(dm.motor1.output_raw.has_value());
  EXPECT_EQ(dm.motor1.status.active()[0], "ALARM");
  EXPECT_EQ(dm.motor2.speed_rpm, -100);
  EXPECT_NEAR(dm.motor2.current_a.value(), 0.6, 0.01);
  EXPECT_EQ(dm.motor2.position, 0);
  EXPECT_TRUE(dm.motor2.status.active().empty());
}

TEST(Status, DecodePntMainDataWrongLength) {
  EXPECT_THROW(decode_pnt_main_data({0,0,0,0,0,0,0}), std::invalid_argument);
}
