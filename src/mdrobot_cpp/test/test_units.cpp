// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
#include <gtest/gtest.h>
#include <cmath>

#include "mdrobot_cpp/units.hpp"

using namespace mdrobot;

TEST(Units, CountsToRadFullRev) {
  EXPECT_NEAR(counts_to_rad(24, 24), 2*M_PI, 1e-9);
  EXPECT_NEAR(counts_to_rad(12, 24), M_PI, 1e-9);
  EXPECT_DOUBLE_EQ(counts_to_rad(0, 24), 0.0);
}

TEST(Units, CountsToRadSignPreserved) {
  EXPECT_NEAR(counts_to_rad(-6, 24), -M_PI/2, 1e-9);
}

TEST(Units, CountsToRadEncoderResolution) {
  double cpr = 4 * 16384;
  EXPECT_NEAR(counts_to_rad(cpr, cpr), 2*M_PI, 1e-9);
  EXPECT_NEAR(counts_to_rad(1, cpr), 2*M_PI/cpr, 1e-12);
}

TEST(Units, RadToCountsInverse) {
  for (double cpr : {24.0, 65536.0}) {
    for (int count : {0, 1, -1, 7, -13, 1000}) {
      EXPECT_EQ(rad_to_counts(counts_to_rad(count, cpr), cpr), count);
    }
  }
}

TEST(Units, RadToCountsRounds) {
  EXPECT_EQ(rad_to_counts(counts_to_rad(0.4, 24), 24), 0);
  EXPECT_EQ(rad_to_counts(counts_to_rad(0.6, 24), 24), 1);
}

TEST(Units, CountsPerRevMustBePositive) {
  EXPECT_THROW(counts_to_rad(10, 0), std::invalid_argument);
  EXPECT_THROW(counts_to_rad(10, -24), std::invalid_argument);
  EXPECT_THROW(rad_to_counts(1.0, 0), std::invalid_argument);
}

TEST(Units, RpmRadSRoundtrip) {
  EXPECT_NEAR(rpm_to_rad_s(60), 2*M_PI, 1e-9);
  EXPECT_DOUBLE_EQ(rpm_to_rad_s(0), 0.0);
  EXPECT_NEAR(rpm_to_rad_s(-30), -M_PI, 1e-9);
  for (double rpm : {-100.0, 0.0, 1.0, 3000.0}) {
    EXPECT_NEAR(rad_s_to_rpm(rpm_to_rad_s(rpm)), rpm, 1e-9);
  }
}

TEST(Units, SlowSecondsToRawDefaultScale) {
  EXPECT_EQ(slow_seconds_to_raw(0), 0);
  EXPECT_EQ(slow_seconds_to_raw(15), SLOW_RAW_MAX);
  EXPECT_EQ(slow_seconds_to_raw(7.5), static_cast<int>(std::round(7.5 / 15 * SLOW_RAW_MAX)));
}

TEST(Units, SlowSecondsToRawClampsAndValidates) {
  EXPECT_EQ(slow_seconds_to_raw(30), SLOW_RAW_MAX);
  EXPECT_EQ(slow_seconds_to_raw(30, 60), static_cast<int>(std::round(30.0 / 60 * SLOW_RAW_MAX)));
  EXPECT_THROW(slow_seconds_to_raw(-1), std::invalid_argument);
  EXPECT_THROW(slow_seconds_to_raw(1, 0), std::invalid_argument);
}

TEST(Units, SlowRawToSeconds) {
  EXPECT_DOUBLE_EQ(slow_raw_to_seconds(0), 0.0);
  EXPECT_NEAR(slow_raw_to_seconds(SLOW_RAW_MAX), 15.0, 1e-9);
  EXPECT_NEAR(slow_raw_to_seconds(512), 512.0/SLOW_RAW_MAX*15.0, 1e-9);
}

TEST(Units, SlowRoundtripWithinQuantization) {
  double step = 15.0 / SLOW_RAW_MAX;
  for (double sec : {0.0, 1.0, 5.0, 15.0}) {
    EXPECT_NEAR(slow_raw_to_seconds(slow_seconds_to_raw(sec)), sec, step + 1e-9);
  }
}
