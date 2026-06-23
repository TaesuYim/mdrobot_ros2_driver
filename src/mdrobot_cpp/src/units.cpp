// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
#include "mdrobot_cpp/units.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mdrobot {

double counts_to_rad(double count, double counts_per_rev) {
  if (counts_per_rev <= 0) {
    throw std::invalid_argument("counts_per_rev must be > 0, got " +
                                std::to_string(counts_per_rev));
  }
  return count * TWO_PI / counts_per_rev;
}

int rad_to_counts(double rad, double counts_per_rev) {
  if (counts_per_rev <= 0) {
    throw std::invalid_argument("counts_per_rev must be > 0, got " +
                                std::to_string(counts_per_rev));
  }
  return static_cast<int>(std::round(rad * counts_per_rev / TWO_PI));
}

int slow_seconds_to_raw(double seconds, double full_scale_s) {
  if (seconds < 0) {
    throw std::invalid_argument("seconds must be >= 0, got " + std::to_string(seconds));
  }
  if (full_scale_s <= 0) {
    throw std::invalid_argument("full_scale_s must be > 0, got " + std::to_string(full_scale_s));
  }
  int raw = static_cast<int>(std::round(seconds / full_scale_s * SLOW_RAW_MAX));
  return std::clamp(raw, 0, SLOW_RAW_MAX);
}

double slow_raw_to_seconds(int raw, double full_scale_s) {
  if (full_scale_s <= 0) {
    throw std::invalid_argument("full_scale_s must be > 0, got " + std::to_string(full_scale_s));
  }
  return static_cast<double>(raw) / SLOW_RAW_MAX * full_scale_s;
}

}  // namespace mdrobot
