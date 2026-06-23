// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file test_plugin_load.cpp
/// Structural test: the SystemInterface plugin is registered and loadable
/// (no motor / no serial port required).

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

namespace {
constexpr char kPluginName[] = "mdrobot_ros2_control/MdrobotSystemHardware";
}

TEST(PluginLoad, Registered) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  const std::vector<std::string> classes = loader.getDeclaredClasses();
  EXPECT_NE(std::find(classes.begin(), classes.end(), kPluginName),
            classes.end())
      << "plugin '" << kPluginName << "' not declared";
}

TEST(PluginLoad, Instantiable) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  std::shared_ptr<hardware_interface::SystemInterface> hw;
  ASSERT_NO_THROW(hw = loader.createSharedInstance(kPluginName));
  EXPECT_NE(hw, nullptr);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
