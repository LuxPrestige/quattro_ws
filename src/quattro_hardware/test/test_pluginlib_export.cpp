// Verifies quattro_hardware.xml / CMakeLists.txt / package.xml are wired
// correctly enough for pluginlib to actually find and instantiate the
// plugin -- catches export/registration mistakes that a pure unit test of
// QuattroSystem's own logic would never touch.

#include <gtest/gtest.h>

#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

TEST(PluginlibExport, LoadsQuattroSystem)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");

  std::shared_ptr<hardware_interface::SystemInterface> instance;
  ASSERT_NO_THROW(instance = loader.createSharedInstance("quattro_hardware/QuattroSystem"));
  ASSERT_NE(instance, nullptr);
}
