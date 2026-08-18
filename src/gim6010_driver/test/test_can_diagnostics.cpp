#include <array>
#include <cstdint>

#include "gim6010_driver/can_diagnostics.hpp"
#include "gtest/gtest.h"

TEST(CanDiagnostics, DecodesFirmware0513Heartbeat)
{
  const std::array<std::uint8_t, 8> data{{0x40, 0x00, 0x00, 0x00, 0x01, 0x89, 0x00, 0xF4}};
  const auto heartbeat = gim6010_driver::decodeHeartbeat(data.data(), data.size());
  EXPECT_EQ(heartbeat.axis_error, 0x40U);
  EXPECT_EQ(heartbeat.axis_state, 1U);
  EXPECT_EQ(heartbeat.flags, 0x89U);
  EXPECT_EQ(heartbeat.life, 0xF4U);
}

TEST(CanDiagnostics, DecodesDetailedErrorsLittleEndian)
{
  const std::array<std::uint8_t, 8> motor{{0x00, 0x00, 0x00, 0x01, 0, 0, 0, 0}};
  const std::array<std::uint8_t, 4> system{{0x02, 0x00, 0x00, 0x00}};
  EXPECT_EQ(
    gim6010_driver::decodeError(
      motor.data(), motor.size(), gim6010_driver::ErrorType::kMotor),
    0x01000000U);
  EXPECT_EQ(
    gim6010_driver::decodeError(
      system.data(), system.size(), gim6010_driver::ErrorType::kSystem),
    0x02U);
}

TEST(CanDiagnostics, DecodesBusVoltageAndCurrent)
{
  const std::array<std::uint8_t, 8> data{{0xA8, 0x05, 0xC0, 0x41, 0, 0, 0, 0}};
  const auto bus = gim6010_driver::decodeBusVoltageCurrent(data.data(), data.size());
  EXPECT_NEAR(bus.voltage, 24.003, 0.001);
  EXPECT_FLOAT_EQ(bus.current, 0.0F);
}
