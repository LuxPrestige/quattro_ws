#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <stdexcept>

#include "gim6010_driver/mit_protocol.hpp"

using gim6010_driver::MitCommand;
using gim6010_driver::MitLimits;

TEST(MitProtocol, FloatToUintAndBackRoundTripsAtRangeEndpoints)
{
  EXPECT_EQ(gim6010_driver::floatToUint(0.0, 0.0, 500.0, 12), 0U);
  EXPECT_EQ(gim6010_driver::floatToUint(500.0, 0.0, 500.0, 12), 4095U);
  EXPECT_DOUBLE_EQ(gim6010_driver::uintToFloat(0U, 0.0, 500.0, 12), 0.0);
  EXPECT_DOUBLE_EQ(gim6010_driver::uintToFloat(4095U, 0.0, 500.0, 12), 500.0);
}

TEST(MitProtocol, ValidateCommandAcceptsBoundaryValues)
{
  const MitCommand command{
    MitLimits::position_max, MitLimits::velocity_min, MitLimits::kp_max, MitLimits::kd_min,
    MitLimits::torque_max};
  EXPECT_NO_THROW(gim6010_driver::validateCommand(command));
}

TEST(MitProtocol, ValidateCommandRejectsOutOfRangeField)
{
  const MitCommand command{MitLimits::position_max + 0.1, 0.0, 0.0, 0.0, 0.0};
  EXPECT_THROW(gim6010_driver::validateCommand(command), std::out_of_range);
}

TEST(MitProtocol, EncodeCommandPacksFieldsMsbFirstAtTheirDocumentedWidths)
{
  // position=0 -> code 0x8000 (16 bits), velocity=0 -> code 0x800 (12 bits),
  // kp=0 -> code 0 (12 bits, range starts at 0), kd=0 -> code 0 (12 bits),
  // torque=0 -> code 0x800 (12 bits). Hand-computed from
  // MitLimits/floatToUint to pin down the exact bit layout.
  const MitCommand command{0.0, 0.0, 0.0, 0.0, 0.0};
  const auto encoded = gim6010_driver::encodeCommand(command);
  const std::array<std::uint8_t, 8> expected{
    0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00};
  EXPECT_EQ(encoded, expected);
}

TEST(MitProtocol, EncodeCommandRejectsOutOfRangeField)
{
  const MitCommand command{0.0, 0.0, MitLimits::kp_max + 1.0, 0.0, 0.0};
  EXPECT_THROW(gim6010_driver::encodeCommand(command), std::out_of_range);
}

TEST(MitProtocol, DecodeFeedbackAtMinimumCodesReturnsMinimumValues)
{
  const std::array<std::uint8_t, 8> raw{9, 0x00, 0x00, 0x00, 0x00, 0x00, 0, 0};
  const auto feedback = gim6010_driver::decodeFeedback(raw.data(), raw.size());
  EXPECT_EQ(feedback.motor_id, 9U);
  EXPECT_DOUBLE_EQ(feedback.output_position_rad, MitLimits::position_min);
  EXPECT_DOUBLE_EQ(feedback.output_velocity_rad_s, MitLimits::velocity_min);
  EXPECT_DOUBLE_EQ(feedback.output_torque_nm, MitLimits::torque_min);
}

TEST(MitProtocol, DecodeFeedbackAtMaximumCodesReturnsMaximumValues)
{
  const std::array<std::uint8_t, 8> raw{3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0};
  const auto feedback = gim6010_driver::decodeFeedback(raw.data(), raw.size());
  EXPECT_EQ(feedback.motor_id, 3U);
  EXPECT_DOUBLE_EQ(feedback.output_position_rad, MitLimits::position_max);
  EXPECT_DOUBLE_EQ(feedback.output_velocity_rad_s, MitLimits::velocity_max);
  EXPECT_DOUBLE_EQ(feedback.output_torque_nm, MitLimits::torque_max);
}

TEST(MitProtocol, DecodeFeedbackRejectsShortPayload)
{
  const std::array<std::uint8_t, 5> raw{0, 0, 0, 0, 0};
  EXPECT_THROW(gim6010_driver::decodeFeedback(raw.data(), raw.size()), std::length_error);
}
