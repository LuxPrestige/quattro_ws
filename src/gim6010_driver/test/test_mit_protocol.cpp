#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include "gim6010_driver/mit_protocol.hpp"

using gim6010_driver::MitCommand;

TEST(MitProtocol, EncodesNeutralCommand)
{
  const auto data = gim6010_driver::encodeCommand(MitCommand{});
  EXPECT_EQ(data[0], 0x7F);
  EXPECT_EQ(data[1], 0xFF);
  EXPECT_EQ(data[2], 0x7F);
  EXPECT_EQ(data[3], 0xF0);
  EXPECT_EQ(data[4], 0x00);
  EXPECT_EQ(data[5], 0x00);
  EXPECT_EQ(data[6], 0x07);
  EXPECT_EQ(data[7], 0xFF);
}

TEST(MitProtocol, DecodesFeedbackLayout)
{
  const std::array<std::uint8_t, 6> data{{5, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF}};
  const auto feedback = gim6010_driver::decodeFeedback(data.data(), data.size());
  EXPECT_EQ(feedback.motor_id, 5);
  EXPECT_NEAR(feedback.position, 0.0, 0.001);
  EXPECT_NEAR(feedback.velocity, 0.0, 0.04);
  EXPECT_NEAR(feedback.torque, 0.0, 0.03);
}

TEST(MitProtocol, RejectsUnsafeInputInsteadOfClamping)
{
  MitCommand command;
  command.position = 12.6;
  EXPECT_THROW(gim6010_driver::encodeCommand(command), std::out_of_range);
  command.position = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(gim6010_driver::encodeCommand(command), std::out_of_range);
}
