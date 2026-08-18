#include <stdexcept>
#include <cstring>

#include "gim6010_driver/gds68_protocol.hpp"
#include "gtest/gtest.h"

TEST(Gds68Protocol, UsesDocumentedCanSimpleIdentifierLayout)
{
  EXPECT_EQ(gim6010_driver::makeArbitrationId(
      5, gim6010_driver::Gds68Command::kSetAxisState), 0xA7U);
  const auto parsed = gim6010_driver::parseArbitrationId(0xA8U);
  EXPECT_EQ(parsed.first, 5U);
  EXPECT_EQ(parsed.second, gim6010_driver::Gds68Command::kMitControl);
}

TEST(Gds68Protocol, EncodesAllDocumentedDirectControlPayloads)
{
  const auto mode = gim6010_driver::encodeControllerMode(
    gim6010_driver::ControlMode::kPosition, gim6010_driver::InputMode::kDirect);
  EXPECT_EQ(mode[0], 3U);
  EXPECT_EQ(mode[4], 1U);

  const auto position = gim6010_driver::encodeDirectPosition(3.14F, 1.0F, 5.0F);
  float position_value = 0.0F;
  std::memcpy(&position_value, position.data(), sizeof(float));
  EXPECT_FLOAT_EQ(position_value, 3.14F);
  EXPECT_EQ(position[4], 0xE8U);
  EXPECT_EQ(position[5], 0x03U);
  EXPECT_EQ(position[6], 0x88U);
  EXPECT_EQ(position[7], 0x13U);

  const auto velocity = gim6010_driver::encodeDirectVelocity(10.0F, 2.0F);
  float velocity_value = 0.0F;
  float velocity_torque = 0.0F;
  std::memcpy(&velocity_value, velocity.data(), sizeof(float));
  std::memcpy(&velocity_torque, velocity.data() + 4, sizeof(float));
  EXPECT_FLOAT_EQ(velocity_value, 10.0F);
  EXPECT_FLOAT_EQ(velocity_torque, 2.0F);

  const auto torque = gim6010_driver::encodeDirectTorque(-3.0F);
  float torque_value = 0.0F;
  std::memcpy(&torque_value, torque.data(), sizeof(float));
  EXPECT_FLOAT_EQ(torque_value, -3.0F);
}

TEST(Gds68Protocol, RejectsNodeIdsOutsideSixBitField)
{
  EXPECT_THROW(gim6010_driver::makeArbitrationId(
      64, gim6010_driver::Gds68Command::kHeartbeat), std::invalid_argument);
}

TEST(Gds68Protocol, EncodesPositionControllerGainsSeparatelyFromMitGains)
{
  const auto position = gim6010_driver::encodePositionGain(20.0F);
  const auto velocity = gim6010_driver::encodeVelocityGains(0.16F, 0.32F);
  float decoded_position = 0.0F;
  float decoded_velocity = 0.0F;
  float decoded_integrator = 0.0F;
  std::memcpy(&decoded_position, position.data(), sizeof(float));
  std::memcpy(&decoded_velocity, velocity.data(), sizeof(float));
  std::memcpy(&decoded_integrator, velocity.data() + 4, sizeof(float));
  EXPECT_FLOAT_EQ(decoded_position, 20.0F);
  EXPECT_FLOAT_EQ(decoded_velocity, 0.16F);
  EXPECT_FLOAT_EQ(decoded_integrator, 0.32F);
  EXPECT_THROW(gim6010_driver::encodePositionGain(-1.0F), std::out_of_range);
}

TEST(Gds68Protocol, EncodesPositionFilterAndTrajectoryModes)
{
  const auto filter = gim6010_driver::encodeControllerMode(
    gim6010_driver::ControlMode::kPosition,
    gim6010_driver::InputMode::kPositionFilter);
  const auto trajectory = gim6010_driver::encodeControllerMode(
    gim6010_driver::ControlMode::kPosition,
    gim6010_driver::InputMode::kTrapezoidalTrajectory);
  EXPECT_EQ(filter[4], 3U);
  EXPECT_EQ(trajectory[4], 5U);
}

TEST(Gds68Protocol, EncodesTrapezoidalTrajectoryLimits)
{
  const auto velocity = gim6010_driver::encodeTrajectoryVelocityLimit(4.0F);
  const auto acceleration =
    gim6010_driver::encodeTrajectoryAccelerationLimits(6.0F, 7.0F);
  float decoded_velocity = 0.0F;
  float decoded_acceleration = 0.0F;
  float decoded_deceleration = 0.0F;
  std::memcpy(&decoded_velocity, velocity.data(), sizeof(float));
  std::memcpy(&decoded_acceleration, acceleration.data(), sizeof(float));
  std::memcpy(&decoded_deceleration, acceleration.data() + 4, sizeof(float));
  EXPECT_FLOAT_EQ(decoded_velocity, 4.0F);
  EXPECT_FLOAT_EQ(decoded_acceleration, 6.0F);
  EXPECT_FLOAT_EQ(decoded_deceleration, 7.0F);
  EXPECT_THROW(
    gim6010_driver::encodeTrajectoryVelocityLimit(-1.0F), std::out_of_range);
}
