#include "gtest/gtest.h"
#include "quattro_hardware/joint_transform.hpp"

TEST(JointTransform, PositiveDirectionRoundTrip)
{
  quattro_hardware::JointTransform transform(1.0, 0.25);
  EXPECT_DOUBLE_EQ(transform.toJointPosition(1.0), 0.75);
  EXPECT_DOUBLE_EQ(transform.toOutputPosition(0.75), 1.0);
}

TEST(JointTransform, NegativeDirectionRoundTrip)
{
  quattro_hardware::JointTransform transform(-1.0, 0.25);
  EXPECT_DOUBLE_EQ(transform.toJointPosition(1.0), -1.25);
  EXPECT_DOUBLE_EQ(transform.toOutputPosition(-1.25), 1.0);
  EXPECT_DOUBLE_EQ(transform.toJointVelocity(2.0), -2.0);
  EXPECT_DOUBLE_EQ(transform.toOutputEffort(3.0), -3.0);
}

TEST(JointTransform, NegativeOffsetRoundTripsForBothDirections)
{
  for (const double direction : {1.0, -1.0}) {
    quattro_hardware::JointTransform transform(direction, -0.4);
    for (const double output_position : {-2.0, 0.0, 1.5}) {
      const double joint_position = transform.toJointPosition(output_position);
      EXPECT_DOUBLE_EQ(transform.toOutputPosition(joint_position), output_position);
    }
  }
}

TEST(JointTransform, VelocityIgnoresOffset)
{
  // Section 3/16 of the calibration convention: offset only shifts
  // position, never velocity.
  quattro_hardware::JointTransform positive(1.0, 5.0);
  quattro_hardware::JointTransform negative(-1.0, -5.0);
  EXPECT_DOUBLE_EQ(positive.toJointVelocity(3.0), 3.0);
  EXPECT_DOUBLE_EQ(positive.toOutputVelocity(3.0), 3.0);
  EXPECT_DOUBLE_EQ(negative.toJointVelocity(3.0), -3.0);
  EXPECT_DOUBLE_EQ(negative.toOutputVelocity(-3.0), 3.0);
}
