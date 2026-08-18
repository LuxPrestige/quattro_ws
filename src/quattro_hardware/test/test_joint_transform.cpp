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
