#include <gtest/gtest.h>

#include "quattro_hardware/joint_transform.hpp"

namespace quattro_hardware
{
namespace
{

TEST(JointTransform, PositionRoundTripsThroughDirectionAndOffset)
{
  JointCalibration calibration{-1.0, 0.3, 8.0};
  const double motor_rev = 2.5;
  const double joint_rad = motor_rev_to_joint_rad(motor_rev, calibration);
  const double recovered = joint_rad_to_motor_rev(joint_rad, calibration);
  EXPECT_NEAR(recovered, motor_rev, 1e-12);
}

TEST(JointTransform, PositiveDirectionZeroOffsetIsPlainGearRatio)
{
  JointCalibration calibration{1.0, 0.0, 8.0};
  // One full motor revolution / 8:1 gearbox = 1/8 revolution at the joint.
  const double joint_rad = motor_rev_to_joint_rad(1.0, calibration);
  EXPECT_NEAR(joint_rad, 2.0 * 3.14159265358979323846 / 8.0, 1e-9);
}

TEST(JointTransform, OffsetShiftsZeroPosition)
{
  JointCalibration calibration{1.0, 0.5, 1.0};
  // motor_rev=0 -> joint_rad = -offset
  EXPECT_NEAR(motor_rev_to_joint_rad(0.0, calibration), -0.5, 1e-12);
  // The motor position that reads back as joint zero is offset/(2*pi) rev.
  EXPECT_NEAR(joint_rad_to_motor_rev(0.0, calibration), 0.5 / (2.0 * 3.14159265358979323846), 1e-9);
}

TEST(JointTransform, VelocityRoundTrips)
{
  JointCalibration calibration{-1.0, 0.0, 8.0};
  const double motor_rev_s = 1.2;
  const double joint_rad_s = motor_rev_s_to_joint_rad_s(motor_rev_s, calibration);
  EXPECT_NEAR(joint_rad_s_to_motor_rev_s(joint_rad_s, calibration), motor_rev_s, 1e-12);
}

TEST(JointTransform, TorqueScalesOppositeDirectionFromPositionThroughGearbox)
{
  JointCalibration calibration{1.0, 0.0, 8.0};
  // Gearbox multiplies torque by the gear ratio (opposite of position, which
  // divides by it) -- this is the sign/magnitude relationship most likely to
  // be gotten backwards, so pin it down explicitly.
  EXPECT_NEAR(motor_Nm_to_joint_Nm(1.0, calibration), 8.0, 1e-12);
  EXPECT_NEAR(joint_Nm_to_motor_Nm(8.0, calibration), 1.0, 1e-12);
}

TEST(JointTransform, TorqueRoundTrips)
{
  JointCalibration calibration{-1.0, 0.0, 8.0};
  const double joint_Nm = 3.7;
  const double motor_Nm = joint_Nm_to_motor_Nm(joint_Nm, calibration);
  EXPECT_NEAR(motor_Nm_to_joint_Nm(motor_Nm, calibration), joint_Nm, 1e-12);
}

// MIT (cmd 0x08) already reports output-shaft units -- gear_ratio must NOT
// be applied a second time, unlike the raw-rotor Get_Encoder_Estimates/
// Set_Input_Pos path above. This is the easiest place to accidentally
// double-apply (or drop) the gearbox factor, so pin the exact relationship.
TEST(JointTransform, MitDomainAppliesDirectionOffsetButNotGearRatio)
{
  JointCalibration calibration{-1.0, 0.2, 8.0};

  // Same numeric output-shaft value through the MIT transform and the
  // gear-ratio=1 rotor transform must agree, since MIT is already
  // gear-reduced (equivalent to a gear_ratio of 1 at this domain boundary).
  JointCalibration unity_gear{calibration.direction, calibration.offset, 1.0};
  const double value = 1.3;
  EXPECT_NEAR(
    mit_output_rad_to_joint_rad(value, calibration),
    motor_rev_to_joint_rad(value / (2.0 * 3.14159265358979323846), unity_gear), 1e-9);
}

TEST(JointTransform, MitPositionRoundTrips)
{
  JointCalibration calibration{1.0, -0.4, 8.0};
  const double joint_rad = 0.75;
  const double mit_output_rad = joint_rad_to_mit_output_rad(joint_rad, calibration);
  EXPECT_NEAR(mit_output_rad_to_joint_rad(mit_output_rad, calibration), joint_rad, 1e-12);
}

TEST(JointTransform, MitVelocityAndTorqueRoundTrip)
{
  JointCalibration calibration{-1.0, 0.0, 8.0};
  EXPECT_NEAR(
    mit_output_rad_s_to_joint_rad_s(joint_rad_s_to_mit_output_rad_s(2.0, calibration), calibration),
    2.0, 1e-12);
  EXPECT_NEAR(
    mit_output_Nm_to_joint_Nm(joint_Nm_to_mit_output_Nm(4.0, calibration), calibration), 4.0, 1e-12);
}

}  // namespace
}  // namespace quattro_hardware
