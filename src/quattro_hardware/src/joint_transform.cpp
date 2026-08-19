#include "quattro_hardware/joint_transform.hpp"

namespace quattro_hardware
{

namespace
{
// Not std::numbers::pi (C++20) -- this project targets C++17 (AGENTS.md).
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
}  // namespace

double motor_rev_to_joint_rad(double motor_rev, const JointCalibration & calibration)
{
  return calibration.direction * (motor_rev * kTwoPi / calibration.gear_ratio) - calibration.offset;
}

double joint_rad_to_motor_rev(double joint_rad, const JointCalibration & calibration)
{
  return calibration.direction * (joint_rad + calibration.offset) * calibration.gear_ratio / kTwoPi;
}

double motor_rev_s_to_joint_rad_s(double motor_rev_s, const JointCalibration & calibration)
{
  return calibration.direction * motor_rev_s * kTwoPi / calibration.gear_ratio;
}

double joint_rad_s_to_motor_rev_s(double joint_rad_s, const JointCalibration & calibration)
{
  return calibration.direction * joint_rad_s * calibration.gear_ratio / kTwoPi;
}

double motor_Nm_to_joint_Nm(double motor_Nm, const JointCalibration & calibration)
{
  return calibration.direction * motor_Nm * calibration.gear_ratio;
}

double joint_Nm_to_motor_Nm(double joint_Nm, const JointCalibration & calibration)
{
  return calibration.direction * joint_Nm / calibration.gear_ratio;
}

double mit_output_rad_to_joint_rad(double mit_output_rad, const JointCalibration & calibration)
{
  return calibration.direction * mit_output_rad - calibration.offset;
}

double joint_rad_to_mit_output_rad(double joint_rad, const JointCalibration & calibration)
{
  return calibration.direction * (joint_rad + calibration.offset);
}

double mit_output_rad_s_to_joint_rad_s(
  double mit_output_rad_s, const JointCalibration & calibration)
{
  return calibration.direction * mit_output_rad_s;
}

double joint_rad_s_to_mit_output_rad_s(double joint_rad_s, const JointCalibration & calibration)
{
  return calibration.direction * joint_rad_s;
}

double mit_output_Nm_to_joint_Nm(double mit_output_Nm, const JointCalibration & calibration)
{
  return calibration.direction * mit_output_Nm;
}

double joint_Nm_to_mit_output_Nm(double joint_Nm, const JointCalibration & calibration)
{
  return calibration.direction * joint_Nm;
}

}  // namespace quattro_hardware
