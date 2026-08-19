#ifndef QUATTRO_HARDWARE__JOINT_TRANSFORM_HPP_
#define QUATTRO_HARDWARE__JOINT_TRANSFORM_HPP_

namespace quattro_hardware
{

// Per-joint calibration: direction (+1/-1), zero-offset (rad), and gear
// ratio between motor rotor and joint output shaft. Pure data, no ROS or
// CAN dependency -- read from calibration.yaml via the URDF at on_init.
struct JointCalibration
{
  double direction{1.0};
  double offset{0.0};
  double gear_ratio{8.0};
};

// joint_rad = direction * (motor_rev * 2*pi / gear_ratio) - offset
double motor_rev_to_joint_rad(double motor_rev, const JointCalibration & calibration);
// motor_rev = direction * (joint_rad + offset) * gear_ratio / (2*pi)
double joint_rad_to_motor_rev(double joint_rad, const JointCalibration & calibration);

double motor_rev_s_to_joint_rad_s(double motor_rev_s, const JointCalibration & calibration);
double joint_rad_s_to_motor_rev_s(double joint_rad_s, const JointCalibration & calibration);

// Torque scales the opposite way through the gearbox from position/velocity:
// joint_Nm = direction * motor_Nm * gear_ratio
double motor_Nm_to_joint_Nm(double motor_Nm, const JointCalibration & calibration);
// motor_Nm = direction * joint_Nm / gear_ratio
double joint_Nm_to_motor_Nm(double joint_Nm, const JointCalibration & calibration);

// --- MIT-domain transforms -------------------------------------------------
// GIM6010-8's MIT motion control (cmd 0x08) reports position/velocity/torque
// already referred to the OUTPUT shaft -- the 8:1 gearbox is applied inside
// the firmware, unlike Get_Encoder_Estimates/Set_Input_Pos which are raw
// rotor units (docs/packages/gim6010_driver.md section 2). gear_ratio must
// NOT be applied a second time here. direction/offset still apply: those
// encode this robot's calibration/mounting convention, which the firmware
// knows nothing about.

// joint_rad = direction * mit_output_rad - offset
double mit_output_rad_to_joint_rad(double mit_output_rad, const JointCalibration & calibration);
// mit_output_rad = direction * (joint_rad + offset)
double joint_rad_to_mit_output_rad(double joint_rad, const JointCalibration & calibration);

// joint_rad_s = direction * mit_output_rad_s
double mit_output_rad_s_to_joint_rad_s(
  double mit_output_rad_s, const JointCalibration & calibration);
// mit_output_rad_s = direction * joint_rad_s
double joint_rad_s_to_mit_output_rad_s(
  double joint_rad_s, const JointCalibration & calibration);

// joint_Nm = direction * mit_output_Nm  (no gear_ratio: already output-referred)
double mit_output_Nm_to_joint_Nm(double mit_output_Nm, const JointCalibration & calibration);
// mit_output_Nm = direction * joint_Nm
double joint_Nm_to_mit_output_Nm(double joint_Nm, const JointCalibration & calibration);

}  // namespace quattro_hardware

#endif  // QUATTRO_HARDWARE__JOINT_TRANSFORM_HPP_
