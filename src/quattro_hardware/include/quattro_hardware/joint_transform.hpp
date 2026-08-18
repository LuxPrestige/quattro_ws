#ifndef QUATTRO_HARDWARE__JOINT_TRANSFORM_HPP_
#define QUATTRO_HARDWARE__JOINT_TRANSFORM_HPP_

#include <cmath>
#include <stdexcept>

namespace quattro_hardware
{

class JointTransform
{
public:
  JointTransform(double direction, double offset)
  : direction_(direction), offset_(offset)
  {
    if ((direction != 1.0 && direction != -1.0) || !std::isfinite(offset)) {
      throw std::invalid_argument("invalid joint transform");
    }
  }

  double toJointPosition(double output_position) const
  {
    return direction_ * output_position - offset_;
  }
  double toOutputPosition(double joint_position) const
  {
    return direction_ * (joint_position + offset_);
  }
  double toJointVelocity(double output_velocity) const {return direction_ * output_velocity;}
  double toOutputVelocity(double joint_velocity) const {return direction_ * joint_velocity;}
  double toJointEffort(double output_effort) const {return direction_ * output_effort;}
  double toOutputEffort(double joint_effort) const {return direction_ * joint_effort;}

private:
  double direction_;
  double offset_;
};

}  // namespace quattro_hardware

#endif  // QUATTRO_HARDWARE__JOINT_TRANSFORM_HPP_
