#ifndef GIM6010_DRIVER__TYPES_HPP_
#define GIM6010_DRIVER__TYPES_HPP_

namespace gim6010_driver
{

enum class MotorControlMode
{
  kUnconfigured,
  kPosition,
  kVelocity,
  kTorque,
  kMit,
};

enum class PositionInputMode
{
  kDirect,
  kPositionFilter,
  kTrapezoidalTrajectory,
};

struct PositionControlGains
{
  double position_gain{0.0};
  double velocity_gain{0.0};
  double velocity_integrator_gain{0.0};
};

struct MitControlGains
{
  double kp{0.0};
  double kd{0.0};
};

struct TrapezoidalTrajectoryLimits
{
  double velocity_rev_s{0.0};
  double acceleration_rev_s2{0.0};
  double deceleration_rev_s2{0.0};
};

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__TYPES_HPP_
