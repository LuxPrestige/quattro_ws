#ifndef GIM6010_DRIVER__TYPES_HPP_
#define GIM6010_DRIVER__TYPES_HPP_

#include <cstdint>

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

enum class AxisState : std::uint32_t
{
  kIdle = 1,
  kClosedLoopControl = 8,
};

// Get_Error (0x03) category selector. Values match the manual's Error_Type
// table exactly (index 2 is not assigned by the manual and is intentionally
// absent here) -- do not renumber these as a plain sequential enum.
enum class ErrorType : std::uint8_t
{
  kMotor = 0,
  kEncoder = 1,
  kController = 3,
  kSystem = 4,
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
