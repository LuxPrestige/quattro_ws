#ifndef GIM6010_DRIVER__GDS68_PROTOCOL_HPP_
#define GIM6010_DRIVER__GDS68_PROTOCOL_HPP_

#include <cstdint>
#include <array>
#include <utility>

namespace gim6010_driver
{

enum class Gds68Command : std::uint8_t
{
  kHeartbeat = 0x01,
  kEstop = 0x02,
  kGetError = 0x03,
  kSetAxisState = 0x07,
  kMitControl = 0x08,
  kEncoderEstimates = 0x09,
  kSetControllerMode = 0x0B,
  kSetInputPosition = 0x0C,
  kSetInputVelocity = 0x0D,
  kSetInputTorque = 0x0E,
  kSetLimits = 0x0F,
  kSetTrajectoryVelocityLimit = 0x11,
  kSetTrajectoryAccelerationLimits = 0x12,
  kIq = 0x14,
  kBusVoltageCurrent = 0x17,
  kClearErrors = 0x18,
  kSetPositionGain = 0x1A,
  kSetVelocityGains = 0x1B,
};

enum class ControlMode : std::uint32_t
{
  kTorque = 1,
  kVelocity = 2,
  kPosition = 3,
};

enum class InputMode : std::uint32_t
{
  kDirect = 1,
  kPositionFilter = 3,
  kTrapezoidalTrajectory = 5,
  kMit = 9,
};

std::array<std::uint8_t, 8> encodeControllerMode(ControlMode control, InputMode input);
std::array<std::uint8_t, 8> encodeDirectPosition(
  float rotor_position_rev, float velocity_feedforward_rev_s = 0.0F,
  float torque_feedforward_nm = 0.0F);
std::array<std::uint8_t, 8> encodeDirectVelocity(
  float rotor_velocity_rev_s, float torque_feedforward_nm = 0.0F);
std::array<std::uint8_t, 4> encodeDirectTorque(float motor_torque_nm);
std::array<std::uint8_t, 4> encodePositionGain(float position_gain);
std::array<std::uint8_t, 8> encodeVelocityGains(
  float velocity_gain, float velocity_integrator_gain);
std::array<std::uint8_t, 4> encodeTrajectoryVelocityLimit(float rotor_velocity_rev_s);
std::array<std::uint8_t, 8> encodeTrajectoryAccelerationLimits(
  float rotor_acceleration_rev_s2, float rotor_deceleration_rev_s2);

constexpr std::uint8_t kMaximumNodeId = 63;

std::uint32_t makeArbitrationId(std::uint8_t node_id, Gds68Command command);
std::pair<std::uint8_t, Gds68Command> parseArbitrationId(std::uint32_t arbitration_id);

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__GDS68_PROTOCOL_HPP_
