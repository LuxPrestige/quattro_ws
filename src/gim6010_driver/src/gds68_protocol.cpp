#include "gim6010_driver/gds68_protocol.hpp"

#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

namespace gim6010_driver
{
namespace
{
template<typename T>
std::array<std::uint8_t, sizeof(T)> littleEndianBytes(T value)
{
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
  std::reverse(bytes.begin(), bytes.end());
#endif
  return bytes;
}

void requireFinite(float value, const char * name)
{
  if (!std::isfinite(value)) {throw std::invalid_argument(std::string(name) + " is not finite");}
}

void requireNonnegative(float value, const char * name)
{
  requireFinite(value, name);
  if (value < 0.0F) {throw std::out_of_range(std::string(name) + " must be nonnegative");}
}
}  // namespace

std::uint32_t makeArbitrationId(std::uint8_t node_id, Gds68Command command)
{
  const auto command_id = static_cast<std::uint8_t>(command);
  if (node_id > kMaximumNodeId || command_id > 0x1FU) {
    throw std::invalid_argument("GDS68 CAN Simple node or command ID is invalid");
  }
  return (static_cast<std::uint32_t>(node_id) << 5U) | command_id;
}

std::array<std::uint8_t, 8> encodeControllerMode(ControlMode control, InputMode input)
{
  std::array<std::uint8_t, 8> payload{};
  const auto control_bytes = littleEndianBytes(static_cast<std::uint32_t>(control));
  const auto input_bytes = littleEndianBytes(static_cast<std::uint32_t>(input));
  std::copy(control_bytes.begin(), control_bytes.end(), payload.begin());
  std::copy(input_bytes.begin(), input_bytes.end(), payload.begin() + 4);
  return payload;
}

std::array<std::uint8_t, 8> encodeDirectPosition(
  float position, float velocity_feedforward, float torque_feedforward)
{
  requireFinite(position, "position");
  requireFinite(velocity_feedforward, "velocity feedforward");
  requireFinite(torque_feedforward, "torque feedforward");
  const double velocity_scaled = std::round(static_cast<double>(velocity_feedforward) * 1000.0);
  const double torque_scaled = std::round(static_cast<double>(torque_feedforward) * 1000.0);
  if (velocity_scaled < std::numeric_limits<std::int16_t>::min() ||
    velocity_scaled > std::numeric_limits<std::int16_t>::max() ||
    torque_scaled < std::numeric_limits<std::int16_t>::min() ||
    torque_scaled > std::numeric_limits<std::int16_t>::max())
  {
    throw std::out_of_range("direct position feedforward exceeds int16 milli-unit range");
  }
  std::array<std::uint8_t, 8> payload{};
  const auto position_bytes = littleEndianBytes(position);
  const auto velocity_bytes = littleEndianBytes(static_cast<std::int16_t>(velocity_scaled));
  const auto torque_bytes = littleEndianBytes(static_cast<std::int16_t>(torque_scaled));
  std::copy(position_bytes.begin(), position_bytes.end(), payload.begin());
  std::copy(velocity_bytes.begin(), velocity_bytes.end(), payload.begin() + 4);
  std::copy(torque_bytes.begin(), torque_bytes.end(), payload.begin() + 6);
  return payload;
}

std::array<std::uint8_t, 8> encodeDirectVelocity(float velocity, float torque_feedforward)
{
  requireFinite(velocity, "velocity");
  requireFinite(torque_feedforward, "torque feedforward");
  std::array<std::uint8_t, 8> payload{};
  const auto velocity_bytes = littleEndianBytes(velocity);
  const auto torque_bytes = littleEndianBytes(torque_feedforward);
  std::copy(velocity_bytes.begin(), velocity_bytes.end(), payload.begin());
  std::copy(torque_bytes.begin(), torque_bytes.end(), payload.begin() + 4);
  return payload;
}

std::array<std::uint8_t, 4> encodeDirectTorque(float torque)
{
  requireFinite(torque, "torque");
  return littleEndianBytes(torque);
}

std::array<std::uint8_t, 4> encodePositionGain(float gain)
{
  requireNonnegative(gain, "position gain");
  return littleEndianBytes(gain);
}

std::array<std::uint8_t, 8> encodeVelocityGains(float gain, float integrator_gain)
{
  requireNonnegative(gain, "velocity gain");
  requireNonnegative(integrator_gain, "velocity integrator gain");
  std::array<std::uint8_t, 8> payload{};
  const auto gain_bytes = littleEndianBytes(gain);
  const auto integrator_bytes = littleEndianBytes(integrator_gain);
  std::copy(gain_bytes.begin(), gain_bytes.end(), payload.begin());
  std::copy(integrator_bytes.begin(), integrator_bytes.end(), payload.begin() + 4);
  return payload;
}

std::array<std::uint8_t, 4> encodeTrajectoryVelocityLimit(float velocity)
{
  requireNonnegative(velocity, "trajectory velocity limit");
  return littleEndianBytes(velocity);
}

std::array<std::uint8_t, 8> encodeTrajectoryAccelerationLimits(
  float acceleration, float deceleration)
{
  requireNonnegative(acceleration, "trajectory acceleration limit");
  requireNonnegative(deceleration, "trajectory deceleration limit");
  std::array<std::uint8_t, 8> payload{};
  const auto acceleration_bytes = littleEndianBytes(acceleration);
  const auto deceleration_bytes = littleEndianBytes(deceleration);
  std::copy(acceleration_bytes.begin(), acceleration_bytes.end(), payload.begin());
  std::copy(deceleration_bytes.begin(), deceleration_bytes.end(), payload.begin() + 4);
  return payload;
}

std::pair<std::uint8_t, Gds68Command> parseArbitrationId(std::uint32_t arbitration_id)
{
  if (arbitration_id > 0x7FFU) {
    throw std::invalid_argument("GDS68 requires an 11-bit standard CAN ID");
  }
  return {static_cast<std::uint8_t>(arbitration_id >> 5U),
    static_cast<Gds68Command>(arbitration_id & 0x1FU)};
}

}  // namespace gim6010_driver
