#include "gim6010_driver/gds68_protocol.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace gim6010_driver
{
namespace
{
void putU32LE(std::uint8_t * out, std::uint32_t value)
{
  out[0] = static_cast<std::uint8_t>(value & 0xFFU);
  out[1] = static_cast<std::uint8_t>((value >> 8) & 0xFFU);
  out[2] = static_cast<std::uint8_t>((value >> 16) & 0xFFU);
  out[3] = static_cast<std::uint8_t>((value >> 24) & 0xFFU);
}

std::uint32_t getU32LE(const std::uint8_t * in)
{
  return static_cast<std::uint32_t>(in[0]) |
         (static_cast<std::uint32_t>(in[1]) << 8) |
         (static_cast<std::uint32_t>(in[2]) << 16) |
         (static_cast<std::uint32_t>(in[3]) << 24);
}

void putI16LE(std::uint8_t * out, std::int16_t value)
{
  const auto raw = static_cast<std::uint16_t>(value);
  out[0] = static_cast<std::uint8_t>(raw & 0xFFU);
  out[1] = static_cast<std::uint8_t>((raw >> 8) & 0xFFU);
}

void putF32LE(std::uint8_t * out, float value)
{
  std::uint32_t bits{};
  std::memcpy(&bits, &value, sizeof(bits));
  putU32LE(out, bits);
}

float getF32LE(const std::uint8_t * in)
{
  const std::uint32_t bits = getU32LE(in);
  float value{};
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void requireLength(std::size_t length, std::size_t minimum, const char * what)
{
  if (length < minimum) {
    throw std::length_error(std::string(what) + " payload is too short");
  }
}
}  // namespace

std::uint32_t makeArbitrationId(std::uint8_t node_id, Gds68Command command)
{
  if (node_id > kMaximumNodeId) {
    throw std::invalid_argument("node_id exceeds the CAN Simple 6-bit range");
  }
  return (static_cast<std::uint32_t>(node_id) << 5) | static_cast<std::uint32_t>(command);
}

std::pair<std::uint8_t, Gds68Command> parseArbitrationId(std::uint32_t arbitration_id)
{
  const auto node_id = static_cast<std::uint8_t>((arbitration_id >> 5) & 0x3FU);
  const auto command = static_cast<Gds68Command>(arbitration_id & 0x1FU);
  return {node_id, command};
}

std::array<std::uint8_t, 4> encodeAxisState(std::uint32_t requested_state)
{
  std::array<std::uint8_t, 4> out{};
  putU32LE(out.data(), requested_state);
  return out;
}

std::array<std::uint8_t, 1> encodeGetErrorRequest(ErrorType error_type)
{
  return {static_cast<std::uint8_t>(error_type)};
}

std::uint64_t decodeGetErrorResponse(
  ErrorType error_type, const std::uint8_t * data, std::size_t length)
{
  if (error_type == ErrorType::kMotor) {
    requireLength(length, 8, "Get_Error motor response");
    const std::uint64_t low = getU32LE(data);
    const std::uint64_t high = getU32LE(data + 4);
    return low | (high << 32);
  }
  requireLength(length, 4, "Get_Error response");
  return getU32LE(data);
}

std::array<std::uint8_t, 8> encodeControllerMode(ControlMode control, InputMode input)
{
  std::array<std::uint8_t, 8> out{};
  putU32LE(out.data(), static_cast<std::uint32_t>(control));
  putU32LE(out.data() + 4, static_cast<std::uint32_t>(input));
  return out;
}

std::array<std::uint8_t, 8> encodeDirectPosition(
  float rotor_position_rev, float velocity_feedforward_rev_s, float torque_feedforward_nm)
{
  std::array<std::uint8_t, 8> out{};
  putF32LE(out.data(), rotor_position_rev);
  putI16LE(out.data() + 4,
      static_cast<std::int16_t>(std::lround(velocity_feedforward_rev_s * 1000.0F)));
  putI16LE(out.data() + 6, static_cast<std::int16_t>(std::lround(torque_feedforward_nm * 1000.0F)));
  return out;
}

std::array<std::uint8_t, 8> encodeDirectVelocity(
  float rotor_velocity_rev_s, float torque_feedforward_nm)
{
  std::array<std::uint8_t, 8> out{};
  putF32LE(out.data(), rotor_velocity_rev_s);
  putF32LE(out.data() + 4, torque_feedforward_nm);
  return out;
}

std::array<std::uint8_t, 4> encodeDirectTorque(float motor_torque_nm)
{
  std::array<std::uint8_t, 4> out{};
  putF32LE(out.data(), motor_torque_nm);
  return out;
}

std::pair<float, float> decodeEncoderEstimates(const std::uint8_t * data, std::size_t length)
{
  requireLength(length, 8, "Get_Encoder_Estimates response");
  return {getF32LE(data), getF32LE(data + 4)};
}

EncoderCount decodeEncoderCount(const std::uint8_t * data, std::size_t length)
{
  requireLength(length, 8, "Get_Encoder_Count response");
  EncoderCount count;
  count.shadow_count = static_cast<std::int32_t>(getU32LE(data));
  count.count_in_cpr = static_cast<std::int32_t>(getU32LE(data + 4));
  return count;
}

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
}  // namespace

double rotorRevToOutputRad(double rotor_rev, double gear_ratio)
{
  return rotor_rev * kTwoPi / gear_ratio;
}

double rotorRevPerSecToOutputRadPerSec(double rotor_rev_s, double gear_ratio)
{
  return rotor_rev_s * kTwoPi / gear_ratio;
}

std::array<std::uint8_t, 8> encodeLimits(float velocity_limit_rev_s, float current_limit_a)
{
  std::array<std::uint8_t, 8> out{};
  putF32LE(out.data(), velocity_limit_rev_s);
  putF32LE(out.data() + 4, current_limit_a);
  return out;
}

std::array<std::uint8_t, 4> encodePositionGain(float position_gain)
{
  std::array<std::uint8_t, 4> out{};
  putF32LE(out.data(), position_gain);
  return out;
}

std::array<std::uint8_t, 8> encodeVelocityGains(float velocity_gain, float velocity_integrator_gain)
{
  std::array<std::uint8_t, 8> out{};
  putF32LE(out.data(), velocity_gain);
  putF32LE(out.data() + 4, velocity_integrator_gain);
  return out;
}

std::array<std::uint8_t, 4> encodeTrajectoryVelocityLimit(float rotor_velocity_rev_s)
{
  std::array<std::uint8_t, 4> out{};
  putF32LE(out.data(), rotor_velocity_rev_s);
  return out;
}

std::array<std::uint8_t, 8> encodeTrajectoryAccelerationLimits(
  float rotor_acceleration_rev_s2, float rotor_deceleration_rev_s2)
{
  std::array<std::uint8_t, 8> out{};
  putF32LE(out.data(), rotor_acceleration_rev_s2);
  putF32LE(out.data() + 4, rotor_deceleration_rev_s2);
  return out;
}

std::pair<float, float> decodeIq(const std::uint8_t * data, std::size_t length)
{
  requireLength(length, 8, "Get_Iq response");
  return {getF32LE(data), getF32LE(data + 4)};
}

std::pair<float, float> decodeBusVoltageCurrent(const std::uint8_t * data, std::size_t length)
{
  requireLength(length, 8, "Get_Bus_Voltage_Current response");
  return {getF32LE(data), getF32LE(data + 4)};
}

Heartbeat decodeHeartbeat(const std::uint8_t * data, std::size_t length)
{
  requireLength(length, 8, "Heartbeat");
  Heartbeat heartbeat;
  heartbeat.axis_error = getU32LE(data);
  heartbeat.axis_state = data[4];
  heartbeat.flags = data[5];
  heartbeat.life = data[7];
  return heartbeat;
}

}  // namespace gim6010_driver
