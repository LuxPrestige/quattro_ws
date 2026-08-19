#include "gim6010_driver/mit_protocol.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace gim6010_driver
{
namespace
{
void checkRange(double value, double minimum, double maximum, const char * name)
{
  if (value < minimum || value > maximum) {
    throw std::out_of_range(std::string(name) + " is outside the MIT command range");
  }
}
}  // namespace

std::uint32_t floatToUint(double value, double minimum, double maximum, unsigned bits)
{
  const double clamped = std::clamp(value, minimum, maximum);
  const double span = maximum - minimum;
  const auto max_code = static_cast<double>((1U << bits) - 1U);
  return static_cast<std::uint32_t>((clamped - minimum) * max_code / span + 0.5);
}

double uintToFloat(std::uint32_t value, double minimum, double maximum, unsigned bits)
{
  const double span = maximum - minimum;
  const auto max_code = static_cast<double>((1U << bits) - 1U);
  return static_cast<double>(value) * span / max_code + minimum;
}

void validateCommand(const MitCommand & command)
{
  checkRange(
    command.output_position_rad, MitLimits::position_min, MitLimits::position_max, "position");
  checkRange(
    command.output_velocity_rad_s, MitLimits::velocity_min, MitLimits::velocity_max, "velocity");
  checkRange(command.kp, MitLimits::kp_min, MitLimits::kp_max, "kp");
  checkRange(command.kd, MitLimits::kd_min, MitLimits::kd_max, "kd");
  checkRange(
    command.output_torque_nm, MitLimits::torque_min, MitLimits::torque_max, "torque");
}

std::array<std::uint8_t, 8> encodeCommand(const MitCommand & command)
{
  validateCommand(command);
  const std::uint32_t position = floatToUint(
    command.output_position_rad, MitLimits::position_min, MitLimits::position_max, 16);
  const std::uint32_t velocity = floatToUint(
    command.output_velocity_rad_s, MitLimits::velocity_min, MitLimits::velocity_max, 12);
  const std::uint32_t kp = floatToUint(command.kp, MitLimits::kp_min, MitLimits::kp_max, 12);
  const std::uint32_t kd = floatToUint(command.kd, MitLimits::kd_min, MitLimits::kd_max, 12);
  const std::uint32_t torque = floatToUint(
    command.output_torque_nm, MitLimits::torque_min, MitLimits::torque_max, 12);

  std::array<std::uint8_t, 8> out{};
  out[0] = static_cast<std::uint8_t>((position >> 8) & 0xFFU);
  out[1] = static_cast<std::uint8_t>(position & 0xFFU);
  out[2] = static_cast<std::uint8_t>((velocity >> 4) & 0xFFU);
  out[3] = static_cast<std::uint8_t>(((velocity & 0x0FU) << 4) | ((kp >> 8) & 0x0FU));
  out[4] = static_cast<std::uint8_t>(kp & 0xFFU);
  out[5] = static_cast<std::uint8_t>((kd >> 4) & 0xFFU);
  out[6] = static_cast<std::uint8_t>(((kd & 0x0FU) << 4) | ((torque >> 8) & 0x0FU));
  out[7] = static_cast<std::uint8_t>(torque & 0xFFU);
  return out;
}

MitFeedback decodeFeedback(const std::uint8_t * data, std::size_t length)
{
  if (length < 6) {
    throw std::length_error("MIT feedback payload is too short");
  }
  const std::uint32_t position = (static_cast<std::uint32_t>(data[1]) << 8) | data[2];
  const std::uint32_t velocity =
    (static_cast<std::uint32_t>(data[3]) << 4) | (data[4] >> 4);
  const std::uint32_t torque =
    ((static_cast<std::uint32_t>(data[4]) & 0x0FU) << 8) | data[5];

  MitFeedback feedback;
  feedback.motor_id = data[0];
  feedback.output_position_rad = uintToFloat(
    position, MitLimits::position_min, MitLimits::position_max, 16);
  feedback.output_velocity_rad_s = uintToFloat(
    velocity, MitLimits::velocity_min, MitLimits::velocity_max, 12);
  feedback.output_torque_nm = uintToFloat(
    torque, MitLimits::torque_min, MitLimits::torque_max, 12);
  return feedback;
}

}  // namespace gim6010_driver
