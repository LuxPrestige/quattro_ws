#include "gim6010_driver/mit_protocol.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace gim6010_driver
{
namespace
{
void requireRange(double value, double low, double high, const char * name)
{
  if (!std::isfinite(value) || value < low || value > high) {
    throw std::out_of_range(std::string(name) + " is outside the GIM6010 MIT range");
  }
}
}  // namespace

std::uint32_t floatToUint(double value, double minimum, double maximum, unsigned bits)
{
  if (!std::isfinite(value) || minimum >= maximum || bits == 0 || bits > 31) {
    throw std::invalid_argument("invalid quantization argument");
  }
  requireRange(value, minimum, maximum, "value");
  const auto maximum_integer = (std::uint32_t{1} << bits) - 1U;
  return static_cast<std::uint32_t>((value - minimum) * maximum_integer / (maximum - minimum));
}

double uintToFloat(std::uint32_t value, double minimum, double maximum, unsigned bits)
{
  if (minimum >= maximum || bits == 0 || bits > 31) {
    throw std::invalid_argument("invalid quantization argument");
  }
  const auto maximum_integer = (std::uint32_t{1} << bits) - 1U;
  if (value > maximum_integer) {throw std::out_of_range("quantized value exceeds bit width");}
  return static_cast<double>(value) * (maximum - minimum) / maximum_integer + minimum;
}

void validateCommand(const MitCommand & c)
{
  requireRange(c.position, MitLimits::position_min, MitLimits::position_max, "position");
  requireRange(c.velocity, MitLimits::velocity_min, MitLimits::velocity_max, "velocity");
  requireRange(c.kp, MitLimits::kp_min, MitLimits::kp_max, "kp");
  requireRange(c.kd, MitLimits::kd_min, MitLimits::kd_max, "kd");
  requireRange(c.torque, MitLimits::torque_min, MitLimits::torque_max, "torque");
}

std::array<std::uint8_t, 8> encodeCommand(const MitCommand & c)
{
  validateCommand(c);
  const auto p = floatToUint(c.position, -12.5, 12.5, 16);
  const auto v = floatToUint(c.velocity, -65.0, 65.0, 12);
  const auto kp = floatToUint(c.kp, 0.0, 500.0, 12);
  const auto kd = floatToUint(c.kd, 0.0, 5.0, 12);
  const auto t = floatToUint(c.torque, -50.0, 50.0, 12);
  return {{
    static_cast<std::uint8_t>(p >> 8), static_cast<std::uint8_t>(p),
    static_cast<std::uint8_t>(v >> 4), static_cast<std::uint8_t>((v << 4) | (kp >> 8)),
    static_cast<std::uint8_t>(kp), static_cast<std::uint8_t>(kd >> 4),
    static_cast<std::uint8_t>((kd << 4) | (t >> 8)), static_cast<std::uint8_t>(t)}};
}

MitFeedback decodeFeedback(const std::uint8_t * data, std::size_t length)
{
  if (data == nullptr || length < 6) {throw std::invalid_argument("MIT feedback requires 6 bytes");}
  const auto p = (static_cast<std::uint32_t>(data[1]) << 8) | data[2];
  const auto v = (static_cast<std::uint32_t>(data[3]) << 4) | (data[4] >> 4);
  const auto t = (static_cast<std::uint32_t>(data[4] & 0x0F) << 8) | data[5];
  return MitFeedback{
    data[0], uintToFloat(p, -12.5, 12.5, 16), uintToFloat(v, -65.0, 65.0, 12),
    uintToFloat(t, -50.0, 50.0, 12)};
}
}  // namespace gim6010_driver
