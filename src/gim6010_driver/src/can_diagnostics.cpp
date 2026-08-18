#include "gim6010_driver/can_diagnostics.hpp"

#include <cstring>
#include <stdexcept>

namespace gim6010_driver
{
namespace
{
std::uint32_t littleEndianUint32(const std::uint8_t * data)
{
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8U) |
         (static_cast<std::uint32_t>(data[2]) << 16U) |
         (static_cast<std::uint32_t>(data[3]) << 24U);
}
}  // namespace

Heartbeat decodeHeartbeat(const std::uint8_t * data, std::size_t length)
{
  if (data == nullptr || length != 8U) {
    throw std::invalid_argument("GIM6010 heartbeat requires 8 bytes");
  }
  return Heartbeat{
    littleEndianUint32(data), data[4], data[5], data[6], data[7]};
}

std::uint64_t decodeError(const std::uint8_t * data, std::size_t length, ErrorType type)
{
  const std::size_t expected_length = type == ErrorType::kMotor ? 8U : 4U;
  if (data == nullptr || length < expected_length) {
    throw std::invalid_argument("GIM6010 error response has invalid length");
  }
  std::uint64_t value = 0;
  for (std::size_t index = 0; index < expected_length; ++index) {
    value |= static_cast<std::uint64_t>(data[index]) << (8U * index);
  }
  return value;
}

BusVoltageCurrent decodeBusVoltageCurrent(const std::uint8_t * data, std::size_t length)
{
  if (data == nullptr || length != 8U) {
    throw std::invalid_argument("GIM6010 bus voltage/current response requires 8 bytes");
  }
  BusVoltageCurrent result;
  std::memcpy(&result.voltage, data, sizeof(float));
  std::memcpy(&result.current, data + sizeof(float), sizeof(float));
  return result;
}

IqFeedback decodeIqFeedback(const std::uint8_t * data, std::size_t length)
{
  if (data == nullptr || length != 8U) {
    throw std::invalid_argument("GIM6010 Iq response requires 8 bytes");
  }
  IqFeedback result;
  std::memcpy(&result.setpoint, data, sizeof(float));
  std::memcpy(&result.measured, data + sizeof(float), sizeof(float));
  return result;
}

}  // namespace gim6010_driver
