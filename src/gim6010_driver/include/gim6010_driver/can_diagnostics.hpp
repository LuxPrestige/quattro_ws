#ifndef GIM6010_DRIVER__CAN_DIAGNOSTICS_HPP_
#define GIM6010_DRIVER__CAN_DIAGNOSTICS_HPP_

#include <cstddef>
#include <cstdint>

namespace gim6010_driver
{

enum class ErrorType : std::uint8_t
{
  kMotor = 0,
  kEncoder = 1,
  kController = 3,
  kSystem = 4,
};

struct Heartbeat
{
  std::uint32_t axis_error{0};
  std::uint8_t axis_state{0};
  std::uint8_t flags{0};
  std::uint8_t reserved{0};
  std::uint8_t life{0};
};

struct BusVoltageCurrent
{
  float voltage{0.0F};
  float current{0.0F};
};

Heartbeat decodeHeartbeat(const std::uint8_t * data, std::size_t length);
std::uint64_t decodeError(const std::uint8_t * data, std::size_t length, ErrorType type);
BusVoltageCurrent decodeBusVoltageCurrent(const std::uint8_t * data, std::size_t length);

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__CAN_DIAGNOSTICS_HPP_
