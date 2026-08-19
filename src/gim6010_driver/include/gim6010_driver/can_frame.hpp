#ifndef GIM6010_DRIVER__CAN_FRAME_HPP_
#define GIM6010_DRIVER__CAN_FRAME_HPP_

#include <array>
#include <cstdint>

namespace gim6010_driver
{

// Pure value type for a standard (11-bit) CAN frame. Never touches a socket
// or a protocol field table by itself.
struct CanFrame
{
  uint32_t id{0};
  uint8_t dlc{0};
  std::array<uint8_t, 8> data{};
  // Remote Transmission Request: used for read-only "Get_*" style requests
  // that carry no payload of their own.
  bool rtr{false};

  bool operator==(const CanFrame & other) const noexcept
  {
    return id == other.id && dlc == other.dlc && rtr == other.rtr && data == other.data;
  }

  bool operator!=(const CanFrame & other) const noexcept { return !(*this == other); }
};

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__CAN_FRAME_HPP_
