#ifndef GIM6010_DRIVER__CAN_FRAME_HPP_
#define GIM6010_DRIVER__CAN_FRAME_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace gim6010_driver
{

// Classic (non-FD) CAN frame, transport-agnostic so protocol code can be unit
// tested without a real SocketCAN interface.
struct CanFrame
{
  // 11-bit standard arbitration id. For a kernel error frame this instead
  // holds the CAN_ERR_* flag word (see can_error.hpp).
  std::uint32_t id{0};
  std::uint8_t dlc{0};
  std::array<std::uint8_t, 8> data{};
  bool remote{false};
  bool error{false};
};

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__CAN_FRAME_HPP_
