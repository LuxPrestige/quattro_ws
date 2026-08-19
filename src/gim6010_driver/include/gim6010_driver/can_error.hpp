#ifndef GIM6010_DRIVER__CAN_ERROR_HPP_
#define GIM6010_DRIVER__CAN_ERROR_HPP_

#include <cstdint>

#include "gim6010_driver/can_frame.hpp"

namespace gim6010_driver
{

// Overall bus health, derived from accumulated Linux SocketCAN error frames.
enum class CanBusState
{
  kErrorActive,
  kErrorWarning,
  kErrorPassive,
  kBusOff,
};

// One decoded Linux SocketCAN error frame (CAN_ERR_FLAG set in frame.id).
// Field layout follows linux/can/error.h: id encodes the top-level error
// classes, data[1] encodes CAN_ERR_CRTL_* controller flags, data[6]/data[7]
// carry the TX/RX error counters when CAN_ERR_CNT is set.
struct DecodedCanError
{
  bool bus_off{false};
  bool error_passive{false};
  bool error_warning{false};
  bool ack_error{false};
  bool has_counters{false};
  std::uint8_t tx_error_counter{0};
  std::uint8_t rx_error_counter{0};
};

// Pure decode: does not touch a socket, safe to unit test with a crafted
// CanFrame. `frame.error` must be true (see CanSocket::receive()).
DecodedCanError decodeErrorFrame(const CanFrame & frame);

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__CAN_ERROR_HPP_
