#include "gim6010_driver/can_error.hpp"

namespace gim6010_driver
{
namespace
{
// linux/can/error.h class flags, carried in the (masked) arbitration id of
// a kernel error frame.
constexpr std::uint32_t kErrCrtl = 0x00000004U;
constexpr std::uint32_t kErrAck = 0x00000020U;
constexpr std::uint32_t kErrBusoff = 0x00000040U;
constexpr std::uint32_t kErrCnt = 0x00000200U;

// linux/can/error.h CAN_ERR_CRTL_* bits, carried in data[1] when kErrCrtl
// is set.
constexpr std::uint8_t kCrtlRxWarning = 0x04U;
constexpr std::uint8_t kCrtlTxWarning = 0x08U;
constexpr std::uint8_t kCrtlRxPassive = 0x10U;
constexpr std::uint8_t kCrtlTxPassive = 0x20U;
}  // namespace

DecodedCanError decodeErrorFrame(const CanFrame & frame)
{
  DecodedCanError result;
  result.bus_off = (frame.id & kErrBusoff) != 0U;
  result.ack_error = (frame.id & kErrAck) != 0U;
  if ((frame.id & kErrCrtl) != 0U && frame.dlc > 1U) {
    const std::uint8_t crtl = frame.data[1];
    result.error_warning = (crtl & (kCrtlRxWarning | kCrtlTxWarning)) != 0U;
    result.error_passive = (crtl & (kCrtlRxPassive | kCrtlTxPassive)) != 0U;
  }
  if ((frame.id & kErrCnt) != 0U && frame.dlc > 7U) {
    result.has_counters = true;
    result.tx_error_counter = frame.data[6];
    result.rx_error_counter = frame.data[7];
  }
  return result;
}

}  // namespace gim6010_driver
