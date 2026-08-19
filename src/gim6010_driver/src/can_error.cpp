#include "gim6010_driver/can_error.hpp"

#include <linux/can/error.h>

namespace gim6010_driver
{

CanBusError decode_error_frame(const struct can_frame & raw)
{
  CanBusError error;

  if (raw.can_id & CAN_ERR_CRTL) {
    const uint8_t control = raw.data[1];
    error.error_warning = (control & (CAN_ERR_CRTL_TX_WARNING | CAN_ERR_CRTL_RX_WARNING)) != 0;
    error.error_passive = (control & (CAN_ERR_CRTL_TX_PASSIVE | CAN_ERR_CRTL_RX_PASSIVE)) != 0;
  }
  error.bus_off = (raw.can_id & CAN_ERR_BUSOFF) != 0;
  error.tx_error_counter = raw.data[6];
  error.rx_error_counter = raw.data[7];

  return error;
}

CanBusState bus_state_from_error(const CanBusError & error) noexcept
{
  if (error.bus_off) {
    return CanBusState::kBusOff;
  }
  if (error.error_passive) {
    return CanBusState::kPassive;
  }
  if (error.error_warning) {
    return CanBusState::kWarning;
  }
  return CanBusState::kActive;
}

}  // namespace gim6010_driver
