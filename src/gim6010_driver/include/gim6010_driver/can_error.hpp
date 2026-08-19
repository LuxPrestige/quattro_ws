#ifndef GIM6010_DRIVER__CAN_ERROR_HPP_
#define GIM6010_DRIVER__CAN_ERROR_HPP_

#include <linux/can.h>

#include "gim6010_driver/types.hpp"

namespace gim6010_driver
{

// Decodes a raw Linux SocketCAN error frame (CAN_ERR_FLAG set in can_id)
// into the driver's own value type. `raw` must be an error frame -- callers
// check CAN_ERR_FLAG before calling this.
CanBusError decode_error_frame(const struct can_frame & raw);

// Reduces a CanBusError down to the single coarse state gim6010_driver and
// its callers act on.
CanBusState bus_state_from_error(const CanBusError & error) noexcept;

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__CAN_ERROR_HPP_
