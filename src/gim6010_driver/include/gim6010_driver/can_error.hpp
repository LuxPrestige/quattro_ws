#ifndef GIM6010_DRIVER__CAN_ERROR_HPP_
#define GIM6010_DRIVER__CAN_ERROR_HPP_

#include <cstdint>

namespace gim6010_driver
{

enum class CanBusState {kErrorActive, kErrorWarning, kErrorPassive, kBusOff};

struct CanErrorStatus
{
  std::uint64_t total_frames{0};
  std::uint64_t warning_frames{0};
  std::uint64_t passive_frames{0};
  std::uint64_t bus_off_frames{0};
  std::uint64_t controller_frames{0};
  std::uint64_t protocol_frames{0};
  std::uint64_t ack_error_frames{0};
  std::uint64_t transceiver_frames{0};
  std::uint8_t tx_error_counter{0};
  std::uint8_t rx_error_counter{0};
  std::uint32_t last_error_class{0};
  std::uint32_t rx_dropped_frames{0};
  CanBusState state{CanBusState::kErrorActive};
};

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__CAN_ERROR_HPP_
