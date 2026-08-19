#ifndef GIM6010_DRIVER__CAN_DIAGNOSTICS_HPP_
#define GIM6010_DRIVER__CAN_DIAGNOSTICS_HPP_

#include <cstdint>

#include "gim6010_driver/can_error.hpp"

namespace gim6010_driver
{

// Running counters for one CAN bus, published to /diagnostics by the caller.
struct CanErrorStatus
{
  CanBusState state{CanBusState::kErrorActive};
  std::uint64_t total_frames{0};
  std::uint64_t ack_error_frames{0};
  std::uint64_t warning_frames{0};
  std::uint64_t passive_frames{0};
  std::uint64_t bus_off_frames{0};
  std::uint32_t tx_error_counter{0};
  std::uint32_t rx_error_counter{0};
  std::uint64_t rx_dropped_frames{0};
};

// Counts of frames MotorManager could not route to a known motor/command.
struct RoutingStatistics
{
  std::uint64_t unknown_node_frames{0};
  std::uint64_t unknown_command_frames{0};
  std::uint64_t malformed_frames{0};
};

// Folds one decoded error frame into the running status. `state` only ever
// escalates within a poll cycle (bus-off is sticky until the caller resets
// the status, e.g. by reconstructing the MotorManager after recovery).
void recordCanError(CanErrorStatus & status, const DecodedCanError & error);

// Called when the kernel socket buffer reports a dropped RX frame
// (SO_RXQ_OVFL), so operators can tell packet loss apart from a quiet bus.
void recordCanRxDrop(CanErrorStatus & status) noexcept;

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__CAN_DIAGNOSTICS_HPP_
