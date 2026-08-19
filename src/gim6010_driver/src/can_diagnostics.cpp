#include "gim6010_driver/can_diagnostics.hpp"

namespace gim6010_driver
{
namespace
{
int severity(CanBusState state)
{
  switch (state) {
    case CanBusState::kErrorActive: return 0;
    case CanBusState::kErrorWarning: return 1;
    case CanBusState::kErrorPassive: return 2;
    case CanBusState::kBusOff: return 3;
  }
  return 0;
}
}  // namespace

void recordCanError(CanErrorStatus & status, const DecodedCanError & error)
{
  ++status.total_frames;
  if (error.ack_error) {++status.ack_error_frames;}
  if (error.has_counters) {
    status.tx_error_counter = error.tx_error_counter;
    status.rx_error_counter = error.rx_error_counter;
  }

  CanBusState observed = CanBusState::kErrorActive;
  if (error.bus_off) {
    ++status.bus_off_frames;
    observed = CanBusState::kBusOff;
  } else if (error.error_passive) {
    ++status.passive_frames;
    observed = CanBusState::kErrorPassive;
  } else if (error.error_warning) {
    ++status.warning_frames;
    observed = CanBusState::kErrorWarning;
  }
  // State only escalates: once bus-off/passive is seen it stays sticky until
  // the caller rebuilds the MotorManager (e.g. during Safe Start reconfigure).
  if (severity(observed) > severity(status.state)) {
    status.state = observed;
  }
}

void recordCanRxDrop(CanErrorStatus & status) noexcept
{
  ++status.rx_dropped_frames;
}

}  // namespace gim6010_driver
