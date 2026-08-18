#include "gim6010_driver/motor_manager.hpp"

#include <stdexcept>
#include <algorithm>

#include <linux/can/error.h>

namespace gim6010_driver
{
MotorManager::MotorManager(const std::string & interface_name)
: socket_(std::make_shared<CanSocket>(interface_name)) {}

void MotorManager::addMotor(std::uint8_t node_id, double gear_ratio)
{
  if (motors_.count(node_id) != 0U) {throw std::invalid_argument("duplicate motor node ID");}
  motors_.emplace(node_id, std::make_unique<Gim6010Motor>(node_id, socket_, gear_ratio));
}
Gim6010Motor & MotorManager::motor(std::uint8_t node_id)
{
  const auto entry = motors_.find(node_id);
  if (entry == motors_.end()) {throw std::out_of_range("unknown motor node ID");}
  return *entry->second;
}
const Gim6010Motor & MotorManager::motor(std::uint8_t node_id) const
{
  const auto entry = motors_.find(node_id);
  if (entry == motors_.end()) {throw std::out_of_range("unknown motor node ID");}
  return *entry->second;
}
bool MotorManager::poll(std::chrono::milliseconds timeout)
{
  CanFrame frame;
  if (!socket_->receive(frame, timeout)) {return false;}
  can_error_status_.rx_dropped_frames = std::max(
    can_error_status_.rx_dropped_frames, frame.rx_overflow_count);
  if (frame.error) {
    ++can_error_status_.total_frames;
    can_error_status_.last_error_class = frame.error_class;
    if ((frame.error_class & CAN_ERR_CRTL) != 0U && frame.dlc >= 8U) {
      ++can_error_status_.controller_frames;
      if ((frame.data[1] & (CAN_ERR_CRTL_TX_WARNING | CAN_ERR_CRTL_RX_WARNING)) != 0U) {
        ++can_error_status_.warning_frames;
        can_error_status_.state = CanBusState::kErrorWarning;
      }
      if ((frame.data[1] & (CAN_ERR_CRTL_TX_PASSIVE | CAN_ERR_CRTL_RX_PASSIVE)) != 0U) {
        ++can_error_status_.passive_frames;
        can_error_status_.state = CanBusState::kErrorPassive;
      }
      can_error_status_.tx_error_counter = frame.data[6];
      can_error_status_.rx_error_counter = frame.data[7];
    }
    if ((frame.error_class & CAN_ERR_BUSOFF) != 0U) {
      ++can_error_status_.bus_off_frames;
      can_error_status_.state = CanBusState::kBusOff;
    }
    if ((frame.error_class & CAN_ERR_RESTARTED) != 0U) {
      can_error_status_.state = CanBusState::kErrorActive;
    }
    if ((frame.error_class & CAN_ERR_ACK) != 0U) {++can_error_status_.ack_error_frames;}
    if ((frame.error_class & CAN_ERR_PROT) != 0U) {++can_error_status_.protocol_frames;}
    if ((frame.error_class & CAN_ERR_TRX) != 0U) {++can_error_status_.transceiver_frames;}
    return true;
  }
  const auto parsed_id = parseArbitrationId(frame.id);
  const auto node_id = parsed_id.first;
  const auto command_id = static_cast<std::uint8_t>(parsed_id.second);
  const auto entry = motors_.find(node_id);
  if (entry == motors_.end()) {
    ++routing_statistics_.unknown_node_frames;
    return true;
  }
  if (command_id == Gim6010Motor::kCommandHeartbeat) {
    if (frame.dlc != 8U || frame.remote) {++routing_statistics_.malformed_frames; return true;}
    entry->second->updateHeartbeat(frame.data.data(), frame.dlc);
    return true;
  }
  if (command_id == Gim6010Motor::kCommandGetError) {
    if ((frame.dlc != 4U && frame.dlc != 8U) || frame.remote) {
      ++routing_statistics_.malformed_frames; return true;
    }
    entry->second->updateError(frame.data.data(), frame.dlc);
    return true;
  }
  if (command_id == Gim6010Motor::kCommandMitControl) {
    if (frame.dlc < 6U || frame.remote) {++routing_statistics_.malformed_frames; return true;}
    auto feedback = decodeFeedback(frame.data.data(), frame.dlc);
    if (feedback.motor_id != node_id) {
      throw std::runtime_error("CAN ID and MIT feedback ID differ");
    }
    entry->second->updateFeedback(feedback);
    return true;
  }
  if (command_id == Gim6010Motor::kCommandEncoderEstimates) {
    if (frame.dlc != 8U || frame.remote) {++routing_statistics_.malformed_frames; return true;}
    entry->second->updateEncoderEstimates(frame.data.data(), frame.dlc);
    return true;
  }
  if (command_id == Gim6010Motor::kCommandGetBusVoltageCurrent) {
    if (frame.dlc != 8U || frame.remote) {++routing_statistics_.malformed_frames; return true;}
    entry->second->updateBusVoltageCurrent(frame.data.data(), frame.dlc);
    return true;
  }
  ++routing_statistics_.unknown_command_frames;
  return true;
}
const CanErrorStatus & MotorManager::canErrorStatus() const noexcept {return can_error_status_;}
const RoutingStatistics & MotorManager::routingStatistics() const noexcept
{
  return routing_statistics_;
}
void MotorManager::disableAll() noexcept
{
  for (auto & entry : motors_) {
    try {
      entry.second->disable();
    } catch (...) {
    }
  }
}
}  // namespace gim6010_driver
