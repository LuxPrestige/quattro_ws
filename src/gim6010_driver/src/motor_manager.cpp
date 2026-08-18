#include "gim6010_driver/motor_manager.hpp"

#include <stdexcept>

namespace gim6010_driver
{
MotorManager::MotorManager(const std::string & interface_name)
: socket_(std::make_shared<CanSocket>(interface_name)) {}

void MotorManager::addMotor(std::uint8_t node_id)
{
  if (motors_.count(node_id) != 0U) {throw std::invalid_argument("duplicate motor node ID");}
  motors_.emplace(node_id, std::make_unique<Gim6010Motor>(node_id, socket_));
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
  const auto node_id = static_cast<std::uint8_t>(frame.id >> 5);
  const auto command_id = static_cast<std::uint8_t>(frame.id & 0x1F);
  const auto entry = motors_.find(node_id);
  if (entry == motors_.end()) {
    return true;
  }
  if (command_id == Gim6010Motor::kCommandHeartbeat && frame.dlc == 8) {
    entry->second->updateHeartbeat(frame.data.data(), frame.dlc);
    return true;
  }
  if (command_id == Gim6010Motor::kCommandGetError && frame.dlc >= 4) {
    entry->second->updateError(frame.data.data(), frame.dlc);
    return true;
  }
  if (command_id == Gim6010Motor::kCommandMitControl && frame.dlc >= 6) {
    auto feedback = decodeFeedback(frame.data.data(), frame.dlc);
    if (feedback.motor_id != node_id) {
      throw std::runtime_error("CAN ID and MIT feedback ID differ");
    }
    entry->second->updateFeedback(feedback);
    return true;
  }
  if (command_id == Gim6010Motor::kCommandEncoderEstimates && frame.dlc == 8) {
    entry->second->updateEncoderEstimates(frame.data.data(), frame.dlc);
    return true;
  }
  if (command_id == Gim6010Motor::kCommandGetBusVoltageCurrent && frame.dlc == 8) {
    entry->second->updateBusVoltageCurrent(frame.data.data(), frame.dlc);
    return true;
  }
  return true;
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
