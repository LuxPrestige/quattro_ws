#include "gim6010_driver/motor_manager.hpp"

#include <stdexcept>
#include <utility>

#include "gim6010_driver/can_error.hpp"
#include "gim6010_driver/gds68_protocol.hpp"

namespace gim6010_driver
{

MotorManager::MotorManager(const std::string & interface_name)
: socket_(std::make_shared<CanSocket>(interface_name))
{
}

void MotorManager::addMotor(std::uint8_t node_id, double gear_ratio)
{
  auto motor = std::make_unique<Gim6010Motor>(node_id, socket_, gear_ratio);
  const auto inserted = motors_.emplace(node_id, std::move(motor)).second;
  if (!inserted) {
    throw std::invalid_argument("duplicate node_id on this CAN bus");
  }
}

Gim6010Motor & MotorManager::motor(std::uint8_t node_id)
{
  const auto found = motors_.find(node_id);
  if (found == motors_.end()) {
    throw std::out_of_range("unknown motor node_id");
  }
  return *found->second;
}

const Gim6010Motor & MotorManager::motor(std::uint8_t node_id) const
{
  const auto found = motors_.find(node_id);
  if (found == motors_.end()) {
    throw std::out_of_range("unknown motor node_id");
  }
  return *found->second;
}

bool MotorManager::poll(std::chrono::milliseconds timeout)
{
  CanFrame frame;
  if (!socket_->receive(frame, timeout)) {
    return false;
  }
  if (frame.error) {
    recordCanError(can_error_status_, decodeErrorFrame(frame));
    return true;
  }

  const auto parsed = parseArbitrationId(frame.id);
  const auto found = motors_.find(parsed.first);
  if (found == motors_.end()) {
    ++routing_statistics_.unknown_node_frames;
    return true;
  }

  try {
    if (!found->second->handleFrame(parsed.second, frame.data.data(), frame.dlc)) {
      ++routing_statistics_.unknown_command_frames;
    }
  } catch (const std::length_error &) {
    ++routing_statistics_.malformed_frames;
  }
  return true;
}

void MotorManager::disableAll() noexcept
{
  for (auto & entry : motors_) {
    try {
      entry.second->disable();
    } catch (...) {
      // Best-effort: a CAN I/O failure while shutting down must not throw.
    }
  }
}

const CanErrorStatus & MotorManager::canErrorStatus() const noexcept {return can_error_status_;}

const RoutingStatistics & MotorManager::routingStatistics() const noexcept
{
  return routing_statistics_;
}

}  // namespace gim6010_driver
