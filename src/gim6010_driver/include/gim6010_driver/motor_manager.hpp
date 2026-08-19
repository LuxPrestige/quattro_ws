#ifndef GIM6010_DRIVER__MOTOR_MANAGER_HPP_
#define GIM6010_DRIVER__MOTOR_MANAGER_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "gim6010_driver/can_diagnostics.hpp"
#include "gim6010_driver/can_socket.hpp"
#include "gim6010_driver/gim6010_motor.hpp"

namespace gim6010_driver
{

// Owns the single CanSocket for one CAN bus and routes inbound frames to the
// Gim6010Motor whose node id the arbitration id encodes. One MotorManager
// per bus (can0, can1, ...); never share a socket across buses.
class MotorManager
{
public:
  explicit MotorManager(const std::string & interface_name);

  void addMotor(std::uint8_t node_id, double gear_ratio = 8.0);
  Gim6010Motor & motor(std::uint8_t node_id);
  const Gim6010Motor & motor(std::uint8_t node_id) const;

  // Drains at most one frame from the socket (blocking up to `timeout`) and
  // routes it. Returns true if a frame was processed, so callers loop
  // `while (poll(0ms)) {}` to drain everything currently queued.
  bool poll(std::chrono::milliseconds timeout = std::chrono::milliseconds{0});

  // Sends Set_Axis_State(idle) to every known motor. Best-effort: swallows
  // CAN I/O failures so a fault during shutdown cannot itself throw.
  void disableAll() noexcept;

  const CanErrorStatus & canErrorStatus() const noexcept;
  const RoutingStatistics & routingStatistics() const noexcept;

private:
  std::shared_ptr<CanSocket> socket_;
  std::unordered_map<std::uint8_t, std::unique_ptr<Gim6010Motor>> motors_;
  CanErrorStatus can_error_status_{};
  RoutingStatistics routing_statistics_{};
};

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__MOTOR_MANAGER_HPP_
