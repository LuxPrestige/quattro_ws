#ifndef GIM6010_DRIVER__MOTOR_MANAGER_HPP_
#define GIM6010_DRIVER__MOTOR_MANAGER_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "gim6010_driver/gim6010_motor.hpp"

namespace gim6010_driver
{

class MotorManager
{
public:
  explicit MotorManager(const std::string & interface_name);
  void addMotor(std::uint8_t node_id);
  Gim6010Motor & motor(std::uint8_t node_id);
  const Gim6010Motor & motor(std::uint8_t node_id) const;
  bool poll(std::chrono::milliseconds timeout = std::chrono::milliseconds{0});
  void disableAll() noexcept;

private:
  std::shared_ptr<CanSocket> socket_;
  std::unordered_map<std::uint8_t, std::unique_ptr<Gim6010Motor>> motors_;
};

}  // namespace gim6010_driver
#endif  // GIM6010_DRIVER__MOTOR_MANAGER_HPP_
