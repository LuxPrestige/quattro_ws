#include "quattro_hardware/quattro_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unordered_set>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace quattro_hardware
{
namespace
{
double parameterDouble(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & key,
  double fallback)
{
  const auto value = parameters.find(key);
  return value == parameters.end() ? fallback : std::stod(value->second);
}

long parameterLong(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & key,
  long fallback)
{
  const auto value = parameters.find(key);
  return value == parameters.end() ? fallback : std::stol(value->second);
}

std::string parameterString(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & key,
  const std::string & fallback)
{
  const auto value = parameters.find(key);
  return value == parameters.end() ? fallback : value->second;
}
}  // namespace

hardware_interface::CallbackReturn QuattroSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    const auto & hp = params.hardware_info.hardware_parameters;
    const auto default_bus = parameterString(hp, "can_interface", "can0");
    feedback_timeout_ = std::chrono::milliseconds(parameterLong(hp, "feedback_timeout_ms", 100));
    startup_timeout_ = std::chrono::milliseconds(parameterLong(hp, "startup_timeout_ms", 1000));
    command_timeout_ = std::chrono::milliseconds(parameterLong(hp, "command_timeout_ms", 100));
    motor_velocity_limit_ = parameterDouble(hp, "motor_velocity_limit", 50.0);
    motor_current_limit_ = parameterDouble(hp, "motor_current_limit", 20.0);
    if (feedback_timeout_.count() <= 0 || startup_timeout_.count() <= 0 ||
      command_timeout_.count() <= 0 || motor_velocity_limit_ <= 0.0 || motor_current_limit_ <= 0.0)
    {
      throw std::invalid_argument("hardware timeout and motor limits must be positive");
    }

    std::unordered_set<std::string> motor_addresses;
    for (const auto & info : params.hardware_info.joints) {
      if (info.command_interfaces.size() != 1 ||
        info.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        throw std::invalid_argument(info.name + " must expose one position command interface");
      }
      if (info.state_interfaces.size() != 3) {
        throw std::invalid_argument(info.name + " must expose position, velocity, effort states");
      }
      std::unordered_set<std::string> state_names;
      for (const auto & state : info.state_interfaces) {
        state_names.insert(state.name);
      }
      if (state_names != std::unordered_set<std::string>{
          hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
          hardware_interface::HW_IF_EFFORT})
      {
        throw std::invalid_argument(info.name + " has invalid state interface names");
      }
      Joint joint;
      joint.name = info.name;
      joint.can_interface = parameterString(info.parameters, "can_interface", default_bus);
      const auto node_id = parameterLong(info.parameters, "can_id", -1);
      if (node_id < 0 || node_id > gim6010_driver::Gim6010Motor::kMaxNodeId) {
        throw std::invalid_argument(info.name + " has invalid can_id");
      }
      joint.node_id = static_cast<std::uint8_t>(node_id);
      joint.direction = parameterDouble(info.parameters, "direction", 1.0);
      joint.offset = parameterDouble(info.parameters, "offset", 0.0);
      joint.kp = parameterDouble(info.parameters, "kp", 20.0);
      joint.kd = parameterDouble(info.parameters, "kd", 0.5);
      if (joint.direction != 1.0 && joint.direction != -1.0) {
        throw std::invalid_argument(info.name + " direction must be 1 or -1");
      }
      const auto & command = info.command_interfaces[0];
      if (!command.min.empty()) {joint.lower = std::stod(command.min);}
      if (!command.max.empty()) {joint.upper = std::stod(command.max);}
      if (!(joint.lower < joint.upper)) {
        throw std::invalid_argument(info.name + " has invalid limits");
      }
      const std::string address = joint.can_interface + ":" + std::to_string(node_id);
      if (!motor_addresses.insert(address).second) {
        throw std::invalid_argument("duplicate motor " + address);
      }
      gim6010_driver::validateCommand({0.0, 0.0, joint.kp, joint.kd, 0.0});
      joints_.push_back(joint);
    }
    if (joints_.empty()) {throw std::invalid_argument("no joints configured");}
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Invalid Quattro hardware configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool QuattroSystem::configureMotors()
{
  for (auto & joint : joints_) {
    auto & manager = managers_[joint.can_interface];
    if (!manager) {manager = std::make_unique<gim6010_driver::MotorManager>(joint.can_interface);}
    manager->addMotor(joint.node_id);
    joint.manager = manager.get();
    auto & motor = manager->motor(joint.node_id);
    motor.clearErrors();
    motor.setLimits(static_cast<float>(motor_velocity_limit_),
        static_cast<float>(motor_current_limit_));
    motor.setMitMode();
    motor.requestEncoderEstimates();
  }
  return true;
}

bool QuattroSystem::waitForInitialFeedback()
{
  const auto deadline = std::chrono::steady_clock::now() + startup_timeout_;
  while (std::chrono::steady_clock::now() < deadline) {
    for (auto & manager : managers_) {
      while (manager.second->poll(std::chrono::milliseconds{0})) {
      }
    }
    const bool ready = std::all_of(joints_.begin(), joints_.end(), [](const Joint & joint) {
          return joint.manager->motor(joint.node_id).hasFeedback();
    });
    if (ready) {return true;}
    for (auto & joint : joints_) {
      if (!joint.manager->motor(joint.node_id).hasFeedback()) {
        joint.manager->motor(joint.node_id).requestEncoderEstimates();
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
  return false;
}

hardware_interface::CallbackReturn QuattroSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    managers_.clear();
    configureMotors();
    if (!waitForInitialFeedback()) {
      RCLCPP_ERROR(get_logger(), "Safe Start failed: not all motors supplied encoder feedback");
      safeStop();
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (const auto & joint : joints_) {
      const auto & feedback = joint.manager->motor(joint.node_id).feedback();
      set_state(interfaceKey(joint, hardware_interface::HW_IF_POSITION),
        joint.direction * feedback.position - joint.offset);
      set_state(interfaceKey(joint, hardware_interface::HW_IF_VELOCITY),
        joint.direction * feedback.velocity);
      set_state(interfaceKey(joint, hardware_interface::HW_IF_EFFORT),
        joint.direction * feedback.torque);
      set_command(interfaceKey(joint, hardware_interface::HW_IF_POSITION),
        joint.direction * feedback.position - joint.offset);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to configure motors: %s", error.what());
    safeStop();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuattroSystem::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    for (auto & joint : joints_) {
      auto & motor = joint.manager->motor(joint.node_id);
      if (motor.feedbackStale(feedback_timeout_)) {
        throw std::runtime_error(joint.name + " feedback is stale before activation");
      }
      const double hold = motor.feedback().position;
      motor.sendCommand({hold, 0.0, joint.kp, joint.kd, 0.0});
      motor.enable();
      motor.sendCommand({hold, 0.0, joint.kp, joint.kd, 0.0});
      set_command(interfaceKey(joint, hardware_interface::HW_IF_POSITION),
        joint.direction * hold - joint.offset);
    }
    last_write_ = std::chrono::steady_clock::now();
    active_ = true;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Safe Start activation failed: %s", error.what());
    safeStop();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuattroSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  safeStop();
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn QuattroSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  safeStop();
  managers_.clear();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type QuattroSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    for (auto & manager : managers_) {
      while (manager.second->poll(std::chrono::milliseconds{0})) {
      }
    }
    for (const auto & joint : joints_) {
      const auto & motor = joint.manager->motor(joint.node_id);
      if (active_ && motor.feedbackStale(feedback_timeout_)) {
        RCLCPP_ERROR(get_logger(), "Stale feedback from %s", joint.name.c_str());
        safeStop();
        return hardware_interface::return_type::ERROR;
      }
      if (!motor.hasFeedback()) {continue;}
      const auto & feedback = motor.feedback();
      set_state(interfaceKey(joint, hardware_interface::HW_IF_POSITION),
        joint.direction * feedback.position - joint.offset);
      set_state(interfaceKey(joint, hardware_interface::HW_IF_VELOCITY),
        joint.direction * feedback.velocity);
      set_state(interfaceKey(joint, hardware_interface::HW_IF_EFFORT),
        joint.direction * feedback.torque);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "CAN read failed: %s", error.what());
    safeStop();
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuattroSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {return hardware_interface::return_type::OK;}
  const auto now = std::chrono::steady_clock::now();
  if (now - last_write_ > command_timeout_) {
    RCLCPP_ERROR(get_logger(), "Hardware command watchdog expired");
    safeStop();
    return hardware_interface::return_type::ERROR;
  }
  try {
    for (const auto & joint : joints_) {
      const double command = get_command<double>(interfaceKey(joint,
          hardware_interface::HW_IF_POSITION));
      if (!std::isfinite(command) || command < joint.lower || command > joint.upper) {
        throw std::out_of_range(joint.name + " position command violates configured limit");
      }
      const double motor_position = joint.direction * (command + joint.offset);
      joint.manager->motor(joint.node_id).sendCommand(
        {motor_position, 0.0, joint.kp, joint.kd, 0.0});
    }
    last_write_ = now;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Unsafe command rejected: %s", error.what());
    safeStop();
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void QuattroSystem::safeStop() noexcept
{
  active_ = false;
  for (auto & manager : managers_) {
    manager.second->disableAll();
  }
}

std::string QuattroSystem::interfaceKey(const Joint & joint, const char * interface_name) const
{
  return joint.name + "/" + interface_name;
}
}  // namespace quattro_hardware

PLUGINLIB_EXPORT_CLASS(quattro_hardware::QuattroSystem, hardware_interface::SystemInterface)
