#include "quattro_hardware/quattro_system.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
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
constexpr auto kConfigureFrameInterval = std::chrono::milliseconds{2};
constexpr std::uint8_t kClosedLoopControl = 8;

std::string hexValue(std::uint64_t value)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << std::uppercase << value;
  return stream.str();
}

diagnostic_msgs::msg::KeyValue diagnosticValue(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string systemErrorName(std::uint64_t value)
{
  if ((value & 0x02U) != 0U) {return "DC_BUS_UNDER_VOLTAGE";}
  if ((value & 0x04U) != 0U) {return "DC_BUS_OVER_VOLTAGE";}
  if ((value & 0x08U) != 0U) {return "DC_BUS_OVER_REGEN_CURRENT";}
  if ((value & 0x10U) != 0U) {return "DC_BUS_OVER_CURRENT";}
  return value == 0U ? "OK" : "UNKNOWN_SYSTEM_ERROR";
}

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
    heartbeat_timeout_ = std::chrono::milliseconds(parameterLong(hp, "heartbeat_timeout_ms", 300));
    startup_timeout_ = std::chrono::milliseconds(parameterLong(hp, "startup_timeout_ms", 1000));
    command_timeout_ = std::chrono::milliseconds(parameterLong(hp, "command_timeout_ms", 100));
    motor_velocity_limit_ = parameterDouble(hp, "motor_velocity_limit", 50.0);
    motor_current_limit_ = parameterDouble(hp, "motor_current_limit", 20.0);
    engagement_duration_ = std::chrono::milliseconds(
      parameterLong(hp, "engagement_duration_ms", 1000));
    telemetry_period_ = std::chrono::milliseconds(parameterLong(hp, "telemetry_period_ms", 500));
    if (feedback_timeout_.count() <= 0 || heartbeat_timeout_.count() <= 0 ||
      startup_timeout_.count() <= 0 || telemetry_period_.count() <= 0 ||
      command_timeout_.count() <= 0 || motor_velocity_limit_ <= 0.0 ||
      motor_current_limit_ <= 0.0 || engagement_duration_.count() <= 0)
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
    diagnostics_publisher_ = get_node()->create_publisher<
      diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
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
    std::this_thread::sleep_for(kConfigureFrameInterval);
    motor.setLimits(static_cast<float>(motor_velocity_limit_),
        static_cast<float>(motor_current_limit_));
    std::this_thread::sleep_for(kConfigureFrameInterval);
    motor.setMitMode();
    std::this_thread::sleep_for(kConfigureFrameInterval);
  }
  return true;
}

bool QuattroSystem::waitForInitialFeedback()
{
  const auto deadline = std::chrono::steady_clock::now() + startup_timeout_;
  while (std::chrono::steady_clock::now() < deadline) {
    pollManagers();
    const bool ready = std::all_of(joints_.begin(), joints_.end(), [this](const Joint & joint) {
          const auto & motor = joint.manager->motor(joint.node_id);
          return motor.hasFeedback() && !motor.feedbackStale(feedback_timeout_);
    });
    if (ready) {return true;}
    for (auto & joint : joints_) {
      auto & motor = joint.manager->motor(joint.node_id);
      if (!motor.hasFeedback() || motor.feedbackStale(feedback_timeout_)) {
        motor.sendCommand({0.0, 0.0, 0.0, 0.0, 0.0});
        motor.requestEncoderEstimates();
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  for (const auto & joint : joints_) {
    const auto & motor = joint.manager->motor(joint.node_id);
    if (!motor.hasFeedback() || motor.feedbackStale(feedback_timeout_)) {
      RCLCPP_ERROR(
        get_logger(),
          "Missing fresh 0x08/0x09 feedback: joint=%s interface=%s can_id=%u received=%s",
        joint.name.c_str(), joint.can_interface.c_str(), static_cast<unsigned>(joint.node_id),
        motor.hasFeedback() ? "yes (stale)" : "no");
    }
  }
  return false;
}

void QuattroSystem::pollManagers()
{
  for (auto & manager : managers_) {
    while (manager.second->poll(std::chrono::milliseconds{0})) {
    }
  }
}

bool QuattroSystem::waitForOperationalHeartbeats()
{
  const auto deadline = std::chrono::steady_clock::now() + startup_timeout_;
  while (std::chrono::steady_clock::now() < deadline) {
    pollManagers();
    const bool ready = std::all_of(joints_.begin(), joints_.end(), [this](const Joint & joint) {
          const auto & motor = joint.manager->motor(joint.node_id);
          return motor.hasHeartbeat() && !motor.heartbeatStale(heartbeat_timeout_) &&
                 motor.heartbeat().axis_error == 0U &&
                 motor.heartbeat().axis_state == kClosedLoopControl;
    });
    if (ready) {return true;}
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  for (const auto & joint : joints_) {
    const auto & motor = joint.manager->motor(joint.node_id);
    if (!motor.hasHeartbeat()) {
      RCLCPP_ERROR(get_logger(), "Missing heartbeat from %s", joint.name.c_str());
      continue;
    }
    const auto & heartbeat = motor.heartbeat();
    RCLCPP_ERROR(
      get_logger(),
      "Motor not operational: joint=%s interface=%s can_id=%u axis_error=%s "
      "axis_state=%u flags=%s life=%u",
      joint.name.c_str(), joint.can_interface.c_str(), static_cast<unsigned>(joint.node_id),
      hexValue(heartbeat.axis_error).c_str(), static_cast<unsigned>(heartbeat.axis_state),
      hexValue(heartbeat.flags).c_str(), static_cast<unsigned>(heartbeat.life));
  }
  return false;
}

hardware_interface::CallbackReturn QuattroSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    managers_.clear();
    configureMotors();
    if (!waitForInitialFeedback()) {
      RCLCPP_ERROR(get_logger(), "Safe Start failed: not all motors supplied feedback");
      safeStop();
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (const auto & joint : joints_) {
      const auto & feedback = joint.manager->motor(joint.node_id).feedback();
      const double joint_position = joint.direction * feedback.position - joint.offset;
      set_state(interfaceKey(joint, hardware_interface::HW_IF_POSITION), joint_position);
      set_state(interfaceKey(joint, hardware_interface::HW_IF_VELOCITY),
        joint.direction * feedback.velocity);
      set_state(interfaceKey(joint, hardware_interface::HW_IF_EFFORT),
        joint.direction * feedback.torque);
      set_command(interfaceKey(joint, hardware_interface::HW_IF_POSITION), joint_position);
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
      motor.enable();
      std::this_thread::sleep_for(kConfigureFrameInterval);
      motor.sendCommand({0.0, 0.0, 0.0, 0.0, 0.0});
      std::this_thread::sleep_for(kConfigureFrameInterval);
    }

    if (!waitForInitialFeedback()) {
      throw std::runtime_error(
              "not all enabled motors supplied fresh 0x08 or 0x09 feedback");
    }
    if (!waitForOperationalHeartbeats()) {
      throw std::runtime_error("not all motors entered closed-loop control without errors");
    }

    constexpr auto control_period = std::chrono::milliseconds{10};

    for (std::size_t refresh = 0; refresh < 3; ++refresh) {
      for (auto & joint : joints_) {
        auto & motor = joint.manager->motor(joint.node_id);
        motor.sendCommand({motor.feedback().position, 0.0, 0.0, joint.kd, 0.0});
      }
      std::this_thread::sleep_for(control_period);
      pollManagers();
    }

    std::vector<double> hold_positions;
    hold_positions.reserve(joints_.size());
    for (const auto & joint : joints_) {
      const auto & motor = joint.manager->motor(joint.node_id);
      if (motor.feedbackStale(feedback_timeout_)) {
        throw std::runtime_error(
                joint.name + " did not supply fresh feedback after zero-Kp activation");
      }
      hold_positions.push_back(motor.feedback().position);
    }

    const auto engagement_steps = std::max<std::int64_t>(
      1, engagement_duration_.count() / control_period.count());
    auto next_cycle = std::chrono::steady_clock::now();
    for (std::int64_t step = 1; step <= engagement_steps; ++step) {
      const double gain_ratio =
        static_cast<double>(step) / static_cast<double>(engagement_steps);
      for (std::size_t index = 0; index < joints_.size(); ++index) {
        const auto & joint = joints_[index];
        joint.manager->motor(joint.node_id).sendCommand(
          {hold_positions[index], 0.0, joint.kp * gain_ratio, joint.kd, 0.0});
      }
      next_cycle += control_period;
      std::this_thread::sleep_until(next_cycle);
      pollManagers();
      for (const auto & joint : joints_) {
        const auto & heartbeat = joint.manager->motor(joint.node_id).heartbeat();
        if (heartbeat.axis_error != 0U || heartbeat.axis_state != kClosedLoopControl) {
          throw std::runtime_error(
                  joint.name + " left closed-loop control during gain engagement");
        }
      }
    }

    for (const auto & joint : joints_) {
      const auto & feedback = joint.manager->motor(joint.node_id).feedback();
      const double joint_position = joint.direction * feedback.position - joint.offset;
      set_state(interfaceKey(joint, hardware_interface::HW_IF_POSITION), joint_position);
      set_state(interfaceKey(joint, hardware_interface::HW_IF_VELOCITY),
        joint.direction * feedback.velocity);
      set_state(interfaceKey(joint, hardware_interface::HW_IF_EFFORT),
        joint.direction * feedback.torque);
      set_command(interfaceKey(joint, hardware_interface::HW_IF_POSITION), joint_position);
    }
    last_write_ = std::chrono::steady_clock::now();
    next_telemetry_request_ = last_write_;
    next_diagnostics_publish_ = last_write_;
    position_limits_enabled_ = false;
    active_ = true;
    RCLCPP_INFO(
      get_logger(),
      "All motors engaged at their current positions; position limits remain disabled until "
      "all initial-pose commands enter their configured ranges");
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Safe Start activation failed: %s", error.what());
    safeStop();
    captureFaultDiagnostics();
    publishDiagnostics(true);
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

void QuattroSystem::requestBusTelemetry()
{
  for (auto & joint : joints_) {
    joint.manager->motor(joint.node_id).requestBusVoltageCurrent();
  }
}

void QuattroSystem::captureFaultDiagnostics()
{
  constexpr std::array<gim6010_driver::ErrorType, 4> error_types{{
    gim6010_driver::ErrorType::kMotor,
    gim6010_driver::ErrorType::kEncoder,
    gim6010_driver::ErrorType::kController,
    gim6010_driver::ErrorType::kSystem}};
  try {
    for (const auto type : error_types) {
      for (auto & joint : joints_) {
        joint.manager->motor(joint.node_id).requestError(type);
      }
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{20};
      while (std::chrono::steady_clock::now() < deadline) {
        pollManagers();
        const bool complete = std::all_of(
          joints_.begin(), joints_.end(), [type](const Joint & joint) {
            return joint.manager->motor(joint.node_id).hasError(type);
          });
        if (complete) {break;}
        std::this_thread::sleep_for(std::chrono::milliseconds{1});
      }
    }
    requestBusTelemetry();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{20};
    while (std::chrono::steady_clock::now() < deadline) {
      pollManagers();
      const bool complete = std::all_of(joints_.begin(), joints_.end(), [](const Joint & joint) {
            return joint.manager->motor(joint.node_id).hasBusVoltageCurrent();
      });
      if (complete) {break;}
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    for (const auto & joint : joints_) {
      const auto & motor = joint.manager->motor(joint.node_id);
      const auto motor_error = motor.hasError(gim6010_driver::ErrorType::kMotor) ?
        motor.error(gim6010_driver::ErrorType::kMotor) : 0U;
      const auto encoder_error = motor.hasError(gim6010_driver::ErrorType::kEncoder) ?
        motor.error(gim6010_driver::ErrorType::kEncoder) : 0U;
      const auto controller_error = motor.hasError(gim6010_driver::ErrorType::kController) ?
        motor.error(gim6010_driver::ErrorType::kController) : 0U;
      const auto system_error = motor.hasError(gim6010_driver::ErrorType::kSystem) ?
        motor.error(gim6010_driver::ErrorType::kSystem) : 0U;
      const double voltage = motor.hasBusVoltageCurrent() ?
        motor.busVoltageCurrent().voltage : std::numeric_limits<double>::quiet_NaN();
      RCLCPP_ERROR(
        get_logger(),
        "Motor fault details: joint=%s interface=%s can_id=%u motor=%s encoder=%s "
        "controller=%s system=%s(%s) bus_voltage=%.3fV",
        joint.name.c_str(), joint.can_interface.c_str(), static_cast<unsigned>(joint.node_id),
        hexValue(motor_error).c_str(), hexValue(encoder_error).c_str(),
        hexValue(controller_error).c_str(), hexValue(system_error).c_str(),
        systemErrorName(system_error).c_str(), voltage);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to collect motor fault details: %s", error.what());
  }
}

void QuattroSystem::publishDiagnostics(bool force)
{
  if (!diagnostics_publisher_) {return;}
  const auto now = std::chrono::steady_clock::now();
  if (!force && now < next_diagnostics_publish_) {return;}
  next_diagnostics_publish_ = now + std::chrono::seconds{1};

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_node()->now();
  array.status.reserve(joints_.size());
  for (const auto & joint : joints_) {
    const auto & motor = joint.manager->motor(joint.node_id);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "quattro_hardware/" + joint.name;
    status.hardware_id = joint.can_interface + ":" + std::to_string(joint.node_id);
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "operational";
    if (!motor.hasHeartbeat() || motor.heartbeatStale(heartbeat_timeout_)) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "heartbeat unavailable or stale";
    } else {
      const auto & heartbeat = motor.heartbeat();
      if (heartbeat.axis_error != 0U || heartbeat.axis_state != kClosedLoopControl) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "motor fault or not in closed-loop control";
      }
      status.values.push_back(diagnosticValue("axis_error", hexValue(heartbeat.axis_error)));
      status.values.push_back(
        diagnosticValue("axis_state", std::to_string(heartbeat.axis_state)));
      status.values.push_back(diagnosticValue("flags", hexValue(heartbeat.flags)));
      status.values.push_back(diagnosticValue("life", std::to_string(heartbeat.life)));
    }
    status.values.push_back(
      diagnosticValue("missed_heartbeats", std::to_string(motor.missedHeartbeats())));
    if (motor.hasBusVoltageCurrent()) {
      status.values.push_back(diagnosticValue(
          "bus_voltage_v", std::to_string(motor.busVoltageCurrent().voltage)));
      status.values.push_back(diagnosticValue(
          "bus_current_a", std::to_string(motor.busVoltageCurrent().current)));
    }
    for (const auto type : {gim6010_driver::ErrorType::kMotor,
        gim6010_driver::ErrorType::kEncoder, gim6010_driver::ErrorType::kController,
        gim6010_driver::ErrorType::kSystem})
    {
      if (motor.hasError(type)) {
        status.values.push_back(diagnosticValue(
            "error_type_" + std::to_string(static_cast<unsigned>(type)),
            hexValue(motor.error(type))));
      }
    }
    array.status.push_back(std::move(status));
  }
  diagnostics_publisher_->publish(array);
}

hardware_interface::return_type QuattroSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    pollManagers();
    for (auto & joint : joints_) {
      const auto & motor = joint.manager->motor(joint.node_id);
      if (active_ && motor.feedbackStale(feedback_timeout_)) {
        RCLCPP_ERROR(get_logger(), "Stale feedback from %s", joint.name.c_str());
        safeStop();
        captureFaultDiagnostics();
        publishDiagnostics(true);
        return hardware_interface::return_type::ERROR;
      }
      if (active_ && motor.heartbeatStale(heartbeat_timeout_)) {
        RCLCPP_ERROR(get_logger(), "Stale heartbeat from %s", joint.name.c_str());
        safeStop();
        captureFaultDiagnostics();
        publishDiagnostics(true);
        return hardware_interface::return_type::ERROR;
      }
      if (active_ && motor.hasHeartbeat()) {
        const auto & heartbeat = motor.heartbeat();
        if (heartbeat.axis_error != 0U || heartbeat.axis_state != kClosedLoopControl) {
          RCLCPP_ERROR(
            get_logger(),
            "Motor left closed-loop control: joint=%s interface=%s can_id=%u "
            "axis_error=%s axis_state=%u flags=%s life=%u",
            joint.name.c_str(), joint.can_interface.c_str(), static_cast<unsigned>(joint.node_id),
            hexValue(heartbeat.axis_error).c_str(), static_cast<unsigned>(heartbeat.axis_state),
            hexValue(heartbeat.flags).c_str(), static_cast<unsigned>(heartbeat.life));
          safeStop();
          captureFaultDiagnostics();
          publishDiagnostics(true);
          return hardware_interface::return_type::ERROR;
        }
        if (motor.missedHeartbeats() > joint.reported_missed_heartbeats) {
          RCLCPP_WARN(
            get_logger(), "Heartbeat gap detected: joint=%s total_missed=%llu",
            joint.name.c_str(),
            static_cast<unsigned long long>(motor.missedHeartbeats()));
          joint.reported_missed_heartbeats = motor.missedHeartbeats();
        }
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
    const auto now = std::chrono::steady_clock::now();
    if (active_ && now >= next_telemetry_request_) {
      requestBusTelemetry();
      next_telemetry_request_ = now + telemetry_period_;
    }
    publishDiagnostics();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "CAN read failed: %s", error.what());
    safeStop();
    captureFaultDiagnostics();
    publishDiagnostics(true);
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
    captureFaultDiagnostics();
    publishDiagnostics(true);
    return hardware_interface::return_type::ERROR;
  }
  try {
    std::vector<double> commands;
    commands.reserve(joints_.size());
    bool all_commands_in_range = true;
    for (const auto & joint : joints_) {
      const double command = get_command<double>(interfaceKey(joint,
          hardware_interface::HW_IF_POSITION));
      if (!std::isfinite(command)) {
        throw std::out_of_range(joint.name + " position command is not finite");
      }
      commands.push_back(command);
      all_commands_in_range = all_commands_in_range &&
        command >= joint.lower && command <= joint.upper;
    }

    if (!position_limits_enabled_ && all_commands_in_range) {
      position_limits_enabled_ = true;
      RCLCPP_INFO(
        get_logger(),
        "All initial-pose commands are within their configured ranges; position limits enabled");
    }

    for (std::size_t index = 0; index < joints_.size(); ++index) {
      const auto & joint = joints_[index];
      const double command = commands[index];
      if (position_limits_enabled_ && (command < joint.lower || command > joint.upper)) {
        throw std::out_of_range(
                joint.name + " position command " + std::to_string(command) +
                " violates limits [" + std::to_string(joint.lower) + ", " +
                std::to_string(joint.upper) + "]");
      }
      const double motor_position = joint.direction * (command + joint.offset);
      joint.manager->motor(joint.node_id).sendCommand(
        {motor_position, 0.0, joint.kp, joint.kd, 0.0});
    }
    last_write_ = now;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Unsafe command rejected: %s", error.what());
    safeStop();
    captureFaultDiagnostics();
    publishDiagnostics(true);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void QuattroSystem::safeStop() noexcept
{
  active_ = false;
  position_limits_enabled_ = false;
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
