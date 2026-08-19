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
#include "quattro_hardware/joint_transform.hpp"
#include "rclcpp/rclcpp.hpp"

namespace quattro_hardware
{
namespace
{
constexpr auto kConfigureFrameInterval = std::chrono::milliseconds{2};
constexpr std::uint8_t kClosedLoopControl = 8;
constexpr std::uint8_t kHeartbeatFaultMask = 0x0F;
constexpr double kTwoPi = 6.28318530717958647692;

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

bool parameterBool(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & key,
  bool fallback)
{
  const auto value = parameters.find(key);
  if (value == parameters.end()) {return fallback;}
  if (value->second == "true" || value->second == "1") {return true;}
  if (value->second == "false" || value->second == "0") {return false;}
  throw std::invalid_argument(key + " must be true or false");
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
    const auto control_method = parameterString(hp, "control_method", "mit");
    if (control_method == "direct_position") {
      control_method_ = ControlMethod::kDirectPosition;
    } else if (control_method == "direct_velocity") {
      control_method_ = ControlMethod::kDirectVelocity;
    } else if (control_method == "direct_torque") {
      control_method_ = ControlMethod::kDirectTorque;
    } else if (control_method == "mit") {
      control_method_ = ControlMethod::kMit;
    } else {
      throw std::invalid_argument(
              "control_method must be direct_position, direct_velocity, direct_torque, or mit");
    }
    feedback_timeout_ = std::chrono::milliseconds(parameterLong(hp, "feedback_timeout_ms", 150));
    feedback_request_period_ = std::chrono::milliseconds(
      parameterLong(hp, "feedback_request_period_ms", 50));
    heartbeat_timeout_ = std::chrono::milliseconds(parameterLong(hp, "heartbeat_timeout_ms", 400));
    startup_timeout_ = std::chrono::milliseconds(parameterLong(hp, "startup_timeout_ms", 1000));
    motor_activation_interval_ = std::chrono::milliseconds(
      parameterLong(hp, "motor_activation_interval_ms", 500));
    command_timeout_ = std::chrono::milliseconds(parameterLong(hp, "command_timeout_ms", 250));
    scheduling_warning_ =
      std::chrono::milliseconds(parameterLong(hp, "scheduling_warning_ms", 50));
    motor_velocity_limit_ = parameterDouble(hp, "rotor_velocity_limit_rev_s", 5.0);
    motor_current_limit_ = parameterDouble(hp, "motor_current_limit_a", 5.0);
    apply_position_gains_ = parameterBool(hp, "apply_position_gains", false);
    position_gains_.position_gain = parameterDouble(hp, "position_gain", 0.0);
    position_gains_.velocity_gain = parameterDouble(hp, "velocity_gain", 0.0);
    position_gains_.velocity_integrator_gain =
      parameterDouble(hp, "velocity_integrator_gain", 0.0);
    engagement_duration_ = std::chrono::milliseconds(
      parameterLong(hp, "engagement_duration_ms", 1000));
    telemetry_period_ = std::chrono::milliseconds(parameterLong(hp, "telemetry_period_ms", 500));
    if (feedback_timeout_.count() <= 0 || feedback_request_period_.count() <= 0 ||
      feedback_request_period_ >= feedback_timeout_ || heartbeat_timeout_.count() <= 0 ||
      startup_timeout_.count() <= 0 || telemetry_period_.count() <= 0 ||
      motor_activation_interval_.count() < 0 ||
      command_timeout_.count() <= 0 || scheduling_warning_.count() <= 0 ||
      scheduling_warning_ >= command_timeout_ || motor_velocity_limit_ <= 0.0 ||
      motor_current_limit_ <= 0.0 || engagement_duration_.count() <= 0)
    {
      throw std::invalid_argument(
              "hardware timeouts and motor limits must be positive, and feedback request "
              "period must be shorter than feedback timeout");
    }
    if (apply_position_gains_ && control_method_ != ControlMethod::kDirectPosition) {
      throw std::invalid_argument("position gains can only be applied in direct_position mode");
    }
    if (apply_position_gains_) {
      (void)gim6010_driver::encodePositionGain(
        static_cast<float>(position_gains_.position_gain));
      (void)gim6010_driver::encodeVelocityGains(
        static_cast<float>(position_gains_.velocity_gain),
        static_cast<float>(position_gains_.velocity_integrator_gain));
    }

    std::unordered_set<std::string> motor_addresses;
    for (const auto & info : params.hardware_info.joints) {
      std::unordered_set<std::string> command_names;
      for (const auto & command : info.command_interfaces) {
        command_names.insert(command.name);
      }
      const std::unordered_set<std::string> expected_commands =
        control_method_ == ControlMethod::kMit ?
        std::unordered_set<std::string>{"position", "velocity", "kp", "kd", "effort"} :
      std::unordered_set<std::string>{commandInterfaceName()};
      if (command_names != expected_commands) {
        throw std::invalid_argument(
                info.name + " command interfaces do not match the selected control method");
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
      joint.gear_ratio = parameterDouble(info.parameters, "gear_ratio", 8.0);
      joint.current_limit = parameterDouble(
        info.parameters, "current_limit", motor_current_limit_);
      joint.mit_kp = parameterDouble(info.parameters, "mit_kp", 20.0);
      joint.mit_kd = parameterDouble(info.parameters, "mit_kd", 0.5);
      if (joint.direction != 1.0 && joint.direction != -1.0) {
        throw std::invalid_argument(info.name + " direction must be 1 or -1");
      }
      if (!std::isfinite(joint.gear_ratio) || joint.gear_ratio <= 0.0) {
        throw std::invalid_argument(info.name + " gear_ratio must be positive");
      }
      if (!std::isfinite(joint.current_limit) || joint.current_limit <= 0.0 ||
        joint.current_limit > 100.0)
      {
        throw std::invalid_argument(info.name + " current_limit must be in (0, 100] A");
      }
      const auto command_entry = std::find_if(
        info.command_interfaces.begin(), info.command_interfaces.end(),
        [this](const auto & entry) {
          return entry.name ==
                 (control_method_ == ControlMethod::kMit ? "position" : commandInterfaceName());
        });
      const auto & command = *command_entry;
      if (!command.min.empty()) {joint.lower = std::stod(command.min);}
      if (!command.max.empty()) {joint.upper = std::stod(command.max);}
      if (!(joint.lower < joint.upper)) {
        throw std::invalid_argument(info.name + " has invalid limits");
      }
      const std::string address = joint.can_interface + ":" + std::to_string(node_id);
      if (!motor_addresses.insert(address).second) {
        throw std::invalid_argument("duplicate motor " + address);
      }
      if (control_method_ == ControlMethod::kMit) {
        gim6010_driver::validateCommand({0.0, 0.0, joint.mit_kp, joint.mit_kd, 0.0});
      }
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
    manager->addMotor(joint.node_id, joint.gear_ratio);
    joint.manager = manager.get();
  }
  return true;
}

void QuattroSystem::configureMotorControllers()
{
  for (auto & joint : joints_) {
    auto & motor = joint.manager->motor(joint.node_id);
    motor.setLimits(static_cast<float>(motor_velocity_limit_),
        static_cast<float>(joint.current_limit));
    RCLCPP_INFO(
      get_logger(), "Configured current limit: joint=%s can_id=%u limit=%.3fA",
      joint.name.c_str(), static_cast<unsigned>(joint.node_id), joint.current_limit);
    std::this_thread::sleep_for(kConfigureFrameInterval);
    switch (control_method_) {
      case ControlMethod::kDirectPosition:
        motor.configurePositionControl(gim6010_driver::PositionInputMode::kDirect);
        if (apply_position_gains_) {motor.setPositionControlGains(position_gains_);}
        break;
      case ControlMethod::kDirectVelocity: motor.configureVelocityControl(); break;
      case ControlMethod::kDirectTorque: motor.configureTorqueControl(); break;
      case ControlMethod::kMit: motor.configureMitControl(); break;
    }
    std::this_thread::sleep_for(kConfigureFrameInterval);
  }
}

// This is Quattro's absolute-position-at-startup step. GIM6010-8 has one
// onboard encoder (a 14-bit single-turn absolute magnetic encoder) and no
// second, independently-readable position sensor -- see
// docs/gim6010_hardware.md section 11. "Confirming absolute position at
// boot" therefore means: wait for a fresh Get_Encoder_Estimates (0x009, or
// the MIT feedback frame 0x008 once MIT is configured) from that one
// encoder, which is already absolute within one rotor revolution the
// instant it arrives. It does NOT resolve which output-shaft rotation the
// joint is in when the joint's range of motion exceeds one rotor-turn's
// output-equivalent span (360deg / gear_ratio); that multi-turn ambiguity
// is a real, currently-unverified gap (does firmware persist the turn count
// across a power cycle?) and is not solved by any code here -- see the
// "실기 검증 필요" note in docs/gim6010_hardware.md section 11.
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
                 (motor.heartbeat().flags & kHeartbeatFaultMask) == 0U &&
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

void QuattroSystem::sendActivationHold(Joint & joint, double output_position)
{
  auto & motor = joint.manager->motor(joint.node_id);
  if (control_method_ == ControlMethod::kDirectPosition) {
    motor.setPosition(static_cast<float>(output_position * joint.gear_ratio / kTwoPi));
  } else if (control_method_ == ControlMethod::kDirectVelocity) {
    motor.setVelocity(0.0F);
  } else if (control_method_ == ControlMethod::kDirectTorque) {
    motor.setTorque(0.0F);
  } else {
    motor.sendMitCommand({output_position, 0.0, 0.0, 0.0, 0.0});
  }
}

bool QuattroSystem::waitForMotorOperational(
  std::size_t motor_index, const std::vector<double> & hold_positions)
{
  auto & target_joint = joints_.at(motor_index);
  const auto deadline = std::chrono::steady_clock::now() + startup_timeout_;
  auto next_feedback_request = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() < deadline) {
    for (std::size_t index = 0; index <= motor_index; ++index) {
      sendActivationHold(joints_[index], hold_positions[index]);
    }
    const auto now = std::chrono::steady_clock::now();
    if (control_method_ != ControlMethod::kMit && now >= next_feedback_request) {
      requestMotorFeedback(motor_index + 1U);
      next_feedback_request = now + feedback_request_period_;
    }
    pollManagers();
    auto & motor = target_joint.manager->motor(target_joint.node_id);
    if (motor.hasHeartbeat() && !motor.heartbeatStale(heartbeat_timeout_)) {
      const auto & heartbeat = motor.heartbeat();
      if (heartbeat.axis_error != 0U || (heartbeat.flags & kHeartbeatFaultMask) != 0U) {
        RCLCPP_ERROR(
          get_logger(),
          "Motor fault during sequential activation: joint=%s axis_error=%s flags=%s",
          target_joint.name.c_str(), hexValue(heartbeat.axis_error).c_str(),
          hexValue(heartbeat.flags).c_str());
        return false;
      }
      if (heartbeat.axis_state == kClosedLoopControl && motor.hasFeedback() &&
        !motor.feedbackStale(feedback_timeout_))
      {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  RCLCPP_ERROR(
    get_logger(), "Sequential activation timed out: joint=%s interface=%s can_id=%u",
    target_joint.name.c_str(), target_joint.can_interface.c_str(),
    static_cast<unsigned>(target_joint.node_id));
  return false;
}

bool QuattroSystem::waitForActivationInterval(
  std::size_t motor_index, const std::vector<double> & hold_positions)
{
  const auto deadline = std::chrono::steady_clock::now() + motor_activation_interval_;
  auto next_feedback_request = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() < deadline) {
    for (std::size_t index = 0; index <= motor_index; ++index) {
      sendActivationHold(joints_[index], hold_positions[index]);
    }
    const auto now = std::chrono::steady_clock::now();
    if (control_method_ != ControlMethod::kMit && now >= next_feedback_request) {
      requestMotorFeedback(motor_index + 1U);
      next_feedback_request = now + feedback_request_period_;
    }
    pollManagers();
    for (std::size_t index = 0; index <= motor_index; ++index) {
      const auto & joint = joints_[index];
      const auto & motor = joint.manager->motor(joint.node_id);
      if (!motor.hasHeartbeat() || motor.heartbeatStale(heartbeat_timeout_) ||
        motor.heartbeat().axis_error != 0U ||
        (motor.heartbeat().flags & kHeartbeatFaultMask) != 0U ||
        motor.heartbeat().axis_state != kClosedLoopControl ||
        !motor.hasFeedback() || motor.feedbackStale(feedback_timeout_))
      {
        RCLCPP_ERROR(
          get_logger(), "Previously enabled motor became unhealthy: joint=%s",
          joint.name.c_str());
        return false;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  return true;
}

bool QuattroSystem::waitForPreflightHeartbeats()
{
  const auto deadline = std::chrono::steady_clock::now() + startup_timeout_;
  while (std::chrono::steady_clock::now() < deadline) {
    pollManagers();
    const bool ready = std::all_of(joints_.begin(), joints_.end(), [this](const Joint & joint) {
          const auto & motor = joint.manager->motor(joint.node_id);
          return motor.hasHeartbeat() && !motor.heartbeatStale(heartbeat_timeout_);
    });
    if (ready) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  for (const auto & joint : joints_) {
    const auto & motor = joint.manager->motor(joint.node_id);
    if (!motor.hasHeartbeat() || motor.heartbeatStale(heartbeat_timeout_)) {
      RCLCPP_ERROR(get_logger(), "Missing fresh preflight heartbeat from %s", joint.name.c_str());
      return false;
    }
    const auto & heartbeat = motor.heartbeat();
    if (heartbeat.axis_error != 0U || (heartbeat.flags & kHeartbeatFaultMask) != 0U) {
      RCLCPP_ERROR(
        get_logger(),
        "Pre-existing motor fault preserved: joint=%s axis_error=%s flags=%s",
        joint.name.c_str(), hexValue(heartbeat.axis_error).c_str(),
        hexValue(heartbeat.flags).c_str());
      return false;
    }
    if (heartbeat.axis_state != static_cast<std::uint8_t>(gim6010_driver::AxisState::kIdle)) {
      RCLCPP_ERROR(
        get_logger(), "Unsafe preflight state: joint=%s axis_state=%u (expected idle)",
        joint.name.c_str(), static_cast<unsigned>(heartbeat.axis_state));
      return false;
    }
  }
  return true;
}

hardware_interface::CallbackReturn QuattroSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    managers_.clear();
    configureMotors();
    if (!waitForInitialFeedback()) {
      RCLCPP_ERROR(get_logger(), "Safe Start failed: not all motors supplied feedback");
      captureFaultDiagnostics();
      safeStop();
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!waitForPreflightHeartbeats()) {
      RCLCPP_ERROR(get_logger(), "Safe Start failed: heartbeat/fault preflight rejected");
      captureFaultDiagnostics();
      safeStop();
      return hardware_interface::CallbackReturn::ERROR;
    }
    configureMotorControllers();
    for (const auto & joint : joints_) {
      const auto & feedback = joint.manager->motor(joint.node_id).feedback();
      const JointTransform transform(joint.direction, joint.offset);
      const double joint_position = transform.toJointPosition(feedback.output_position_rad);
      set_state(interfaceKey(joint, hardware_interface::HW_IF_POSITION), joint_position);
      set_state(interfaceKey(joint, hardware_interface::HW_IF_VELOCITY),
        transform.toJointVelocity(feedback.output_velocity_rad_s));
      set_state(interfaceKey(joint, hardware_interface::HW_IF_EFFORT),
        transform.toJointEffort(feedback.output_torque_nm));
      const double initial_command =
        (control_method_ == ControlMethod::kDirectPosition ||
        control_method_ == ControlMethod::kMit) ?
        joint_position : 0.0;
      set_command(interfaceKey(joint, commandInterfaceName()), initial_command);
      if (control_method_ == ControlMethod::kMit) {
        set_command(interfaceKey(joint, hardware_interface::HW_IF_VELOCITY), 0.0);
        set_command(interfaceKey(joint, "kp"), joint.mit_kp);
        set_command(interfaceKey(joint, "kd"), joint.mit_kd);
        set_command(interfaceKey(joint, hardware_interface::HW_IF_EFFORT), 0.0);
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to configure motors: %s", error.what());
    captureFaultDiagnostics();
    safeStop();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuattroSystem::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    std::vector<double> hold_positions;
    hold_positions.reserve(joints_.size());
    for (const auto & joint : joints_) {
      hold_positions.push_back(joint.manager->motor(joint.node_id).feedback().output_position_rad);
    }

    for (std::size_t index = 0; index < joints_.size(); ++index) {
      auto & joint = joints_[index];
      auto & motor = joint.manager->motor(joint.node_id);
      sendActivationHold(joint, hold_positions[index]);
      std::this_thread::sleep_for(kConfigureFrameInterval);
      motor.enable();
      if (!waitForMotorOperational(index, hold_positions)) {
        throw std::runtime_error("motor failed sequential activation: " + joint.name);
      }
      RCLCPP_INFO(
        get_logger(), "Activated motor %zu/%zu: joint=%s interface=%s can_id=%u",
        index + 1U, joints_.size(), joint.name.c_str(), joint.can_interface.c_str(),
        static_cast<unsigned>(joint.node_id));
      if (!waitForActivationInterval(index, hold_positions)) {
        throw std::runtime_error("motor fault during activation interval: " + joint.name);
      }
    }

    if (!waitForInitialFeedback()) {
      throw std::runtime_error(
              "not all enabled motors supplied fresh 0x08 or 0x09 feedback");
    }
    if (!waitForOperationalHeartbeats()) {
      throw std::runtime_error("not all motors entered closed-loop control without errors");
    }

    constexpr auto control_period = std::chrono::milliseconds{10};

    for (std::size_t refresh = 0;
      control_method_ == ControlMethod::kMit && refresh < 3; ++refresh)
    {
      for (auto & joint : joints_) {
        auto & motor = joint.manager->motor(joint.node_id);
        motor.sendMitCommand({motor.feedback().output_position_rad, 0.0, 0.0, joint.mit_kd, 0.0});
      }
      std::this_thread::sleep_for(control_period);
      pollManagers();
    }

    for (std::size_t index = 0; index < joints_.size(); ++index) {
      const auto & joint = joints_[index];
      const auto & motor = joint.manager->motor(joint.node_id);
      if (motor.feedbackStale(feedback_timeout_)) {
        throw std::runtime_error(
                joint.name + " did not supply fresh feedback after zero-Kp activation");
      }
      hold_positions[index] = motor.feedback().output_position_rad;
    }

    const auto engagement_steps = control_method_ == ControlMethod::kMit ?
      std::max<std::int64_t>(1, engagement_duration_.count() / control_period.count()) : 0;
    auto next_cycle = std::chrono::steady_clock::now();
    for (std::int64_t step = 1; step <= engagement_steps; ++step) {
      const double gain_ratio =
        static_cast<double>(step) / static_cast<double>(engagement_steps);
      for (std::size_t index = 0; index < joints_.size(); ++index) {
        const auto & joint = joints_[index];
        joint.manager->motor(joint.node_id).sendMitCommand(
          {hold_positions[index], 0.0, joint.mit_kp * gain_ratio, joint.mit_kd, 0.0});
      }
      next_cycle += control_period;
      std::this_thread::sleep_until(next_cycle);
      pollManagers();
      for (const auto & joint : joints_) {
        const auto & heartbeat = joint.manager->motor(joint.node_id).heartbeat();
        if (heartbeat.axis_error != 0U ||
          (heartbeat.flags & kHeartbeatFaultMask) != 0U ||
          heartbeat.axis_state != kClosedLoopControl)
        {
          throw std::runtime_error(
                  joint.name + " left closed-loop control during gain engagement");
        }
      }
    }

    if (control_method_ == ControlMethod::kMit) {
      // Leave one complete control period between the final engagement burst and
      // the controller manager's first write cycle so a small SocketCAN TX queue
      // cannot retain frames from both cycles.
      std::this_thread::sleep_for(control_period);
      pollManagers();
    }

    for (const auto & joint : joints_) {
      const auto & feedback = joint.manager->motor(joint.node_id).feedback();
      const JointTransform transform(joint.direction, joint.offset);
      const double joint_position = transform.toJointPosition(feedback.output_position_rad);
      set_state(interfaceKey(joint, hardware_interface::HW_IF_POSITION), joint_position);
      set_state(interfaceKey(joint, hardware_interface::HW_IF_VELOCITY),
        transform.toJointVelocity(feedback.output_velocity_rad_s));
      set_state(interfaceKey(joint, hardware_interface::HW_IF_EFFORT),
        transform.toJointEffort(feedback.output_torque_nm));
      const double initial_command =
        (control_method_ == ControlMethod::kDirectPosition ||
        control_method_ == ControlMethod::kMit) ?
        joint_position : 0.0;
      set_command(interfaceKey(joint, commandInterfaceName()), initial_command);
      if (control_method_ == ControlMethod::kMit) {
        set_command(interfaceKey(joint, hardware_interface::HW_IF_VELOCITY), 0.0);
        set_command(interfaceKey(joint, "kp"), joint.mit_kp);
        set_command(interfaceKey(joint, "kd"), joint.mit_kd);
        set_command(interfaceKey(joint, hardware_interface::HW_IF_EFFORT), 0.0);
      }
    }
    last_write_ = std::chrono::steady_clock::now();
    next_telemetry_request_ = last_write_;
    next_telemetry_motor_ = 0;
    next_feedback_request_ = last_write_;
    next_diagnostics_publish_ = last_write_;
    active_ = true;
    RCLCPP_INFO(
      get_logger(),
      "All motors engaged at their current positions; joint limits are enforced immediately");
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Safe Start activation failed: %s", error.what());
    captureFaultDiagnostics();
    publishDiagnostics(true);
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

void QuattroSystem::requestMotorTelemetry()
{
  for (auto & joint : joints_) {
    auto & motor = joint.manager->motor(joint.node_id);
    motor.requestIq();
    motor.requestBusVoltageCurrent();
  }
}

void QuattroSystem::requestMotorFeedback(std::size_t motor_count)
{
  const auto count = std::min(motor_count, joints_.size());
  for (std::size_t index = 0; index < count; ++index) {
    joints_[index].manager->motor(joints_[index].node_id).requestEncoderEstimates();
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
    requestMotorTelemetry();
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
      if (heartbeat.axis_error != 0U ||
        (heartbeat.flags & kHeartbeatFaultMask) != 0U ||
        heartbeat.axis_state != kClosedLoopControl)
      {
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
    status.values.push_back(diagnosticValue("control_method",
        control_method_ == ControlMethod::kDirectPosition ? "direct_position" :
        control_method_ == ControlMethod::kDirectVelocity ? "direct_velocity" :
        control_method_ == ControlMethod::kDirectTorque ? "direct_torque" : "mit"));
    if (motor.hasFeedback()) {
      status.values.push_back(diagnosticValue("feedback_age_ms", std::to_string(
          std::chrono::duration_cast<std::chrono::milliseconds>(motor.feedbackAge()).count())));
      status.values.push_back(diagnosticValue("position_rad",
          std::to_string(motor.feedback().output_position_rad)));
      status.values.push_back(diagnosticValue("velocity_rad_s",
          std::to_string(motor.feedback().output_velocity_rad_s)));
      status.values.push_back(diagnosticValue("torque_nm",
          std::to_string(motor.feedback().output_torque_nm)));
    }
    if (motor.hasHeartbeat()) {
      status.values.push_back(diagnosticValue("heartbeat_age_ms", std::to_string(
          std::chrono::duration_cast<std::chrono::milliseconds>(motor.heartbeatAge()).count())));
    }
    if (motor.hasBusVoltageCurrent()) {
      status.values.push_back(diagnosticValue(
          "bus_voltage_v", std::to_string(motor.busVoltageCurrent().voltage)));
      status.values.push_back(diagnosticValue(
          "bus_current_a", std::to_string(motor.busVoltageCurrent().current)));
    }
    if (motor.hasIq()) {
      status.values.push_back(diagnosticValue(
          "iq_setpoint_a", std::to_string(motor.iq().setpoint)));
      status.values.push_back(diagnosticValue(
          "iq_measured_a", std::to_string(motor.iq().measured)));
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
  for (const auto & manager_entry : managers_) {
    const auto & error = manager_entry.second->canErrorStatus();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "quattro_hardware/can/" + manager_entry.first;
    status.hardware_id = manager_entry.first;
    status.level = error.bus_off_frames > 0U ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      (error.passive_frames > 0U || error.warning_frames > 0U ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK);
    status.message = error.bus_off_frames > 0U ? "CAN bus-off observed" :
      (error.total_frames > 0U ? "CAN error frames observed" : "CAN bus healthy");
    status.values.push_back(diagnosticValue("error_frames", std::to_string(error.total_frames)));
    status.values.push_back(diagnosticValue("ack_error_frames",
        std::to_string(error.ack_error_frames)));
    status.values.push_back(diagnosticValue("warning_frames",
        std::to_string(error.warning_frames)));
    status.values.push_back(diagnosticValue("passive_frames",
        std::to_string(error.passive_frames)));
    status.values.push_back(diagnosticValue("bus_off_frames",
        std::to_string(error.bus_off_frames)));
    status.values.push_back(diagnosticValue("tx_error_counter",
        std::to_string(error.tx_error_counter)));
    status.values.push_back(diagnosticValue("rx_error_counter",
        std::to_string(error.rx_error_counter)));
    status.values.push_back(diagnosticValue("rx_dropped_frames",
        std::to_string(error.rx_dropped_frames)));
    array.status.push_back(std::move(status));
  }
  diagnostics_publisher_->publish(array);
}

// Runtime position/velocity state has exactly one source: the feedback
// GIM6010-8's single onboard encoder delivers via Get_Encoder_Estimates
// (0x009) or, in MIT mode, the MIT feedback frame (0x008) -- both decoded
// by gim6010_driver and exposed as Gim6010Motor::feedback(). There is no
// secondary encoder to read here, and a stale/missing feedback below always
// becomes a fault (see the feedbackStale() branch) rather than falling back
// to anything else.
hardware_interface::return_type QuattroSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    const auto now = std::chrono::steady_clock::now();
    if (active_ && control_method_ != ControlMethod::kMit &&
      now >= next_feedback_request_)
    {
      requestMotorFeedback(joints_.size());
      next_feedback_request_ = now + feedback_request_period_;
    }
    pollManagers();
    if (active_) {
      for (const auto & manager_entry : managers_) {
        const auto state = manager_entry.second->canErrorStatus().state;
        if (state == gim6010_driver::CanBusState::kBusOff) {
          fault_reason_ = FaultReason::kCanBusOff;
          throw std::runtime_error("CAN bus-off detected on " + manager_entry.first);
        }
        if (state == gim6010_driver::CanBusState::kErrorPassive) {
          fault_reason_ = FaultReason::kCanPassive;
          throw std::runtime_error("CAN error-passive detected on " + manager_entry.first);
        }
      }
    }
    for (auto & joint : joints_) {
      const auto & motor = joint.manager->motor(joint.node_id);
      if (active_ && motor.feedbackStale(feedback_timeout_)) {
        fault_reason_ = FaultReason::kFeedbackTimeout;
        RCLCPP_ERROR(get_logger(), "Stale feedback from %s", joint.name.c_str());
        captureFaultDiagnostics();
        publishDiagnostics(true);
        safeStop();
        return hardware_interface::return_type::ERROR;
      }
      if (active_ && motor.heartbeatStale(heartbeat_timeout_)) {
        fault_reason_ = FaultReason::kHeartbeatTimeout;
        RCLCPP_ERROR(get_logger(), "Stale heartbeat from %s", joint.name.c_str());
        captureFaultDiagnostics();
        publishDiagnostics(true);
        safeStop();
        return hardware_interface::return_type::ERROR;
      }
      if (active_ && motor.hasHeartbeat()) {
        const auto & heartbeat = motor.heartbeat();
        if (heartbeat.axis_error != 0U ||
          (heartbeat.flags & kHeartbeatFaultMask) != 0U ||
          heartbeat.axis_state != kClosedLoopControl)
        {
          fault_reason_ = FaultReason::kMotorFault;
          RCLCPP_ERROR(
            get_logger(),
            "Motor left closed-loop control: joint=%s interface=%s can_id=%u "
            "axis_error=%s axis_state=%u flags=%s life=%u",
            joint.name.c_str(), joint.can_interface.c_str(), static_cast<unsigned>(joint.node_id),
            hexValue(heartbeat.axis_error).c_str(), static_cast<unsigned>(heartbeat.axis_state),
            hexValue(heartbeat.flags).c_str(), static_cast<unsigned>(heartbeat.life));
          captureFaultDiagnostics();
          publishDiagnostics(true);
          safeStop();
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
      const JointTransform transform(joint.direction, joint.offset);
      set_state(interfaceKey(joint, hardware_interface::HW_IF_POSITION),
        transform.toJointPosition(feedback.output_position_rad));
      set_state(interfaceKey(joint, hardware_interface::HW_IF_VELOCITY),
        transform.toJointVelocity(feedback.output_velocity_rad_s));
      set_state(interfaceKey(joint, hardware_interface::HW_IF_EFFORT),
        transform.toJointEffort(feedback.output_torque_nm));
    }
    if (active_ && now >= next_telemetry_request_) {
      auto & joint = joints_.at(next_telemetry_motor_);
      auto & motor = joint.manager->motor(joint.node_id);
      motor.requestIq();
      motor.requestBusVoltageCurrent();
      next_telemetry_motor_ = (next_telemetry_motor_ + 1U) % joints_.size();
      const auto slot_period = std::max(
        std::chrono::milliseconds{1},
        std::chrono::milliseconds{
          telemetry_period_.count() / static_cast<std::int64_t>(joints_.size())});
      next_telemetry_request_ = now + slot_period;
    }
    publishDiagnostics();
  } catch (const std::exception & error) {
    if (fault_reason_ == FaultReason::kNone) {fault_reason_ = FaultReason::kCanIo;}
    RCLCPP_ERROR(get_logger(), "CAN read failed: %s", error.what());
    captureFaultDiagnostics();
    publishDiagnostics(true);
    safeStop();
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuattroSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {return hardware_interface::return_type::OK;}
  const auto now = std::chrono::steady_clock::now();
  if (now - last_write_ > scheduling_warning_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_node()->get_clock(), 1000,
      "controller_manager scheduling delay exceeded warning threshold");
  }
  if (now - last_write_ > command_timeout_) {
    fault_reason_ = FaultReason::kCommandTimeout;
    RCLCPP_ERROR(get_logger(), "Hardware command watchdog expired");
    captureFaultDiagnostics();
    publishDiagnostics(true);
    safeStop();
    return hardware_interface::return_type::ERROR;
  }
  try {
    std::vector<double> commands;
    commands.reserve(joints_.size());
    for (const auto & joint : joints_) {
      const double command = get_command<double>(interfaceKey(joint, commandInterfaceName()));
      if (!std::isfinite(command)) {
        throw std::out_of_range(
                joint.name + " " + commandInterfaceName() + " command is not finite");
      }
      if (command < joint.lower || command > joint.upper) {
        throw std::out_of_range(
                joint.name + " " + commandInterfaceName() + " command " +
                std::to_string(command) +
                " violates limits [" + std::to_string(joint.lower) + ", " +
                std::to_string(joint.upper) + "]");
      }
      commands.push_back(command);
    }

    for (std::size_t index = 0; index < joints_.size(); ++index) {
      const auto & joint = joints_[index];
      const double command = commands[index];
      auto & motor = joint.manager->motor(joint.node_id);
      const JointTransform transform(joint.direction, joint.offset);
      if (control_method_ == ControlMethod::kDirectPosition) {
        const double output_position = transform.toOutputPosition(command);
        motor.setPosition(
          static_cast<float>(output_position * joint.gear_ratio / kTwoPi));
      } else if (control_method_ == ControlMethod::kDirectVelocity) {
        motor.setVelocity(
          static_cast<float>(
            transform.toOutputVelocity(command) * joint.gear_ratio / kTwoPi));
      } else if (control_method_ == ControlMethod::kDirectTorque) {
        motor.setTorque(static_cast<float>(
            transform.toOutputEffort(command) / joint.gear_ratio));
      } else {
        const double output_position = transform.toOutputPosition(command);
        const double velocity = get_command<double>(
          interfaceKey(joint, hardware_interface::HW_IF_VELOCITY));
        const double kp = get_command<double>(interfaceKey(joint, "kp"));
        const double kd = get_command<double>(interfaceKey(joint, "kd"));
        const double effort = get_command<double>(
          interfaceKey(joint, hardware_interface::HW_IF_EFFORT));
        if (!std::isfinite(velocity) || velocity < -4.0 || velocity > 4.0 ||
          !std::isfinite(kp) || kp < 0.0 || kp > 500.0 ||
          !std::isfinite(kd) || kd < 0.0 || kd > 5.0 ||
          !std::isfinite(effort) || effort < -10.0 || effort > 10.0)
        {
          throw std::out_of_range(joint.name + " has an invalid MIT command field");
        }
        motor.sendMitCommand(
          {output_position, transform.toOutputVelocity(velocity), kp, kd,
            transform.toOutputEffort(effort)});
      }
    }
    last_write_ = now;
  } catch (const std::exception & error) {
    fault_reason_ = FaultReason::kInvalidCommand;
    RCLCPP_ERROR(get_logger(), "Unsafe command rejected: %s", error.what());
    captureFaultDiagnostics();
    publishDiagnostics(true);
    safeStop();
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuattroSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces, const std::vector<std::string> &)
{
  if (control_method_ != ControlMethod::kMit || start_interfaces.empty()) {
    return hardware_interface::return_type::OK;
  }
  const std::unordered_set<std::string> requested(start_interfaces.begin(), start_interfaces.end());
  for (const auto & joint : joints_) {
    for (const auto * interface_name : {"position", "velocity", "kp", "kd", "effort"}) {
      if (requested.count(interfaceKey(joint, interface_name)) == 0U) {
        RCLCPP_ERROR(
          get_logger(), "MIT controller must claim all five command interfaces for every joint");
        return hardware_interface::return_type::ERROR;
      }
    }
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

const char * QuattroSystem::commandInterfaceName() const noexcept
{
  switch (control_method_) {
    case ControlMethod::kDirectPosition:
    case ControlMethod::kMit: return hardware_interface::HW_IF_POSITION;
    case ControlMethod::kDirectVelocity: return hardware_interface::HW_IF_VELOCITY;
    case ControlMethod::kDirectTorque: return hardware_interface::HW_IF_EFFORT;
  }
  return hardware_interface::HW_IF_POSITION;
}
}  // namespace quattro_hardware

PLUGINLIB_EXPORT_CLASS(quattro_hardware::QuattroSystem, hardware_interface::SystemInterface)
