#include "quattro_hardware/quattro_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace quattro_hardware
{

namespace
{

constexpr std::chrono::milliseconds kPollInterval{5};
// A single rejected Set_Input_Pos is treated as transient CAN TX queue
// pressure, not a fault -- see write() and docs/packages/quattro_hardware.md
// section 2 ("write()"). Observed on real hardware: a handful of motors
// (docs/development_status.md, "encoder estimate 자발적 브로드캐스트") keep
// broadcasting Get_Encoder_Estimates on their own well beyond what anyone
// requested, and that extra bus load occasionally fills the kernel's CAN TX
// queue (default qlen 10) for exactly one write() cycle. A joint with a real
// problem (disconnected, bus down, ...) fails every cycle and still trips
// this within kMaxConsecutiveWriteFailures * 10 ms.
constexpr int kMaxConsecutiveWriteFailures = 3;

bool get_required(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  std::string & out, const rclcpp::Logger & logger)
{
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty()) {
    RCLCPP_ERROR(logger, "Missing required parameter '%s'", key.c_str());
    return false;
  }
  out = it->second;
  return true;
}

bool parse_double_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  double & out, const rclcpp::Logger & logger)
{
  std::string text;
  if (!get_required(params, key, text, logger)) {
    return false;
  }
  try {
    size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed != text.size()) {
      throw std::invalid_argument("trailing characters");
    }
    out = value;
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger, "Parameter '%s' is not a valid number: '%s'", key.c_str(), text.c_str());
    return false;
  }
  return true;
}

bool parse_ms_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  std::chrono::milliseconds & out, const rclcpp::Logger & logger)
{
  double value = 0.0;
  if (!parse_double_param(params, key, value, logger)) {
    return false;
  }
  if (value < 0.0) {
    RCLCPP_ERROR(logger, "Parameter '%s' must not be negative, got %f", key.c_str(), value);
    return false;
  }
  out = std::chrono::milliseconds(static_cast<long>(std::lround(value)));
  return true;
}

// on_configure sends 3 frames per joint (Set_Limits, Set_Pos_Gain,
// Set_Vel_Gains) back to back for all 12 joints with no pacing -- observed
// on real hardware to overrun the kernel's CAN TX queue (default qlen 10)
// partway through each bus, which makes CanSocket::send() (a non-blocking
// write()) return false with no fault of the motor's own. This is a
// one-time startup burst, not the real-time write() hot path, so a bounded
// retry-with-backoff here is safe and lets the queue drain instead of
// failing configure outright.
template <typename SendFn>
bool send_with_retry(SendFn && send_fn)
{
  constexpr int kMaxAttempts = 20;
  constexpr auto kRetryDelay = std::chrono::milliseconds(2);
  for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
    if (send_fn()) {
      return true;
    }
    std::this_thread::sleep_for(kRetryDelay);
  }
  return false;
}

}  // namespace

hardware_interface::CallbackReturn QuattroSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  const auto base_result = hardware_interface::SystemInterface::on_init(params);
  if (base_result != hardware_interface::CallbackReturn::SUCCESS) {
    return base_result;
  }

  if (!parse_hardware_parameters()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!parse_joints()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!validate_interfaces()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "QuattroSystem initialized: %zu joints", joints_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool QuattroSystem::parse_hardware_parameters()
{
  const auto & params = info_.hardware_parameters;
  const auto & logger = get_logger();

  bool ok = true;
  ok = parse_double_param(params, "current_limit", current_limit_, logger) && ok;
  ok = parse_double_param(params, "position_gain", position_gain_, logger) && ok;
  ok = parse_double_param(params, "velocity_gain", velocity_gain_, logger) && ok;
  ok = parse_double_param(params, "velocity_integrator_gain", velocity_integrator_gain_, logger) &&
    ok;
  ok = parse_ms_param(params, "feedback_timeout_ms", feedback_timeout_, logger) && ok;
  ok = parse_ms_param(params, "feedback_request_period_ms", feedback_request_period_, logger) && ok;
  ok = parse_ms_param(params, "heartbeat_timeout_ms", heartbeat_timeout_, logger) && ok;
  ok = parse_ms_param(params, "startup_timeout_ms", startup_timeout_, logger) && ok;
  ok = parse_ms_param(params, "motor_activation_interval_ms", motor_activation_interval_, logger) &&
    ok;
  ok = parse_ms_param(params, "command_timeout_ms", command_timeout_, logger) && ok;
  ok = parse_ms_param(params, "scheduling_warning_ms", scheduling_warning_, logger) && ok;
  ok = parse_double_param(params, "rotor_velocity_limit_rev_s", rotor_velocity_limit_rev_s_, logger) &&
    ok;
  ok = parse_ms_param(params, "telemetry_period_ms", telemetry_period_, logger) && ok;

  if (ok && (!(current_limit_ > 0.0) || !(position_gain_ >= 0.0) ||
    !(velocity_gain_ >= 0.0) || !(velocity_integrator_gain_ >= 0.0)))
  {
    RCLCPP_ERROR(
      logger, "Direct Position current_limit must be positive and all gains must be non-negative");
    ok = false;
  }

  if (!ok) {
    RCLCPP_ERROR(logger, "One or more hardware_parameters failed to parse (see errors above)");
  }
  return ok;
}

bool QuattroSystem::parse_joints()
{
  const auto & logger = get_logger();
  joints_.clear();
  joints_.reserve(info_.joints.size());

  std::set<uint8_t> seen_node_ids;
  for (const auto & joint_info : info_.joints) {
    JointContext joint;
    joint.name = joint_info.name;
    const auto & params = joint_info.parameters;

    if (!get_required(params, "can_interface", joint.can_bus, logger)) {
      return false;
    }

    std::string can_id_text;
    if (!get_required(params, "can_id", can_id_text, logger)) {
      return false;
    }
    int can_id_value = -1;
    try {
      size_t consumed = 0;
      can_id_value = std::stoi(can_id_text, &consumed);
      if (consumed != can_id_text.size()) {
        throw std::invalid_argument("trailing characters");
      }
    } catch (const std::exception &) {
      RCLCPP_ERROR(
        logger, "Joint '%s' has invalid can_id '%s'", joint.name.c_str(), can_id_text.c_str());
      return false;
    }
    if (can_id_value < 0 || can_id_value > static_cast<int>(gim6010_driver::kMaxNodeId)) {
      RCLCPP_ERROR(
        logger, "Joint '%s' can_id %d is outside [0, %u]", joint.name.c_str(), can_id_value,
        gim6010_driver::kMaxNodeId);
      return false;
    }
    joint.node_id = static_cast<uint8_t>(can_id_value);
    if (!seen_node_ids.insert(joint.node_id).second) {
      RCLCPP_ERROR(logger, "Duplicate can_id %u across joints", joint.node_id);
      return false;
    }

    double direction = 0.0;
    if (!parse_double_param(params, "direction", direction, logger)) {
      return false;
    }
    if (direction != 1.0 && direction != -1.0) {
      RCLCPP_ERROR(
        logger, "Joint '%s' direction must be exactly 1.0 or -1.0, got %f", joint.name.c_str(),
        direction);
      return false;
    }
    joint.calibration.direction = direction;

    if (!parse_double_param(params, "offset", joint.calibration.offset, logger)) {
      return false;
    }

    if (!parse_double_param(params, "gear_ratio", joint.calibration.gear_ratio, logger)) {
      return false;
    }
    if (!(joint.calibration.gear_ratio > 0.0)) {
      RCLCPP_ERROR(
        logger, "Joint '%s' gear_ratio must be positive, got %f", joint.name.c_str(),
        joint.calibration.gear_ratio);
      return false;
    }

    joints_.push_back(std::move(joint));
  }

  if (joints_.empty()) {
    RCLCPP_ERROR(logger, "QuattroSystem requires at least one joint");
    return false;
  }
  return true;
}

bool QuattroSystem::validate_interfaces() const
{
  const auto & logger = get_logger();

  const std::vector<std::string> expected_command = {hardware_interface::HW_IF_POSITION};
  const std::vector<std::string> expected_state = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_EFFORT};

  for (const auto & joint_info : info_.joints) {
    if (joint_info.command_interfaces.size() != expected_command.size()) {
      RCLCPP_ERROR(
        logger, "Joint '%s' has %zu command interfaces, expected exactly %zu",
        joint_info.name.c_str(), joint_info.command_interfaces.size(),
        expected_command.size());
      return false;
    }
    for (const auto & wanted : expected_command) {
      const bool found = std::any_of(
        joint_info.command_interfaces.begin(), joint_info.command_interfaces.end(),
        [&wanted](const auto & interface_info) { return interface_info.name == wanted; });
      if (!found) {
        RCLCPP_ERROR(
          logger, "Joint '%s' is missing required command interface '%s'",
          joint_info.name.c_str(), wanted.c_str());
        return false;
      }
    }

    if (joint_info.state_interfaces.size() != expected_state.size()) {
      RCLCPP_ERROR(
        logger, "Joint '%s' has %zu state interfaces, expected exactly %zu (position/velocity/"
        "effort)", joint_info.name.c_str(), joint_info.state_interfaces.size(),
        expected_state.size());
      return false;
    }
    for (const auto & wanted : expected_state) {
      const bool found = std::any_of(
        joint_info.state_interfaces.begin(), joint_info.state_interfaces.end(),
        [&wanted](const auto & interface_info) { return interface_info.name == wanted; });
      if (!found) {
        RCLCPP_ERROR(
          logger, "Joint '%s' is missing required state interface '%s'", joint_info.name.c_str(),
          wanted.c_str());
        return false;
      }
    }
  }
  return true;
}

hardware_interface::CallbackReturn QuattroSystem::on_configure(const rclcpp_lifecycle::State &)
{
  const auto & logger = get_logger();

  buses_.clear();
  std::vector<gim6010_driver::MotorRoute> routes;
  std::set<std::string> seen_buses;
  for (const auto & joint : joints_) {
    if (seen_buses.insert(joint.can_bus).second) {
      buses_.push_back(joint.can_bus);
    }
    routes.push_back(gim6010_driver::MotorRoute{joint.node_id, joint.can_bus});
  }

  try {
    motor_manager_ = std::make_unique<gim6010_driver::MotorManager>(buses_, routes);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "Failed to construct MotorManager: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!motor_manager_->open()) {
    RCLCPP_ERROR(logger, "Failed to open one or more CAN buses");
    motor_manager_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  bool ok = true;
  for (const auto & joint : joints_) {
    const bool limits_ok = send_with_retry([&] {
        return motor_manager_->send_set_limits(
          joint.node_id, static_cast<float>(rotor_velocity_limit_rev_s_),
          static_cast<float>(current_limit_));
      });
    if (!limits_ok) {
      RCLCPP_ERROR(logger, "Failed to send Set_Limits to joint '%s'", joint.name.c_str());
      ok = false;
    }
    const bool gains_ok =
      send_with_retry([&] {
        return motor_manager_->send_set_pos_gain(joint.node_id, static_cast<float>(position_gain_));
      }) &&
      send_with_retry([&] {
        return motor_manager_->send_set_vel_gains(
          joint.node_id, static_cast<float>(velocity_gain_),
          static_cast<float>(velocity_integrator_gain_));
      });
    if (!gains_ok) {
      RCLCPP_ERROR(
        logger, "Failed to apply position/velocity gains to joint '%s'", joint.name.c_str());
      ok = false;
    }
  }

  // Deliberately no Clear_Errors here: a pre-existing fault must remain
  // visible until on_activate's fault check and an operator decide what to
  // do about it (docs/packages/quattro_hardware.md section 4).

  if (!ok) {
    motor_manager_->close();
    motor_manager_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger, "QuattroSystem configured: %zu joints across %zu CAN bus(es)", joints_.size(),
    buses_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuattroSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (motor_manager_) {
    motor_manager_->close();
    motor_manager_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool QuattroSystem::wait_for_all_motors_fresh_feedback(std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  std::set<uint8_t> pending;
  for (const auto & joint : joints_) {
    pending.insert(joint.node_id);
  }

  while (!pending.empty() && std::chrono::steady_clock::now() < deadline) {
    for (const auto & joint : joints_) {
      if (pending.count(joint.node_id) != 0) {
        motor_manager_->request_encoder_estimate(joint.node_id);
      }
    }
    std::this_thread::sleep_for(kPollInterval);
    motor_manager_->poll();

    const auto now = std::chrono::steady_clock::now();
    for (auto it = pending.begin(); it != pending.end();) {
      const auto * motor = motor_manager_->motor(*it);
      if (motor != nullptr && motor->has_fresh_feedback(feedback_timeout_, now) &&
        motor->has_fresh_heartbeat(heartbeat_timeout_, now))
      {
        it = pending.erase(it);
      } else {
        ++it;
      }
    }
  }

  return pending.empty();
}

bool QuattroSystem::wait_for_fresh_feedback_and_no_faults()
{
  const auto & logger = get_logger();

  if (!wait_for_all_motors_fresh_feedback(startup_timeout_)) {
    RCLCPP_ERROR(
      logger, "One or more motors never answered with fresh encoder feedback and heartbeat "
      "within startup_timeout_ms (%ld ms)", startup_timeout_.count());
    return false;
  }

  // Get_Error (0x03) goes unanswered on this firmware even though
  // Get_Encoder_Estimates (0x09) and Heartbeat (0x01) both respond normally
  // (confirmed on the bus with candump/cansend against several nodes) -- a
  // wait on request_get_error()/last_error() here would never resolve, so
  // pre-existing faults are read from Heartbeat's axis_error field instead
  // (docs/packages/quattro_hardware.md section 0).
  bool fault_free = true;
  for (const auto & joint : joints_) {
    const auto * motor = motor_manager_->motor(joint.node_id);
    const auto heartbeat = motor != nullptr ? motor->last_heartbeat() : std::nullopt;
    if (!heartbeat || heartbeat->axis_error != 0) {
      RCLCPP_ERROR(
        logger, "Joint '%s' (node %u) has a pre-existing fault or no heartbeat "
        "(axis_error=0x%08X) -- refusing to activate. Clear the fault explicitly before "
        "retrying.",
        joint.name.c_str(), joint.node_id, heartbeat ? heartbeat->axis_error : 0U);
      fault_free = false;
    }
  }
  return fault_free;
}

bool QuattroSystem::activate_joint(const JointContext & joint)
{
  const auto & logger = get_logger();
  auto * motor = motor_manager_->motor(joint.node_id);
  const auto estimate = motor != nullptr ? motor->last_encoder_estimate() : std::nullopt;
  if (!estimate) {
    RCLCPP_ERROR(
      logger, "Joint '%s' has no encoder estimate at activation time", joint.name.c_str());
    return false;
  }
  const double current_joint_rad = motor_rev_to_joint_rad(estimate->position_rev, joint.calibration);

  constexpr auto control_mode = gim6010_driver::ControlMode::kPositionControl;
  constexpr auto input_mode = gim6010_driver::InputMode::kDirect;

  if (!motor_manager_->send_set_controller_mode(joint.node_id, control_mode, input_mode)) {
    RCLCPP_ERROR(logger, "Failed to set controller mode for joint '%s'", joint.name.c_str());
    return false;
  }

  // Send the hold-at-current-position target BEFORE requesting closed-loop
  // control, not after: Input_Pos is just a register and stays whatever it
  // was last set to (e.g. from a previous session, possibly many turns
  // away) until we write it, regardless of axis state. Enabling closed-loop
  // first left a real window where the axis actively drove toward that
  // stale setpoint before our correcting Set_Input_Pos arrived -- on real
  // hardware this showed up as the joint visibly snapping in the wrong
  // direction (by a large, sometimes 180+ degree margin) the instant
  // activation reached it, even though the target and the just-measured
  // current position are mathematically identical. Writing Input_Pos while
  // still idle means closed-loop starts from a stationary target with no
  // window to chase a stale one.
  gim6010_driver::SetInputPosCommand command;
  command.position_rev = static_cast<float>(joint_rad_to_motor_rev(current_joint_rad, joint.calibration));
  if (!motor_manager_->send_set_input_pos(joint.node_id, command)) {
    RCLCPP_ERROR(logger, "Joint '%s' rejected initial hold position", joint.name.c_str());
    return false;
  }

  if (!motor_manager_->send_set_axis_state(
      joint.node_id, gim6010_driver::AxisState::kClosedLoopControl))
  {
    RCLCPP_ERROR(
      logger, "Failed to request closed-loop control for joint '%s'", joint.name.c_str());
    return false;
  }

  // Stability check: hold for motor_activation_interval_ and confirm no
  // fault appears before moving on to the next motor. Get_Error never
  // answers on this firmware (see wait_for_fresh_feedback_and_no_faults()),
  // so this reads Heartbeat's axis_error instead.
  const auto stable_until = std::chrono::steady_clock::now() + motor_activation_interval_;
  while (std::chrono::steady_clock::now() < stable_until) {
    std::this_thread::sleep_for(kPollInterval);
    motor_manager_->poll();

    const auto now = std::chrono::steady_clock::now();
    if (!motor->has_fresh_heartbeat(heartbeat_timeout_, now)) {
      RCLCPP_ERROR(
        logger, "Joint '%s' heartbeat went stale during activation stabilization",
        joint.name.c_str());
      return false;
    }
    const auto heartbeat = motor->last_heartbeat();
    if (heartbeat && heartbeat->axis_error != 0) {
      RCLCPP_ERROR(
        logger, "Joint '%s' faulted during activation stabilization (axis_error=0x%08X)",
        joint.name.c_str(), heartbeat->axis_error);
      return false;
    }
  }

  return true;
}

void QuattroSystem::safe_stop_all()
{
  if (!motor_manager_) {
    return;
  }
  for (const auto & joint : joints_) {
    motor_manager_->send_set_axis_state(joint.node_id, gim6010_driver::AxisState::kIdle);
  }
  active_ = false;
}

hardware_interface::CallbackReturn QuattroSystem::on_activate(const rclcpp_lifecycle::State &)
{
  const auto & logger = get_logger();
  if (!motor_manager_) {
    RCLCPP_ERROR(logger, "on_activate called before on_configure succeeded");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!wait_for_fresh_feedback_and_no_faults()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : joints_) {
    if (!activate_joint(joint)) {
      RCLCPP_ERROR(
        logger, "Activation failed at joint '%s' -- stopping all motors (no partial "
        "activation)", joint.name.c_str());
      safe_stop_all();
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Sequentially activating all 12 joints (motor_activation_interval_ms
  // each) takes well over feedback_timeout_ms in total, and nothing
  // re-requests an already-activated joint's encoder estimate while later
  // joints are still being activated -- so by the time this loop finishes,
  // the earliest-activated joints' feedback is already older than
  // feedback_timeout_ms (observed on real hardware: read()'s first cycle
  // after activation immediately flagged most joints "stale feedback" and
  // safe-stopped everything moments after a successful activation). Refresh
  // every joint's feedback here, before handing control back to read(), so
  // the normal read() polling cadence starts from fresh timestamps.
  if (!wait_for_all_motors_fresh_feedback(startup_timeout_)) {
    RCLCPP_ERROR(
      logger, "Feedback went stale for one or more joints immediately after sequential "
      "activation -- stopping all motors");
    safe_stop_all();
    return hardware_interface::CallbackReturn::ERROR;
  }

  consecutive_write_failures_.assign(joints_.size(), 0);
  active_ = true;
  last_write_time_ = std::chrono::steady_clock::now();
  last_feedback_request_time_ = last_write_time_;
  RCLCPP_INFO(logger, "QuattroSystem activated: %zu joints in closed-loop control", joints_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QuattroSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  safe_stop_all();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type QuattroSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!motor_manager_) {
    return hardware_interface::return_type::ERROR;
  }

  const auto now = std::chrono::steady_clock::now();
  if (active_ && has_prior_read_time_ && (now - last_read_time_) > scheduling_warning_) {
    RCLCPP_WARN(
      get_logger(), "read() scheduling gap %ld ms exceeds scheduling_warning_ms (%ld ms) -- "
      "not a fault by itself, but watch for repeated warnings",
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_read_time_).count(),
      scheduling_warning_.count());
  }
  last_read_time_ = now;
  has_prior_read_time_ = true;

  motor_manager_->poll();

  if (now - last_feedback_request_time_ >= feedback_request_period_) {
    for (const auto & joint : joints_) {
      // A few motors on this firmware broadcast Get_Encoder_Estimates on
      // their own at a much higher rate than we would ever ask for (RxSdo
      // does not respond on this firmware, so there is no way to configure
      // which -- docs/packages/gim6010_driver.md section 0). dispatch()
      // updates the same "last feedback" timestamp regardless of whether a
      // reading was requested or arrived unprompted, so if it is already
      // fresher than what we're about to ask for, requesting again only
      // adds bus load for nothing. This has no node list to maintain: it
      // just naturally stops polling whichever motors happen to keep
      // themselves fresh on their own.
      const auto * motor = motor_manager_->motor(joint.node_id);
      if (motor != nullptr && motor->has_fresh_feedback(feedback_request_period_, now)) {
        continue;
      }
      motor_manager_->request_encoder_estimate(joint.node_id);
    }
    last_feedback_request_time_ = now;
  }

  bool faulted = false;
  for (const auto & joint : joints_) {
    const auto * motor = motor_manager_->motor(joint.node_id);

    double position_rad = std::numeric_limits<double>::quiet_NaN();
    double velocity_rad_s = std::numeric_limits<double>::quiet_NaN();
    double effort_Nm = std::numeric_limits<double>::quiet_NaN();

    if (motor != nullptr) {
      if (const auto estimate = motor->last_encoder_estimate()) {
        position_rad = motor_rev_to_joint_rad(estimate->position_rev, joint.calibration);
        velocity_rad_s = motor_rev_s_to_joint_rad_s(estimate->velocity_rev_s, joint.calibration);
      }
      // effort_Nm stays NaN: no measured-torque path is wired in for
      // Direct Position mode (docs/packages/quattro_hardware.md section 3).
    }

    set_state<double>(joint.name + "/" + hardware_interface::HW_IF_POSITION, position_rad);
    set_state<double>(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, velocity_rad_s);
    set_state<double>(joint.name + "/" + hardware_interface::HW_IF_EFFORT, effort_Nm);

    if (!active_ || motor == nullptr) {
      continue;
    }
    if (!motor->has_fresh_feedback(feedback_timeout_, now)) {
      RCLCPP_ERROR(get_logger(), "Joint '%s': stale feedback", joint.name.c_str());
      faulted = true;
    }
    if (!motor->has_fresh_heartbeat(heartbeat_timeout_, now)) {
      RCLCPP_ERROR(get_logger(), "Joint '%s': stale heartbeat", joint.name.c_str());
      faulted = true;
    }
    // Get_Error never answers on this firmware (see
    // wait_for_fresh_feedback_and_no_faults()); Heartbeat's axis_error is
    // the only fault source that actually arrives during read().
    if (const auto heartbeat = motor->last_heartbeat()) {
      if (heartbeat->axis_error != 0) {
        RCLCPP_ERROR(
          get_logger(), "Joint '%s': axis fault (axis_error=0x%08X)", joint.name.c_str(),
          heartbeat->axis_error);
        faulted = true;
      }
    }
  }

  if (active_) {
    for (const auto & bus : buses_) {
      if (motor_manager_->bus_state(bus) != gim6010_driver::CanBusState::kActive) {
        RCLCPP_ERROR(
          get_logger(), "CAN bus '%s' is not ERROR-ACTIVE (state=%d)", bus.c_str(),
          static_cast<int>(motor_manager_->bus_state(bus)));
        faulted = true;
      }
    }
    if (now - last_write_time_ > command_timeout_) {
      RCLCPP_ERROR(
        get_logger(), "Command watchdog expired: write() has not run for %ld ms",
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_write_time_).count());
      faulted = true;
    }
  }

  if (faulted) {
    safe_stop_all();
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuattroSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!motor_manager_ || !active_) {
    return hardware_interface::return_type::OK;
  }

  last_write_time_ = std::chrono::steady_clock::now();

  bool faulted = false;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto & joint = joints_[i];
    const double joint_rad = get_command<double>(joint.name + "/" + hardware_interface::HW_IF_POSITION);
    gim6010_driver::SetInputPosCommand command;
    command.position_rev =
      static_cast<float>(joint_rad_to_motor_rev(joint_rad, joint.calibration));
    if (motor_manager_->send_set_input_pos(joint.node_id, command)) {
      consecutive_write_failures_[i] = 0;
      continue;
    }
    const int failures = ++consecutive_write_failures_[i];
    if (failures >= kMaxConsecutiveWriteFailures) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s': position command rejected %d cycles in a row -- treating as "
        "a real fault", joint.name.c_str(), failures);
      faulted = true;
    } else {
      RCLCPP_WARN(
        get_logger(), "Joint '%s': position command rejected (%d/%d consecutive) -- likely "
        "transient CAN TX queue pressure, not yet a fault", joint.name.c_str(), failures,
        kMaxConsecutiveWriteFailures);
    }
  }

  if (faulted) {
    safe_stop_all();
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace quattro_hardware

PLUGINLIB_EXPORT_CLASS(quattro_hardware::QuattroSystem, hardware_interface::SystemInterface)
