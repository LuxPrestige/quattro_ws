#include "quattro_hardware/quattro_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

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

bool parse_int_param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key, int & out,
  const rclcpp::Logger & logger)
{
  std::string text;
  if (!get_required(params, key, text, logger)) {
    return false;
  }
  try {
    size_t consumed = 0;
    const int value = std::stoi(text, &consumed);
    if (consumed != text.size()) {
      throw std::invalid_argument("trailing characters");
    }
    out = value;
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger, "Parameter '%s' is not a valid integer: '%s'", key.c_str(), text.c_str());
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
template<typename SendFn>
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
  ok = parse_ms_param(params, "heartbeat_timeout_ms", heartbeat_timeout_, logger) && ok;
  ok = parse_ms_param(params, "startup_timeout_ms", startup_timeout_, logger) && ok;
  ok = parse_ms_param(params, "closed_loop_timeout_ms", closed_loop_timeout_, logger) && ok;
  ok = parse_ms_param(params, "encoder_sync_timeout_ms", encoder_sync_timeout_, logger) && ok;
  ok = parse_int_param(params, "encoder_sync_frames", encoder_sync_frames_, logger) && ok;
  ok = parse_ms_param(params, "command_timeout_ms", command_timeout_, logger) && ok;
  ok = parse_ms_param(params, "scheduling_warning_ms", scheduling_warning_, logger) && ok;
  ok = parse_double_param(params, "rotor_velocity_limit_rev_s", rotor_velocity_limit_rev_s_,
      logger) &&
    ok;
  ok = parse_ms_param(params, "telemetry_period_ms", telemetry_period_, logger) && ok;

  if (ok && (!(current_limit_ > 0.0) || !(position_gain_ >= 0.0) ||
    !(velocity_gain_ >= 0.0) || !(velocity_integrator_gain_ >= 0.0)))
  {
    RCLCPP_ERROR(
      logger, "position_control current_limit must be positive and all gains must be "
      "non-negative");
    ok = false;
  }

  if (ok && encoder_sync_frames_ < 1) {
    RCLCPP_ERROR(
      logger, "encoder_sync_frames must be at least 1, got %d -- activation must consume at "
      "least one EncoderEstimate that arrived after closed-loop control was confirmed",
      encoder_sync_frames_);
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
        [&wanted](const auto & interface_info) {return interface_info.name == wanted;});
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
        [&wanted](const auto & interface_info) {return interface_info.name == wanted;});
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

std::unique_ptr<gim6010_driver::MotorManager> QuattroSystem::create_motor_manager(
  const std::vector<std::string> & buses,
  const std::vector<gim6010_driver::MotorRoute> & routes)
{
  return std::make_unique<gim6010_driver::MotorManager>(buses, routes);
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
    motor_manager_ = create_motor_manager(buses_, routes);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "Failed to construct MotorManager: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!motor_manager_->open()) {
    RCLCPP_ERROR(logger, "Failed to open one or more CAN buses");
    motor_manager_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Presence check before any configuration is sent. Heartbeat only: an
  // EncoderEstimate arriving here would prove the motor is alive too, but
  // its position field is meaningless before closed-loop control and must
  // not be allowed to look like a usable reading (AGENTS.md hardware
  // contract item 2), so nothing in configure reads one.
  if (!wait_for_all_heartbeats(startup_timeout_)) {
    RCLCPP_ERROR(
      logger, "One or more motors never sent a Heartbeat within startup_timeout_ms (%ld ms)",
      startup_timeout_.count());
    motor_manager_->close();
    motor_manager_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!check_pre_activation_faults()) {
    motor_manager_->close();
    motor_manager_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  bool ok = true;
  for (const auto & joint : joints_) {
    // Order matters and is the one confirmed on real hardware: limits,
    // then gains, then the controller mode, and only later (in
    // on_activate) closed-loop control.
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
          return motor_manager_->send_set_pos_gain(joint.node_id,
          static_cast<float>(position_gain_));
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
    // Position Control + Pos Filter is what makes closed-loop entry alone
    // hold the axis at its current position, which is why on_activate needs
    // no Set_Input_Pos of its own.
    const bool mode_ok = send_with_retry([&] {
          return motor_manager_->send_set_controller_mode(
          joint.node_id, gim6010_driver::ControlMode::kPositionControl,
          gim6010_driver::InputMode::kPosFilter);
      });
    if (!mode_ok) {
      RCLCPP_ERROR(
        logger, "Failed to set Position Control + Pos Filter on joint '%s'", joint.name.c_str());
      ok = false;
    }
  }

  // Deliberately no Clear_Errors here: a pre-existing fault must remain
  // visible until an operator decides what to do about it
  // (docs/packages/quattro_hardware.md section 4).

  if (!ok) {
    motor_manager_->close();
    motor_manager_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger, "QuattroSystem configured: %zu joints across %zu CAN bus(es), Position Control + "
    "Pos Filter", joints_.size(), buses_.size());
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

namespace
{

// Shared shape of every startup wait in this file: drain the buses every
// kPollInterval and re-test, until `predicate` holds or `timeout` elapses.
// Polling drains *all* buses, so waiting on one motor keeps every other
// motor's cached state and freshness timestamps current at the same time.
template<typename Predicate>
bool poll_until(
  gim6010_driver::MotorManager & manager, std::chrono::milliseconds timeout, Predicate predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  for (;; ) {
    manager.poll();
    if (predicate()) {
      return true;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(kPollInterval);
  }
}

}  // namespace

bool QuattroSystem::wait_for_all_heartbeats(std::chrono::milliseconds timeout)
{
  return poll_until(*motor_manager_, timeout, [this] {
             const auto now = std::chrono::steady_clock::now();
             return std::all_of(joints_.begin(), joints_.end(), [&](const JointContext & joint) {
                      const auto * motor = motor_manager_->motor(joint.node_id);
                      return motor != nullptr && motor->has_fresh_heartbeat(heartbeat_timeout_,
          now);
        });
    });
}

bool QuattroSystem::wait_for_all_fresh_feedback(std::chrono::milliseconds timeout)
{
  return poll_until(*motor_manager_, timeout, [this] {
             const auto now = std::chrono::steady_clock::now();
             return std::all_of(joints_.begin(), joints_.end(), [&](const JointContext & joint) {
                      const auto * motor = motor_manager_->motor(joint.node_id);
                      return motor != nullptr && motor->has_fresh_heartbeat(heartbeat_timeout_,
          now) &&
                             motor->has_fresh_feedback(feedback_timeout_, now);
        });
    });
}

bool QuattroSystem::check_pre_activation_faults()
{
  const auto & logger = get_logger();

  // Get_Error (0x03) goes unanswered on this firmware even though
  // Get_Encoder_Estimates (0x09) and Heartbeat (0x01) both arrive normally
  // (confirmed on the bus with candump/cansend against several nodes) -- a
  // wait on request_get_error()/last_error() here would never resolve, so
  // pre-existing faults are read from Heartbeat's axis_error field instead.
  bool fault_free = true;
  for (const auto & joint : joints_) {
    const auto * motor = motor_manager_->motor(joint.node_id);
    const auto heartbeat = motor != nullptr ? motor->last_heartbeat() : std::nullopt;
    if (!heartbeat || heartbeat->axis_error != 0) {
      RCLCPP_ERROR(
        logger, "Joint '%s' (node %u) has a pre-existing fault or no heartbeat "
        "(axis_error=0x%08X) -- refusing to continue. Clear the fault explicitly before "
        "retrying.",
        joint.name.c_str(), joint.node_id, heartbeat ? heartbeat->axis_error : 0U);
      fault_free = false;
    }
  }
  return fault_free;
}

bool QuattroSystem::wait_for_closed_loop(const JointContext & joint)
{
  const auto & logger = get_logger();
  auto * motor = motor_manager_->motor(joint.node_id);
  if (motor == nullptr) {
    RCLCPP_ERROR(logger, "Joint '%s' has no motor entry", joint.name.c_str());
    return false;
  }

  bool faulted = false;
  const bool reached = poll_until(*motor_manager_, closed_loop_timeout_, [&] {
        const auto now = std::chrono::steady_clock::now();
        if (!motor->has_fresh_heartbeat(heartbeat_timeout_, now)) {
          return false;
        }
        const auto heartbeat = motor->last_heartbeat();
        if (heartbeat->axis_error != 0) {
          faulted = true;
          return true;
        }
        return heartbeat->axis_state == gim6010_driver::AxisState::kClosedLoopControl;
    });

  if (faulted) {
    RCLCPP_ERROR(
      logger, "Joint '%s' reported axis_error=0x%08X while entering closed-loop control",
      joint.name.c_str(), motor->last_heartbeat()->axis_error);
    return false;
  }
  if (!reached) {
    RCLCPP_ERROR(
      logger, "Joint '%s' did not report Closed Loop Control within closed_loop_timeout_ms "
      "(%ld ms)", joint.name.c_str(), closed_loop_timeout_.count());
    return false;
  }
  return true;
}

bool QuattroSystem::wait_for_post_closed_loop_encoder(
  const JointContext & joint, std::uint64_t baseline_sequence, double & motor_rev_out)
{
  const auto & logger = get_logger();
  auto * motor = motor_manager_->motor(joint.node_id);
  if (motor == nullptr) {
    return false;
  }

  const std::uint64_t required =
    baseline_sequence + static_cast<std::uint64_t>(encoder_sync_frames_);
  const bool synced = poll_until(*motor_manager_, encoder_sync_timeout_, [&] {
        return motor->encoder_sequence() >= required;
    });
  if (!synced) {
    RCLCPP_ERROR(
      logger, "Joint '%s' produced only %lu of the %d EncoderEstimates required after "
      "closed-loop control was confirmed, within encoder_sync_timeout_ms (%ld ms)",
      joint.name.c_str(),
      static_cast<unsigned long>(motor->encoder_sequence() - baseline_sequence),
      encoder_sync_frames_, encoder_sync_timeout_.count());
    return false;
  }

  const auto estimate = motor->last_encoder_estimate();
  if (!estimate) {
    RCLCPP_ERROR(logger, "Joint '%s' encoder sequence advanced without a stored estimate",
      joint.name.c_str());
    return false;
  }
  motor_rev_out = estimate->position_rev;
  return true;
}

bool QuattroSystem::activate_joint(const JointContext & joint)
{
  const auto & logger = get_logger();
  auto * motor = motor_manager_->motor(joint.node_id);
  if (motor == nullptr) {
    RCLCPP_ERROR(logger, "Joint '%s' has no motor entry", joint.name.c_str());
    return false;
  }

  // No Set_Input_Pos anywhere in this function. Position Control + Pos
  // Filter (set in on_configure) makes the axis hold its current position
  // the moment it enters closed loop, so the old "read encoder, command it
  // back as a hold target, then enable" safe start is both unnecessary and
  // unsafe here: the encoder value it depended on is not trustworthy until
  // after the transition it was trying to prepare for.
  if (!motor_manager_->send_set_axis_state(
      joint.node_id, gim6010_driver::AxisState::kClosedLoopControl))
  {
    RCLCPP_ERROR(
      logger, "Failed to request closed-loop control for joint '%s'", joint.name.c_str());
    return false;
  }

  if (!wait_for_closed_loop(joint)) {
    return false;
  }

  // Baseline taken here, after closed loop is confirmed -- not before
  // Set_Axis_State. Frames counted from this point were all sampled by the
  // motor while it was already reporting closed-loop control, which is the
  // condition that makes the position trustworthy.
  const std::uint64_t baseline_sequence = motor->encoder_sequence();
  double motor_rev = 0.0;
  if (!wait_for_post_closed_loop_encoder(joint, baseline_sequence, motor_rev)) {
    return false;
  }

  const double joint_rad = motor_rev_to_joint_rad(motor_rev, joint.calibration);
  set_state<double>(joint.name + "/" + hardware_interface::HW_IF_POSITION, joint_rad);
  // The command interface starts at the same value the state does, so the
  // first write() after activation commands exactly where the joint already
  // is. Without this the first cycle would send whatever the interface was
  // default-initialised to (0.0 rad) and the joint would snap there.
  set_command<double>(joint.name + "/" + hardware_interface::HW_IF_POSITION, joint_rad);

  RCLCPP_INFO(
    logger, "Joint '%s' (node %u) closed loop, synchronized at %.6f rad (%.6f motor rev)",
    joint.name.c_str(), joint.node_id, joint_rad, motor_rev);
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

  if (!wait_for_all_heartbeats(startup_timeout_)) {
    RCLCPP_ERROR(
      logger, "One or more motors stopped sending Heartbeats before activation");
    safe_stop_all();
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!check_pre_activation_faults()) {
    safe_stop_all();
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : joints_) {
    if (!activate_joint(joint)) {
      RCLCPP_ERROR(
        logger, "Activation failed at joint '%s' -- idling all motors (no partial activation)",
        joint.name.c_str());
      safe_stop_all();
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Activating 12 joints in sequence takes longer than feedback_timeout_ms
  // in total. poll() during those waits drains every bus, so no joint's
  // feedback should have gone stale -- confirm that before handing control
  // to read(), which would otherwise safe-stop everything on its first
  // cycle if it had.
  if (!wait_for_all_fresh_feedback(startup_timeout_)) {
    RCLCPP_ERROR(
      logger, "Feedback went stale for one or more joints immediately after activation -- "
      "idling all motors");
    safe_stop_all();
    return hardware_interface::CallbackReturn::ERROR;
  }

  consecutive_write_failures_.assign(joints_.size(), 0);
  active_ = true;
  last_write_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    logger, "QuattroSystem activated: %zu joints in closed-loop control, synchronized to "
    "post-closed-loop encoder positions", joints_.size());
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

  // poll() only. The motors broadcast Get_Encoder_Estimates (~10 ms) and
  // Heartbeat (~100 ms) by themselves, so read() never requests either --
  // doing so would add 12 TX frames per cycle to a bus whose TX queue is
  // already the tightest resource here.
  motor_manager_->poll();

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
      // effort_Nm stays NaN: Position Control + Pos Filter reports no
      // measured torque, and no separate torque feedback path is wired in
      // (docs/packages/quattro_hardware.md section 7).
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
    // check_pre_activation_faults()); Heartbeat's axis_error is the only
    // fault source that actually arrives during read().
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
    const double joint_rad = get_command<double>(joint.name + "/" +
        hardware_interface::HW_IF_POSITION);
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
