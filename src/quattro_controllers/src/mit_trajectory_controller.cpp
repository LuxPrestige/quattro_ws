#include "quattro_controllers/mit_trajectory_controller.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace quattro_controllers
{

controller_interface::CallbackReturn MitTrajectoryController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("kp", {});
    auto_declare<std::vector<double>>("kd", {});
    auto_declare<double>("command_timeout", 0.5);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "MIT controller init failed: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MitTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    for (const auto * interface_name : kCommandInterfaces) {
      config.names.push_back(joint + "/" + interface_name);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
MitTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

controller_interface::CallbackReturn MitTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  kp_ = get_node()->get_parameter("kp").as_double_array();
  kd_ = get_node()->get_parameter("kd").as_double_array();
  command_timeout_ = get_node()->get_parameter("command_timeout").as_double();
  if (joints_.empty() || kp_.size() != joints_.size() || kd_.size() != joints_.size() ||
    !std::isfinite(command_timeout_) || command_timeout_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "joints, per-joint kp/kd, and positive timeout are required");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    if (!std::isfinite(kp_[index]) || kp_[index] < 0.0 || kp_[index] > 500.0 ||
      !std::isfinite(kd_[index]) || kd_[index] < 0.0 || kd_[index] > 5.0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "MIT gain is outside the GIM6010 range");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  subscription_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory", 10,
    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr message) {
      auto normalized = std::make_shared<PendingTrajectory>();
      if (!normalizeTrajectory(*message, normalized->trajectory)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Rejected invalid MIT joint trajectory");
        return;
      }
      normalized->sequence = ++received_sequence_;
      pending_trajectory_.writeFromNonRT(normalized);
    });
  last_positions_.assign(joints_.size(), 0.0);
  segment_start_positions_.assign(joints_.size(), 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitTrajectoryController::on_activate(
  const rclcpp_lifecycle::State &)
{
  command_index_.clear();
  state_index_.clear();
  for (std::size_t index = 0; index < command_interfaces_.size(); ++index) {
    command_index_[command_interfaces_[index].get_name()] = index;
  }
  for (std::size_t index = 0; index < state_interfaces_.size(); ++index) {
    state_index_[state_interfaces_[index].get_name()] = index;
  }
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    const auto state = state_index_.find(joints_[index] + "/position");
    if (state == state_index_.end()) {return controller_interface::CallbackReturn::ERROR;}
    const auto value = state_interfaces_[state->second].get_optional();
    if (!value || !std::isfinite(*value)) {return controller_interface::CallbackReturn::ERROR;}
    last_positions_[index] = *value;
  }
  active_trajectory_.reset();
  active_sequence_ = 0;
  if (!writeCommands(last_positions_, std::vector<double>(joints_.size(), 0.0))) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  const auto configured_kp = kp_;
  std::fill(kp_.begin(), kp_.end(), 0.0);
  (void)writeCommands(last_positions_, std::vector<double>(joints_.size(), 0.0));
  kp_ = configured_kp;
  return controller_interface::CallbackReturn::SUCCESS;
}

double MitTrajectoryController::pointTime(
  const trajectory_msgs::msg::JointTrajectoryPoint & point) const
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1.0e-9;
}

bool MitTrajectoryController::normalizeTrajectory(
  const trajectory_msgs::msg::JointTrajectory & input,
  trajectory_msgs::msg::JointTrajectory & output) const
{
  if (input.joint_names.size() != joints_.size() || input.points.empty()) {return false;}
  std::vector<std::size_t> mapping(joints_.size());
  for (std::size_t target = 0; target < joints_.size(); ++target) {
    const auto found = std::find(input.joint_names.begin(), input.joint_names.end(),
        joints_[target]);
    if (found == input.joint_names.end()) {return false;}
    mapping[target] = static_cast<std::size_t>(std::distance(input.joint_names.begin(), found));
  }
  output.joint_names = joints_;
  output.points.reserve(input.points.size());
  double previous_time = -1.0;
  for (const auto & input_point : input.points) {
    if (input_point.positions.size() != joints_.size()) {return false;}
    const double time = pointTime(input_point);
    if (!std::isfinite(time) || time < 0.0 || time <= previous_time) {return false;}
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(joints_.size());
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      point.positions[index] = input_point.positions[mapping[index]];
      if (!std::isfinite(point.positions[index])) {return false;}
    }
    point.time_from_start = input_point.time_from_start;
    output.points.push_back(std::move(point));
    previous_time = time;
  }
  return true;
}

bool MitTrajectoryController::writeCommands(
  const std::vector<double> & positions, const std::vector<double> & velocities)
{
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    const std::array<double, 5> values{
      positions[index], velocities[index], kp_[index], kd_[index], 0.0};
    for (std::size_t field = 0; field < kCommandInterfaces.size(); ++field) {
      const auto command = command_index_.find(
        joints_[index] + "/" + kCommandInterfaces[field]);
      if (command == command_index_.end() ||
        !command_interfaces_[command->second].set_value(values[field]))
      {
        return false;
      }
    }
  }
  return true;
}

controller_interface::return_type MitTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  const auto pending = *pending_trajectory_.readFromRT();
  if (pending && pending->sequence != active_sequence_) {
    active_trajectory_ = pending;
    active_sequence_ = pending->sequence;
    trajectory_start_ = time;
    segment_start_positions_ = last_positions_;
  }
  if (!active_trajectory_) {
    return writeCommands(last_positions_, std::vector<double>(joints_.size(), 0.0)) ?
           controller_interface::return_type::OK : controller_interface::return_type::ERROR;
  }

  const double elapsed = (time - trajectory_start_).seconds();
  const auto & points = active_trajectory_->trajectory.points;
  std::size_t upper = 0;
  while (upper < points.size() && pointTime(points[upper]) < elapsed) {++upper;}
  std::vector<double> positions(joints_.size());
  if (upper == points.size()) {
    positions = points.back().positions;
    if (elapsed > pointTime(points.back()) + command_timeout_) {
      active_trajectory_.reset();
    }
  } else {
    const auto & end = points[upper];
    const std::vector<double> & start = upper == 0 ?
      segment_start_positions_ : points[upper - 1].positions;
    const double start_time = upper == 0 ? 0.0 : pointTime(points[upper - 1]);
    const double duration = pointTime(end) - start_time;
    const double ratio = duration <= 0.0 ? 1.0 :
      std::clamp((elapsed - start_time) / duration, 0.0, 1.0);
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      positions[index] = start[index] + (end.positions[index] - start[index]) * ratio;
    }
  }
  last_positions_ = positions;
  return writeCommands(positions, std::vector<double>(joints_.size(), 0.0)) ?
         controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

}  // namespace quattro_controllers

PLUGINLIB_EXPORT_CLASS(
  quattro_controllers::MitTrajectoryController,
  controller_interface::ControllerInterface)
