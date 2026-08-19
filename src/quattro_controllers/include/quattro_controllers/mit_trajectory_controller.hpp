#ifndef QUATTRO_CONTROLLERS__MIT_TRAJECTORY_CONTROLLER_HPP_
#define QUATTRO_CONTROLLERS__MIT_TRAJECTORY_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace quattro_controllers
{

class MitTrajectoryController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct PendingTrajectory
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    std::uint64_t sequence{0};
  };

  bool normalizeTrajectory(
    const trajectory_msgs::msg::JointTrajectory & input,
    trajectory_msgs::msg::JointTrajectory & output) const;
  bool writeCommands(
    const std::vector<double> & positions, const std::vector<double> & velocities);
  double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint & point) const;

  static constexpr std::array<const char *, 5> kCommandInterfaces{
    "position", "velocity", "kp", "kd", "effort"};
  std::vector<std::string> joints_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  double command_timeout_{0.5};
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<PendingTrajectory>> pending_trajectory_;
  std::shared_ptr<PendingTrajectory> active_trajectory_;
  std::uint64_t received_sequence_{0};
  std::uint64_t active_sequence_{0};
  rclcpp::Time trajectory_start_{0, 0, RCL_ROS_TIME};
  std::vector<double> segment_start_positions_;
  std::vector<double> last_positions_;
  std::unordered_map<std::string, std::size_t> command_index_;
  std::unordered_map<std::string, std::size_t> state_index_;
};

}  // namespace quattro_controllers

#endif  // QUATTRO_CONTROLLERS__MIT_TRAJECTORY_CONTROLLER_HPP_
