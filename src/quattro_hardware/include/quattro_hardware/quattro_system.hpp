#ifndef QUATTRO_HARDWARE__QUATTRO_SYSTEM_HPP_
#define QUATTRO_HARDWARE__QUATTRO_SYSTEM_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "gim6010_driver/motor_manager.hpp"
#include "hardware_interface/system_interface.hpp"
#include "quattro_hardware/joint_transform.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace quattro_hardware
{

// hardware_interface::SystemInterface connecting the 12 Quattro joints to
// real GIM6010-8/GDS68 motors over gim6010_driver. Command/state interface
// names, per-joint parameters, and hardware parameters are the contract
// already fixed by quattro_description/urdf/quattro.urdf.xacro -- see
// docs/packages/quattro_hardware.md for the full design rationale.
class QuattroSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct JointContext
  {
    std::string name;
    JointCalibration calibration;
    uint8_t node_id{0};
    std::string can_bus;
    double current_limit{5.0};
  };

  bool parse_hardware_parameters();
  bool parse_joints();
  bool validate_interfaces() const;

  // Blocks (with bounded sleeps) until every motor has answered with fresh
  // position feedback and no pre-existing fault, or startup_timeout_
  // elapses. Called once at the top of on_activate.
  bool wait_for_fresh_feedback_and_no_faults();
  // Brings a single motor into closed-loop control, holding at its current
  // measured position. Blocks for up to ~motor_activation_interval_.
  bool activate_joint(const JointContext & joint);
  // Idles every configured motor. Safe to call even if some were never
  // activated.
  void safe_stop_all();

  bool apply_position_gains_{false};
  double position_gain_{0.0};
  double velocity_gain_{0.0};
  double velocity_integrator_gain_{0.0};
  std::chrono::milliseconds feedback_timeout_{150};
  std::chrono::milliseconds feedback_request_period_{50};
  std::chrono::milliseconds heartbeat_timeout_{400};
  std::chrono::milliseconds startup_timeout_{1000};
  std::chrono::milliseconds motor_activation_interval_{100};
  std::chrono::milliseconds command_timeout_{250};
  std::chrono::milliseconds scheduling_warning_{50};
  double rotor_velocity_limit_rev_s_{5.0};
  double motor_current_limit_a_{5.0};
  std::chrono::milliseconds telemetry_period_{500};

  std::vector<JointContext> joints_;
  std::vector<std::string> buses_;
  std::unique_ptr<gim6010_driver::MotorManager> motor_manager_;

  std::chrono::steady_clock::time_point last_feedback_request_time_{};
  // Updated at the top of write() while active; read() checks this to
  // detect "write() has stopped being called" (see quattro_system.cpp for
  // why command staleness can't be detected by comparing command values).
  std::chrono::steady_clock::time_point last_write_time_{};
  // Interval between consecutive read() calls, used only to log a
  // scheduling-jitter warning (never triggers safe_stop_all() by itself --
  // that is what feedback_timeout_/heartbeat_timeout_ are for).
  std::chrono::steady_clock::time_point last_read_time_{};
  bool has_prior_read_time_{false};
  bool active_{false};
};

}  // namespace quattro_hardware

#endif  // QUATTRO_HARDWARE__QUATTRO_SYSTEM_HPP_
