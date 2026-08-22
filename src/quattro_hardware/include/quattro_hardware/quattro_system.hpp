#ifndef QUATTRO_HARDWARE__QUATTRO_SYSTEM_HPP_
#define QUATTRO_HARDWARE__QUATTRO_SYSTEM_HPP_

#include <chrono>
#include <cstdint>
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
//
// The startup sequence follows the hardware contract confirmed on the real
// GIM6010-8 (AGENTS.md, "GIM6010-8 실기 확인 하드웨어 계약"):
//
//   on_configure:  Set_Limits -> Set_Pos_Gain -> Set_Vel_Gains
//                  -> Set_Controller_Mode(Position Control, Pos Filter)
//   on_activate:   Set_Axis_State(Closed Loop) -> confirm via Heartbeat
//                  -> first EncoderEstimate that arrived after that
//                  -> sync ROS state AND the command interface to it
//
// Two rules from that contract shape everything below: an EncoderEstimate
// received before closed-loop control is not a usable joint position, and
// closed-loop entry alone makes the axis hold its current position, so no
// Set_Input_Pos is ever sent as a startup "safe start" measure.
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

protected:
  // The single seam a test needs to run this whole class against an
  // in-memory CAN bus: a test subclass returns a MotorManager built on a
  // fake transport (gim6010_driver::MotorManager::SocketFactory) and every
  // other line of the startup sequence runs unmodified. Production
  // overrides nothing.
  virtual std::unique_ptr<gim6010_driver::MotorManager> create_motor_manager(
    const std::vector<std::string> & buses,
    const std::vector<gim6010_driver::MotorRoute> & routes);

private:
  struct JointContext
  {
    std::string name;
    JointCalibration calibration;
    uint8_t node_id{0};
    std::string can_bus;
  };

  bool parse_hardware_parameters();
  bool parse_joints();
  bool validate_interfaces() const;

  // Blocks (with bounded sleeps), draining every bus via poll() each
  // kPollInterval, until every motor has a fresh Heartbeat or `timeout`
  // elapses. Nothing is requested: motors broadcast Heartbeat and
  // Get_Encoder_Estimates on their own. Used by on_configure as the
  // "is every motor present and talking?" check -- Heartbeat only, because
  // before closed-loop control an EncoderEstimate proves liveness but its
  // position field is not a usable joint position.
  bool wait_for_all_heartbeats(std::chrono::milliseconds timeout);
  // Same wait, additionally requiring fresh encoder feedback. Only valid
  // after every motor has reached closed-loop control; called at the end of
  // on_activate so read() starts from timestamps stamped moments ago rather
  // than from whenever the first joint in the activation loop was synced.
  bool wait_for_all_fresh_feedback(std::chrono::milliseconds timeout);
  // Checks Heartbeat.axis_error on every motor. Get_Error (0x03) goes
  // unanswered on this firmware, so Heartbeat is the only fault source that
  // actually arrives (docs/packages/gim6010_driver.md section 5).
  bool check_pre_activation_faults();

  // Waits until this motor's Heartbeat reports Closed Loop Control with
  // axis_error == 0, or closed_loop_timeout_ elapses.
  bool wait_for_closed_loop(const JointContext & joint);
  // Waits for encoder_sync_frames_ Get_Encoder_Estimates frames to arrive
  // *after* `baseline_sequence`, and returns the position from the last of
  // them. `baseline_sequence` is captured at the instant closed loop was
  // confirmed, so every frame counted here was sampled by the motor while
  // already in closed-loop control -- the only condition under which the
  // GIM6010-8's reported position is trustworthy.
  bool wait_for_post_closed_loop_encoder(
    const JointContext & joint, std::uint64_t baseline_sequence, double & motor_rev_out);
  // Full per-joint activation: request closed loop, confirm it via
  // Heartbeat, wait for a post-closed-loop encoder, and sync both the ROS
  // position state and the position command interface to it. Sends no
  // Set_Input_Pos.
  bool activate_joint(const JointContext & joint);
  // Idles every configured motor. Safe to call even if some were never
  // activated.
  void safe_stop_all();

  double current_limit_{5.0};
  double position_gain_{20.0};
  double velocity_gain_{0.16};
  double velocity_integrator_gain_{0.32};
  std::chrono::milliseconds feedback_timeout_{150};
  std::chrono::milliseconds heartbeat_timeout_{400};
  std::chrono::milliseconds startup_timeout_{1000};
  // Budget for one motor's Heartbeat to report Closed Loop Control after
  // Set_Axis_State. Replaces the old fixed motor_activation_interval_ms
  // sleep: waiting on the reported state is both faster when the motor
  // answers promptly and correct when it does not answer at all.
  std::chrono::milliseconds closed_loop_timeout_{500};
  // Budget for encoder_sync_frames_ post-closed-loop EncoderEstimates.
  std::chrono::milliseconds encoder_sync_timeout_{200};
  // How many EncoderEstimates after the closed-loop Heartbeat must arrive
  // before the last one is accepted as the joint's initial position. See
  // docs/packages/quattro_hardware.md section 6 for why the default is 2
  // and not 1.
  int encoder_sync_frames_{2};
  std::chrono::milliseconds command_timeout_{250};
  std::chrono::milliseconds scheduling_warning_{50};
  double rotor_velocity_limit_rev_s_{5.0};
  std::chrono::milliseconds telemetry_period_{500};

  std::vector<JointContext> joints_;
  std::vector<std::string> buses_;
  std::unique_ptr<gim6010_driver::MotorManager> motor_manager_;
  // Per-joint count of consecutive write() cycles whose Set_Input_Pos was
  // rejected (CanSocket::send() is a non-blocking write() -- a full kernel
  // CAN TX queue makes it fail immediately). Reset to 0 on activation and
  // on every successful send; see write() for why a single rejection does
  // not by itself trigger safe_stop_all().
  std::vector<int> consecutive_write_failures_;

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
