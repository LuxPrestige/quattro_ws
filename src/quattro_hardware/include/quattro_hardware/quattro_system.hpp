#ifndef QUATTRO_HARDWARE__QUATTRO_SYSTEM_HPP_
#define QUATTRO_HARDWARE__QUATTRO_SYSTEM_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gim6010_driver/motor_manager.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace quattro_hardware
{

class QuattroSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  enum class ControlMethod {kDirectPosition, kDirectVelocity, kDirectTorque, kMit};
  enum class FaultReason
  {
    kNone, kFeedbackTimeout, kHeartbeatTimeout, kCommandTimeout, kCanPassive,
    kCanBusOff, kMotorFault, kInvalidCommand, kCanIo
  };

  struct Joint
  {
    std::string name;
    std::string can_interface;
    std::uint8_t node_id{0};
    double direction{1.0};
    double offset{0.0};
    double gear_ratio{8.0};
    double lower{-12.5};
    double upper{12.5};
    double velocity_limit{4.0};
    double effort_limit{40.0};
    double current_limit{5.0};
    double mit_kp{20.0};
    double mit_kd{0.5};
    std::uint64_t reported_missed_heartbeats{0};
    gim6010_driver::MotorManager * manager{nullptr};
  };

  bool configureMotors();
  void configureMotorControllers();
  bool waitForInitialFeedback();
  bool waitForPreflightHeartbeats();
  bool waitForOperationalHeartbeats();
  void sendActivationHold(Joint & joint, double output_position);
  bool waitForMotorOperational(
    std::size_t motor_index, const std::vector<double> & hold_positions);
  bool waitForActivationInterval(
    std::size_t motor_index, const std::vector<double> & hold_positions);
  void pollManagers();
  void requestMotorFeedback(std::size_t motor_count);
  void requestMotorTelemetry();
  void captureFaultDiagnostics();
  void publishDiagnostics(bool force = false);
  void safeStop() noexcept;
  std::string interfaceKey(const Joint & joint, const char * interface_name) const;
  const char * commandInterfaceName() const noexcept;

  std::vector<Joint> joints_;
  std::unordered_map<std::string, std::unique_ptr<gim6010_driver::MotorManager>> managers_;
  std::chrono::milliseconds feedback_timeout_{200};
  std::chrono::milliseconds feedback_request_period_{50};
  std::chrono::milliseconds heartbeat_timeout_{1000};
  std::chrono::milliseconds startup_timeout_{1000};
  std::chrono::milliseconds motor_activation_interval_{500};
  std::chrono::milliseconds command_timeout_{250};
  std::chrono::milliseconds scheduling_warning_{50};
  std::chrono::steady_clock::time_point last_write_{};
  double motor_velocity_limit_{5.0};
  double motor_current_limit_{5.0};
  bool apply_position_gains_{false};
  gim6010_driver::PositionControlGains position_gains_{};
  std::chrono::milliseconds engagement_duration_{1000};
  std::chrono::milliseconds telemetry_period_{500};
  std::chrono::steady_clock::time_point next_telemetry_request_{};
  std::size_t next_telemetry_motor_{0};
  std::chrono::steady_clock::time_point next_feedback_request_{};
  std::chrono::steady_clock::time_point next_diagnostics_publish_{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  bool active_{false};
  ControlMethod control_method_{ControlMethod::kMit};
  FaultReason fault_reason_{FaultReason::kNone};
};

}  // namespace quattro_hardware
#endif  // QUATTRO_HARDWARE__QUATTRO_SYSTEM_HPP_
