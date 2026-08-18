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

private:
  struct Joint
  {
    std::string name;
    std::string can_interface;
    std::uint8_t node_id{0};
    double direction{1.0};
    double offset{0.0};
    double lower{-12.5};
    double upper{12.5};
    double velocity_limit{4.0};
    double effort_limit{40.0};
    double kp{20.0};
    double kd{0.5};
    std::uint64_t reported_missed_heartbeats{0};
    gim6010_driver::MotorManager * manager{nullptr};
  };

  bool configureMotors();
  bool waitForInitialFeedback();
  bool waitForOperationalHeartbeats();
  void pollManagers();
  void requestBusTelemetry();
  void captureFaultDiagnostics();
  void publishDiagnostics(bool force = false);
  void safeStop() noexcept;
  std::string interfaceKey(const Joint & joint, const char * interface_name) const;

  std::vector<Joint> joints_;
  std::unordered_map<std::string, std::unique_ptr<gim6010_driver::MotorManager>> managers_;
  std::chrono::milliseconds feedback_timeout_{100};
  std::chrono::milliseconds heartbeat_timeout_{300};
  std::chrono::milliseconds startup_timeout_{1000};
  std::chrono::milliseconds command_timeout_{100};
  std::chrono::steady_clock::time_point last_write_{};
  double motor_velocity_limit_{50.0};
  double motor_current_limit_{20.0};
  std::chrono::milliseconds engagement_duration_{1000};
  std::chrono::milliseconds telemetry_period_{500};
  std::chrono::steady_clock::time_point next_telemetry_request_{};
  std::chrono::steady_clock::time_point next_diagnostics_publish_{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  bool active_{false};
  bool position_limits_enabled_{false};
};

}  // namespace quattro_hardware
#endif  // QUATTRO_HARDWARE__QUATTRO_SYSTEM_HPP_
