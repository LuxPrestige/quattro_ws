#ifndef GIM6010_DRIVER__GIM6010_MOTOR_HPP_
#define GIM6010_DRIVER__GIM6010_MOTOR_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>

#include "gim6010_driver/can_diagnostics.hpp"
#include "gim6010_driver/can_socket.hpp"
#include "gim6010_driver/gds68_protocol.hpp"
#include "gim6010_driver/mit_protocol.hpp"
#include "gim6010_driver/types.hpp"

namespace gim6010_driver
{

enum class AxisState : std::uint32_t { kIdle = 1, kClosedLoopControl = 8 };

class Gim6010Motor
{
public:
  static constexpr std::uint8_t kMaxNodeId = kMaximumNodeId;
  static constexpr std::uint8_t kCommandHeartbeat = 0x01;
  static constexpr std::uint8_t kCommandGetError = 0x03;
  static constexpr std::uint8_t kCommandSetAxisState = 0x07;
  static constexpr std::uint8_t kCommandMitControl = 0x08;
  static constexpr std::uint8_t kCommandEncoderEstimates = 0x09;
  static constexpr std::uint8_t kCommandSetControllerMode = 0x0B;
  static constexpr std::uint8_t kCommandSetLimits = 0x0F;
  static constexpr std::uint8_t kCommandGetIq = 0x14;
  static constexpr std::uint8_t kCommandGetBusVoltageCurrent = 0x17;
  static constexpr std::uint8_t kCommandClearErrors = 0x18;

  Gim6010Motor(std::uint8_t node_id, std::shared_ptr<CanSocket> socket, double gear_ratio = 8.0);
  std::uint8_t nodeId() const noexcept;
  std::uint32_t arbitrationId(std::uint8_t command_id) const;

  void clearErrors();
  void setLimits(float velocity_limit, float current_limit);
  void configurePositionControl(PositionInputMode input_mode);
  void configureVelocityControl();
  void configureTorqueControl();
  void configureMitControl();
  MotorControlMode controlMode() const noexcept;
  void setPositionControlGains(const PositionControlGains & gains);
  void setTrapezoidalTrajectoryLimits(const TrapezoidalTrajectoryLimits & limits);
  void enable();
  void disable();
  void requestEncoderEstimates();
  void requestError(ErrorType type);
  void requestIq();
  void requestBusVoltageCurrent();
  void setPosition(
    float rotor_position_rev, float velocity_feedforward_rev_s = 0.0F,
    float torque_feedforward_nm = 0.0F);
  void setVelocity(float rotor_velocity_rev_s, float torque_feedforward_nm = 0.0F);
  void setTorque(float motor_torque_nm);
  void sendMitCommand(const MitCommand & command);
  void updateHeartbeat(const std::uint8_t * data, std::size_t length);
  void updateError(const std::uint8_t * data, std::size_t length);
  void updateIq(const std::uint8_t * data, std::size_t length);
  void updateBusVoltageCurrent(const std::uint8_t * data, std::size_t length);
  void updateFeedback(const MitFeedback & feedback);
  void updateEncoderEstimates(const std::uint8_t * data, std::size_t length);

  bool hasFeedback() const noexcept;
  bool feedbackStale(std::chrono::steady_clock::duration timeout) const;
  std::chrono::steady_clock::duration feedbackAge() const;
  const MitFeedback & feedback() const noexcept;
  bool hasHeartbeat() const noexcept;
  bool heartbeatStale(std::chrono::steady_clock::duration timeout) const;
  std::chrono::steady_clock::duration heartbeatAge() const;
  const Heartbeat & heartbeat() const noexcept;
  std::uint64_t missedHeartbeats() const noexcept;
  bool hasError(ErrorType type) const noexcept;
  std::uint64_t error(ErrorType type) const;
  bool hasIq() const noexcept;
  const IqFeedback & iq() const noexcept;
  bool hasBusVoltageCurrent() const noexcept;
  const BusVoltageCurrent & busVoltageCurrent() const noexcept;
  bool hasEncoderEstimates() const noexcept;
  bool encoderEstimatesStale(std::chrono::steady_clock::duration timeout) const;
  const MitFeedback & encoderEstimates() const noexcept;

private:
  void sendRaw(
    std::uint8_t command_id, const std::uint8_t * data, std::size_t length,
    bool remote = false);
  std::uint8_t node_id_;
  std::shared_ptr<CanSocket> socket_;
  MitFeedback feedback_{};
  std::chrono::steady_clock::time_point feedback_time_{};
  bool has_feedback_{false};
  Heartbeat heartbeat_{};
  std::chrono::steady_clock::time_point heartbeat_time_{};
  bool has_heartbeat_{false};
  std::uint64_t missed_heartbeats_{0};
  std::optional<ErrorType> pending_error_type_;
  std::unordered_map<ErrorType, std::uint64_t> errors_;
  IqFeedback iq_{};
  bool has_iq_{false};
  BusVoltageCurrent bus_voltage_current_{};
  bool has_bus_voltage_current_{false};
  MitFeedback encoder_estimates_{};
  std::chrono::steady_clock::time_point encoder_estimates_time_{};
  bool has_encoder_estimates_{false};
  double gear_ratio_{8.0};
  MotorControlMode control_mode_{MotorControlMode::kUnconfigured};
  std::optional<PositionInputMode> position_input_mode_;
};

}  // namespace gim6010_driver
#endif  // GIM6010_DRIVER__GIM6010_MOTOR_HPP_
