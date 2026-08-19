#ifndef GIM6010_DRIVER__GIM6010_MOTOR_HPP_
#define GIM6010_DRIVER__GIM6010_MOTOR_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <unordered_map>

#include "gim6010_driver/can_socket.hpp"
#include "gim6010_driver/gds68_protocol.hpp"
#include "gim6010_driver/mit_protocol.hpp"
#include "gim6010_driver/types.hpp"

namespace gim6010_driver
{

struct IqFeedback
{
  double setpoint{0.0};
  double measured{0.0};
};

struct BusVoltageCurrent
{
  double voltage{0.0};
  double current{0.0};
};

// Stateful abstraction of one GIM6010-8/GDS68 node on a CAN bus: tracks the
// commanded control mode, sends mode-appropriate commands, and caches the
// latest decoded feedback/heartbeat/diagnostics. Owns no socket -- frames go
// through the CanSocket handed in by MotorManager, and this class never
// blocks or sleeps; all "wait until ready" policy lives in the caller
// (quattro_hardware).
//
// GIM6010-8 has exactly one onboard encoder (a 14-bit single-turn absolute
// magnetic encoder; see docs/gim6010_hardware.md section 11) -- there is no
// second, independently-readable position sensor. feedback()/
// encoderEstimates() are both sourced from that single encoder via
// Get_Encoder_Estimates (0x009) or, in MIT mode, from the MIT feedback
// frame (0x008), which the manual documents as reporting the same physical
// encoder's estimate on the output shaft. encoderCount() (0x0A) exposes the
// same sensor's raw multi-turn/single-turn counters for startup diagnostics
// only; it is volatile firmware state, not a second sensor, and must never
// be blended into runtime position/velocity feedback.
class Gim6010Motor
{
public:
  static constexpr std::uint8_t kMaxNodeId = kMaximumNodeId;

  Gim6010Motor(std::uint8_t node_id, std::shared_ptr<CanSocket> socket, double gear_ratio = 8.0);

  std::uint8_t nodeId() const noexcept;
  std::uint32_t arbitrationId(Gds68Command command) const;
  MotorControlMode controlMode() const noexcept;

  // Configuration / lifecycle.
  void clearErrors();
  void setLimits(float velocity_limit_rev_s, float current_limit_a);
  void configurePositionControl(PositionInputMode input_mode);
  void configureVelocityControl();
  void configureTorqueControl();
  void configureMitControl();
  void setPositionControlGains(const PositionControlGains & gains);
  void setTrapezoidalTrajectoryLimits(const TrapezoidalTrajectoryLimits & limits);
  void enable();
  void disable();

  // Requests (responses arrive later via handleFrame()).
  void requestEncoderEstimates();
  // Get_Error responses carry no type tag; queues `type` so the next
  // Get_Error response is attributed correctly even if multiple requests
  // (for different types) are in flight at once.
  void requestError(ErrorType type);
  void requestIq();
  void requestBusVoltageCurrent();
  void requestEncoderCount();

  // Commands. Each throws std::logic_error if the motor was not configured
  // for the matching control mode, to catch integration bugs immediately
  // instead of silently sending a frame the firmware will ignore.
  void setPosition(
    float rotor_position_rev, float velocity_feedforward_rev_s = 0.0F,
    float torque_feedforward_nm = 0.0F);
  void setVelocity(float rotor_velocity_rev_s, float torque_feedforward_nm = 0.0F);
  void setTorque(float motor_torque_nm);
  void sendMitCommand(const MitCommand & command);

  // Dispatches one inbound frame already known to belong to this node.
  // Returns false for a command this driver does not decode (caller should
  // count it as unrouted); throws std::length_error for a too-short payload
  // on a recognized command.
  bool handleFrame(Gds68Command command, const std::uint8_t * data, std::size_t length);

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

  bool hasEncoderCount() const noexcept;
  const EncoderCount & encoderCount() const noexcept;

private:
  void send(
    Gds68Command command, const std::uint8_t * data, std::size_t length,
    bool remote = false);
  void requireMode(MotorControlMode expected, const char * action) const;
  MitFeedback convertEncoderEstimates(float rotor_position_rev, float rotor_velocity_rev_s) const;

  std::uint8_t node_id_;
  std::shared_ptr<CanSocket> socket_;
  double gear_ratio_;
  MotorControlMode control_mode_{MotorControlMode::kUnconfigured};
  PositionInputMode position_input_mode_{PositionInputMode::kDirect};

  MitFeedback feedback_{};
  std::chrono::steady_clock::time_point feedback_time_{};
  bool has_feedback_{false};

  Heartbeat heartbeat_{};
  std::chrono::steady_clock::time_point heartbeat_time_{};
  bool has_heartbeat_{false};
  bool has_previous_life_{false};
  std::uint8_t previous_life_{0};
  std::uint64_t missed_heartbeats_{0};

  std::unordered_map<int, std::uint64_t> errors_;
  std::deque<ErrorType> pending_error_requests_;

  IqFeedback iq_{};
  bool has_iq_{false};

  BusVoltageCurrent bus_voltage_current_{};
  bool has_bus_voltage_current_{false};

  MitFeedback encoder_estimates_{};
  std::chrono::steady_clock::time_point encoder_estimates_time_{};
  bool has_encoder_estimates_{false};

  EncoderCount encoder_count_{};
  bool has_encoder_count_{false};
};

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__GIM6010_MOTOR_HPP_
