#include "gim6010_driver/gim6010_motor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <limits>

namespace gim6010_driver
{
namespace
{
template<typename T>
std::array<std::uint8_t, sizeof(T)> littleEndianBytes(T value)
{
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
  std::reverse(bytes.begin(), bytes.end());
#endif
  return bytes;
}
}  // namespace

Gim6010Motor::Gim6010Motor(
  std::uint8_t node_id, std::shared_ptr<CanSocket> socket, double gear_ratio)
: node_id_(node_id), socket_(std::move(socket)), gear_ratio_(gear_ratio)
{
  if (node_id > kMaxNodeId) {throw std::invalid_argument("GIM6010 node ID must be 0..63");}
  if (!socket_) {throw std::invalid_argument("CAN socket is null");}
  if (!std::isfinite(gear_ratio) || gear_ratio <= 0.0) {
    throw std::invalid_argument("invalid gear ratio");
  }
}

std::uint8_t Gim6010Motor::nodeId() const noexcept {return node_id_;}
std::uint32_t Gim6010Motor::arbitrationId(std::uint8_t command_id) const
{
  if (command_id > 0x1F) {throw std::invalid_argument("CAN Simple command ID must be 0..31");}
  return makeArbitrationId(node_id_, static_cast<Gds68Command>(command_id));
}

void Gim6010Motor::sendRaw(
  std::uint8_t command_id, const std::uint8_t * data, std::size_t length, bool remote)
{
  if (length > 8 || (length != 0 && data == nullptr)) {
    throw std::invalid_argument("invalid CAN payload");
  }
  CanFrame frame;
  frame.id = arbitrationId(command_id);
  frame.dlc = static_cast<std::uint8_t>(length);
  frame.remote = remote;
  if (length > 0) {std::copy_n(data, length, frame.data.begin());}
  socket_->send(frame);
}

void Gim6010Motor::clearErrors() {sendRaw(kCommandClearErrors, nullptr, 0);}
void Gim6010Motor::setLimits(float velocity_limit, float current_limit)
{
  if (!(velocity_limit > 0.0F) || !(current_limit > 0.0F) || current_limit > 100.0F) {
    throw std::invalid_argument(
            "motor limits must be positive and current must not exceed the GDS68 100 A limit");
  }
  std::array<std::uint8_t, 8> payload{};
  const auto velocity = littleEndianBytes(velocity_limit);
  const auto current = littleEndianBytes(current_limit);
  std::copy(velocity.begin(), velocity.end(), payload.begin());
  std::copy(current.begin(), current.end(), payload.begin() + 4);
  sendRaw(kCommandSetLimits, payload.data(), payload.size());
}
void Gim6010Motor::configureMitControl()
{
  const auto payload = encodeControllerMode(ControlMode::kPosition, InputMode::kMit);
  sendRaw(kCommandSetControllerMode, payload.data(), payload.size());
  control_mode_ = MotorControlMode::kMit;
  position_input_mode_.reset();
}
void Gim6010Motor::configurePositionControl(PositionInputMode input_mode)
{
  InputMode protocol_input;
  switch (input_mode) {
    case PositionInputMode::kDirect: protocol_input = InputMode::kDirect; break;
    case PositionInputMode::kPositionFilter: protocol_input = InputMode::kPositionFilter; break;
    case PositionInputMode::kTrapezoidalTrajectory:
      protocol_input = InputMode::kTrapezoidalTrajectory;
      break;
  }
  const auto payload = encodeControllerMode(ControlMode::kPosition, protocol_input);
  sendRaw(kCommandSetControllerMode, payload.data(), payload.size());
  control_mode_ = MotorControlMode::kPosition;
  position_input_mode_ = input_mode;
}
void Gim6010Motor::configureVelocityControl()
{
  const auto payload = encodeControllerMode(ControlMode::kVelocity, InputMode::kDirect);
  sendRaw(kCommandSetControllerMode, payload.data(), payload.size());
  control_mode_ = MotorControlMode::kVelocity;
  position_input_mode_.reset();
}
void Gim6010Motor::configureTorqueControl()
{
  const auto payload = encodeControllerMode(ControlMode::kTorque, InputMode::kDirect);
  sendRaw(kCommandSetControllerMode, payload.data(), payload.size());
  control_mode_ = MotorControlMode::kTorque;
  position_input_mode_.reset();
}
MotorControlMode Gim6010Motor::controlMode() const noexcept {return control_mode_;}
void Gim6010Motor::setPositionControlGains(const PositionControlGains & gains)
{
  if (control_mode_ != MotorControlMode::kPosition) {
    throw std::logic_error("position controller gains require configured position mode");
  }
  const auto position = encodePositionGain(static_cast<float>(gains.position_gain));
  const auto velocity = encodeVelocityGains(
    static_cast<float>(gains.velocity_gain),
    static_cast<float>(gains.velocity_integrator_gain));
  sendRaw(static_cast<std::uint8_t>(Gds68Command::kSetPositionGain),
    position.data(), position.size());
  sendRaw(static_cast<std::uint8_t>(Gds68Command::kSetVelocityGains),
    velocity.data(), velocity.size());
}
void Gim6010Motor::setTrapezoidalTrajectoryLimits(
  const TrapezoidalTrajectoryLimits & limits)
{
  if (control_mode_ != MotorControlMode::kPosition ||
    position_input_mode_ != PositionInputMode::kTrapezoidalTrajectory)
  {
    throw std::logic_error("trajectory limits require trapezoidal position mode");
  }
  const auto velocity = encodeTrajectoryVelocityLimit(
    static_cast<float>(limits.velocity_rev_s));
  const auto acceleration = encodeTrajectoryAccelerationLimits(
    static_cast<float>(limits.acceleration_rev_s2),
    static_cast<float>(limits.deceleration_rev_s2));
  sendRaw(static_cast<std::uint8_t>(Gds68Command::kSetTrajectoryVelocityLimit),
    velocity.data(), velocity.size());
  sendRaw(static_cast<std::uint8_t>(Gds68Command::kSetTrajectoryAccelerationLimits),
    acceleration.data(), acceleration.size());
}
void Gim6010Motor::enable()
{
  const auto payload = littleEndianBytes(static_cast<std::uint32_t>(AxisState::kClosedLoopControl));
  sendRaw(kCommandSetAxisState, payload.data(), payload.size());
}
void Gim6010Motor::disable()
{
  const auto payload = littleEndianBytes(static_cast<std::uint32_t>(AxisState::kIdle));
  sendRaw(kCommandSetAxisState, payload.data(), payload.size());
}
void Gim6010Motor::requestEncoderEstimates()
{
  has_encoder_estimates_ = false;
  sendRaw(kCommandEncoderEstimates, nullptr, 0, true);
}
void Gim6010Motor::requestError(ErrorType type)
{
  const auto value = static_cast<std::uint8_t>(type);
  errors_.erase(type);
  pending_error_type_ = type;
  sendRaw(kCommandGetError, &value, 1);
}
void Gim6010Motor::requestIq()
{
  has_iq_ = false;
  sendRaw(kCommandGetIq, nullptr, 0, true);
}
void Gim6010Motor::requestBusVoltageCurrent()
{
  has_bus_voltage_current_ = false;
  sendRaw(kCommandGetBusVoltageCurrent, nullptr, 0, true);
}
void Gim6010Motor::sendMitCommand(const MitCommand & command)
{
  if (control_mode_ != MotorControlMode::kMit) {
    throw std::logic_error("MIT command requires configured MIT control mode");
  }
  const auto payload = encodeCommand(command);
  sendRaw(kCommandMitControl, payload.data(), payload.size());
}
void Gim6010Motor::setPosition(float position, float velocity_ff, float torque_ff)
{
  if (control_mode_ != MotorControlMode::kPosition) {
    throw std::logic_error("position command requires configured position control mode");
  }
  const auto payload = encodeDirectPosition(position, velocity_ff, torque_ff);
  sendRaw(static_cast<std::uint8_t>(Gds68Command::kSetInputPosition), payload.data(), payload.size());
}
void Gim6010Motor::setVelocity(float velocity, float torque_ff)
{
  if (control_mode_ != MotorControlMode::kVelocity) {
    throw std::logic_error("velocity command requires configured velocity control mode");
  }
  const auto payload = encodeDirectVelocity(velocity, torque_ff);
  sendRaw(static_cast<std::uint8_t>(Gds68Command::kSetInputVelocity), payload.data(), payload.size());
}
void Gim6010Motor::setTorque(float torque)
{
  if (control_mode_ != MotorControlMode::kTorque) {
    throw std::logic_error("torque command requires configured torque control mode");
  }
  const auto payload = encodeDirectTorque(torque);
  sendRaw(static_cast<std::uint8_t>(Gds68Command::kSetInputTorque), payload.data(), payload.size());
}
void Gim6010Motor::updateHeartbeat(const std::uint8_t * data, std::size_t length)
{
  const auto next = decodeHeartbeat(data, length);
  if (has_heartbeat_) {
    const auto advance = static_cast<std::uint8_t>(next.life - heartbeat_.life);
    if (advance > 1U) {
      missed_heartbeats_ += static_cast<std::uint8_t>(advance - 1U);
    }
  }
  heartbeat_ = next;
  heartbeat_time_ = std::chrono::steady_clock::now();
  has_heartbeat_ = true;
}
void Gim6010Motor::updateError(const std::uint8_t * data, std::size_t length)
{
  if (!pending_error_type_) {return;}
  errors_[*pending_error_type_] = decodeError(data, length, *pending_error_type_);
  pending_error_type_.reset();
}
void Gim6010Motor::updateIq(const std::uint8_t * data, std::size_t length)
{
  iq_ = decodeIqFeedback(data, length);
  has_iq_ = true;
}
void Gim6010Motor::updateBusVoltageCurrent(const std::uint8_t * data, std::size_t length)
{
  bus_voltage_current_ = decodeBusVoltageCurrent(data, length);
  has_bus_voltage_current_ = true;
}
void Gim6010Motor::updateFeedback(const MitFeedback & feedback)
{
  if (feedback.motor_id != node_id_) {throw std::invalid_argument("feedback motor ID mismatch");}
  feedback_ = feedback;
  feedback_time_ = std::chrono::steady_clock::now();
  has_feedback_ = true;
}
void Gim6010Motor::updateEncoderEstimates(const std::uint8_t * data, std::size_t length)
{
  if (data == nullptr || length != 8) {
    throw std::invalid_argument("encoder estimates require 8 bytes");
  }
  float position_turns = 0.0F;
  float velocity_turns_per_second = 0.0F;
  std::memcpy(&position_turns, data, sizeof(float));
  std::memcpy(&velocity_turns_per_second, data + 4, sizeof(float));
  constexpr double two_pi = 6.28318530717958647692;
  encoder_estimates_ = MitFeedback{
    node_id_, position_turns * two_pi / gear_ratio_,
    velocity_turns_per_second * two_pi / gear_ratio_,
    std::numeric_limits<double>::quiet_NaN()};
  encoder_estimates_time_ = std::chrono::steady_clock::now();
  has_encoder_estimates_ = true;
  feedback_ = encoder_estimates_;
  feedback_time_ = encoder_estimates_time_;
  has_feedback_ = true;
}
bool Gim6010Motor::hasFeedback() const noexcept {return has_feedback_;}
bool Gim6010Motor::feedbackStale(std::chrono::steady_clock::duration timeout) const
{
  return !has_feedback_ || std::chrono::steady_clock::now() - feedback_time_ > timeout;
}
std::chrono::steady_clock::duration Gim6010Motor::feedbackAge() const
{
  return has_feedback_ ? std::chrono::steady_clock::now() - feedback_time_ :
         std::chrono::steady_clock::duration::max();
}
const MitFeedback & Gim6010Motor::feedback() const noexcept {return feedback_;}
bool Gim6010Motor::hasHeartbeat() const noexcept {return has_heartbeat_;}
bool Gim6010Motor::heartbeatStale(std::chrono::steady_clock::duration timeout) const
{
  return !has_heartbeat_ || std::chrono::steady_clock::now() - heartbeat_time_ > timeout;
}
std::chrono::steady_clock::duration Gim6010Motor::heartbeatAge() const
{
  return has_heartbeat_ ? std::chrono::steady_clock::now() - heartbeat_time_ :
         std::chrono::steady_clock::duration::max();
}
const Heartbeat & Gim6010Motor::heartbeat() const noexcept {return heartbeat_;}
std::uint64_t Gim6010Motor::missedHeartbeats() const noexcept {return missed_heartbeats_;}
bool Gim6010Motor::hasError(ErrorType type) const noexcept
{
  return errors_.count(type) != 0U;
}
std::uint64_t Gim6010Motor::error(ErrorType type) const
{
  const auto entry = errors_.find(type);
  if (entry == errors_.end()) {throw std::logic_error("GIM6010 error value is unavailable");}
  return entry->second;
}
bool Gim6010Motor::hasIq() const noexcept {return has_iq_;}
const IqFeedback & Gim6010Motor::iq() const noexcept {return iq_;}
bool Gim6010Motor::hasBusVoltageCurrent() const noexcept {return has_bus_voltage_current_;}
const BusVoltageCurrent & Gim6010Motor::busVoltageCurrent() const noexcept
{
  return bus_voltage_current_;
}
bool Gim6010Motor::hasEncoderEstimates() const noexcept {return has_encoder_estimates_;}
bool Gim6010Motor::encoderEstimatesStale(
  std::chrono::steady_clock::duration timeout) const
{
  return !has_encoder_estimates_ ||
         std::chrono::steady_clock::now() - encoder_estimates_time_ > timeout;
}
const MitFeedback & Gim6010Motor::encoderEstimates() const noexcept
{
  return encoder_estimates_;
}
}  // namespace gim6010_driver
