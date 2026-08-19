#include "gim6010_driver/gim6010_motor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace gim6010_driver
{

Gim6010Motor::Gim6010Motor(
  std::uint8_t node_id, std::shared_ptr<CanSocket> socket,
  double gear_ratio)
: node_id_(node_id), socket_(std::move(socket)), gear_ratio_(gear_ratio)
{
  if (node_id_ > kMaxNodeId) {
    throw std::invalid_argument("node_id exceeds the CAN Simple 6-bit range");
  }
  if (!socket_) {
    throw std::invalid_argument("Gim6010Motor requires a CAN socket");
  }
  if (!std::isfinite(gear_ratio_) || gear_ratio_ <= 0.0) {
    throw std::invalid_argument("gear_ratio must be positive");
  }
}

std::uint8_t Gim6010Motor::nodeId() const noexcept {return node_id_;}

std::uint32_t Gim6010Motor::arbitrationId(Gds68Command command) const
{
  return makeArbitrationId(node_id_, command);
}

MotorControlMode Gim6010Motor::controlMode() const noexcept {return control_mode_;}

void Gim6010Motor::send(
  Gds68Command command, const std::uint8_t * data, std::size_t length, bool remote)
{
  CanFrame frame;
  frame.id = arbitrationId(command);
  frame.dlc = static_cast<std::uint8_t>(length);
  frame.remote = remote;
  if (!remote && data != nullptr) {
    std::copy(data, data + std::min<std::size_t>(length, 8U), frame.data.begin());
  }
  socket_->send(frame);
}

void Gim6010Motor::requireMode(MotorControlMode expected, const char * action) const
{
  if (control_mode_ != expected) {
    throw std::logic_error(
            std::string(action) + " requires the matching control mode to be configured first");
  }
}

void Gim6010Motor::clearErrors()
{
  send(Gds68Command::kClearErrors, nullptr, 0);
}

void Gim6010Motor::setLimits(float velocity_limit_rev_s, float current_limit_a)
{
  const auto payload = encodeLimits(velocity_limit_rev_s, current_limit_a);
  send(Gds68Command::kSetLimits, payload.data(), payload.size());
}

void Gim6010Motor::configurePositionControl(PositionInputMode input_mode)
{
  InputMode mapped{};
  switch (input_mode) {
    case PositionInputMode::kDirect: mapped = InputMode::kDirect; break;
    case PositionInputMode::kPositionFilter: mapped = InputMode::kPositionFilter; break;
    case PositionInputMode::kTrapezoidalTrajectory:
      mapped = InputMode::kTrapezoidalTrajectory;
      break;
  }
  const auto payload = encodeControllerMode(ControlMode::kPosition, mapped);
  send(Gds68Command::kSetControllerMode, payload.data(), payload.size());
  control_mode_ = MotorControlMode::kPosition;
  position_input_mode_ = input_mode;
}

void Gim6010Motor::configureVelocityControl()
{
  const auto payload = encodeControllerMode(ControlMode::kVelocity, InputMode::kDirect);
  send(Gds68Command::kSetControllerMode, payload.data(), payload.size());
  control_mode_ = MotorControlMode::kVelocity;
}

void Gim6010Motor::configureTorqueControl()
{
  const auto payload = encodeControllerMode(ControlMode::kTorque, InputMode::kDirect);
  send(Gds68Command::kSetControllerMode, payload.data(), payload.size());
  control_mode_ = MotorControlMode::kTorque;
}

void Gim6010Motor::configureMitControl()
{
  const auto payload = encodeControllerMode(ControlMode::kPosition, InputMode::kMit);
  send(Gds68Command::kSetControllerMode, payload.data(), payload.size());
  control_mode_ = MotorControlMode::kMit;
}

void Gim6010Motor::setPositionControlGains(const PositionControlGains & gains)
{
  requireMode(MotorControlMode::kPosition, "setPositionControlGains");
  const auto position_gain = encodePositionGain(static_cast<float>(gains.position_gain));
  send(Gds68Command::kSetPositionGain, position_gain.data(), position_gain.size());
  const auto velocity_gains = encodeVelocityGains(
    static_cast<float>(gains.velocity_gain), static_cast<float>(gains.velocity_integrator_gain));
  send(Gds68Command::kSetVelocityGains, velocity_gains.data(), velocity_gains.size());
}

void Gim6010Motor::setTrapezoidalTrajectoryLimits(const TrapezoidalTrajectoryLimits & limits)
{
  requireMode(MotorControlMode::kPosition, "setTrapezoidalTrajectoryLimits");
  if (position_input_mode_ != PositionInputMode::kTrapezoidalTrajectory) {
    throw std::logic_error(
            "setTrapezoidalTrajectoryLimits requires the trapezoidal trajectory input mode");
  }
  const auto velocity_limit = encodeTrajectoryVelocityLimit(
    static_cast<float>(limits.velocity_rev_s));
  send(Gds68Command::kSetTrajectoryVelocityLimit, velocity_limit.data(), velocity_limit.size());
  const auto accel_limits = encodeTrajectoryAccelerationLimits(
    static_cast<float>(limits.acceleration_rev_s2), static_cast<float>(limits.deceleration_rev_s2));
  send(
    Gds68Command::kSetTrajectoryAccelerationLimits, accel_limits.data(), accel_limits.size());
}

void Gim6010Motor::enable()
{
  const auto payload = encodeAxisState(static_cast<std::uint32_t>(AxisState::kClosedLoopControl));
  send(Gds68Command::kSetAxisState, payload.data(), payload.size());
}

void Gim6010Motor::disable()
{
  const auto payload = encodeAxisState(static_cast<std::uint32_t>(AxisState::kIdle));
  send(Gds68Command::kSetAxisState, payload.data(), payload.size());
}

void Gim6010Motor::requestEncoderEstimates()
{
  send(Gds68Command::kEncoderEstimates, nullptr, 8, /*remote=*/true);
}

void Gim6010Motor::requestError(ErrorType type)
{
  const auto payload = encodeGetErrorRequest(type);
  send(Gds68Command::kGetError, payload.data(), payload.size());
  pending_error_requests_.push_back(type);
}

void Gim6010Motor::requestIq()
{
  send(Gds68Command::kIq, nullptr, 8, /*remote=*/true);
}

void Gim6010Motor::requestBusVoltageCurrent()
{
  send(Gds68Command::kBusVoltageCurrent, nullptr, 8, /*remote=*/true);
}

void Gim6010Motor::requestEncoderCount()
{
  send(Gds68Command::kEncoderCount, nullptr, 8, /*remote=*/true);
}

void Gim6010Motor::setPosition(
  float rotor_position_rev, float velocity_feedforward_rev_s, float torque_feedforward_nm)
{
  requireMode(MotorControlMode::kPosition, "setPosition");
  const auto payload = encodeDirectPosition(
    rotor_position_rev, velocity_feedforward_rev_s, torque_feedforward_nm);
  send(Gds68Command::kSetInputPosition, payload.data(), payload.size());
}

void Gim6010Motor::setVelocity(float rotor_velocity_rev_s, float torque_feedforward_nm)
{
  requireMode(MotorControlMode::kVelocity, "setVelocity");
  const auto payload = encodeDirectVelocity(rotor_velocity_rev_s, torque_feedforward_nm);
  send(Gds68Command::kSetInputVelocity, payload.data(), payload.size());
}

void Gim6010Motor::setTorque(float motor_torque_nm)
{
  requireMode(MotorControlMode::kTorque, "setTorque");
  const auto payload = encodeDirectTorque(motor_torque_nm);
  send(Gds68Command::kSetInputTorque, payload.data(), payload.size());
}

void Gim6010Motor::sendMitCommand(const MitCommand & command)
{
  requireMode(MotorControlMode::kMit, "sendMitCommand");
  const auto payload = encodeCommand(command);
  send(Gds68Command::kMitControl, payload.data(), payload.size());
}

MitFeedback Gim6010Motor::convertEncoderEstimates(
  float rotor_position_rev, float rotor_velocity_rev_s) const
{
  MitFeedback converted;
  converted.motor_id = node_id_;
  converted.output_position_rad = rotorRevToOutputRad(rotor_position_rev, gear_ratio_);
  converted.output_velocity_rad_s =
    rotorRevPerSecToOutputRadPerSec(rotor_velocity_rev_s, gear_ratio_);
  // 0x009 does not report torque; never surface a command value as feedback
  // for a field that has no measurement.
  converted.output_torque_nm = std::numeric_limits<double>::quiet_NaN();
  return converted;
}

bool Gim6010Motor::handleFrame(Gds68Command command, const std::uint8_t * data, std::size_t length)
{
  const auto now = std::chrono::steady_clock::now();
  switch (command) {
    case Gds68Command::kHeartbeat: {
        const auto decoded = decodeHeartbeat(data, length);
        if (has_previous_life_) {
          const auto expected = static_cast<std::uint8_t>(previous_life_ + 1U);
          if (decoded.life != expected) {
            missed_heartbeats_ += static_cast<std::uint8_t>(decoded.life - expected);
          }
        }
        previous_life_ = decoded.life;
        has_previous_life_ = true;
        heartbeat_ = decoded;
        heartbeat_time_ = now;
        has_heartbeat_ = true;
        return true;
      }
    case Gds68Command::kMitControl: {
        feedback_ = decodeFeedback(data, length);
        feedback_time_ = now;
        has_feedback_ = true;
        return true;
      }
    case Gds68Command::kEncoderEstimates: {
        const auto estimates = decodeEncoderEstimates(data, length);
        const auto converted = convertEncoderEstimates(estimates.first, estimates.second);
        encoder_estimates_ = converted;
        encoder_estimates_time_ = now;
        has_encoder_estimates_ = true;
      // In MIT mode 0x008 already delivers fresh output-SI feedback with
      // every command/response cycle; do not let a slower, separately
      // requested 0x009 poll overwrite it.
        if (control_mode_ != MotorControlMode::kMit) {
          feedback_ = converted;
          feedback_time_ = now;
          has_feedback_ = true;
        }
        return true;
      }
    case Gds68Command::kGetError: {
        // The response carries no type tag; attribute it to the oldest
        // still-pending request. If nothing is pending (e.g. a stray or
        // duplicate reply), drop it rather than guess.
        if (pending_error_requests_.empty()) {
          return false;
        }
        const ErrorType type = pending_error_requests_.front();
        pending_error_requests_.pop_front();
        errors_[static_cast<int>(type)] = decodeGetErrorResponse(type, data, length);
        return true;
      }
    case Gds68Command::kIq: {
        const auto response = decodeIq(data, length);
        iq_ = {response.first, response.second};
        has_iq_ = true;
        return true;
      }
    case Gds68Command::kBusVoltageCurrent: {
        const auto response = decodeBusVoltageCurrent(data, length);
        bus_voltage_current_ = {response.first, response.second};
        has_bus_voltage_current_ = true;
        return true;
      }
    case Gds68Command::kEncoderCount: {
        encoder_count_ = decodeEncoderCount(data, length);
        has_encoder_count_ = true;
        return true;
      }
    default:
      return false;
  }
}

bool Gim6010Motor::hasFeedback() const noexcept {return has_feedback_;}

bool Gim6010Motor::feedbackStale(std::chrono::steady_clock::duration timeout) const
{
  return !has_feedback_ || (std::chrono::steady_clock::now() - feedback_time_) > timeout;
}

std::chrono::steady_clock::duration Gim6010Motor::feedbackAge() const
{
  return std::chrono::steady_clock::now() - feedback_time_;
}

const MitFeedback & Gim6010Motor::feedback() const noexcept {return feedback_;}

bool Gim6010Motor::hasHeartbeat() const noexcept {return has_heartbeat_;}

bool Gim6010Motor::heartbeatStale(std::chrono::steady_clock::duration timeout) const
{
  return !has_heartbeat_ || (std::chrono::steady_clock::now() - heartbeat_time_) > timeout;
}

std::chrono::steady_clock::duration Gim6010Motor::heartbeatAge() const
{
  return std::chrono::steady_clock::now() - heartbeat_time_;
}

const Heartbeat & Gim6010Motor::heartbeat() const noexcept {return heartbeat_;}

std::uint64_t Gim6010Motor::missedHeartbeats() const noexcept {return missed_heartbeats_;}

bool Gim6010Motor::hasError(ErrorType type) const noexcept
{
  return errors_.find(static_cast<int>(type)) != errors_.end();
}

std::uint64_t Gim6010Motor::error(ErrorType type) const
{
  const auto found = errors_.find(static_cast<int>(type));
  return found == errors_.end() ? 0U : found->second;
}

bool Gim6010Motor::hasIq() const noexcept {return has_iq_;}

const IqFeedback & Gim6010Motor::iq() const noexcept {return iq_;}

bool Gim6010Motor::hasBusVoltageCurrent() const noexcept {return has_bus_voltage_current_;}

const BusVoltageCurrent & Gim6010Motor::busVoltageCurrent() const noexcept
{
  return bus_voltage_current_;
}

bool Gim6010Motor::hasEncoderEstimates() const noexcept {return has_encoder_estimates_;}

bool Gim6010Motor::encoderEstimatesStale(std::chrono::steady_clock::duration timeout) const
{
  return !has_encoder_estimates_ ||
         (std::chrono::steady_clock::now() - encoder_estimates_time_) > timeout;
}

const MitFeedback & Gim6010Motor::encoderEstimates() const noexcept {return encoder_estimates_;}

bool Gim6010Motor::hasEncoderCount() const noexcept {return has_encoder_count_;}

const EncoderCount & Gim6010Motor::encoderCount() const noexcept {return encoder_count_;}

}  // namespace gim6010_driver
