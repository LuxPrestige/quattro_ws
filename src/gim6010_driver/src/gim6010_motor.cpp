#include "gim6010_driver/gim6010_motor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

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
  return (static_cast<std::uint32_t>(node_id_) << 5) | command_id;
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
  if (!(velocity_limit > 0.0F) || !(current_limit > 0.0F)) {
    throw std::invalid_argument("motor limits must be positive");
  }
  std::array<std::uint8_t, 8> payload{};
  const auto velocity = littleEndianBytes(velocity_limit);
  const auto current = littleEndianBytes(current_limit);
  std::copy(velocity.begin(), velocity.end(), payload.begin());
  std::copy(current.begin(), current.end(), payload.begin() + 4);
  sendRaw(kCommandSetLimits, payload.data(), payload.size());
}
void Gim6010Motor::setMitMode()
{
  constexpr std::uint32_t control_mode_position = 3;
  constexpr std::uint32_t input_mode_mit = 9;
  std::array<std::uint8_t, 8> payload{};
  const auto control = littleEndianBytes(control_mode_position);
  const auto input = littleEndianBytes(input_mode_mit);
  std::copy(control.begin(), control.end(), payload.begin());
  std::copy(input.begin(), input.end(), payload.begin() + 4);
  sendRaw(kCommandSetControllerMode, payload.data(), payload.size());
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
void Gim6010Motor::requestEncoderEstimates() {sendRaw(kCommandEncoderEstimates, nullptr, 0, true);}
void Gim6010Motor::sendCommand(const MitCommand & command)
{
  const auto payload = encodeCommand(command);
  sendRaw(kCommandMitControl, payload.data(), payload.size());
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
    velocity_turns_per_second * two_pi / gear_ratio_, 0.0};
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
const MitFeedback & Gim6010Motor::feedback() const noexcept {return feedback_;}
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
