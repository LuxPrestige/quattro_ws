#include "gim6010_driver/gim6010_motor.hpp"

namespace gim6010_driver
{

void Gim6010Motor::on_heartbeat(const Heartbeat & message, std::chrono::steady_clock::time_point now)
{
  heartbeat_ = message;
  heartbeat_time_ = now;
}

void Gim6010Motor::on_encoder_estimate(
  const EncoderEstimate & message, std::chrono::steady_clock::time_point now)
{
  encoder_estimate_ = message;
  feedback_time_ = now;
}

void Gim6010Motor::on_mit_feedback(
  const MitFeedback & message, std::chrono::steady_clock::time_point now)
{
  mit_feedback_ = message;
  feedback_time_ = now;
}

void Gim6010Motor::on_error_response(
  const AxisErrorResponse & message, std::chrono::steady_clock::time_point /*now*/)
{
  error_ = message;
}

void Gim6010Motor::on_encoder_count(
  const EncoderCount & message, std::chrono::steady_clock::time_point /*now*/)
{
  encoder_count_ = message;
}

void Gim6010Motor::on_bus_voltage_current(
  const BusVoltageCurrent & message, std::chrono::steady_clock::time_point /*now*/)
{
  bus_voltage_current_ = message;
}

void Gim6010Motor::on_torques(const Torques & message, std::chrono::steady_clock::time_point /*now*/)
{
  torques_ = message;
}

void Gim6010Motor::on_sdo_response(
  const TxSdoResponse & message, std::chrono::steady_clock::time_point /*now*/)
{
  sdo_response_ = message;
}

bool Gim6010Motor::has_fresh_feedback(
  std::chrono::steady_clock::duration timeout, std::chrono::steady_clock::time_point now) const
{
  if (!encoder_estimate_ && !mit_feedback_) {
    return false;
  }
  return (now - feedback_time_) <= timeout;
}

bool Gim6010Motor::has_fresh_heartbeat(
  std::chrono::steady_clock::duration timeout, std::chrono::steady_clock::time_point now) const
{
  if (!heartbeat_) {
    return false;
  }
  return (now - heartbeat_time_) <= timeout;
}

}  // namespace gim6010_driver
