#ifndef GIM6010_DRIVER__GIM6010_MOTOR_HPP_
#define GIM6010_DRIVER__GIM6010_MOTOR_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

#include "gim6010_driver/can_simple_messages.hpp"
#include "gim6010_driver/mit_protocol.hpp"

namespace gim6010_driver
{

// Tracks the last-known state of a single GIM6010-8/GDS68 motor: the most
// recent value of each feedback message type plus when it arrived, so
// callers can judge staleness themselves. Holds no socket and sends
// nothing -- MotorManager owns routing and I/O, this class only records
// what has already been decoded.
class Gim6010Motor
{
public:
  explicit Gim6010Motor(uint8_t node_id) : node_id_(node_id) {}

  uint8_t node_id() const noexcept { return node_id_; }

  void on_heartbeat(const Heartbeat & message, std::chrono::steady_clock::time_point now);
  // Position/velocity feedback can arrive either as a polled
  // Get_Encoder_Estimates response or embedded in every MIT response frame
  // -- both update the same "last feedback" freshness clock, since either
  // is a valid position/velocity source depending on hardware_control_method.
  // Only this one bumps encoder_sequence(): the counter exists so a caller
  // can tell "a new 0x009 arrived" from "the cached one is still there",
  // and MIT feedback is a different message on a different code path.
  void on_encoder_estimate(const EncoderEstimate & message, std::chrono::steady_clock::time_point now);
  void on_mit_feedback(const MitFeedback & message, std::chrono::steady_clock::time_point now);
  void on_error_response(const AxisErrorResponse & message, std::chrono::steady_clock::time_point now);
  void on_encoder_count(const EncoderCount & message, std::chrono::steady_clock::time_point now);
  // Only the most recent TxSdo response is kept. TxSdo carries its
  // endpoint_id back (self-tagged), so a caller pipelining several RxSdo
  // requests to the same motor should check endpoint_id on each read and
  // poll again if an older response was overwritten before it was read.
  void on_sdo_response(const TxSdoResponse & message, std::chrono::steady_clock::time_point now);
  void on_bus_voltage_current(
    const BusVoltageCurrent & message, std::chrono::steady_clock::time_point now);
  void on_torques(const Torques & message, std::chrono::steady_clock::time_point now);

  bool has_fresh_feedback(
    std::chrono::steady_clock::duration timeout, std::chrono::steady_clock::time_point now) const;
  bool has_fresh_heartbeat(
    std::chrono::steady_clock::duration timeout, std::chrono::steady_clock::time_point now) const;

  std::optional<Heartbeat> last_heartbeat() const { return heartbeat_; }
  std::optional<EncoderEstimate> last_encoder_estimate() const { return encoder_estimate_; }
  // Monotonic count of Get_Encoder_Estimates frames received from this
  // motor since construction. Timestamps alone cannot answer "did a frame
  // arrive after this point in the startup sequence?" -- two frames closer
  // together than the clock's usable resolution, or a caller that captured
  // its reference time between dispatch and read, both look identical. The
  // counter makes that question exact, which matters because on the
  // GIM6010-8 an encoder position is only trustworthy once it was sampled
  // after the axis reached closed-loop control
  // (docs/packages/gim6010_driver.md section 4). This class does not judge
  // which frames are valid; it only makes them countable.
  std::uint64_t encoder_sequence() const noexcept { return encoder_sequence_; }
  std::optional<MitFeedback> last_mit_feedback() const { return mit_feedback_; }
  std::optional<AxisErrorResponse> last_error() const { return error_; }
  std::optional<EncoderCount> last_encoder_count() const { return encoder_count_; }
  std::optional<BusVoltageCurrent> last_bus_voltage_current() const
  {
    return bus_voltage_current_;
  }
  std::optional<Torques> last_torques() const { return torques_; }
  std::optional<TxSdoResponse> last_sdo_response() const { return sdo_response_; }

private:
  uint8_t node_id_;

  std::optional<Heartbeat> heartbeat_;
  std::chrono::steady_clock::time_point heartbeat_time_{};

  std::optional<EncoderEstimate> encoder_estimate_;
  std::uint64_t encoder_sequence_{0};
  std::optional<MitFeedback> mit_feedback_;
  std::chrono::steady_clock::time_point feedback_time_{};

  std::optional<AxisErrorResponse> error_;
  std::optional<EncoderCount> encoder_count_;
  std::optional<BusVoltageCurrent> bus_voltage_current_;
  std::optional<Torques> torques_;
  std::optional<TxSdoResponse> sdo_response_;
};

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__GIM6010_MOTOR_HPP_
