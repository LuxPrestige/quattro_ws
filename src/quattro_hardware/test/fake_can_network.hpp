#ifndef QUATTRO_HARDWARE__TEST__FAKE_CAN_NETWORK_HPP_
#define QUATTRO_HARDWARE__TEST__FAKE_CAN_NETWORK_HPP_

#include <algorithm>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "gim6010_driver/byte_utils.hpp"
#include "gim6010_driver/can_simple_messages.hpp"
#include "gim6010_driver/can_socket_interface.hpp"
#include "gim6010_driver/types.hpp"

namespace quattro_hardware_test
{

// An in-memory stand-in for a CAN bus carrying GIM6010-8 motors, used to
// drive QuattroSystem's startup sequence deterministically.
//
// Substituting the transport rather than MotorManager is deliberate: the
// real encode/decode, routing, dispatch and encoder-sequence bookkeeping all
// stay in the loop, so a test can make claims about actual wire traffic --
// "no Set_Input_Pos frame was ever sent" is checked by looking at frames,
// not by trusting a mock's call count.
//
// Each simulated motor answers Set_Axis_State and, on every poll cycle,
// broadcasts a Heartbeat and (optionally) a Get_Encoder_Estimates frame,
// exactly as the real motors do.
class FakeCanNetwork
{
public:
  struct MotorSim
  {
    std::string bus;

    // What the axis reports. Starts idle, like a motor at power-on.
    gim6010_driver::AxisState axis_state{gim6010_driver::AxisState::kIdle};
    uint32_t axis_error{0};

    // Position reported before closed-loop control is reached. On real
    // hardware this can be garbage, which is the entire reason startup must
    // not use it, so tests deliberately set it to a value that would be
    // obvious if it leaked into the ROS state.
    float pre_closed_loop_position_rev{0.0F};
    // Position reported once closed loop is reached. Advances by
    // position_step_rev per frame so a test can tell exactly which frame a
    // value came from.
    float closed_loop_position_rev{0.0F};
    float position_step_rev{0.0F};
    float velocity_rev_s{0.0F};

    bool heartbeat_enabled{true};
    bool encoder_enabled{true};
    // If false, Set_Axis_State(ClosedLoopControl) is ignored and the axis
    // stays idle -- a motor that will not enable.
    bool accepts_closed_loop{true};
    // Number of encoder frames still to broadcast after reaching closed
    // loop; negative means unlimited. Set to 0 to model a motor that goes
    // silent exactly when its position is needed.
    int post_closed_loop_encoder_budget{-1};
    // Fault raised the moment closed loop is requested, modelling a motor
    // that trips on enable.
    uint32_t axis_error_on_enable{0};
  };

  void add_motor(uint8_t node_id, MotorSim motor) {motors_.emplace(node_id, std::move(motor));}

  MotorSim & motor(uint8_t node_id) {return motors_.at(node_id);}

  // Every frame this network has been asked to transmit, in order.
  const std::vector<gim6010_driver::CanFrame> & sent_frames() const {return sent_frames_;}

  std::size_t count_sent(uint8_t node_id, gim6010_driver::CommandId command) const
  {
    return static_cast<std::size_t>(std::count_if(
      sent_frames_.begin(), sent_frames_.end(),
             [node_id, command](const gim6010_driver::CanFrame & frame) {
               return gim6010_driver::node_id_from_arbitration_id(frame.id) == node_id &&
                      gim6010_driver::cmd_id_from_arbitration_id(frame.id) ==
                      static_cast<uint8_t>(command);
      }));
  }

  std::size_t count_sent(gim6010_driver::CommandId command) const
  {
    return static_cast<std::size_t>(std::count_if(
      sent_frames_.begin(), sent_frames_.end(),
             [command](const gim6010_driver::CanFrame & frame) {
               return gim6010_driver::cmd_id_from_arbitration_id(frame.id) ==
                      static_cast<uint8_t>(command);
      }));
  }

  std::optional<gim6010_driver::CanFrame> last_sent(
    uint8_t node_id, gim6010_driver::CommandId command) const
  {
    for (auto it = sent_frames_.rbegin(); it != sent_frames_.rend(); ++it) {
      if (gim6010_driver::node_id_from_arbitration_id(it->id) == node_id &&
        gim6010_driver::cmd_id_from_arbitration_id(it->id) == static_cast<uint8_t>(command))
      {
        return *it;
      }
    }
    return std::nullopt;
  }

  // Index into sent_frames_ of the first frame matching node/command, or
  // sent_frames_.size() if there is none. Lets a test assert ordering
  // ("gains before controller mode before closed loop").
  std::size_t first_sent_index(uint8_t node_id, gim6010_driver::CommandId command) const
  {
    for (std::size_t i = 0; i < sent_frames_.size(); ++i) {
      if (gim6010_driver::node_id_from_arbitration_id(sent_frames_[i].id) == node_id &&
        gim6010_driver::cmd_id_from_arbitration_id(sent_frames_[i].id) ==
        static_cast<uint8_t>(command))
      {
        return i;
      }
    }
    return sent_frames_.size();
  }

  void clear_sent() {sent_frames_.clear();}

  std::unique_ptr<gim6010_driver::CanSocketInterface> make_socket(const std::string & bus);

private:
  friend class FakeCanSocket;

  void on_send(const gim6010_driver::CanFrame & frame);
  // Queues one broadcast cycle's worth of frames for every motor on `bus`.
  void refill(const std::string & bus);

  std::map<uint8_t, MotorSim> motors_;
  std::vector<gim6010_driver::CanFrame> sent_frames_;
  std::map<std::string, std::deque<gim6010_driver::CanFrame>> rx_queues_;
};

class FakeCanSocket : public gim6010_driver::CanSocketInterface
{
public:
  FakeCanSocket(FakeCanNetwork & network, std::string bus)
  : network_(network), bus_(std::move(bus)) {}

  bool open() override
  {
    open_ = true;
    return true;
  }
  void close() override {open_ = false;}
  bool is_open() const noexcept override {return open_;}

  bool send(const gim6010_driver::CanFrame & frame) override
  {
    if (!open_) {
      return false;
    }
    network_.on_send(frame);
    return true;
  }

  // MotorManager::poll() drains with `while (receive_nonblocking())`, so
  // this hands out exactly one broadcast cycle per drain and then reports
  // empty; the next drain refills. That mirrors real polling -- each poll()
  // sees whatever accumulated since the last one -- without ever making the
  // drain loop unable to terminate.
  std::optional<gim6010_driver::CanFrame> receive_nonblocking() override
  {
    auto & queue = network_.rx_queues_[bus_];
    if (queue.empty()) {
      if (drained_) {
        network_.refill(bus_);
        drained_ = false;
      } else {
        drained_ = true;
        return std::nullopt;
      }
    }
    if (queue.empty()) {
      drained_ = true;
      return std::nullopt;
    }
    const auto frame = queue.front();
    queue.pop_front();
    return frame;
  }

  gim6010_driver::CanBusState bus_state() const noexcept override
  {
    return gim6010_driver::CanBusState::kActive;
  }
  const std::string & interface_name() const noexcept override {return bus_;}

private:
  FakeCanNetwork & network_;
  std::string bus_;
  bool open_{false};
  // Starts true so the very first receive_nonblocking() refills rather than
  // reporting an empty bus.
  bool drained_{true};
};

inline std::unique_ptr<gim6010_driver::CanSocketInterface> FakeCanNetwork::make_socket(
  const std::string & bus)
{
  rx_queues_[bus];
  return std::make_unique<FakeCanSocket>(*this, bus);
}

inline void FakeCanNetwork::on_send(const gim6010_driver::CanFrame & frame)
{
  sent_frames_.push_back(frame);

  const uint8_t node_id = gim6010_driver::node_id_from_arbitration_id(frame.id);
  const auto it = motors_.find(node_id);
  if (it == motors_.end()) {
    return;
  }
  auto & motor = it->second;

  if (gim6010_driver::cmd_id_from_arbitration_id(frame.id) !=
    static_cast<uint8_t>(gim6010_driver::CommandId::kSetAxisState))
  {
    return;
  }

  const auto requested = static_cast<gim6010_driver::AxisState>(
    gim6010_driver::detail::read_le<uint32_t>(frame.data, 0));

  if (requested == gim6010_driver::AxisState::kClosedLoopControl) {
    if (motor.axis_error_on_enable != 0) {
      motor.axis_error = motor.axis_error_on_enable;
      return;
    }
    if (motor.accepts_closed_loop) {
      motor.axis_state = gim6010_driver::AxisState::kClosedLoopControl;
    }
  } else if (requested == gim6010_driver::AxisState::kIdle) {
    motor.axis_state = gim6010_driver::AxisState::kIdle;
  }
}

inline void FakeCanNetwork::refill(const std::string & bus)
{
  auto & queue = rx_queues_[bus];
  for (auto & [node_id, motor] : motors_) {
    if (motor.bus != bus) {
      continue;
    }

    if (motor.heartbeat_enabled) {
      gim6010_driver::CanFrame heartbeat;
      heartbeat.id = gim6010_driver::make_arbitration_id(
        node_id, static_cast<uint8_t>(gim6010_driver::CommandId::kHeartbeat));
      heartbeat.dlc = 8;
      gim6010_driver::detail::write_le<uint32_t>(heartbeat.data, 0, motor.axis_error);
      heartbeat.data[4] = static_cast<uint8_t>(motor.axis_state);
      queue.push_back(heartbeat);
    }

    const bool in_closed_loop =
      motor.axis_state == gim6010_driver::AxisState::kClosedLoopControl;
    if (!motor.encoder_enabled) {
      continue;
    }
    if (in_closed_loop && motor.post_closed_loop_encoder_budget == 0) {
      continue;
    }
    if (in_closed_loop && motor.post_closed_loop_encoder_budget > 0) {
      --motor.post_closed_loop_encoder_budget;
    }

    float position = motor.pre_closed_loop_position_rev;
    if (in_closed_loop) {
      position = motor.closed_loop_position_rev;
      motor.closed_loop_position_rev += motor.position_step_rev;
    }

    gim6010_driver::CanFrame encoder;
    encoder.id = gim6010_driver::make_arbitration_id(
      node_id, static_cast<uint8_t>(gim6010_driver::CommandId::kGetEncoderEstimates));
    encoder.dlc = 8;
    gim6010_driver::detail::write_le<float>(encoder.data, 0, position);
    gim6010_driver::detail::write_le<float>(encoder.data, 4, motor.velocity_rev_s);
    queue.push_back(encoder);
  }
}

}  // namespace quattro_hardware_test

#endif  // QUATTRO_HARDWARE__TEST__FAKE_CAN_NETWORK_HPP_
