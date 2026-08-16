#ifndef GIM6010_DRIVER__GIM6010_MOTOR_HPP_
#define GIM6010_DRIVER__GIM6010_MOTOR_HPP_

#include <chrono>
#include <cstdint>
#include <memory>

#include "gim6010_driver/can_socket.hpp"
#include "gim6010_driver/mit_protocol.hpp"

namespace gim6010_driver
{

enum class AxisState : std::uint32_t { kIdle = 1, kClosedLoopControl = 8 };

class Gim6010Motor
{
public:
  static constexpr std::uint8_t kMaxNodeId = 63;
  static constexpr std::uint8_t kCommandSetAxisState = 0x07;
  static constexpr std::uint8_t kCommandMitControl = 0x08;
  static constexpr std::uint8_t kCommandEncoderEstimates = 0x09;
  static constexpr std::uint8_t kCommandSetControllerMode = 0x0B;
  static constexpr std::uint8_t kCommandSetLimits = 0x0F;
  static constexpr std::uint8_t kCommandClearErrors = 0x18;

  Gim6010Motor(std::uint8_t node_id, std::shared_ptr<CanSocket> socket, double gear_ratio = 8.0);
  std::uint8_t nodeId() const noexcept;
  std::uint32_t arbitrationId(std::uint8_t command_id) const;

  void clearErrors();
  void setLimits(float velocity_limit, float current_limit);
  void setMitMode();
  void enable();
  void disable();
  void requestEncoderEstimates();
  void sendCommand(const MitCommand & command);
  void updateFeedback(const MitFeedback & feedback);
  void updateEncoderEstimates(const std::uint8_t * data, std::size_t length);

  bool hasFeedback() const noexcept;
  bool feedbackStale(std::chrono::steady_clock::duration timeout) const;
  const MitFeedback & feedback() const noexcept;

private:
  void sendRaw(
    std::uint8_t command_id, const std::uint8_t * data, std::size_t length,
    bool remote = false);
  std::uint8_t node_id_;
  std::shared_ptr<CanSocket> socket_;
  MitFeedback feedback_{};
  std::chrono::steady_clock::time_point feedback_time_{};
  bool has_feedback_{false};
  double gear_ratio_{8.0};
};

}  // namespace gim6010_driver
#endif  // GIM6010_DRIVER__GIM6010_MOTOR_HPP_
