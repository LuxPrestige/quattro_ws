#include "gim6010_driver/mit_protocol.hpp"

#include <algorithm>
#include <cmath>

#include "gim6010_driver/can_simple_messages.hpp"

namespace gim6010_driver
{

namespace
{

constexpr int kPositionBits = 16;
constexpr int kVelocityBits = 12;
constexpr int kKpBits = 12;
constexpr int kKdBits = 12;
constexpr int kTorqueBits = 12;

constexpr uint32_t kPositionMaxInt = (1U << kPositionBits) - 1U;
constexpr uint32_t kVelocityMaxInt = (1U << kVelocityBits) - 1U;
constexpr uint32_t kKpMaxInt = (1U << kKpBits) - 1U;
constexpr uint32_t kKdMaxInt = (1U << kKdBits) - 1U;
constexpr uint32_t kTorqueMaxInt = (1U << kTorqueBits) - 1U;

bool in_range(double value, double min, double max) { return value >= min && value <= max; }

// Linear float -> unsigned-int packing shared by every MIT field:
// int = round((value - min) * max_int / (max - min)).
uint32_t pack(double value, double min, double max, uint32_t max_int)
{
  const double scaled = (value - min) * static_cast<double>(max_int) / (max - min);
  const double rounded = std::lround(scaled);
  return static_cast<uint32_t>(std::clamp(rounded, 0.0, static_cast<double>(max_int)));
}

double unpack(uint32_t raw, double min, double max, uint32_t max_int)
{
  return static_cast<double>(raw) * (max - min) / static_cast<double>(max_int) + min;
}

}  // namespace

std::optional<CanFrame> encode_mit_command(uint8_t node_id, const MitCommand & command)
{
  if (!in_range(command.position_rad, -kMitPositionRangeRad, kMitPositionRangeRad) ||
    !in_range(command.velocity_rad_s, -kMitVelocityRangeRadS, kMitVelocityRangeRadS) ||
    !in_range(command.kp, 0.0, kMitKpMax) ||
    !in_range(command.kd, 0.0, kMitKdMax) ||
    !in_range(command.torque_Nm, -kMitTorqueRangeNm, kMitTorqueRangeNm))
  {
    return std::nullopt;
  }

  const uint32_t pos_int =
    pack(command.position_rad, -kMitPositionRangeRad, kMitPositionRangeRad, kPositionMaxInt);
  const uint32_t vel_int =
    pack(command.velocity_rad_s, -kMitVelocityRangeRadS, kMitVelocityRangeRadS, kVelocityMaxInt);
  const uint32_t kp_int = pack(command.kp, 0.0, kMitKpMax, kKpMaxInt);
  const uint32_t kd_int = pack(command.kd, 0.0, kMitKdMax, kKdMaxInt);
  const uint32_t torque_int =
    pack(command.torque_Nm, -kMitTorqueRangeNm, kMitTorqueRangeNm, kTorqueMaxInt);

  CanFrame frame;
  frame.id = make_arbitration_id(node_id, static_cast<uint8_t>(CommandId::kMitControl));
  frame.dlc = 8;
  frame.data[0] = static_cast<uint8_t>(pos_int >> 8);
  frame.data[1] = static_cast<uint8_t>(pos_int & 0xFFU);
  frame.data[2] = static_cast<uint8_t>(vel_int >> 4);
  frame.data[3] =
    static_cast<uint8_t>(((vel_int & 0x0FU) << 4) | ((kp_int >> 8) & 0x0FU));
  frame.data[4] = static_cast<uint8_t>(kp_int & 0xFFU);
  frame.data[5] = static_cast<uint8_t>(kd_int >> 4);
  frame.data[6] =
    static_cast<uint8_t>(((kd_int & 0x0FU) << 4) | ((torque_int >> 8) & 0x0FU));
  frame.data[7] = static_cast<uint8_t>(torque_int & 0xFFU);
  return frame;
}

MitFeedback decode_mit_feedback(const CanFrame & frame)
{
  MitFeedback feedback;
  feedback.node_id = frame.data[0];

  const uint32_t pos_int =
    (static_cast<uint32_t>(frame.data[1]) << 8) | static_cast<uint32_t>(frame.data[2]);
  const uint32_t vel_int =
    (static_cast<uint32_t>(frame.data[3]) << 4) | (static_cast<uint32_t>(frame.data[4]) >> 4);
  const uint32_t torque_int =
    ((static_cast<uint32_t>(frame.data[4]) & 0x0FU) << 8) | static_cast<uint32_t>(frame.data[5]);

  feedback.position_rad =
    unpack(pos_int, -kMitPositionRangeRad, kMitPositionRangeRad, kPositionMaxInt);
  feedback.velocity_rad_s =
    unpack(vel_int, -kMitVelocityRangeRadS, kMitVelocityRangeRadS, kVelocityMaxInt);
  feedback.torque_Nm = unpack(torque_int, -kMitTorqueRangeNm, kMitTorqueRangeNm, kTorqueMaxInt);
  return feedback;
}

}  // namespace gim6010_driver
