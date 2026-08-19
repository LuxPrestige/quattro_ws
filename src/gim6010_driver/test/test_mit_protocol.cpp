#include <gtest/gtest.h>

#include <cmath>

#include "gim6010_driver/mit_protocol.hpp"

namespace gim6010_driver
{
namespace
{

MitCommand make_command(
  double position, double velocity, double kp, double kd, double torque)
{
  MitCommand command;
  command.position_rad = position;
  command.velocity_rad_s = velocity;
  command.kp = kp;
  command.kd = kd;
  command.torque_Nm = torque;
  return command;
}

TEST(MitProtocol, EncodesMidRangeCommandWithCorrectArbitrationId)
{
  const auto command = make_command(1.0, -2.0, 60.0, 0.8, -3.5);
  const auto frame = encode_mit_command(7, command);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(node_id_from_arbitration_id(frame->id), 7);
  EXPECT_EQ(cmd_id_from_arbitration_id(frame->id), 0x08);
  EXPECT_EQ(frame->dlc, 8);
}

// The feedback frame's byte layout is NOT the command frame's byte layout
// with a node_id prefix tacked on: feedback carries no Kp/Kd, so
// position/velocity/torque are packed more tightly (6 bytes total, not 8).
// This builds a feedback frame by hand, independent of encode_mit_command,
// to verify decode_mit_feedback's own bit layout in isolation.
TEST(MitProtocol, DecodesFeedbackFrameBitLayout)
{
  auto pack = [](double value, double min, double max, uint32_t max_int) {
    const double scaled = (value - min) * static_cast<double>(max_int) / (max - min);
    return static_cast<uint32_t>(std::lround(scaled));
  };

  const double position = 1.0;
  const double velocity = -2.0;
  const double torque = -3.5;
  const uint32_t pos_int = pack(position, -kMitPositionRangeRad, kMitPositionRangeRad, 65535);
  const uint32_t vel_int = pack(velocity, -kMitVelocityRangeRadS, kMitVelocityRangeRadS, 4095);
  const uint32_t torque_int = pack(torque, -kMitTorqueRangeNm, kMitTorqueRangeNm, 4095);

  CanFrame frame;
  frame.dlc = 6;
  frame.data[0] = 7;  // node_id
  frame.data[1] = static_cast<uint8_t>(pos_int >> 8);
  frame.data[2] = static_cast<uint8_t>(pos_int & 0xFFU);
  frame.data[3] = static_cast<uint8_t>(vel_int >> 4);
  frame.data[4] = static_cast<uint8_t>(((vel_int & 0x0FU) << 4) | (torque_int >> 8));
  frame.data[5] = static_cast<uint8_t>(torque_int & 0xFFU);

  const auto decoded = decode_mit_feedback(frame);
  EXPECT_EQ(decoded.node_id, 7);
  EXPECT_NEAR(decoded.position_rad, position, 2 * kMitPositionRangeRad / 65535.0);
  EXPECT_NEAR(decoded.velocity_rad_s, velocity, 2 * kMitVelocityRangeRadS / 4095.0);
  EXPECT_NEAR(decoded.torque_Nm, torque, 2 * kMitTorqueRangeNm / 4095.0);
}

TEST(MitProtocol, EncodesExactBoundaryValues)
{
  const auto min_command = make_command(
    -kMitPositionRangeRad, -kMitVelocityRangeRadS, 0.0, 0.0, -kMitTorqueRangeNm);
  const auto min_frame = encode_mit_command(0, min_command);
  ASSERT_TRUE(min_frame.has_value());
  EXPECT_EQ(min_frame->data[0], 0);
  EXPECT_EQ(min_frame->data[1], 0);

  const auto max_command =
    make_command(kMitPositionRangeRad, kMitVelocityRangeRadS, kMitKpMax, kMitKdMax, kMitTorqueRangeNm);
  const auto max_frame = encode_mit_command(0, max_command);
  ASSERT_TRUE(max_frame.has_value());
  EXPECT_EQ(max_frame->data[0], 0xFF);
  EXPECT_EQ(max_frame->data[1], 0xFF);
}

TEST(MitProtocol, RejectsOutOfRangeInputsWithoutClamping)
{
  EXPECT_FALSE(encode_mit_command(0, make_command(kMitPositionRangeRad + 0.1, 0, 0, 0, 0)).has_value());
  EXPECT_FALSE(encode_mit_command(0, make_command(0, kMitVelocityRangeRadS + 0.1, 0, 0, 0)).has_value());
  EXPECT_FALSE(encode_mit_command(0, make_command(0, 0, kMitKpMax + 0.1, 0, 0)).has_value());
  EXPECT_FALSE(encode_mit_command(0, make_command(0, 0, 0, kMitKdMax + 0.1, 0)).has_value());
  EXPECT_FALSE(encode_mit_command(0, make_command(0, 0, 0, 0, kMitTorqueRangeNm + 0.1)).has_value());
  EXPECT_FALSE(encode_mit_command(0, make_command(0, 0, -1.0, 0, 0)).has_value());
}

TEST(MitProtocol, DecodesZeroFeedback)
{
  CanFrame frame;
  frame.dlc = 8;
  // Zero-valued raw fields map to the bottom of each field's range, not to
  // physical zero -- this checks the decode formula, not a "zero command".
  const auto decoded = decode_mit_feedback(frame);
  EXPECT_NEAR(decoded.position_rad, -kMitPositionRangeRad, 1e-9);
  EXPECT_NEAR(decoded.velocity_rad_s, -kMitVelocityRangeRadS, 1e-9);
  EXPECT_NEAR(decoded.torque_Nm, -kMitTorqueRangeNm, 1e-9);
}

}  // namespace
}  // namespace gim6010_driver
