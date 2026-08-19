#include <gtest/gtest.h>

#include <cstring>

#include "gim6010_driver/can_simple_messages.hpp"

namespace gim6010_driver
{
namespace
{

TEST(ArbitrationId, PacksAndUnpacksNodeAndCommand)
{
  const uint32_t id = make_arbitration_id(11, 0x0C);
  EXPECT_EQ(id, (11U << 5) | 0x0CU);
  EXPECT_EQ(node_id_from_arbitration_id(id), 11);
  EXPECT_EQ(cmd_id_from_arbitration_id(id), 0x0C);
}

TEST(Heartbeat, DecodesAllFields)
{
  CanFrame frame;
  frame.dlc = 8;
  frame.data = {0x2A, 0x00, 0x00, 0x00, 8, 0b10000101, 0x00, 42};
  const Heartbeat heartbeat = decode_heartbeat(frame);
  EXPECT_EQ(heartbeat.axis_error, 0x2AU);
  EXPECT_EQ(heartbeat.axis_state, AxisState::kClosedLoopControl);
  EXPECT_TRUE(heartbeat.flags.motor_error);
  EXPECT_FALSE(heartbeat.flags.encoder_error);
  EXPECT_TRUE(heartbeat.flags.controller_error);
  EXPECT_FALSE(heartbeat.flags.system_error);
  EXPECT_TRUE(heartbeat.flags.trajectory_done);
  EXPECT_EQ(heartbeat.life_counter, 42);
}

TEST(GetError, RoundTrips)
{
  CanFrame frame = encode_get_error_request(5);
  EXPECT_EQ(cmd_id_from_arbitration_id(frame.id), 0x03);
  EXPECT_TRUE(frame.rtr);

  CanFrame response;
  response.dlc = 8;
  response.data = {0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
  const AxisErrorResponse decoded = decode_get_error_response(response);
  EXPECT_EQ(decoded.active_errors, 1U);
  EXPECT_EQ(decoded.disarm_reason, 2U);
}

TEST(SetAxisState, EncodesRequestedStateAsUint32)
{
  const CanFrame frame = encode_set_axis_state(3, AxisState::kClosedLoopControl);
  EXPECT_EQ(cmd_id_from_arbitration_id(frame.id), 0x07);
  EXPECT_EQ(node_id_from_arbitration_id(frame.id), 3);
  ASSERT_EQ(frame.dlc, 4);
  EXPECT_EQ(frame.data[0], 8);
  EXPECT_EQ(frame.data[1], 0);
  EXPECT_EQ(frame.data[2], 0);
  EXPECT_EQ(frame.data[3], 0);
}

TEST(SetControllerMode, EncodesBothFields)
{
  const CanFrame frame =
    encode_set_controller_mode(0, ControlMode::kPositionControl, InputMode::kMitMotionControl);
  ASSERT_EQ(frame.dlc, 8);
  EXPECT_EQ(frame.data[0], 3);  // kPositionControl
  EXPECT_EQ(frame.data[4], 9);  // kMitMotionControl
}

TEST(SetInputPos, RoundTripsScaledFeedForward)
{
  SetInputPosCommand command;
  command.position_rev = 1.5F;
  command.velocity_ff_rev_s = 2.0F;
  command.torque_ff_Nm = -1.0F;

  const auto frame = encode_set_input_pos(0, command);
  ASSERT_TRUE(frame.has_value());
  float position{};
  std::memcpy(&position, frame->data.data(), sizeof(position));
  EXPECT_FLOAT_EQ(position, 1.5F);
}

TEST(SetInputPos, RejectsFeedForwardOutOfInt16Range)
{
  SetInputPosCommand command;
  command.velocity_ff_rev_s = 1000.0F;  // far beyond +/-32.767 rev/s at 0.001 scale
  EXPECT_FALSE(encode_set_input_pos(0, command).has_value());
}

TEST(EncoderEstimates, RoundTrips)
{
  CanFrame frame;
  frame.dlc = 8;
  const float position = 3.25F;
  const float velocity = -0.5F;
  std::memcpy(frame.data.data(), &position, sizeof(position));
  std::memcpy(frame.data.data() + 4, &velocity, sizeof(velocity));

  const EncoderEstimate estimate = decode_encoder_estimates(frame);
  EXPECT_FLOAT_EQ(estimate.position_rev, position);
  EXPECT_FLOAT_EQ(estimate.velocity_rev_s, velocity);
}

TEST(SetLimits, EncodesVelocityAndCurrentAsFloats)
{
  const CanFrame frame = encode_set_limits(0, 5.0F, 10.0F);
  ASSERT_EQ(frame.dlc, 8);
  float velocity{};
  float current{};
  std::memcpy(&velocity, frame.data.data(), sizeof(velocity));
  std::memcpy(&current, frame.data.data() + 4, sizeof(current));
  EXPECT_FLOAT_EQ(velocity, 5.0F);
  EXPECT_FLOAT_EQ(current, 10.0F);
}

TEST(ClearErrorsAndSaveConfiguration, HaveNoPayload)
{
  EXPECT_EQ(encode_clear_errors(0).dlc, 0);
  EXPECT_EQ(encode_save_configuration(0).dlc, 0);
}

TEST(DisableCan, HasNoPayload)
{
  const CanFrame frame = encode_disable_can(4);
  EXPECT_EQ(cmd_id_from_arbitration_id(frame.id), 0x1E);
  EXPECT_EQ(frame.dlc, 0);
}

TEST(TrajectoryLimits, EncodeCorrectPayloads)
{
  const CanFrame vel = encode_set_traj_vel_limit(0, 3.0F);
  ASSERT_EQ(vel.dlc, 4);
  float decoded_vel{};
  std::memcpy(&decoded_vel, vel.data.data(), sizeof(decoded_vel));
  EXPECT_FLOAT_EQ(decoded_vel, 3.0F);

  const CanFrame accel = encode_set_traj_accel_limits(0, 1.0F, 2.0F);
  ASSERT_EQ(accel.dlc, 8);
  float decoded_accel{};
  float decoded_decel{};
  std::memcpy(&decoded_accel, accel.data.data(), sizeof(decoded_accel));
  std::memcpy(&decoded_decel, accel.data.data() + 4, sizeof(decoded_decel));
  EXPECT_FLOAT_EQ(decoded_accel, 1.0F);
  EXPECT_FLOAT_EQ(decoded_decel, 2.0F);

  const CanFrame inertia = encode_set_traj_inertia(0, 0.5F);
  EXPECT_EQ(inertia.dlc, 4);
}

TEST(SdoValue, RoundTripsEachSupportedType)
{
  EXPECT_FLOAT_EQ(sdo_value_as_float(make_sdo_value(1.5F)), 1.5F);
  EXPECT_EQ(sdo_value_as_int32(make_sdo_value(int32_t{-7})), -7);
  EXPECT_EQ(sdo_value_as_uint32(make_sdo_value(uint32_t{42})), 42U);
  EXPECT_EQ(sdo_value_as_uint8(make_sdo_value(uint8_t{9})), 9);
  EXPECT_TRUE(sdo_value_as_bool(make_sdo_value(true)));
  EXPECT_FALSE(sdo_value_as_bool(make_sdo_value(false)));
}

TEST(RxSdoTxSdo, EncodesReadRequestAndDecodesSelfTaggedResponse)
{
  const CanFrame request = encode_rxsdo(3, SdoOpcode::kRead, 271);
  EXPECT_EQ(cmd_id_from_arbitration_id(request.id), 0x04);
  EXPECT_EQ(node_id_from_arbitration_id(request.id), 3);
  ASSERT_EQ(request.dlc, 8);
  EXPECT_EQ(request.data[0], static_cast<uint8_t>(SdoOpcode::kRead));
  uint16_t decoded_endpoint{};
  std::memcpy(&decoded_endpoint, request.data.data() + 1, sizeof(decoded_endpoint));
  EXPECT_EQ(decoded_endpoint, 271);

  CanFrame response;
  response.id = make_arbitration_id(3, 0x05);
  response.dlc = 8;
  response.data[1] = 271 & 0xFF;
  response.data[2] = (271 >> 8) & 0xFF;
  const float value = 12.5F;
  std::memcpy(response.data.data() + 4, &value, sizeof(value));

  const TxSdoResponse decoded = decode_txsdo(response);
  EXPECT_EQ(decoded.endpoint_id, 271);
  EXPECT_FLOAT_EQ(sdo_value_as_float(decoded.value), 12.5F);
}

TEST(RxSdoTxSdo, EncodesWriteRequestWithValue)
{
  const CanFrame frame = encode_rxsdo(0, SdoOpcode::kWrite, 100, make_sdo_value(2.0F));
  EXPECT_EQ(frame.data[0], static_cast<uint8_t>(SdoOpcode::kWrite));
  float decoded_value{};
  std::memcpy(&decoded_value, frame.data.data() + 4, sizeof(decoded_value));
  EXPECT_FLOAT_EQ(decoded_value, 2.0F);
}

}  // namespace
}  // namespace gim6010_driver
