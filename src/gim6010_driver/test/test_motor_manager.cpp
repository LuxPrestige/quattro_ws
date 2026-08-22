#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "gim6010_driver/can_simple_messages.hpp"
#include "gim6010_driver/motor_manager.hpp"

namespace gim6010_driver
{
namespace
{

TEST(MotorManager, RejectsDuplicateNodeId)
{
  EXPECT_THROW(
    MotorManager(
      {"can0"}, {MotorRoute{0, "can0"}, MotorRoute{0, "can0"}}),
    std::invalid_argument);
}

TEST(MotorManager, RejectsRouteToUnknownBus)
{
  EXPECT_THROW(MotorManager({"can0"}, {MotorRoute{0, "can1"}}), std::invalid_argument);
}

TEST(MotorManager, RejectsNodeIdBeyondMax)
{
  EXPECT_THROW(
    MotorManager({"can0"}, {MotorRoute{static_cast<uint8_t>(kMaxNodeId + 1), "can0"}}),
    std::invalid_argument);
}

TEST(MotorManager, RejectsEmptyBusList)
{
  EXPECT_THROW(MotorManager({}, {}), std::invalid_argument);
}

TEST(MotorManager, NodeIdsAreSortedAndComplete)
{
  MotorManager manager(
    {"can0", "can1"},
    {MotorRoute{5, "can1"}, MotorRoute{0, "can0"}, MotorRoute{3, "can0"}});
  EXPECT_EQ(manager.node_ids(), (std::vector<uint8_t>{0, 3, 5}));
}

TEST(MotorManager, MotorLookupReturnsNullForUnknownId)
{
  MotorManager manager({"can0"}, {MotorRoute{0, "can0"}});
  EXPECT_NE(manager.motor(0), nullptr);
  EXPECT_EQ(manager.motor(1), nullptr);
}

TEST(MotorManager, DispatchRoutesHeartbeatToMatchingMotor)
{
  MotorManager manager({"can0", "can1"}, {MotorRoute{2, "can0"}, MotorRoute{9, "can1"}});

  CanFrame frame;
  frame.id = make_arbitration_id(2, static_cast<uint8_t>(CommandId::kHeartbeat));
  frame.dlc = 8;
  frame.data = {0, 0, 0, 0, static_cast<uint8_t>(AxisState::kClosedLoopControl), 0, 0, 1};
  manager.dispatch(frame);

  ASSERT_NE(manager.motor(2), nullptr);
  const auto heartbeat = manager.motor(2)->last_heartbeat();
  ASSERT_TRUE(heartbeat.has_value());
  EXPECT_EQ(heartbeat->axis_state, AxisState::kClosedLoopControl);

  // The other configured motor must be unaffected.
  EXPECT_FALSE(manager.motor(9)->last_heartbeat().has_value());
}

TEST(MotorManager, DispatchIgnoresFrameForUnconfiguredNode)
{
  MotorManager manager({"can0"}, {MotorRoute{0, "can0"}});
  CanFrame frame;
  frame.id = make_arbitration_id(42, static_cast<uint8_t>(CommandId::kHeartbeat));
  frame.dlc = 8;
  EXPECT_NO_THROW(manager.dispatch(frame));
}

TEST(MotorManager, SendToUnknownNodeIdFailsWithoutOpenSocket)
{
  MotorManager manager({"can0"}, {MotorRoute{0, "can0"}});
  EXPECT_FALSE(manager.send_estop(1));  // node 1 was never configured
}

TEST(MotorManager, DispatchRoutesTxSdoResponseByEndpointId)
{
  MotorManager manager({"can0"}, {MotorRoute{0, "can0"}});

  CanFrame frame;
  frame.id = make_arbitration_id(0, static_cast<uint8_t>(CommandId::kTxSdo));
  frame.dlc = 8;
  frame.data[1] = 5;  // endpoint_id low byte
  frame.data[2] = 0;  // endpoint_id high byte
  manager.dispatch(frame);

  const auto response = manager.motor(0)->last_sdo_response();
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(response->endpoint_id, 5);
}

TEST(MotorManager, EncoderSequenceStartsAtZeroAndCountsOnlyEncoderFrames)
{
  MotorManager manager({"can0"}, {MotorRoute{0, "can0"}});
  ASSERT_NE(manager.motor(0), nullptr);
  EXPECT_EQ(manager.motor(0)->encoder_sequence(), 0U);

  CanFrame encoder_frame;
  encoder_frame.id = make_arbitration_id(0, static_cast<uint8_t>(CommandId::kGetEncoderEstimates));
  encoder_frame.dlc = 8;
  manager.dispatch(encoder_frame);
  EXPECT_EQ(manager.motor(0)->encoder_sequence(), 1U);
  manager.dispatch(encoder_frame);
  manager.dispatch(encoder_frame);
  EXPECT_EQ(manager.motor(0)->encoder_sequence(), 3U);

  // Heartbeat and MIT feedback must not advance it: callers use the counter
  // to answer "did a new 0x009 arrive?", and nothing else can answer that.
  CanFrame heartbeat_frame;
  heartbeat_frame.id = make_arbitration_id(0, static_cast<uint8_t>(CommandId::kHeartbeat));
  heartbeat_frame.dlc = 8;
  manager.dispatch(heartbeat_frame);
  CanFrame mit_frame;
  mit_frame.id = make_arbitration_id(0, static_cast<uint8_t>(CommandId::kMitControl));
  mit_frame.dlc = 8;
  manager.dispatch(mit_frame);
  EXPECT_EQ(manager.motor(0)->encoder_sequence(), 3U);
}

TEST(MotorManager, EncoderSequenceIsPerMotor)
{
  MotorManager manager({"can0"}, {MotorRoute{0, "can0"}, MotorRoute{1, "can0"}});

  CanFrame frame;
  frame.id = make_arbitration_id(1, static_cast<uint8_t>(CommandId::kGetEncoderEstimates));
  frame.dlc = 8;
  manager.dispatch(frame);

  EXPECT_EQ(manager.motor(0)->encoder_sequence(), 0U);
  EXPECT_EQ(manager.motor(1)->encoder_sequence(), 1U);
}

TEST(MotorManager, RejectsEmptySocketFactory)
{
  EXPECT_THROW(
    MotorManager({"can0"}, {MotorRoute{0, "can0"}}, MotorManager::SocketFactory{}),
    std::invalid_argument);
}

TEST(MotorManager, RejectsSocketFactoryThatReturnsNothing)
{
  EXPECT_THROW(
    MotorManager(
      {"can0"}, {MotorRoute{0, "can0"}},
      [](const std::string &) { return std::unique_ptr<CanSocketInterface>{}; }),
    std::invalid_argument);
}

TEST(MotorManager, DispatchUpdatesFeedbackFromEitherSource)
{
  MotorManager manager({"can0"}, {MotorRoute{0, "can0"}});

  CanFrame encoder_frame;
  encoder_frame.id = make_arbitration_id(0, static_cast<uint8_t>(CommandId::kGetEncoderEstimates));
  encoder_frame.dlc = 8;
  manager.dispatch(encoder_frame);
  EXPECT_TRUE(manager.motor(0)->last_encoder_estimate().has_value());

  CanFrame mit_frame;
  mit_frame.id = make_arbitration_id(0, static_cast<uint8_t>(CommandId::kMitControl));
  mit_frame.dlc = 8;
  manager.dispatch(mit_frame);
  EXPECT_TRUE(manager.motor(0)->last_mit_feedback().has_value());
}

}  // namespace
}  // namespace gim6010_driver
