// Startup-sequence tests for QuattroSystem, driven against an in-memory CAN
// bus (fake_can_network.hpp) so the real gim6010_driver encode/decode,
// routing and encoder-sequence bookkeeping stay in the loop.
//
// What is being protected here is the GIM6010-8 hardware contract confirmed
// on the real robot (AGENTS.md): a position read before closed-loop control
// is not a usable position, and closed-loop entry alone holds the axis, so
// startup never sends Set_Input_Pos. Both are easy to regress by
// "restoring" the older safe-start logic, and neither shows up as a
// compile error -- only as a joint that snaps to the wrong angle on a real
// robot.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "gim6010_driver/byte_utils.hpp"
#include "gim6010_driver/can_simple_messages.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "quattro_hardware/joint_transform.hpp"
#include "quattro_hardware/quattro_system.hpp"
#include "rclcpp/rclcpp.hpp"

#include "fake_can_network.hpp"

namespace
{

using gim6010_driver::CommandId;
using quattro_hardware_test::FakeCanNetwork;

constexpr double kPi = 3.14159265358979323846;
constexpr double kGearRatio = 8.0;

// QuattroSystem with the one seam the tests need: its MotorManager is built
// on the fake transport instead of a real SocketCAN socket. Nothing else
// about the class is replaced or stubbed.
class TestableQuattroSystem : public quattro_hardware::QuattroSystem
{
public:
  explicit TestableQuattroSystem(FakeCanNetwork & network)
  : network_(network) {}

protected:
  std::unique_ptr<gim6010_driver::MotorManager> create_motor_manager(
    const std::vector<std::string> & buses,
    const std::vector<gim6010_driver::MotorRoute> & routes) override
  {
    FakeCanNetwork * network = &network_;
    return std::make_unique<gim6010_driver::MotorManager>(
      buses, routes,
      [network](const std::string & bus) {return network->make_socket(bus);});
  }

private:
  FakeCanNetwork & network_;
};

hardware_interface::ComponentInfo make_joint(
  const std::string & name, const std::string & bus, int can_id, double direction)
{
  hardware_interface::ComponentInfo joint;
  joint.name = name;

  hardware_interface::InterfaceInfo position_command;
  position_command.name = hardware_interface::HW_IF_POSITION;
  joint.command_interfaces.push_back(position_command);

  for (const char * interface : {
      hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT})
  {
    hardware_interface::InterfaceInfo state;
    state.name = interface;
    joint.state_interfaces.push_back(state);
  }

  joint.parameters["can_interface"] = bus;
  joint.parameters["can_id"] = std::to_string(can_id);
  joint.parameters["direction"] = direction > 0.0 ? "1.0" : "-1.0";
  joint.parameters["offset"] = "0.0";
  joint.parameters["gear_ratio"] = "8.0";
  return joint;
}

hardware_interface::HardwareInfo make_hardware_info(std::size_t joint_count)
{
  hardware_interface::HardwareInfo info;
  info.name = "QuattroSystem";
  info.type = "system";
  info.hardware_plugin_name = "quattro_hardware/QuattroSystem";

  info.hardware_parameters = {
    {"current_limit", "5.0"},
    {"position_gain", "20.0"},
    {"velocity_gain", "0.16"},
    {"velocity_integrator_gain", "0.32"},
    {"feedback_timeout_ms", "150"},
    {"heartbeat_timeout_ms", "400"},
    {"startup_timeout_ms", "500"},
    {"closed_loop_timeout_ms", "200"},
    {"encoder_sync_timeout_ms", "200"},
    {"encoder_sync_frames", "2"},
    {"command_timeout_ms", "250"},
    {"scheduling_warning_ms", "50"},
    {"rotor_velocity_limit_rev_s", "5.0"},
    {"telemetry_period_ms", "500"},
  };

  for (std::size_t i = 0; i < joint_count; ++i) {
    info.joints.push_back(
      make_joint("joint_" + std::to_string(i), "can0", static_cast<int>(i), 1.0));
  }

  // Mirrors the <gpio> block in quattro.urdf.xacro: gait_controller's
  // walking_active flag, read by QuattroSystem::read() to relax the stale
  // feedback/heartbeat checks while walking (never axis_error).
  hardware_interface::ComponentInfo safety_mode;
  safety_mode.name = "safety_mode";
  hardware_interface::InterfaceInfo walking_active;
  walking_active.name = "walking_active";
  safety_mode.command_interfaces.push_back(walking_active);
  info.gpios.push_back(safety_mode);

  return info;
}

// Builds a system that is initialized and has its interface storage
// allocated, i.e. ready for on_configure(). Mirrors what ResourceManager
// does before a lifecycle transition.
std::unique_ptr<TestableQuattroSystem> make_system(
  FakeCanNetwork & network, std::size_t joint_count)
{
  auto system = std::make_unique<TestableQuattroSystem>(network);

  hardware_interface::HardwareComponentParams params;
  params.hardware_info = make_hardware_info(joint_count);
  params.logger = rclcpp::get_logger("test_quattro_system_startup");
  params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

  if (system->init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return nullptr;
  }
  system->on_export_state_interfaces();
  system->on_export_command_interfaces();
  return system;
}

// A motor that behaves like a healthy GIM6010-8: garbage position while
// idle, a real position once closed loop is reached.
FakeCanNetwork::MotorSim healthy_motor(float closed_loop_rev)
{
  FakeCanNetwork::MotorSim motor;
  motor.bus = "can0";
  // Deliberately nothing like the closed-loop value: if this ever reaches
  // the ROS state, the assertion difference is unmistakable.
  motor.pre_closed_loop_position_rev = 987.0F;
  motor.closed_loop_position_rev = closed_loop_rev;
  return motor;
}

const rclcpp_lifecycle::State kUnconfigured;

hardware_interface::CallbackReturn configure(TestableQuattroSystem & system)
{
  return system.on_configure(kUnconfigured);
}

hardware_interface::CallbackReturn activate(TestableQuattroSystem & system)
{
  return system.on_activate(kUnconfigured);
}

double state_position(TestableQuattroSystem & system, const std::string & joint)
{
  return system.get_state<double>(joint + "/" + hardware_interface::HW_IF_POSITION);
}

// Every node that was ever commanded to idle.
bool was_idled(const FakeCanNetwork & network, uint8_t node_id)
{
  const auto frame = network.last_sent(node_id, CommandId::kSetAxisState);
  if (!frame) {
    return false;
  }
  return static_cast<gim6010_driver::AxisState>(
    gim6010_driver::detail::read_le<uint32_t>(frame->data, 0)) ==
         gim6010_driver::AxisState::kIdle;
}

class QuattroSystemStartup : public ::testing::Test
{
protected:
  FakeCanNetwork network;
};

// --- 1. pre-closed-loop encoder is never used as the startup position ----

TEST_F(QuattroSystemStartup, PreClosedLoopEncoderIsNotUsedAsStartupPosition)
{
  network.add_motor(0, healthy_motor(2.0F));
  auto system = make_system(network, 1);
  ASSERT_NE(system, nullptr);

  ASSERT_EQ(configure(*system), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(activate(*system), hardware_interface::CallbackReturn::SUCCESS);

  const double expected =
    quattro_hardware::motor_rev_to_joint_rad(2.0, {1.0, 0.0, kGearRatio});
  EXPECT_NEAR(state_position(*system, "joint_0"), expected, 1e-9);

  // The value the motor reported while idle (987 rev) must not appear
  // anywhere near the state, no matter how many frames of it arrived first.
  const double garbage =
    quattro_hardware::motor_rev_to_joint_rad(987.0, {1.0, 0.0, kGearRatio});
  EXPECT_GT(std::abs(state_position(*system, "joint_0") - garbage), 1.0);
}

// --- 2. controller mode is Position Control + Pos Filter -----------------

TEST_F(QuattroSystemStartup, ConfiguresPositionControlWithPosFilter)
{
  network.add_motor(0, healthy_motor(1.0F));
  network.add_motor(1, healthy_motor(1.0F));
  auto system = make_system(network, 2);
  ASSERT_NE(system, nullptr);

  ASSERT_EQ(configure(*system), hardware_interface::CallbackReturn::SUCCESS);

  for (uint8_t node : {0, 1}) {
    const auto frame = network.last_sent(node, CommandId::kSetControllerMode);
    ASSERT_TRUE(frame.has_value()) << "no Set_Controller_Mode for node " << int{node};
    EXPECT_EQ(
      *frame, gim6010_driver::encode_set_controller_mode(
        node, gim6010_driver::ControlMode::kPositionControl,
        gim6010_driver::InputMode::kPosFilter));

    // The confirmed order: limits, gains, then mode -- and closed loop only
    // later, in on_activate.
    EXPECT_LT(
      network.first_sent_index(node, CommandId::kSetLimits),
      network.first_sent_index(node, CommandId::kSetPosGain));
    EXPECT_LT(
      network.first_sent_index(node, CommandId::kSetVelGains),
      network.first_sent_index(node, CommandId::kSetControllerMode));
    EXPECT_EQ(network.count_sent(node, CommandId::kSetAxisState), 0U);
  }
}

// --- 3. startup sends no Set_Input_Pos at all ----------------------------

TEST_F(QuattroSystemStartup, StartupNeverSendsSetInputPos)
{
  for (uint8_t node = 0; node < 3; ++node) {
    network.add_motor(node, healthy_motor(0.5F * static_cast<float>(node + 1)));
  }
  auto system = make_system(network, 3);
  ASSERT_NE(system, nullptr);

  ASSERT_EQ(configure(*system), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(activate(*system), hardware_interface::CallbackReturn::SUCCESS);

  // The whole point of the Pos Filter contract: the axis holds itself, so
  // nothing has to be commanded to make it hold.
  EXPECT_EQ(network.count_sent(CommandId::kSetInputPos), 0U);
}

// --- 4. only post-closed-loop encoder frames initialize the state --------

TEST_F(QuattroSystemStartup, InitialStateComesFromPostClosedLoopEncoderFrames)
{
  auto motor = healthy_motor(1.0F);
  // Each post-closed-loop frame reports a different position, so the value
  // that lands in the state identifies exactly which frame was used.
  motor.position_step_rev = 1.0F;
  network.add_motor(0, motor);

  auto system = make_system(network, 1);
  ASSERT_NE(system, nullptr);
  ASSERT_EQ(configure(*system), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(activate(*system), hardware_interface::CallbackReturn::SUCCESS);

  // The 1.0 frame arrives in the same poll batch as the closed-loop
  // Heartbeat, so it is at or below the baseline and deliberately not
  // counted: a frame that landed alongside the Heartbeat cannot be proven
  // to have been sampled after the transition the Heartbeat reports. The
  // two frames that follow (2.0 and 3.0) are the ones encoder_sync_frames
  // requires, and the last of them is what the state must hold.
  const double expected =
    quattro_hardware::motor_rev_to_joint_rad(3.0, {1.0, 0.0, kGearRatio});
  EXPECT_NEAR(state_position(*system, "joint_0"), expected, 1e-6);

  // Stated as an inequality too, so the intent survives a change to the
  // fake's frame pacing: the state must never come from the first frame
  // seen after closed loop was requested.
  const double first_frame =
    quattro_hardware::motor_rev_to_joint_rad(1.0, {1.0, 0.0, kGearRatio});
  EXPECT_GT(state_position(*system, "joint_0"), first_frame);
}

// --- 5. post-closed-loop encoder timeout fails activation, idles all -----

TEST_F(QuattroSystemStartup, EncoderTimeoutAfterClosedLoopFailsActivationAndIdlesAll)
{
  network.add_motor(0, healthy_motor(1.0F));
  auto silent = healthy_motor(1.0F);
  // Reaches closed loop, then stops reporting position -- the case where
  // the axis is enabled but there is no trustworthy angle to hand to ROS.
  silent.post_closed_loop_encoder_budget = 0;
  network.add_motor(1, silent);

  auto system = make_system(network, 2);
  ASSERT_NE(system, nullptr);
  ASSERT_EQ(configure(*system), hardware_interface::CallbackReturn::SUCCESS);

  EXPECT_EQ(activate(*system), hardware_interface::CallbackReturn::ERROR);
  EXPECT_TRUE(was_idled(network, 0));
  EXPECT_TRUE(was_idled(network, 1));
}

// --- 6. a reported axis fault fails activation, idles all ----------------

TEST_F(QuattroSystemStartup, AxisErrorOnEnableFailsActivationAndIdlesAll)
{
  network.add_motor(0, healthy_motor(1.0F));
  auto faulty = healthy_motor(1.0F);
  faulty.axis_error_on_enable = 0x00000040;
  network.add_motor(1, faulty);

  auto system = make_system(network, 2);
  ASSERT_NE(system, nullptr);
  ASSERT_EQ(configure(*system), hardware_interface::CallbackReturn::SUCCESS);

  EXPECT_EQ(activate(*system), hardware_interface::CallbackReturn::ERROR);
  EXPECT_TRUE(was_idled(network, 0));
  EXPECT_TRUE(was_idled(network, 1));
}

TEST_F(QuattroSystemStartup, PreExistingAxisErrorBlocksConfigure)
{
  auto faulty = healthy_motor(1.0F);
  faulty.axis_error = 0x00000001;
  network.add_motor(0, faulty);

  auto system = make_system(network, 1);
  ASSERT_NE(system, nullptr);

  // Refusing here, before any gains or modes are written, is what keeps a
  // pre-existing fault visible instead of quietly configured over.
  EXPECT_EQ(configure(*system), hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(network.count_sent(0, CommandId::kSetControllerMode), 0U);
  EXPECT_EQ(network.count_sent(0, CommandId::kClearErrors), 0U);
}

// --- 7. failure at joint N idles the joints already activated ------------

TEST_F(QuattroSystemStartup, FailureAtLaterJointIdlesEarlierActivatedJoints)
{
  network.add_motor(0, healthy_motor(1.0F));
  network.add_motor(1, healthy_motor(1.0F));
  auto stuck = healthy_motor(1.0F);
  // Never leaves idle, so activation fails at joint 2 with joints 0 and 1
  // already in closed loop -- the partial-activation case.
  stuck.accepts_closed_loop = false;
  network.add_motor(2, stuck);
  network.add_motor(3, healthy_motor(1.0F));

  auto system = make_system(network, 4);
  ASSERT_NE(system, nullptr);
  ASSERT_EQ(configure(*system), hardware_interface::CallbackReturn::SUCCESS);

  EXPECT_EQ(activate(*system), hardware_interface::CallbackReturn::ERROR);
  for (uint8_t node = 0; node < 4; ++node) {
    EXPECT_TRUE(was_idled(network, node)) << "node " << int{node} << " was left enabled";
  }
}

TEST_F(QuattroSystemStartup, MissingHeartbeatBlocksConfigure)
{
  auto silent = healthy_motor(1.0F);
  silent.heartbeat_enabled = false;
  network.add_motor(0, silent);

  auto system = make_system(network, 1);
  ASSERT_NE(system, nullptr);
  EXPECT_EQ(configure(*system), hardware_interface::CallbackReturn::ERROR);
}

// --- 9. the first normal command continues from the synced position ------

TEST_F(QuattroSystemStartup, FirstCommandContinuesFromSynchronizedPosition)
{
  network.add_motor(0, healthy_motor(1.25F));
  network.add_motor(1, healthy_motor(-0.75F));
  auto system = make_system(network, 2);
  ASSERT_NE(system, nullptr);
  ASSERT_EQ(configure(*system), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(activate(*system), hardware_interface::CallbackReturn::SUCCESS);

  // Activation must leave command == state. Otherwise the first write()
  // would command the interface's default 0.0 rad and the joint would snap
  // there the moment the controller takes over.
  for (const std::string joint : {"joint_0", "joint_1"}) {
    const double commanded =
      system->get_command<double>(joint + "/" + hardware_interface::HW_IF_POSITION);
    EXPECT_NEAR(commanded, state_position(*system, joint), 1e-9) << joint;
  }

  // And the first write() must put that same position on the wire.
  ASSERT_EQ(
    system->write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);

  const auto frame = network.last_sent(0, CommandId::kSetInputPos);
  ASSERT_TRUE(frame.has_value());
  const float commanded_rev = gim6010_driver::detail::read_le<float>(frame->data, 0);
  EXPECT_NEAR(commanded_rev, 1.25F, 1e-3F);
}

TEST_F(QuattroSystemStartup, ReadPublishesPostClosedLoopPositionWithoutRequestingEncoders)
{
  network.add_motor(0, healthy_motor(3.0F));
  auto system = make_system(network, 1);
  ASSERT_NE(system, nullptr);
  ASSERT_EQ(configure(*system), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(activate(*system), hardware_interface::CallbackReturn::SUCCESS);

  network.clear_sent();
  ASSERT_EQ(
    system->read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);

  const double expected =
    quattro_hardware::motor_rev_to_joint_rad(3.0, {1.0, 0.0, kGearRatio});
  EXPECT_NEAR(state_position(*system, "joint_0"), expected, 1e-6);
  // The motors broadcast 0x009 themselves; read() adding requests would be
  // pure extra TX load on the tightest resource in the system.
  EXPECT_EQ(network.count_sent(0, CommandId::kGetEncoderEstimates), 0U);
}

TEST_F(QuattroSystemStartup, DirectionAndOffsetAreAppliedToTheSynchronizedPosition)
{
  network.add_motor(0, healthy_motor(4.0F));
  auto system = std::make_unique<TestableQuattroSystem>(network);

  hardware_interface::HardwareComponentParams params;
  params.hardware_info = make_hardware_info(1);
  params.hardware_info.joints[0].parameters["direction"] = "-1.0";
  params.hardware_info.joints[0].parameters["offset"] = "0.25";
  params.logger = rclcpp::get_logger("test_quattro_system_startup");
  params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  ASSERT_EQ(system->init(params), hardware_interface::CallbackReturn::SUCCESS);
  system->on_export_state_interfaces();
  system->on_export_command_interfaces();

  ASSERT_EQ(configure(*system), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(activate(*system), hardware_interface::CallbackReturn::SUCCESS);

  const double expected = -1.0 * (4.0 * 2.0 * kPi / kGearRatio) - 0.25;
  EXPECT_NEAR(state_position(*system, "joint_0"), expected, 1e-9);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
