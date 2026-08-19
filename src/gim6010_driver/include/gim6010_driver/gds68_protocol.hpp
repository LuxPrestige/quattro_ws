#ifndef GIM6010_DRIVER__GDS68_PROTOCOL_HPP_
#define GIM6010_DRIVER__GDS68_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <utility>

#include "gim6010_driver/types.hpp"

namespace gim6010_driver
{

// GDS68 CAN Simple command ids. GDS68's CAN Simple layer is protocol-
// compatible with ODrive CAN Simple; command 0x008 is repurposed by the
// vendor firmware to carry MIT-mode bit-packed commands/feedback instead of
// the stock "Get_Motor_Error" slot.
enum class Gds68Command : std::uint8_t
{
  kHeartbeat = 0x01,
  kEstop = 0x02,
  kGetError = 0x03,
  kSetAxisState = 0x07,
  kMitControl = 0x08,
  kEncoderEstimates = 0x09,
  kEncoderCount = 0x0A,
  kSetControllerMode = 0x0B,
  kSetInputPosition = 0x0C,
  kSetInputVelocity = 0x0D,
  kSetInputTorque = 0x0E,
  kSetLimits = 0x0F,
  kSetTrajectoryVelocityLimit = 0x11,
  kSetTrajectoryAccelerationLimits = 0x12,
  kIq = 0x14,
  kBusVoltageCurrent = 0x17,
  kClearErrors = 0x18,
  kSetPositionGain = 0x1A,
  kSetVelocityGains = 0x1B,
};

enum class ControlMode : std::uint32_t
{
  kTorque = 1,
  kVelocity = 2,
  kPosition = 3,
};

enum class InputMode : std::uint32_t
{
  kDirect = 1,
  kPositionFilter = 3,
  kTrapezoidalTrajectory = 5,
  kMit = 9,
};

constexpr std::uint8_t kMaximumNodeId = 63;

// arbitration_id = (node_id << 5) | command_id, an 11-bit standard frame.
std::uint32_t makeArbitrationId(std::uint8_t node_id, Gds68Command command);
std::pair<std::uint8_t, Gds68Command> parseArbitrationId(std::uint32_t arbitration_id);

// Set_Axis_State (0x07): 1 (idle) or 8 (closed-loop control) as a
// little-endian uint32.
std::array<std::uint8_t, 4> encodeAxisState(std::uint32_t requested_state);

// Get_Error (0x03) request: manual section 4.1.2, Error_Type selects which
// category the following response reports (0=motor, 1=encoder, 3=controller,
// 4=system -- see ErrorType in types.hpp).
std::array<std::uint8_t, 1> encodeGetErrorRequest(ErrorType error_type);

// Get_Error (0x03) response. The response carries no type tag of its own --
// the width and meaning depend entirely on which ErrorType was requested,
// so the caller must track that itself (Gim6010Motor keeps a per-motor
// pending-request queue for this reason). kMotor decodes as a uint64 across
// all 8 bytes; kEncoder/kController/kSystem decode as a uint32 across the
// first 4 bytes.
std::uint64_t decodeGetErrorResponse(
  ErrorType error_type, const std::uint8_t * data, std::size_t length);

// Set_Controller_Mode (0x0B): control_mode, then input_mode, each a
// little-endian uint32.
std::array<std::uint8_t, 8> encodeControllerMode(ControlMode control, InputMode input);

// Set_Input_Pos (0x0C): Input_Pos float32 (rev), Vel_FF int16 (0.001 rev/s
// LSB), Torque_FF int16 (0.001 N*m LSB).
std::array<std::uint8_t, 8> encodeDirectPosition(
  float rotor_position_rev, float velocity_feedforward_rev_s = 0.0F,
  float torque_feedforward_nm = 0.0F);

// Set_Input_Vel (0x0D): Input_Vel float32 (rev/s), Torque_FF float32 (N*m).
std::array<std::uint8_t, 8> encodeDirectVelocity(
  float rotor_velocity_rev_s, float torque_feedforward_nm = 0.0F);

// Set_Input_Torque (0x0E): Input_Torque float32 (N*m).
std::array<std::uint8_t, 4> encodeDirectTorque(float motor_torque_nm);

// Get_Encoder_Estimates (0x09) response: Pos_Estimate float32 (rev),
// Vel_Estimate float32 (rev/s). This is the single onboard MA732 absolute
// encoder's estimate -- GIM6010-8 has no second physical encoder (see
// docs/gim6010_hardware.md section 11); this is the sole runtime position
// source.
std::pair<float, float> decodeEncoderEstimates(const std::uint8_t * data, std::size_t length);

// Get_Encoder_Count (0x0A) response: the same onboard encoder's raw
// counters -- Shadow_Count is the accumulated multi-turn rotor count,
// Count_In_Cpr is the single-turn count within one rotor revolution.
// Diagnostic only: this is volatile firmware state, not a second sensor,
// and is not known to survive a power cycle, so it must not be blended
// into runtime position/velocity feedback.
struct EncoderCount
{
  std::int32_t shadow_count{0};
  std::int32_t count_in_cpr{0};
};
EncoderCount decodeEncoderCount(const std::uint8_t * data, std::size_t length);

// Rotor rev / rev-s -> output-shaft rad / rad-s, for a `gear_ratio`
// output-per-rotor reduction (8.0 for GIM6010-8's 8:1 gearbox). Pure
// function so the conversion used to turn Get_Encoder_Estimates into
// runtime output-shaft feedback is independently unit-testable.
double rotorRevToOutputRad(double rotor_rev, double gear_ratio);
double rotorRevPerSecToOutputRadPerSec(double rotor_rev_s, double gear_ratio);

// Set_Limits (0x0F): Velocity_Limit float32 (rev/s), Current_Limit float32
// (A).
std::array<std::uint8_t, 8> encodeLimits(float velocity_limit_rev_s, float current_limit_a);

// Set_Pos_Gain (0x1A): Pos_Gain float32 ((rev/s)/rev).
std::array<std::uint8_t, 4> encodePositionGain(float position_gain);

// Set_Vel_Gains (0x1B): Vel_Gain float32 (N*m/(rev/s)), Vel_Integrator_Gain
// float32 (N*m/rev).
std::array<std::uint8_t, 8> encodeVelocityGains(
  float velocity_gain,
  float velocity_integrator_gain);

// Set_Traj_Vel_Limit (0x11): Traj_Vel_Limit float32 (rev/s).
std::array<std::uint8_t, 4> encodeTrajectoryVelocityLimit(float rotor_velocity_rev_s);

// Set_Traj_Accel_Limits (0x12): Traj_Accel_Limit, Traj_Decel_Limit, both
// float32 (rev/s^2).
std::array<std::uint8_t, 8> encodeTrajectoryAccelerationLimits(
  float rotor_acceleration_rev_s2, float rotor_deceleration_rev_s2);

// Get_Iq (0x14) response: Iq_Setpoint float32 (A), Iq_Measured float32 (A).
std::pair<float, float> decodeIq(const std::uint8_t * data, std::size_t length);

// Get_Bus_Voltage_Current (0x17) response: Bus_Voltage float32 (V),
// Bus_Current float32 (A).
std::pair<float, float> decodeBusVoltageCurrent(const std::uint8_t * data, std::size_t length);

// Heartbeat (0x01), GIM6010 firmware 0.5.13+ layout: Axis_Error uint32,
// Axis_State uint8, Flags uint8 (fault bits in the low nibble), reserved
// uint8, Life uint8 (increments every heartbeat; used to detect drops).
struct Heartbeat
{
  std::uint32_t axis_error{0};
  std::uint8_t axis_state{0};
  std::uint8_t flags{0};
  std::uint8_t life{0};
};
Heartbeat decodeHeartbeat(const std::uint8_t * data, std::size_t length);

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__GDS68_PROTOCOL_HPP_
