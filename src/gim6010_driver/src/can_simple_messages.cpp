#include "gim6010_driver/can_simple_messages.hpp"

#include <cmath>
#include <cstring>
#include <limits>

#include "gim6010_driver/byte_utils.hpp"

namespace gim6010_driver
{

using detail::read_le;
using detail::write_le;

namespace
{

CanFrame make_request(uint8_t node_id, CommandId cmd)
{
  CanFrame frame;
  frame.id = make_arbitration_id(node_id, static_cast<uint8_t>(cmd));
  frame.dlc = 0;
  frame.rtr = true;
  return frame;
}

CanFrame make_command(uint8_t node_id, CommandId cmd, uint8_t dlc)
{
  CanFrame frame;
  frame.id = make_arbitration_id(node_id, static_cast<uint8_t>(cmd));
  frame.dlc = dlc;
  return frame;
}

}  // namespace

Heartbeat decode_heartbeat(const CanFrame & frame)
{
  Heartbeat heartbeat;
  heartbeat.axis_error = read_le<uint32_t>(frame.data, 0);
  heartbeat.axis_state = static_cast<AxisState>(frame.data[4]);
  const uint8_t flags = frame.data[5];
  heartbeat.flags.motor_error = (flags & 0x01U) != 0;
  heartbeat.flags.encoder_error = (flags & 0x02U) != 0;
  heartbeat.flags.controller_error = (flags & 0x04U) != 0;
  heartbeat.flags.system_error = (flags & 0x08U) != 0;
  heartbeat.flags.trajectory_done = (flags & 0x80U) != 0;
  heartbeat.life_counter = frame.data[7];
  return heartbeat;
}

CanFrame encode_estop(uint8_t node_id) { return make_command(node_id, CommandId::kEstop, 0); }

CanFrame encode_get_error_request(uint8_t node_id)
{
  return make_request(node_id, CommandId::kGetError);
}

AxisErrorResponse decode_get_error_response(const CanFrame & frame)
{
  AxisErrorResponse response;
  response.active_errors = read_le<uint32_t>(frame.data, 0);
  response.disarm_reason = read_le<uint32_t>(frame.data, 4);
  return response;
}

SdoValue make_sdo_value(float value)
{
  SdoValue result;
  std::memcpy(result.bytes.data(), &value, sizeof(value));
  return result;
}

SdoValue make_sdo_value(int32_t value)
{
  SdoValue result;
  std::memcpy(result.bytes.data(), &value, sizeof(value));
  return result;
}

SdoValue make_sdo_value(uint32_t value)
{
  SdoValue result;
  std::memcpy(result.bytes.data(), &value, sizeof(value));
  return result;
}

SdoValue make_sdo_value(uint8_t value)
{
  SdoValue result;
  result.bytes[0] = value;
  return result;
}

SdoValue make_sdo_value(bool value) { return make_sdo_value(static_cast<uint8_t>(value ? 1 : 0)); }

float sdo_value_as_float(const SdoValue & value)
{
  float result{};
  std::memcpy(&result, value.bytes.data(), sizeof(result));
  return result;
}

int32_t sdo_value_as_int32(const SdoValue & value)
{
  int32_t result{};
  std::memcpy(&result, value.bytes.data(), sizeof(result));
  return result;
}

uint32_t sdo_value_as_uint32(const SdoValue & value)
{
  uint32_t result{};
  std::memcpy(&result, value.bytes.data(), sizeof(result));
  return result;
}

uint8_t sdo_value_as_uint8(const SdoValue & value) { return value.bytes[0]; }

bool sdo_value_as_bool(const SdoValue & value) { return value.bytes[0] != 0; }

CanFrame encode_rxsdo(uint8_t node_id, SdoOpcode opcode, uint16_t endpoint_id, SdoValue value)
{
  CanFrame frame = make_command(node_id, CommandId::kRxSdo, 8);
  frame.data[0] = static_cast<uint8_t>(opcode);
  write_le<uint16_t>(frame.data, 1, endpoint_id);
  frame.data[3] = 0;
  std::memcpy(frame.data.data() + 4, value.bytes.data(), value.bytes.size());
  return frame;
}

TxSdoResponse decode_txsdo(const CanFrame & frame)
{
  TxSdoResponse response;
  response.endpoint_id = read_le<uint16_t>(frame.data, 1);
  std::memcpy(response.value.bytes.data(), frame.data.data() + 4, response.value.bytes.size());
  return response;
}

CanFrame encode_set_axis_node_id(uint8_t node_id, uint8_t new_node_id)
{
  CanFrame frame = make_command(node_id, CommandId::kSetAxisNodeId, 4);
  write_le<uint32_t>(frame.data, 0, static_cast<uint32_t>(new_node_id));
  return frame;
}

CanFrame encode_set_axis_state(uint8_t node_id, AxisState requested_state)
{
  CanFrame frame = make_command(node_id, CommandId::kSetAxisState, 4);
  write_le<uint32_t>(frame.data, 0, static_cast<uint32_t>(requested_state));
  return frame;
}

CanFrame encode_set_controller_mode(
  uint8_t node_id, ControlMode control_mode, InputMode input_mode)
{
  CanFrame frame = make_command(node_id, CommandId::kSetControllerMode, 8);
  write_le<uint32_t>(frame.data, 0, static_cast<uint32_t>(control_mode));
  write_le<uint32_t>(frame.data, 4, static_cast<uint32_t>(input_mode));
  return frame;
}

namespace
{
constexpr float kFeedForwardScale = 0.001F;
constexpr float kInt16Min = static_cast<float>(std::numeric_limits<int16_t>::min());
constexpr float kInt16Max = static_cast<float>(std::numeric_limits<int16_t>::max());

std::optional<int16_t> scale_to_int16(float value, float scale)
{
  const float scaled = value / scale;
  if (!std::isfinite(scaled) || scaled < kInt16Min || scaled > kInt16Max) {
    return std::nullopt;
  }
  return static_cast<int16_t>(std::lround(scaled));
}
}  // namespace

std::optional<CanFrame> encode_set_input_pos(uint8_t node_id, const SetInputPosCommand & command)
{
  const auto vel_ff = scale_to_int16(command.velocity_ff_rev_s, kFeedForwardScale);
  const auto torque_ff = scale_to_int16(command.torque_ff_Nm, kFeedForwardScale);
  if (!vel_ff || !torque_ff) {
    return std::nullopt;
  }

  CanFrame frame = make_command(node_id, CommandId::kSetInputPos, 8);
  write_le<float>(frame.data, 0, command.position_rev);
  write_le<int16_t>(frame.data, 4, *vel_ff);
  write_le<int16_t>(frame.data, 6, *torque_ff);
  return frame;
}

CanFrame encode_set_input_vel(uint8_t node_id, float velocity_rev_s, float torque_ff_Nm)
{
  CanFrame frame = make_command(node_id, CommandId::kSetInputVel, 8);
  write_le<float>(frame.data, 0, velocity_rev_s);
  write_le<float>(frame.data, 4, torque_ff_Nm);
  return frame;
}

CanFrame encode_set_input_torque(uint8_t node_id, float torque_Nm)
{
  CanFrame frame = make_command(node_id, CommandId::kSetInputTorque, 4);
  write_le<float>(frame.data, 0, torque_Nm);
  return frame;
}

CanFrame encode_get_encoder_estimates_request(uint8_t node_id)
{
  return make_request(node_id, CommandId::kGetEncoderEstimates);
}

EncoderEstimate decode_encoder_estimates(const CanFrame & frame)
{
  EncoderEstimate estimate;
  estimate.position_rev = read_le<float>(frame.data, 0);
  estimate.velocity_rev_s = read_le<float>(frame.data, 4);
  return estimate;
}

CanFrame encode_get_encoder_count_request(uint8_t node_id)
{
  return make_request(node_id, CommandId::kGetEncoderCount);
}

EncoderCount decode_encoder_count(const CanFrame & frame)
{
  EncoderCount count;
  count.shadow_count = read_le<int32_t>(frame.data, 0);
  count.count_in_cpr = read_le<int32_t>(frame.data, 4);
  return count;
}

CanFrame encode_set_limits(uint8_t node_id, float velocity_limit_rev_s, float current_limit_A)
{
  CanFrame frame = make_command(node_id, CommandId::kSetLimits, 8);
  write_le<float>(frame.data, 0, velocity_limit_rev_s);
  write_le<float>(frame.data, 4, current_limit_A);
  return frame;
}

CanFrame encode_set_traj_vel_limit(uint8_t node_id, float traj_vel_limit_rev_s)
{
  CanFrame frame = make_command(node_id, CommandId::kSetTrajVelLimit, 4);
  write_le<float>(frame.data, 0, traj_vel_limit_rev_s);
  return frame;
}

CanFrame encode_set_traj_accel_limits(
  uint8_t node_id, float traj_accel_limit_rev_s2, float traj_decel_limit_rev_s2)
{
  CanFrame frame = make_command(node_id, CommandId::kSetTrajAccelLimits, 8);
  write_le<float>(frame.data, 0, traj_accel_limit_rev_s2);
  write_le<float>(frame.data, 4, traj_decel_limit_rev_s2);
  return frame;
}

CanFrame encode_set_traj_inertia(uint8_t node_id, float traj_inertia)
{
  CanFrame frame = make_command(node_id, CommandId::kSetTrajInertia, 4);
  write_le<float>(frame.data, 0, traj_inertia);
  return frame;
}

CanFrame encode_get_bus_voltage_current_request(uint8_t node_id)
{
  return make_request(node_id, CommandId::kGetBusVoltageCurrent);
}

BusVoltageCurrent decode_bus_voltage_current(const CanFrame & frame)
{
  BusVoltageCurrent value;
  value.bus_voltage_V = read_le<float>(frame.data, 0);
  value.bus_current_A = read_le<float>(frame.data, 4);
  return value;
}

CanFrame encode_clear_errors(uint8_t node_id)
{
  return make_command(node_id, CommandId::kClearErrors, 0);
}

CanFrame encode_set_pos_gain(uint8_t node_id, float pos_gain)
{
  CanFrame frame = make_command(node_id, CommandId::kSetPosGain, 4);
  write_le<float>(frame.data, 0, pos_gain);
  return frame;
}

CanFrame encode_set_vel_gains(uint8_t node_id, float vel_gain, float vel_integrator_gain)
{
  CanFrame frame = make_command(node_id, CommandId::kSetVelGains, 8);
  write_le<float>(frame.data, 0, vel_gain);
  write_le<float>(frame.data, 4, vel_integrator_gain);
  return frame;
}

CanFrame encode_get_torques_request(uint8_t node_id)
{
  return make_request(node_id, CommandId::kGetTorques);
}

Torques decode_torques(const CanFrame & frame)
{
  Torques torques;
  torques.torque_target_Nm = read_le<float>(frame.data, 0);
  torques.torque_estimate_Nm = read_le<float>(frame.data, 4);
  return torques;
}

CanFrame encode_disable_can(uint8_t node_id)
{
  return make_command(node_id, CommandId::kDisableCan, 0);
}

CanFrame encode_save_configuration(uint8_t node_id)
{
  return make_command(node_id, CommandId::kSaveConfiguration, 0);
}

}  // namespace gim6010_driver
