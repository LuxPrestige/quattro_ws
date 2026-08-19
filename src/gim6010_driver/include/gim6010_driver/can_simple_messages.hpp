#ifndef GIM6010_DRIVER__CAN_SIMPLE_MESSAGES_HPP_
#define GIM6010_DRIVER__CAN_SIMPLE_MESSAGES_HPP_

#include <array>
#include <cstdint>
#include <optional>

#include "gim6010_driver/can_frame.hpp"
#include "gim6010_driver/types.hpp"

// CAN Simple encode/decode for GDS68's ODrive-derived command set. Every
// function here is a pure transform between value types and CanFrame -- no
// socket, no ROS, no Quattro-specific concept -- so it is unit-testable
// without a CAN bus and reusable by any project driving a GDS68/GIM6010-8
// (or other CAN-Simple-compatible ODrive-lineage driver) over SocketCAN.
//
// Coverage: axis lifecycle (Estop/Set_Axis_State/Set_Axis_Node_Id/
// Disable_Can), Direct Position/Velocity/Torque control, trap-trajectory
// shaping limits, limits and Direct-mode controller gains, encoder/bus/
// torque telemetry, error query/clear/save, and a generic RxSdo/TxSdo
// parameter read/write escape hatch for anything not covered by a named
// command. MIT motion control lives in mit_protocol.hpp because its wire
// format (MSB-first bit-packing) is unrelated to the little-endian
// byte-aligned fields used everywhere in this file.
//
// Voltage control (ControlMode::kVoltageControl) is intentionally NOT
// paired with a dedicated "Set_Input_Voltage" command: no CAN Simple
// message for driving voltage control directly was confirmed against the
// GIM6010-8 manual, and guessing a command ID/payload for a mode that can
// spin a motor is not an acceptable risk. If a project confirms this on
// real hardware, add the command here rather than working around its
// absence.
namespace gim6010_driver
{

enum class CommandId : uint8_t
{
  kHeartbeat = 0x01,
  kEstop = 0x02,
  kGetError = 0x03,
  kRxSdo = 0x04,
  kTxSdo = 0x05,
  kSetAxisNodeId = 0x06,
  kSetAxisState = 0x07,
  kMitControl = 0x08,
  kGetEncoderEstimates = 0x09,
  kGetEncoderCount = 0x0A,
  kSetControllerMode = 0x0B,
  kSetInputPos = 0x0C,
  kSetInputVel = 0x0D,
  kSetInputTorque = 0x0E,
  kSetLimits = 0x0F,
  kSetTrajVelLimit = 0x11,
  kSetTrajAccelLimits = 0x12,
  kSetTrajInertia = 0x13,
  kGetBusVoltageCurrent = 0x17,
  kClearErrors = 0x18,
  kSetPosGain = 0x1A,
  kSetVelGains = 0x1B,
  kGetTorques = 0x1C,
  kDisableCan = 0x1E,
  kSaveConfiguration = 0x1F,
};

// --- Heartbeat (0x01, motor -> host, periodic broadcast) -------------------

struct HeartbeatFlags
{
  bool motor_error{false};
  bool encoder_error{false};
  bool controller_error{false};
  bool system_error{false};
  bool trajectory_done{false};
};

struct Heartbeat
{
  uint32_t axis_error{0};
  AxisState axis_state{AxisState::kUndefined};
  HeartbeatFlags flags{};
  // Free-running 0-255 counter the device increments every heartbeat; used
  // to detect a missed frame without depending on wall-clock timing alone.
  uint8_t life_counter{0};
};

Heartbeat decode_heartbeat(const CanFrame & frame);

// --- Estop (0x02, host -> motor) -------------------------------------------

CanFrame encode_estop(uint8_t node_id);

// --- Get_Error (0x03) -------------------------------------------------------

struct AxisErrorResponse
{
  uint32_t active_errors{0};
  uint32_t disarm_reason{0};
};

CanFrame encode_get_error_request(uint8_t node_id);
AxisErrorResponse decode_get_error_response(const CanFrame & frame);

// --- RxSdo / TxSdo (0x04/0x05) — generic parameter read/write --------------
// Reads or writes any device parameter by its numeric "endpoint ID", the
// same mechanism odrivetool/Set_Pos_Gain/Set_Limits/etc. are themselves
// built on. This is the escape hatch for parameters this driver does not
// name explicitly (pole pairs, torque constant, thermistor limits, ...):
// the GIM6010-8 manual only points at a firmware-version-specific JSON
// endpoint table hosted elsewhere, not a fixed table in the manual itself,
// so endpoint IDs cannot be hardcoded here -- callers must supply the
// endpoint ID for their firmware version and know the value's real type.
// TxSdo's response carries the endpoint_id back, so unlike Get_Error this
// is self-tagged and needs no request queue even with several requests in
// flight.
//
// SdoValue is 4 raw bytes; use the make_sdo_value/sdo_value_as_* helpers to
// interpret them as whatever type the target endpoint actually is.

enum class SdoOpcode : uint8_t
{
  kRead = 0x00,
  kWrite = 0x01,
};

struct SdoValue
{
  std::array<uint8_t, 4> bytes{};
};

SdoValue make_sdo_value(float value);
SdoValue make_sdo_value(int32_t value);
SdoValue make_sdo_value(uint32_t value);
SdoValue make_sdo_value(uint8_t value);
SdoValue make_sdo_value(bool value);

float sdo_value_as_float(const SdoValue & value);
int32_t sdo_value_as_int32(const SdoValue & value);
uint32_t sdo_value_as_uint32(const SdoValue & value);
uint8_t sdo_value_as_uint8(const SdoValue & value);
bool sdo_value_as_bool(const SdoValue & value);

// For a read, `value` is ignored by the device (send a default-constructed
// SdoValue{}). For a write, `value` is the new value to set.
CanFrame encode_rxsdo(
  uint8_t node_id, SdoOpcode opcode, uint16_t endpoint_id, SdoValue value = {});

struct TxSdoResponse
{
  uint16_t endpoint_id{0};
  SdoValue value{};
};

TxSdoResponse decode_txsdo(const CanFrame & frame);

// --- Set_Axis_Node_Id (0x06, host -> motor) --------------------------------
// Reassigns the addressed motor's own node ID. Changes take effect
// immediately on the bus; callers must update their own routing table
// (MotorRoute) to match before sending anything else to this motor.

CanFrame encode_set_axis_node_id(uint8_t node_id, uint8_t new_node_id);

// --- Set_Axis_State (0x07, host -> motor) ----------------------------------

CanFrame encode_set_axis_state(uint8_t node_id, AxisState requested_state);

// --- Set_Controller_Mode (0x0B, host -> motor) -----------------------------

CanFrame encode_set_controller_mode(
  uint8_t node_id, ControlMode control_mode, InputMode input_mode);

// --- Set_Input_Pos / Set_Input_Vel / Set_Input_Torque (Direct control) -----

struct SetInputPosCommand
{
  float position_rev{0.0F};
  float velocity_ff_rev_s{0.0F};
  float torque_ff_Nm{0.0F};
};

// vel_ff/torque_ff are transmitted as 0.001-scaled int16 (range
// approximately +/-32.767). Returns nullopt without clamping if either
// feed-forward term does not fit -- callers must reject rather than send a
// silently truncated command.
std::optional<CanFrame> encode_set_input_pos(uint8_t node_id, const SetInputPosCommand & command);

CanFrame encode_set_input_vel(uint8_t node_id, float velocity_rev_s, float torque_ff_Nm);
CanFrame encode_set_input_torque(uint8_t node_id, float torque_Nm);

// --- Get_Encoder_Estimates (0x09) ------------------------------------------

struct EncoderEstimate
{
  float position_rev{0.0F};
  float velocity_rev_s{0.0F};
};

CanFrame encode_get_encoder_estimates_request(uint8_t node_id);
EncoderEstimate decode_encoder_estimates(const CanFrame & frame);

// --- Get_Encoder_Count (0x0A, diagnostic only, not a runtime feedback source)

struct EncoderCount
{
  int32_t shadow_count{0};
  int32_t count_in_cpr{0};
};

CanFrame encode_get_encoder_count_request(uint8_t node_id);
EncoderCount decode_encoder_count(const CanFrame & frame);

// --- Set_Limits (0x0F, host -> motor) --------------------------------------
// Both fields are motor/rotor units (rev/s, A), not ROS joint units.

CanFrame encode_set_limits(uint8_t node_id, float velocity_limit_rev_s, float current_limit_A);

// --- Set_Traj_Vel_Limit / Set_Traj_Accel_Limits / Set_Traj_Inertia --------
// (0x11/0x12/0x13, host -> motor) Configure the shaping used by
// InputMode::kTrapTraj. Selecting kTrapTraj via Set_Controller_Mode without
// ever calling these leaves it running on whatever limits are already
// stored on the device -- callers that actually want trapezoidal shaping
// should call all three explicitly rather than assume a default.
// quattro_hardware does not use this mode (its own trajectory controllers
// already shape the path -- see docs/packages/quattro_bringup.md), but it
// is part of the CAN Simple surface other projects may need.

CanFrame encode_set_traj_vel_limit(uint8_t node_id, float traj_vel_limit_rev_s);
CanFrame encode_set_traj_accel_limits(
  uint8_t node_id, float traj_accel_limit_rev_s2, float traj_decel_limit_rev_s2);
CanFrame encode_set_traj_inertia(uint8_t node_id, float traj_inertia);

// --- Get_Bus_Voltage_Current (0x17) ----------------------------------------

struct BusVoltageCurrent
{
  float bus_voltage_V{0.0F};
  float bus_current_A{0.0F};
};

CanFrame encode_get_bus_voltage_current_request(uint8_t node_id);
BusVoltageCurrent decode_bus_voltage_current(const CanFrame & frame);

// --- Clear_Errors (0x18, host -> motor) ------------------------------------
// Never call this automatically on startup: clearing on activation would
// erase the evidence of a pre-existing fault before it can be diagnosed.

CanFrame encode_clear_errors(uint8_t node_id);

// --- Set_Pos_Gain / Set_Vel_Gains (0x1A/0x1B, Direct Position/Velocity only)
// Distinct from MIT's per-command Kp/Kd (mit_protocol.hpp): these configure
// GDS68's own cascade controller and persist until changed or reset.

CanFrame encode_set_pos_gain(uint8_t node_id, float pos_gain);
CanFrame encode_set_vel_gains(uint8_t node_id, float vel_gain, float vel_integrator_gain);

// --- Get_Torques (0x1C) -----------------------------------------------------

struct Torques
{
  float torque_target_Nm{0.0F};
  float torque_estimate_Nm{0.0F};
};

CanFrame encode_get_torques_request(uint8_t node_id);
Torques decode_torques(const CanFrame & frame);

// --- Disable_Can (0x1E, host -> motor) --------------------------------------
// Stops this axis from acting on further CAN Simple commands. Depending on
// firmware this may require a power cycle (not just Estop/Set_Axis_State)
// to re-enable -- callers must treat this as effectively irreversible from
// software and only send it when that is genuinely intended.

CanFrame encode_disable_can(uint8_t node_id);

// --- Save_Configuration (0x1F, host -> motor) ------------------------------
// Never called automatically; persisting runtime gains/limits to flash is
// an explicit, separate step from applying them at runtime.

CanFrame encode_save_configuration(uint8_t node_id);

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__CAN_SIMPLE_MESSAGES_HPP_
