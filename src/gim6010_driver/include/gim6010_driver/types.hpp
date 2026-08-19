#ifndef GIM6010_DRIVER__TYPES_HPP_
#define GIM6010_DRIVER__TYPES_HPP_

#include <cstdint>
#include <string>

namespace gim6010_driver
{

// GDS68 axis lifecycle state (Set_Axis_State / Heartbeat). Only the values
// this driver actually drives or observes are listed -- do not guess at
// additional ODrive-lineage codes that were not confirmed against the
// GIM6010-8 manual.
enum class AxisState : uint32_t
{
  kUndefined = 0,
  kIdle = 1,
  kFullCalibration = 3,
  kMotorCalibration = 4,
  kEncoderCalibration = 7,
  kClosedLoopControl = 8,
};

// Set_Controller_Mode Control_Mode field.
enum class ControlMode : uint32_t
{
  kVoltageControl = 0,
  kTorqueControl = 1,
  kVelocityControl = 2,
  kPositionControl = 3,
};

// Set_Controller_Mode Input_Mode field.
enum class InputMode : uint32_t
{
  kInactive = 0,
  kDirect = 1,
  kVelRamp = 2,
  kPosFilter = 3,
  kTrapTraj = 5,
  kTorqueRamp = 6,
  kMitMotionControl = 9,
};

// Linux SocketCAN error-frame derived bus health, coarsest to worst.
enum class CanBusState : uint8_t
{
  kActive = 0,
  kWarning = 1,
  kPassive = 2,
  kBusOff = 3,
};

struct CanBusError
{
  bool error_warning{false};
  bool error_passive{false};
  bool bus_off{false};
  uint8_t tx_error_counter{0};
  uint8_t rx_error_counter{0};
};

// Maps a motor's CAN Simple node ID to the physical bus (SocketCAN
// interface name, e.g. "can0") it is wired to.
struct MotorRoute
{
  uint8_t node_id{0};
  std::string bus;
};

// Arbitration ID is an 11-bit standard ID: (node_id << 5) | cmd_id. cmd_id
// therefore only ever occupies the low 5 bits, leaving node_id up to 63.
constexpr uint8_t kMaxNodeId = 63;
constexpr uint8_t kCommandIdMask = 0x1F;

constexpr uint32_t make_arbitration_id(uint8_t node_id, uint8_t cmd_id) noexcept
{
  return (static_cast<uint32_t>(node_id) << 5) | (cmd_id & kCommandIdMask);
}

constexpr uint8_t node_id_from_arbitration_id(uint32_t arbitration_id) noexcept
{
  return static_cast<uint8_t>(arbitration_id >> 5);
}

constexpr uint8_t cmd_id_from_arbitration_id(uint32_t arbitration_id) noexcept
{
  return static_cast<uint8_t>(arbitration_id & kCommandIdMask);
}

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__TYPES_HPP_
