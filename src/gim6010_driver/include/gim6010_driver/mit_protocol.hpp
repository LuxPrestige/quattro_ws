#ifndef GIM6010_DRIVER__MIT_PROTOCOL_HPP_
#define GIM6010_DRIVER__MIT_PROTOCOL_HPP_

#include <cstdint>
#include <optional>

#include "gim6010_driver/can_frame.hpp"
#include "gim6010_driver/types.hpp"

// MIT motion control (CAN Simple cmd_id 0x08). Kept separate from
// can_simple_messages.hpp because the wire format is MSB-first bit-packing
// across nibble boundaries, unlike every other CAN Simple field which is a
// plain little-endian byte-aligned value.
//
// Encoding source: GIM6010-8 manual rev2.2, section 4.1.2 "Mit_Control".
// Fields are output-shaft units (post 8:1 gearbox): rad, rad/s, N*m.
namespace gim6010_driver
{

// Protocol-fixed ranges (not per-motor tuning values -- hardcode, do not
// make these configurable).
constexpr double kMitPositionRangeRad = 12.5;    // +/- rad
constexpr double kMitVelocityRangeRadS = 65.0;   // +/- rad/s
constexpr double kMitKpMax = 500.0;              // [0, 500]
constexpr double kMitKdMax = 5.0;                // [0, 5]
constexpr double kMitTorqueRangeNm = 50.0;        // +/- N*m

struct MitCommand
{
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double kp{0.0};
  double kd{0.0};
  double torque_Nm{0.0};
};

struct MitFeedback
{
  uint8_t node_id{0};
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double torque_Nm{0.0};
};

// Returns nullopt without clamping if any field is outside its protocol
// range -- callers must reject the command, not silently saturate it.
std::optional<CanFrame> encode_mit_command(uint8_t node_id, const MitCommand & command);

// `frame` must have dlc >= 6 (node_id + position + velocity/torque nibble
// pair); shorter frames yield an all-zero MitFeedback.
MitFeedback decode_mit_feedback(const CanFrame & frame);

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__MIT_PROTOCOL_HPP_
