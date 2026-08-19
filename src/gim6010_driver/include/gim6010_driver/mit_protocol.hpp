#ifndef GIM6010_DRIVER__MIT_PROTOCOL_HPP_
#define GIM6010_DRIVER__MIT_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace gim6010_driver
{

// Output-axis SI ranges the 8-byte MIT command/feedback frame can represent.
// Command 0x008: position 16 bits, velocity/kp/kd/torque 12 bits each.
struct MitLimits
{
  static constexpr double position_min = -12.5;
  static constexpr double position_max = 12.5;
  static constexpr double velocity_min = -65.0;
  static constexpr double velocity_max = 65.0;
  static constexpr double kp_min = 0.0;
  static constexpr double kp_max = 500.0;
  static constexpr double kd_min = 0.0;
  static constexpr double kd_max = 5.0;
  static constexpr double torque_min = -50.0;
  static constexpr double torque_max = 50.0;
};

// Field names are prefixed output_* because MIT position/velocity/torque
// are output-shaft quantities per the manual (unlike Direct Position/
// Velocity/Torque, which are rotor-side) -- see docs/gim6010_hardware.md.
struct MitCommand
{
  double output_position_rad{0.0};
  double output_velocity_rad_s{0.0};
  double kp{0.0};
  double kd{0.0};
  double output_torque_nm{0.0};
};

struct MitFeedback
{
  std::uint8_t motor_id{0};
  double output_position_rad{0.0};
  double output_velocity_rad_s{0.0};
  double output_torque_nm{0.0};
};

// Quantizes `value` (clamped to [minimum, maximum]) into an unsigned integer
// of `bits` width. Pure function, used by both encode and decode paths.
std::uint32_t floatToUint(double value, double minimum, double maximum, unsigned bits);
double uintToFloat(std::uint32_t value, double minimum, double maximum, unsigned bits);

// Throws std::out_of_range if any field falls outside MitLimits.
void validateCommand(const MitCommand & command);

// Bit-packed MSB-first, unlike every other GDS68 CAN Simple payload which is
// little-endian -- see docs/gim6010_hardware.md section 3.
// [pos:16][vel:12][kp:12][kd:12][torque:12] = 64 bits.
std::array<std::uint8_t, 8> encodeCommand(const MitCommand & command);
MitFeedback decodeFeedback(const std::uint8_t * data, std::size_t length);

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__MIT_PROTOCOL_HPP_
