#ifndef GIM6010_DRIVER__MIT_PROTOCOL_HPP_
#define GIM6010_DRIVER__MIT_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace gim6010_driver
{

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

struct MitCommand
{
  double position{0.0};
  double velocity{0.0};
  double kp{0.0};
  double kd{0.0};
  double torque{0.0};
};

struct MitFeedback
{
  std::uint8_t motor_id{0};
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
};

std::uint32_t floatToUint(double value, double minimum, double maximum, unsigned bits);
double uintToFloat(std::uint32_t value, double minimum, double maximum, unsigned bits);
void validateCommand(const MitCommand & command);
std::array<std::uint8_t, 8> encodeCommand(const MitCommand & command);
MitFeedback decodeFeedback(const std::uint8_t * data, std::size_t length);

}  // namespace gim6010_driver
#endif  // GIM6010_DRIVER__MIT_PROTOCOL_HPP_
