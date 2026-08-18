#ifndef GIM6010_DRIVER__CAN_FRAME_HPP_
#define GIM6010_DRIVER__CAN_FRAME_HPP_

#include <array>
#include <cstdint>

namespace gim6010_driver
{

struct CanFrame
{
  std::uint32_t id{0};
  std::uint8_t dlc{0};
  std::array<std::uint8_t, 8> data{};
  bool remote{false};
  bool error{false};
  std::uint32_t error_class{0};
  std::uint32_t rx_overflow_count{0};
};

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__CAN_FRAME_HPP_
