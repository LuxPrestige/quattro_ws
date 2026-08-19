#ifndef GIM6010_DRIVER__BYTE_UTILS_HPP_
#define GIM6010_DRIVER__BYTE_UTILS_HPP_

#include <array>
#include <cstdint>
#include <cstring>

namespace gim6010_driver::detail
{

// CAN Simple payload fields are little-endian. Every GDS68/quattro_hardware
// target (Raspberry Pi 5 aarch64, x86_64 dev containers) is little-endian
// too, so a plain memcpy is correct without an explicit byte-swap step.
template <typename T>
T read_le(const std::array<uint8_t, 8> & data, std::size_t offset)
{
  T value{};
  std::memcpy(&value, data.data() + offset, sizeof(T));
  return value;
}

template <typename T>
void write_le(std::array<uint8_t, 8> & data, std::size_t offset, T value)
{
  std::memcpy(data.data() + offset, &value, sizeof(T));
}

}  // namespace gim6010_driver::detail

#endif  // GIM6010_DRIVER__BYTE_UTILS_HPP_
