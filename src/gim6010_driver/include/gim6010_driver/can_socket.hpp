#ifndef GIM6010_DRIVER__CAN_SOCKET_HPP_
#define GIM6010_DRIVER__CAN_SOCKET_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "gim6010_driver/can_frame.hpp"

namespace gim6010_driver
{

class CanSocket
{
public:
  CanSocket() = default;
  explicit CanSocket(const std::string & interface_name);
  ~CanSocket();
  CanSocket(const CanSocket &) = delete;
  CanSocket & operator=(const CanSocket &) = delete;
  CanSocket(CanSocket && other) noexcept;
  CanSocket & operator=(CanSocket && other) noexcept;

  void open(const std::string & interface_name);
  void close() noexcept;
  bool isOpen() const noexcept;
  const std::string & interfaceName() const noexcept;
  void send(const CanFrame & frame) const;
  bool receive(CanFrame & frame, std::chrono::milliseconds timeout) const;
  void setStandardFilters(const std::vector<std::uint32_t> & ids);

private:
  int fd_{-1};
  std::string interface_name_;
};

}  // namespace gim6010_driver
#endif  // GIM6010_DRIVER__CAN_SOCKET_HPP_
