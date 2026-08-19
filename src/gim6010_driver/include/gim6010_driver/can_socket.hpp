#ifndef GIM6010_DRIVER__CAN_SOCKET_HPP_
#define GIM6010_DRIVER__CAN_SOCKET_HPP_

#include <chrono>
#include <string>

#include "gim6010_driver/can_frame.hpp"

namespace gim6010_driver
{

// RAII wrapper around one Linux SocketCAN raw socket. This is the only class
// in the driver that touches the OS; everything above it (Gim6010Motor,
// MotorManager) only exchanges CanFrame values so protocol logic stays
// testable without a real interface.
class CanSocket
{
public:
  explicit CanSocket(const std::string & interface_name);
  ~CanSocket();
  CanSocket(const CanSocket &) = delete;
  CanSocket & operator=(const CanSocket &) = delete;
  CanSocket(CanSocket &&) = delete;
  CanSocket & operator=(CanSocket &&) = delete;

  // Throws std::runtime_error if the frame cannot be written (e.g. TX queue
  // full or interface down). Never blocks.
  void send(const CanFrame & frame);

  // Waits up to `timeout` for one frame (data or kernel error frame).
  // Returns false on timeout; throws std::runtime_error on socket failure.
  bool receive(CanFrame & frame, std::chrono::milliseconds timeout);

  const std::string & interfaceName() const noexcept;

private:
  std::string interface_name_;
  int fd_{-1};
};

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__CAN_SOCKET_HPP_
