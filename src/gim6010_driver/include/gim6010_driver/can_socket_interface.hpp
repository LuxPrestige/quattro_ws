#ifndef GIM6010_DRIVER__CAN_SOCKET_INTERFACE_HPP_
#define GIM6010_DRIVER__CAN_SOCKET_INTERFACE_HPP_

#include <optional>
#include <string>

#include "gim6010_driver/can_frame.hpp"
#include "gim6010_driver/types.hpp"

namespace gim6010_driver
{

// The frame-level transport MotorManager sits on top of. CanSocket is the
// only production implementation; the abstraction exists so a test can
// substitute an in-memory transport and exercise the real MotorManager
// routing/dispatch code (and, above it, the real startup sequence in
// quattro_hardware) without a physical bus.
//
// Deliberately the narrowest interface MotorManager actually needs:
// error-frame introspection (CanSocket::poll_error_frame()) stays on the
// concrete class, since nothing routes through this abstraction to reach
// it.
class CanSocketInterface
{
public:
  virtual ~CanSocketInterface() = default;

  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool is_open() const noexcept = 0;
  virtual bool send(const CanFrame & frame) = 0;
  virtual std::optional<CanFrame> receive_nonblocking() = 0;
  virtual CanBusState bus_state() const noexcept = 0;
  virtual const std::string & interface_name() const noexcept = 0;
};

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__CAN_SOCKET_INTERFACE_HPP_
