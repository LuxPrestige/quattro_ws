#ifndef GIM6010_DRIVER__CAN_SOCKET_HPP_
#define GIM6010_DRIVER__CAN_SOCKET_HPP_

#include <optional>
#include <string>

#include "gim6010_driver/can_frame.hpp"
#include "gim6010_driver/can_socket_interface.hpp"
#include "gim6010_driver/types.hpp"

namespace gim6010_driver
{

// RAII wrapper around one Linux SocketCAN raw socket bound to a single
// interface (e.g. "can0"). Knows nothing about CAN Simple, MIT, or any
// other GDS68 protocol detail -- it only moves CanFrame values in and out.
//
// Non-blocking, no CAN ID filters are installed: the interface's full
// traffic is delivered and MotorManager routes by arbitration ID. With at
// most a handful of motors per bus, filtering would save little and a
// misconfigured filter would silently drop frames -- a worse failure mode.
class CanSocket : public CanSocketInterface
{
public:
  explicit CanSocket(std::string interface_name);
  ~CanSocket() override;

  CanSocket(const CanSocket &) = delete;
  CanSocket & operator=(const CanSocket &) = delete;
  CanSocket(CanSocket && other) noexcept;
  CanSocket & operator=(CanSocket && other) noexcept;

  // Opens and binds the socket. Returns false (with no exception) on any
  // failure -- interface missing, permission denied, etc. -- so callers can
  // report the failure through their own error path instead of unwinding.
  bool open() override;
  void close() override;
  bool is_open() const noexcept override { return fd_ >= 0; }

  // Returns false on send failure (would-block, interface down, ...). Never
  // throws: this may be called from a real-time control loop.
  bool send(const CanFrame & frame) override;

  // Drains and returns the next queued data frame, if any. Error frames
  // encountered along the way update bus_state()/poll_error_frame()
  // internally and are not returned as data.
  std::optional<CanFrame> receive_nonblocking() override;

  // Returns and clears the most recent error frame observed since the last
  // call, if any.
  std::optional<CanBusError> poll_error_frame();

  CanBusState bus_state() const noexcept override { return bus_state_; }
  const std::string & interface_name() const noexcept override { return interface_name_; }

private:
  std::string interface_name_;
  int fd_{-1};
  CanBusState bus_state_{CanBusState::kActive};
  std::optional<CanBusError> last_error_;
};

}  // namespace gim6010_driver

#endif  // GIM6010_DRIVER__CAN_SOCKET_HPP_
