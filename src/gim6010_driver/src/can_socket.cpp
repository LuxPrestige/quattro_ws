#include "gim6010_driver/can_socket.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace gim6010_driver
{
namespace
{
[[noreturn]] void throwErrno(const std::string & what)
{
  throw std::runtime_error(what + ": " + std::strerror(errno));
}
}  // namespace

CanSocket::CanSocket(const std::string & interface_name)
: interface_name_(interface_name)
{
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    throwErrno("failed to open a CAN_RAW socket");
  }

  struct ifreq interface_request {};
  std::strncpy(interface_request.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &interface_request) < 0) {
    const int saved_errno = errno;
    ::close(fd_);
    fd_ = -1;
    errno = saved_errno;
    throwErrno("unknown CAN interface " + interface_name_);
  }

  // Receive kernel error frames (bus-off, error-passive/warning, ack error,
  // TX/RX error counters) in addition to data frames.
  can_err_mask_t error_mask = CAN_ERR_MASK;
  if (::setsockopt(
      fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &error_mask, sizeof(error_mask)) < 0)
  {
    const int saved_errno = errno;
    ::close(fd_);
    fd_ = -1;
    errno = saved_errno;
    throwErrno("failed to enable CAN error frames on " + interface_name_);
  }

  struct sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = interface_request.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
    const int saved_errno = errno;
    ::close(fd_);
    fd_ = -1;
    errno = saved_errno;
    throwErrno("failed to bind the CAN socket to " + interface_name_);
  }
}

CanSocket::~CanSocket()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void CanSocket::send(const CanFrame & frame)
{
  struct can_frame raw {};
  raw.can_id = frame.id & CAN_SFF_MASK;
  if (frame.remote) {
    raw.can_id |= CAN_RTR_FLAG;
  }
  raw.can_dlc = frame.dlc;
  if (!frame.remote) {
    std::memcpy(raw.data, frame.data.data(), std::min<std::size_t>(frame.dlc, 8U));
  }
  auto written = ::write(fd_, &raw, sizeof(raw));
  if (written != static_cast<decltype(written)>(sizeof(raw))) {
    throwErrno("failed to write a CAN frame on " + interface_name_);
  }
}

bool CanSocket::receive(CanFrame & frame, std::chrono::milliseconds timeout)
{
  struct pollfd poll_fd {};
  poll_fd.fd = fd_;
  poll_fd.events = POLLIN;
  const int ready = ::poll(&poll_fd, 1, static_cast<int>(timeout.count()));
  if (ready < 0) {
    throwErrno("poll() failed on " + interface_name_);
  }
  if (ready == 0 || (poll_fd.revents & POLLIN) == 0) {
    return false;
  }

  struct can_frame raw {};
  auto received = ::read(fd_, &raw, sizeof(raw));
  if (received != static_cast<decltype(received)>(sizeof(raw))) {
    throwErrno("failed to read a CAN frame on " + interface_name_);
  }

  frame.error = (raw.can_id & CAN_ERR_FLAG) != 0U;
  frame.remote = !frame.error && (raw.can_id & CAN_RTR_FLAG) != 0U;
  frame.id = frame.error ? (raw.can_id & CAN_ERR_MASK) : (raw.can_id & CAN_EFF_MASK);
  frame.dlc = raw.can_dlc;
  std::memcpy(frame.data.data(), raw.data, 8);
  return true;
}

const std::string & CanSocket::interfaceName() const noexcept
{
  return interface_name_;
}

}  // namespace gim6010_driver
