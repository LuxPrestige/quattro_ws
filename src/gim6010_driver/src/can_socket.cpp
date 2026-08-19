#include "gim6010_driver/can_socket.hpp"

#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "gim6010_driver/can_error.hpp"

namespace gim6010_driver
{

CanSocket::CanSocket(std::string interface_name)
: interface_name_(std::move(interface_name))
{
}

CanSocket::~CanSocket() { close(); }

CanSocket::CanSocket(CanSocket && other) noexcept
: interface_name_(std::move(other.interface_name_)),
  fd_(other.fd_),
  bus_state_(other.bus_state_),
  last_error_(other.last_error_)
{
  other.fd_ = -1;
}

CanSocket & CanSocket::operator=(CanSocket && other) noexcept
{
  if (this != &other) {
    close();
    interface_name_ = std::move(other.interface_name_);
    fd_ = other.fd_;
    bus_state_ = other.bus_state_;
    last_error_ = other.last_error_;
    other.fd_ = -1;
  }
  return *this;
}

bool CanSocket::open()
{
  close();

  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    return false;
  }

  struct ifreq interface_request{};
  std::strncpy(interface_request.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFINDEX, &interface_request) < 0) {
    ::close(fd);
    return false;
  }

  // Deliver error frames alongside data frames on the same fd so
  // receive_nonblocking() can observe bus health without a second socket.
  can_err_mask_t error_mask = CAN_ERR_MASK;
  if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &error_mask, sizeof(error_mask)) < 0) {
    ::close(fd);
    return false;
  }

  struct sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = interface_request.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
    ::close(fd);
    return false;
  }

  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    ::close(fd);
    return false;
  }

  fd_ = fd;
  bus_state_ = CanBusState::kActive;
  last_error_.reset();
  return true;
}

void CanSocket::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool CanSocket::send(const CanFrame & frame)
{
  if (fd_ < 0) {
    return false;
  }

  struct can_frame raw{};
  raw.can_id = frame.id & CAN_SFF_MASK;
  if (frame.rtr) {
    raw.can_id |= CAN_RTR_FLAG;
  }
  raw.can_dlc = frame.dlc;
  std::memcpy(raw.data, frame.data.data(), frame.data.size());

  const ssize_t written = ::write(fd_, &raw, sizeof(raw));
  return written == static_cast<ssize_t>(sizeof(raw));
}

std::optional<CanFrame> CanSocket::receive_nonblocking()
{
  if (fd_ < 0) {
    return std::nullopt;
  }

  while (true) {
    struct can_frame raw{};
    const ssize_t bytes_read = ::read(fd_, &raw, sizeof(raw));
    if (bytes_read != static_cast<ssize_t>(sizeof(raw))) {
      return std::nullopt;
    }

    if (raw.can_id & CAN_ERR_FLAG) {
      const CanBusError error = decode_error_frame(raw);
      bus_state_ = bus_state_from_error(error);
      last_error_ = error;
      continue;
    }

    CanFrame frame;
    frame.id = raw.can_id & CAN_SFF_MASK;
    frame.rtr = (raw.can_id & CAN_RTR_FLAG) != 0;
    frame.dlc = raw.can_dlc;
    std::memcpy(frame.data.data(), raw.data, frame.data.size());
    return frame;
  }
}

std::optional<CanBusError> CanSocket::poll_error_frame()
{
  std::optional<CanBusError> result = last_error_;
  last_error_.reset();
  return result;
}

}  // namespace gim6010_driver
