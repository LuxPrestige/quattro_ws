#include "gim6010_driver/can_socket.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
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
std::runtime_error systemError(const std::string & operation)
{
  return std::runtime_error(operation + ": " + std::strerror(errno));
}
}  // namespace

CanSocket::CanSocket(const std::string & interface_name) {open(interface_name);}
CanSocket::~CanSocket() {close();}

CanSocket::CanSocket(CanSocket && other) noexcept
: fd_(std::exchange(other.fd_, -1)), interface_name_(std::move(other.interface_name_)) {}

CanSocket & CanSocket::operator=(CanSocket && other) noexcept
{
  if (this != &other) {
    close();
    fd_ = std::exchange(other.fd_, -1);
    interface_name_ = std::move(other.interface_name_);
  }
  return *this;
}

void CanSocket::open(const std::string & interface_name)
{
  if (interface_name.empty() || interface_name.size() >= IFNAMSIZ) {
    throw std::invalid_argument("invalid CAN interface name");
  }
  close();
  const int socket_fd = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
  if (socket_fd < 0) {throw systemError("create CAN socket");}

  struct ifreq request {};
  std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_fd, SIOCGIFINDEX, &request) < 0) {
    const auto error = systemError("resolve CAN interface " + interface_name);
    ::close(socket_fd);
    throw error;
  }
  struct sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (::bind(socket_fd, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
    const auto error = systemError("bind CAN interface " + interface_name);
    ::close(socket_fd);
    throw error;
  }
  const can_err_mask_t error_mask = CAN_ERR_MASK;
  if (::setsockopt(
      socket_fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &error_mask, sizeof(error_mask)) < 0)
  {
    const auto error = systemError("enable CAN error frames on " + interface_name);
    ::close(socket_fd);
    throw error;
  }
  const int receive_overflow = 1;
  if (::setsockopt(
      socket_fd, SOL_SOCKET, SO_RXQ_OVFL, &receive_overflow, sizeof(receive_overflow)) < 0)
  {
    const auto error = systemError("enable CAN receive overflow reporting on " + interface_name);
    ::close(socket_fd);
    throw error;
  }
  fd_ = socket_fd;
  interface_name_ = interface_name;
}

void CanSocket::close() noexcept
{
  if (fd_ >= 0) {::close(fd_);}
  fd_ = -1;
  interface_name_.clear();
}

bool CanSocket::isOpen() const noexcept {return fd_ >= 0;}
const std::string & CanSocket::interfaceName() const noexcept {return interface_name_;}

void CanSocket::send(const CanFrame & input) const
{
  if (!isOpen()) {throw std::logic_error("CAN socket is not open");}
  if (input.id > CAN_SFF_MASK || input.dlc > CAN_MAX_DLEN) {
    throw std::invalid_argument("invalid standard CAN frame");
  }
  struct can_frame frame {};
  frame.can_id = input.id | (input.remote ? CAN_RTR_FLAG : 0U);
  frame.can_dlc = input.dlc;
  std::copy_n(input.data.begin(), input.dlc, frame.data);
  ssize_t written;
  do {
    written = ::write(fd_, &frame, sizeof(frame));
  } while (written < 0 && errno == EINTR);
  if (written != static_cast<ssize_t>(sizeof(frame))) {throw systemError("write CAN frame");}
}

bool CanSocket::receive(CanFrame & output, std::chrono::milliseconds timeout) const
{
  if (!isOpen()) {throw std::logic_error("CAN socket is not open");}
  struct pollfd descriptor {fd_, POLLIN, 0};
  int ready;
  do {
    ready = ::poll(&descriptor, 1, static_cast<int>(timeout.count()));
  } while (ready < 0 && errno == EINTR);
  if (ready == 0) {return false;}
  if (ready < 0) {
    throw systemError("poll CAN socket");
  }
  if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    throw std::runtime_error("CAN socket poll reported an interface failure");
  }
  struct can_frame frame {};
  struct iovec iov {&frame, sizeof(frame)};
  std::array<std::uint8_t, CMSG_SPACE(sizeof(std::uint32_t))> control{};
  struct msghdr message {};
  message.msg_iov = &iov;
  message.msg_iovlen = 1;
  message.msg_control = control.data();
  message.msg_controllen = control.size();
  ssize_t received;
  do {
    received = ::recvmsg(fd_, &message, 0);
  } while (received < 0 && errno == EINTR);
  if (received != static_cast<ssize_t>(sizeof(frame))) {throw systemError("read CAN frame");}
  output = CanFrame{};
  for (auto * header = CMSG_FIRSTHDR(&message); header != nullptr;
    header = CMSG_NXTHDR(&message, header))
  {
    if (header->cmsg_level == SOL_SOCKET && header->cmsg_type == SO_RXQ_OVFL &&
      header->cmsg_len >= CMSG_LEN(sizeof(std::uint32_t)))
    {
      std::memcpy(&output.rx_overflow_count, CMSG_DATA(header), sizeof(std::uint32_t));
    }
  }
  if ((frame.can_id & CAN_ERR_FLAG) != 0U) {
    output.error = true;
    output.error_class = frame.can_id & CAN_ERR_MASK;
    output.dlc = frame.can_dlc;
    std::copy_n(frame.data, frame.can_dlc, output.data.begin());
    return true;
  }
  if ((frame.can_id & CAN_EFF_FLAG) != 0U) {
    throw std::runtime_error("unexpected extended CAN frame on GDS68 bus");
  }
  output.id = frame.can_id & CAN_SFF_MASK;
  output.remote = (frame.can_id & CAN_RTR_FLAG) != 0U;
  output.dlc = frame.can_dlc;
  std::copy_n(frame.data, frame.can_dlc, output.data.begin());
  return true;
}

void CanSocket::setStandardFilters(const std::vector<std::uint32_t> & ids)
{
  if (!isOpen()) {throw std::logic_error("CAN socket is not open");}
  std::vector<struct can_filter> filters;
  filters.reserve(ids.size());
  for (const auto id : ids) {
    if (id > CAN_SFF_MASK) {throw std::invalid_argument("invalid standard CAN filter ID");}
    filters.push_back(can_filter{id, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG});
  }
  const void * data = filters.empty() ? nullptr : filters.data();
  const auto size = static_cast<socklen_t>(filters.size() * sizeof(struct can_filter));
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, data, size) < 0) {
    throw systemError("set CAN receive filters");
  }
}
}  // namespace gim6010_driver
