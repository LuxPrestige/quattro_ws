#include "gim6010_driver/can_socket.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

#include <fcntl.h>
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
  const auto written = ::write(fd_, &frame, sizeof(frame));
  if (written != static_cast<ssize_t>(sizeof(frame))) {throw systemError("write CAN frame");}
}

bool CanSocket::receive(CanFrame & output, std::chrono::milliseconds timeout) const
{
  if (!isOpen()) {throw std::logic_error("CAN socket is not open");}
  struct pollfd descriptor {fd_, POLLIN, 0};
  const int ready = ::poll(&descriptor, 1, static_cast<int>(timeout.count()));
  if (ready == 0) {return false;}
  if (ready < 0) {
    if (errno == EINTR) {return false;}
    throw systemError("poll CAN socket");
  }
  struct can_frame frame {};
  const auto received = ::read(fd_, &frame, sizeof(frame));
  if (received != static_cast<ssize_t>(sizeof(frame))) {throw systemError("read CAN frame");}
  if ((frame.can_id & (CAN_EFF_FLAG | CAN_ERR_FLAG)) != 0U) {return false;}
  output.id = frame.can_id & CAN_SFF_MASK;
  output.remote = (frame.can_id & CAN_RTR_FLAG) != 0U;
  output.dlc = frame.can_dlc;
  std::copy_n(frame.data, frame.can_dlc, output.data.begin());
  return true;
}
}  // namespace gim6010_driver
