// SPDX-License-Identifier: Apache-2.0
#include "galata/hardware/can_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#endif

namespace galata::hardware {
namespace {

constexpr std::size_t kPacketHeaderSize = 24;
constexpr std::size_t kPacketCrcSize = 4;
constexpr std::size_t kCanFdPayloadSize = 64;
constexpr std::uint32_t kCanExtendedIdMask = 0x1fffffffU;

std::size_t packet_size(std::size_t channel_count) {
  return kPacketHeaderSize + channel_count * sizeof(double) + kPacketCrcSize;
}

void validate_endpoint(const CanFdEndpoint& endpoint) {
  if (endpoint.interface_name.empty()) {
    throw std::invalid_argument("hardware: CAN-FD interface_name is required");
  }
  if (endpoint.receive_can_id == 0 || endpoint.transmit_can_id == 0
      || endpoint.receive_can_id > kCanExtendedIdMask
      || endpoint.transmit_can_id > kCanExtendedIdMask) {
    throw std::invalid_argument("hardware: CAN-FD IDs must be non-zero 29-bit identifiers");
  }
  if (endpoint.receive_can_id == endpoint.transmit_can_id) {
    throw std::invalid_argument("hardware: CAN-FD receive and transmit IDs must differ");
  }
  if (endpoint.receive_timeout_ms == 0 || endpoint.transmit_timeout_ms == 0) {
    throw std::invalid_argument("hardware: CAN-FD I/O timeouts must be non-zero");
  }
}

void validate_can_fd_width(const InterfaceSpec& specification) {
  if (packet_size(specification.sensor_channels.size()) > kCanFdPayloadSize
      || packet_size(specification.actuator_channels.size()) > kCanFdPayloadSize) {
    throw std::invalid_argument(
        "hardware: CAN-FD FrameCodec packet exceeds one 64-byte payload; use fewer channels "
        "or a reviewed fragmentation protocol");
  }
}

#ifdef __linux__

void validate_monotonic(const Frame& frame,
                        std::uint64_t previous_sequence,
                        double previous_timestamp_s,
                        bool has_previous,
                        double sample_period_s,
                        const char* direction) {
  if (!has_previous) {
    return;
  }
  if (frame.sequence <= previous_sequence) {
    throw std::invalid_argument(std::string("hardware: CAN-FD ") + direction
                                + " frame sequence must increase strictly");
  }
  const double interval = frame.timestamp_s - previous_timestamp_s;
  const double tolerance = 1.0e-9 * std::max(1.0, sample_period_s);
  if (std::abs(interval - sample_period_s) > tolerance) {
    throw std::invalid_argument(std::string("hardware: CAN-FD ") + direction
                                + " frame interval does not match sample_period_s");
  }
}

[[noreturn]] void throw_can_error(const char* operation) {
  throw std::runtime_error(std::string("hardware: CAN-FD ") + operation + ": "
                           + std::strerror(errno));
}

void close_socket(int& descriptor) noexcept {
  if (descriptor >= 0) {
    (void)::close(descriptor);
    descriptor = -1;
  }
}

int remaining_milliseconds(std::chrono::steady_clock::time_point deadline) {
  const auto now = std::chrono::steady_clock::now();
  if (now >= deadline) {
    return 0;
  }
  const auto remaining =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
  return static_cast<int>(std::min<std::int64_t>(remaining + 1, INT_MAX));
}

bool wait_for(int descriptor, short events, std::uint32_t timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (true) {
    pollfd ready{descriptor, events, 0};
    int result = 0;
    do {
      result = ::poll(&ready, 1, remaining_milliseconds(deadline));
    } while (result < 0 && errno == EINTR);
    if (result == 0) {
      return false;
    }
    if (result < 0) {
      throw_can_error("poll");
    }
    if ((ready.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      throw std::runtime_error("hardware: CAN-FD socket reported a link fault");
    }
    return (ready.revents & events) != 0;
  }
}

#endif

}  // namespace

CanFdTransport::CanFdTransport(CanFdEndpoint endpoint) : endpoint_(std::move(endpoint)) {
  validate_endpoint(endpoint_);
}

CanFdTransport::~CanFdTransport() {
  disconnect();
}

void CanFdTransport::connect(const InterfaceSpec& specification) {
  if (state_ != LinkState::Disconnected) {
    throw std::logic_error("hardware: CAN-FD connect requires a disconnected link");
  }
  validate_interface(specification);
  validate_endpoint(endpoint_);
  validate_can_fd_width(specification);

#ifdef __linux__
  const unsigned int interface_index = ::if_nametoindex(endpoint_.interface_name.c_str());
  if (interface_index == 0) {
    throw_can_error("find interface");
  }

  const int descriptor = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (descriptor < 0) {
    throw_can_error("open socket");
  }
  socket_ = descriptor;

  const int flags = ::fcntl(socket_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(socket_, F_SETFL, flags | O_NONBLOCK) != 0) {
    close_socket(socket_);
    throw_can_error("configure non-blocking I/O");
  }

  int enable_fd = 1;
  if (::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) != 0) {
    close_socket(socket_);
    throw_can_error("enable CAN-FD");
  }

  can_filter filter{};
  filter.can_id = endpoint_.receive_can_id | CAN_EFF_FLAG;
  filter.can_mask = CAN_EFF_MASK | CAN_EFF_FLAG;
  if (::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) != 0) {
    close_socket(socket_);
    throw_can_error("install receive filter");
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = static_cast<int>(interface_index);
  if (::bind(socket_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
    close_socket(socket_);
    throw_can_error("bind interface");
  }

  specification_ = specification;
  last_received_sequence_ = 0;
  last_sent_sequence_ = 0;
  last_received_timestamp_s_ = 0.0;
  last_sent_timestamp_s_ = 0.0;
  has_received_ = false;
  has_sent_ = false;
  state_ = LinkState::Ready;
#else
  state_ = LinkState::Faulted;
  throw std::runtime_error("hardware: CAN-FD SocketCAN transport is supported on Linux only");
#endif
}

void CanFdTransport::disconnect() noexcept {
#ifdef __linux__
  close_socket(socket_);
#else
  socket_ = -1;
#endif
  state_ = LinkState::Disconnected;
#ifdef __linux__
  has_received_ = false;
  has_sent_ = false;
#endif
}

bool CanFdTransport::receive(Frame& frame) {
#ifndef __linux__
  (void)frame;
  throw std::logic_error("hardware: CAN-FD receive requires Linux SocketCAN");
#else
  if (state_ != LinkState::Ready || socket_ < 0) {
    throw std::logic_error("hardware: CAN-FD receive requires a ready link");
  }
  if (!wait_for(socket_, POLLIN, endpoint_.receive_timeout_ms)) {
    return false;
  }

  canfd_frame wire{};
  const ssize_t received = ::recv(socket_, &wire, sizeof(wire), 0);
  if (received < 0) {
    state_ = LinkState::Faulted;
    throw_can_error("receive");
  }
  if (received != CANFD_MTU || wire.can_id != (endpoint_.receive_can_id | CAN_EFF_FLAG)) {
    state_ = LinkState::Faulted;
    throw std::runtime_error("hardware: CAN-FD received an unexpected frame or CAN ID");
  }
  const std::size_t expected_size = packet_size(specification_.sensor_channels.size());
  if (wire.len != expected_size) {
    state_ = LinkState::Faulted;
    throw std::invalid_argument("hardware: CAN-FD received a packet with the wrong payload size");
  }

  try {
    const std::vector<std::uint8_t> bytes(wire.data, wire.data + wire.len);
    Frame decoded = FrameCodec::decode(bytes, specification_.sensor_channels.size());
    validate_frame(decoded, specification_.sensor_channels, "sensor");
    validate_monotonic(decoded,
                       last_received_sequence_,
                       last_received_timestamp_s_,
                       has_received_,
                       specification_.sample_period_s,
                       "sensor");
    frame = std::move(decoded);
    last_received_sequence_ = frame.sequence;
    last_received_timestamp_s_ = frame.timestamp_s;
    has_received_ = true;
    return true;
  } catch (...) {
    state_ = LinkState::Faulted;
    throw;
  }
#endif
}

void CanFdTransport::send(const Frame& frame) {
#ifndef __linux__
  (void)frame;
  throw std::logic_error("hardware: CAN-FD send requires Linux SocketCAN");
#else
  if (state_ != LinkState::Ready || socket_ < 0) {
    throw std::logic_error("hardware: CAN-FD send requires a ready link");
  }
  validate_frame(frame, specification_.actuator_channels, "actuator");
  validate_monotonic(frame,
                     last_sent_sequence_,
                     last_sent_timestamp_s_,
                     has_sent_,
                     specification_.sample_period_s,
                     "actuator");
  const std::vector<std::uint8_t> bytes =
      FrameCodec::encode(frame, specification_.actuator_channels.size());

  canfd_frame wire{};
  wire.can_id = endpoint_.transmit_can_id | CAN_EFF_FLAG;
  wire.len = static_cast<__u8>(bytes.size());
  std::copy(bytes.begin(), bytes.end(), wire.data);
  try {
    if (!wait_for(socket_, POLLOUT, endpoint_.transmit_timeout_ms)) {
      throw std::runtime_error("hardware: CAN-FD send exceeded its bounded timeout");
    }
    const ssize_t sent = ::write(socket_, &wire, CANFD_MTU);
    if (sent < 0) {
      throw_can_error("send");
    }
    if (sent != CANFD_MTU) {
      throw std::runtime_error("hardware: CAN-FD send was unexpectedly partial");
    }
  } catch (...) {
    state_ = LinkState::Faulted;
    throw;
  }
  last_sent_sequence_ = frame.sequence;
  last_sent_timestamp_s_ = frame.timestamp_s;
  has_sent_ = true;
#endif
}

}  // namespace galata::hardware
