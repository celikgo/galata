// SPDX-License-Identifier: Apache-2.0
#include "galata/hardware/udp_transport.hpp"

#include <sys/socket.h>
#include <sys/types.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <unistd.h>

namespace galata::hardware {
namespace {

constexpr std::size_t kPacketHeaderSize = 24;
constexpr std::size_t kPacketCrcSize = 4;

std::size_t packet_size(std::size_t channel_count) {
  return kPacketHeaderSize + channel_count * sizeof(double) + kPacketCrcSize;
}

[[noreturn]] void throw_socket_error(const char* operation) {
  throw std::runtime_error(std::string("hardware: UDP ") + operation + ": " + std::strerror(errno));
}

void close_socket(int& descriptor) noexcept {
  if (descriptor >= 0) {
    (void)::close(descriptor);
    descriptor = -1;
  }
}

void validate_endpoint(const UdpEndpoint& endpoint) {
  if (endpoint.host.empty()) {
    throw std::invalid_argument("hardware: UDP endpoint host is required");
  }
  if (endpoint.port == 0) {
    throw std::invalid_argument("hardware: UDP endpoint port must be non-zero");
  }
  if (endpoint.receive_timeout_ms == 0) {
    throw std::invalid_argument("hardware: UDP receive_timeout_ms must be non-zero");
  }
  if (endpoint.transmit_timeout_ms == 0) {
    throw std::invalid_argument("hardware: UDP transmit_timeout_ms must be non-zero");
  }
}

int bind_local_port(int family, std::uint16_t local_port) {
  if (local_port == 0U) {
    return -1;
  }

  addrinfo hints{};
  hints.ai_family = family;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;
  hints.ai_flags = AI_PASSIVE;
  addrinfo* addresses = nullptr;
  const std::string port = std::to_string(local_port);
  const int result = ::getaddrinfo(nullptr, port.c_str(), &hints, &addresses);
  if (result != 0) {
    throw std::runtime_error(std::string("hardware: UDP local bind address: ")
                             + ::gai_strerror(result));
  }

  int descriptor = -1;
  for (addrinfo* address = addresses; address != nullptr; address = address->ai_next) {
    descriptor = ::socket(address->ai_family, address->ai_socktype, address->ai_protocol);
    if (descriptor < 0) {
      continue;
    }
    if (::bind(descriptor, address->ai_addr, address->ai_addrlen) == 0) {
      break;
    }
    close_socket(descriptor);
  }
  ::freeaddrinfo(addresses);
  if (descriptor < 0) {
    throw_socket_error("bind local port");
  }
  return descriptor;
}

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
    throw std::invalid_argument(std::string("hardware: UDP ") + direction
                                + " frame sequence must increase strictly");
  }
  const double interval = frame.timestamp_s - previous_timestamp_s;
  const double tolerance = 1.0e-9 * std::max(1.0, sample_period_s);
  if (std::abs(interval - sample_period_s) > tolerance) {
    throw std::invalid_argument(std::string("hardware: UDP ") + direction
                                + " frame interval does not match sample_period_s");
  }
}

}  // namespace

UdpTransport::UdpTransport(UdpEndpoint endpoint) : endpoint_(std::move(endpoint)) {
  validate_endpoint(endpoint_);
}

UdpTransport::~UdpTransport() {
  disconnect();
}

void UdpTransport::connect(const InterfaceSpec& specification) {
  if (state_ != LinkState::Disconnected) {
    throw std::logic_error("hardware: UDP connect requires a disconnected link");
  }
  validate_interface(specification);
  validate_endpoint(endpoint_);

  addrinfo hints{};
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_family = AF_UNSPEC;
  hints.ai_protocol = IPPROTO_UDP;
  addrinfo* addresses = nullptr;
  const std::string port = std::to_string(endpoint_.port);
  const int result = ::getaddrinfo(endpoint_.host.c_str(), port.c_str(), &hints, &addresses);
  if (result != 0) {
    throw std::runtime_error(std::string("hardware: UDP getaddrinfo: ") + ::gai_strerror(result));
  }

  int descriptor = -1;
  for (addrinfo* address = addresses; address != nullptr; address = address->ai_next) {
    descriptor = endpoint_.local_port == 0U
                     ? ::socket(address->ai_family, address->ai_socktype, address->ai_protocol)
                     : bind_local_port(address->ai_family, endpoint_.local_port);
    if (descriptor < 0) {
      continue;
    }
    if (::connect(descriptor, address->ai_addr, address->ai_addrlen) == 0) {
      const int flags = ::fcntl(descriptor, F_GETFL, 0);
      if (flags < 0 || ::fcntl(descriptor, F_SETFL, flags | O_NONBLOCK) != 0) {
        close_socket(descriptor);
        continue;
      }
      break;
    }
    close_socket(descriptor);
  }
  ::freeaddrinfo(addresses);
  if (descriptor < 0) {
    throw_socket_error("connect");
  }

  socket_ = descriptor;
  specification_ = specification;
  last_received_sequence_ = 0;
  last_sent_sequence_ = 0;
  last_received_timestamp_s_ = 0.0;
  last_sent_timestamp_s_ = 0.0;
  has_received_ = false;
  has_sent_ = false;
  state_ = LinkState::Ready;
}

void UdpTransport::disconnect() noexcept {
  close_socket(socket_);
  state_ = LinkState::Disconnected;
  has_received_ = false;
  has_sent_ = false;
}

bool UdpTransport::receive(Frame& frame) {
  if (state_ != LinkState::Ready || socket_ < 0) {
    throw std::logic_error("hardware: UDP receive requires a ready link");
  }

  pollfd descriptor{socket_, POLLIN, 0};
  int ready = 0;
  do {
    ready = ::poll(&descriptor, 1, static_cast<int>(endpoint_.receive_timeout_ms));
  } while (ready < 0 && errno == EINTR);
  if (ready == 0) {
    return false;
  }
  if (ready < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    state_ = LinkState::Faulted;
    throw_socket_error("receive readiness");
  }

  const std::size_t expected_size = packet_size(specification_.sensor_channels.size());
  std::vector<std::uint8_t> bytes(expected_size + 1U);
  const ssize_t received = ::recv(socket_, bytes.data(), bytes.size(), 0);
  if (received < 0) {
    state_ = LinkState::Faulted;
    throw_socket_error("receive");
  }
  bytes.resize(static_cast<std::size_t>(received));

  try {
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
}

void UdpTransport::send(const Frame& frame) {
  if (state_ != LinkState::Ready || socket_ < 0) {
    throw std::logic_error("hardware: UDP send requires a ready link");
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

  pollfd descriptor{socket_, POLLOUT, 0};
  int ready = 0;
  do {
    ready = ::poll(&descriptor, 1, static_cast<int>(endpoint_.transmit_timeout_ms));
  } while (ready < 0 && errno == EINTR);
  if (ready == 0) {
    state_ = LinkState::Faulted;
    throw std::runtime_error("hardware: UDP send exceeded its bounded timeout");
  }
  if (ready < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    state_ = LinkState::Faulted;
    throw_socket_error("send readiness");
  }
  const ssize_t sent = ::send(socket_, bytes.data(), bytes.size(), 0);
  if (sent < 0) {
    state_ = LinkState::Faulted;
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      throw std::runtime_error("hardware: UDP send became unavailable before its bounded timeout");
    }
    throw_socket_error("send");
  }
  if (static_cast<std::size_t>(sent) != bytes.size()) {
    state_ = LinkState::Faulted;
    throw std::runtime_error("hardware: UDP send was unexpectedly partial");
  }
  last_sent_sequence_ = frame.sequence;
  last_sent_timestamp_s_ = frame.timestamp_s;
  has_sent_ = true;
}

}  // namespace galata::hardware
