// SPDX-License-Identifier: Apache-2.0
#include "galata/hardware/serial_transport.hpp"

#include <sys/types.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace galata::hardware {
namespace {

constexpr std::size_t kPacketHeaderSize = 24;
constexpr std::size_t kPacketCrcSize = 4;

std::size_t packet_size(std::size_t channel_count) {
  return kPacketHeaderSize + channel_count * sizeof(double) + kPacketCrcSize;
}

[[noreturn]] void throw_serial_error(const char* operation) {
  throw std::runtime_error(std::string("hardware: serial ") + operation + ": "
                           + std::strerror(errno));
}

void close_descriptor(int& descriptor) noexcept {
  if (descriptor >= 0) {
    (void)::close(descriptor);
    descriptor = -1;
  }
}

void validate_endpoint(const SerialEndpoint& endpoint) {
  if (endpoint.device.empty()) {
    throw std::invalid_argument("hardware: serial device is required");
  }
  if (endpoint.baud_rate == 0) {
    throw std::invalid_argument("hardware: serial baud_rate must be non-zero");
  }
  if (endpoint.receive_timeout_ms == 0) {
    throw std::invalid_argument("hardware: serial receive_timeout_ms must be non-zero");
  }
  if (endpoint.transmit_timeout_ms == 0) {
    throw std::invalid_argument("hardware: serial transmit_timeout_ms must be non-zero");
  }
}

speed_t baud_constant(std::uint32_t baud_rate) {
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      throw std::invalid_argument("hardware: serial baud_rate is not supported by this build");
  }
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
    throw std::invalid_argument(std::string("hardware: serial ") + direction
                                + " frame sequence must increase strictly");
  }
  const double interval = frame.timestamp_s - previous_timestamp_s;
  const double tolerance = 1.0e-9 * std::max(1.0, sample_period_s);
  if (std::abs(interval - sample_period_s) > tolerance) {
    throw std::invalid_argument(std::string("hardware: serial ") + direction
                                + " frame interval does not match sample_period_s");
  }
}

int remaining_milliseconds(std::chrono::steady_clock::time_point deadline) {
  const auto now = std::chrono::steady_clock::now();
  if (now >= deadline) {
    return 0;
  }
  const auto remaining =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
  return static_cast<int>(std::min<std::int64_t>(remaining + 1, std::numeric_limits<int>::max()));
}

bool read_exact(int descriptor, std::vector<std::uint8_t>& bytes, std::uint32_t timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    pollfd input{descriptor, POLLIN, 0};
    int ready = 0;
    do {
      ready = ::poll(&input, 1, remaining_milliseconds(deadline));
    } while (ready < 0 && errno == EINTR);
    if (ready == 0) {
      return false;
    }
    if (ready < 0 || (input.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      throw_serial_error("read readiness");
    }
    const ssize_t count = ::read(descriptor, bytes.data() + offset, bytes.size() - offset);
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      throw_serial_error("read");
    }
    if (count == 0) {
      throw std::runtime_error("hardware: serial device closed while reading a frame");
    }
    offset += static_cast<std::size_t>(count);
  }
  return true;
}

void write_exact(int descriptor, const std::vector<std::uint8_t>& bytes, std::uint32_t timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    pollfd output{descriptor, POLLOUT, 0};
    int ready = 0;
    do {
      ready = ::poll(&output, 1, remaining_milliseconds(deadline));
    } while (ready < 0 && errno == EINTR);
    if (ready == 0) {
      throw std::runtime_error("hardware: serial write exceeded its bounded timeout");
    }
    if (ready < 0 || (output.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      throw_serial_error("write readiness");
    }
    const ssize_t count = ::write(descriptor, bytes.data() + offset, bytes.size() - offset);
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      throw_serial_error("write");
    }
    if (count == 0) {
      throw std::runtime_error("hardware: serial device accepted no bytes");
    }
    offset += static_cast<std::size_t>(count);
  }
}

}  // namespace

SerialTransport::SerialTransport(SerialEndpoint endpoint) : endpoint_(std::move(endpoint)) {
  validate_endpoint(endpoint_);
  (void)baud_constant(endpoint_.baud_rate);
}

SerialTransport::~SerialTransport() {
  disconnect();
}

void SerialTransport::connect(const InterfaceSpec& specification) {
  if (state_ != LinkState::Disconnected) {
    throw std::logic_error("hardware: serial connect requires a disconnected link");
  }
  validate_interface(specification);
  validate_endpoint(endpoint_);
  const speed_t baud = baud_constant(endpoint_.baud_rate);
  descriptor_ = ::open(endpoint_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (descriptor_ < 0) {
    throw_serial_error("open");
  }

  termios settings{};
  if (::tcgetattr(descriptor_, &settings) != 0) {
    close_descriptor(descriptor_);
    throw_serial_error("tcgetattr");
  }
  ::cfmakeraw(&settings);
  settings.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  settings.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
  if (::cfsetispeed(&settings, baud) != 0 || ::cfsetospeed(&settings, baud) != 0
      || ::tcsetattr(descriptor_, TCSANOW, &settings) != 0) {
    close_descriptor(descriptor_);
    throw_serial_error("configure");
  }
  if (::tcflush(descriptor_, TCIOFLUSH) != 0) {
    close_descriptor(descriptor_);
    throw_serial_error("flush");
  }

  specification_ = specification;
  last_received_sequence_ = 0;
  last_sent_sequence_ = 0;
  last_received_timestamp_s_ = 0.0;
  last_sent_timestamp_s_ = 0.0;
  has_received_ = false;
  has_sent_ = false;
  state_ = LinkState::Ready;
}

void SerialTransport::disconnect() noexcept {
  close_descriptor(descriptor_);
  state_ = LinkState::Disconnected;
  has_received_ = false;
  has_sent_ = false;
}

bool SerialTransport::receive(Frame& frame) {
  if (state_ != LinkState::Ready || descriptor_ < 0) {
    throw std::logic_error("hardware: serial receive requires a ready link");
  }
  const std::size_t expected_size = packet_size(specification_.sensor_channels.size());
  std::vector<std::uint8_t> bytes(expected_size);
  try {
    if (!read_exact(descriptor_, bytes, endpoint_.receive_timeout_ms)) {
      return false;
    }
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

void SerialTransport::send(const Frame& frame) {
  if (state_ != LinkState::Ready || descriptor_ < 0) {
    throw std::logic_error("hardware: serial send requires a ready link");
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
  try {
    write_exact(descriptor_, bytes, endpoint_.transmit_timeout_ms);
  } catch (...) {
    state_ = LinkState::Faulted;
    throw;
  }
  last_sent_sequence_ = frame.sequence;
  last_sent_timestamp_s_ = frame.timestamp_s;
  has_sent_ = true;
}

}  // namespace galata::hardware
