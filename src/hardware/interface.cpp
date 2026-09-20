// SPDX-License-Identifier: Apache-2.0
#include "galata/hardware/interface.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace galata::hardware {
namespace {

constexpr std::array<std::uint8_t, 4> kPacketMagic{'G', 'L', 'H', 'W'};
constexpr std::size_t kPacketHeaderSize = 24;
constexpr std::size_t kPacketCrcSize = 4;
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
              "hardware frame codec requires IEEE-754 binary64 doubles");

void append_u16(std::vector<std::uint8_t>& packet, std::uint16_t value) {
  packet.push_back(static_cast<std::uint8_t>(value & 0xffU));
  packet.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xffU));
}

void append_u64(std::vector<std::uint8_t>& packet, std::uint64_t value) {
  for (unsigned int byte = 0; byte < 8; ++byte) {
    packet.push_back(static_cast<std::uint8_t>((value >> (8U * byte)) & 0xffU));
  }
}

std::uint16_t read_u16(const std::vector<std::uint8_t>& packet, std::size_t offset) {
  const std::uint32_t value = static_cast<std::uint32_t>(packet[offset])
                              | (static_cast<std::uint32_t>(packet[offset + 1]) << 8U);
  return static_cast<std::uint16_t>(value);
}

std::uint64_t read_u64(const std::vector<std::uint8_t>& packet, std::size_t offset) {
  std::uint64_t value = 0;
  for (unsigned int byte = 0; byte < 8; ++byte) {
    value |= static_cast<std::uint64_t>(packet[offset + byte]) << (8U * byte);
  }
  return value;
}

std::uint32_t crc32(const std::vector<std::uint8_t>& packet, std::size_t length) {
  std::uint32_t result = 0xffffffffU;
  for (std::size_t index = 0; index < length; ++index) {
    result ^= packet[index];
    for (int bit = 0; bit < 8; ++bit) {
      result = (result & 1U) != 0U ? (result >> 1U) ^ 0xedb88320U : result >> 1U;
    }
  }
  return result ^ 0xffffffffU;
}

std::uint32_t packet_checksum(const std::vector<std::uint8_t>& packet) {
  const std::size_t offset = packet.size() - kPacketCrcSize;
  return static_cast<std::uint32_t>(packet[offset])
         | static_cast<std::uint32_t>(packet[offset + 1]) << 8U
         | static_cast<std::uint32_t>(packet[offset + 2]) << 16U
         | static_cast<std::uint32_t>(packet[offset + 3]) << 24U;
}

void validate_channels(const std::vector<ChannelSpec>& channels, const char* direction) {
  if (channels.empty()) {
    throw std::invalid_argument(std::string("hardware: ") + direction
                                + " channel list must not be empty");
  }
  std::unordered_set<std::string> names;
  for (const ChannelSpec& channel : channels) {
    if (channel.name.empty() || channel.unit.empty() || channel.frame.empty()) {
      throw std::invalid_argument(std::string("hardware: every ") + direction
                                  + " channel needs a name, SI unit and frame");
    }
    if (!names.insert(channel.name).second) {
      throw std::invalid_argument(std::string("hardware: duplicate ") + direction + " channel '"
                                  + channel.name + "'");
    }
  }
}

void validate_sequence(std::uint64_t sequence,
                       std::uint64_t previous,
                       bool has_previous,
                       const char* direction) {
  if (has_previous && sequence <= previous) {
    throw std::invalid_argument(std::string("hardware: ") + direction
                                + " frame sequence must increase strictly");
  }
}

void validate_timing(double timestamp,
                     double previous_timestamp,
                     bool has_previous,
                     double sample_period_s,
                     const char* direction) {
  if (!has_previous) {
    return;
  }
  const double interval = timestamp - previous_timestamp;
  const double tolerance = 1.0e-9 * std::max(1.0, sample_period_s);
  if (std::fabs(interval - sample_period_s) > tolerance) {
    throw std::invalid_argument(std::string("hardware: ") + direction
                                + " frame interval does not match sample_period_s");
  }
}

}  // namespace

std::vector<std::uint8_t> FrameCodec::encode(const Frame& frame, std::size_t expected_value_count) {
  if (expected_value_count > std::numeric_limits<std::uint16_t>::max()) {
    throw std::invalid_argument("hardware: frame codec channel count exceeds uint16 wire limit");
  }
  if (frame.values.size() != expected_value_count) {
    throw std::invalid_argument("hardware: frame codec width does not match its contract");
  }
  if (!std::isfinite(frame.timestamp_s) || frame.timestamp_s < 0.0) {
    throw std::invalid_argument("hardware: frame codec timestamp must be finite and non-negative");
  }
  std::vector<std::uint8_t> packet;
  packet.reserve(kPacketHeaderSize + expected_value_count * sizeof(double) + kPacketCrcSize);
  packet.insert(packet.end(), kPacketMagic.begin(), kPacketMagic.end());
  packet.push_back(kVersion);
  packet.push_back(0);  // Reserved flags; non-zero values are refused by decode.
  append_u16(packet, static_cast<std::uint16_t>(expected_value_count));
  append_u64(packet, frame.sequence);
  std::uint64_t timestamp_bits = 0;
  static_assert(sizeof(timestamp_bits) == sizeof(frame.timestamp_s));
  std::memcpy(&timestamp_bits, &frame.timestamp_s, sizeof(timestamp_bits));
  append_u64(packet, timestamp_bits);
  for (const double value : frame.values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("hardware: frame codec refuses a non-finite value");
    }
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    append_u64(packet, bits);
  }
  const std::uint32_t checksum = crc32(packet, packet.size());
  packet.push_back(static_cast<std::uint8_t>(checksum & 0xffU));
  packet.push_back(static_cast<std::uint8_t>((checksum >> 8U) & 0xffU));
  packet.push_back(static_cast<std::uint8_t>((checksum >> 16U) & 0xffU));
  packet.push_back(static_cast<std::uint8_t>((checksum >> 24U) & 0xffU));
  return packet;
}

Frame FrameCodec::decode(const std::vector<std::uint8_t>& packet,
                         std::size_t expected_value_count) {
  if (expected_value_count > std::numeric_limits<std::uint16_t>::max()) {
    throw std::invalid_argument("hardware: frame codec channel count exceeds uint16 wire limit");
  }
  const std::size_t expected_size =
      kPacketHeaderSize + expected_value_count * sizeof(double) + kPacketCrcSize;
  if (packet.size() != expected_size) {
    throw std::invalid_argument("hardware: frame codec packet has an unexpected size");
  }
  if (!std::equal(kPacketMagic.begin(), kPacketMagic.end(), packet.begin())) {
    throw std::invalid_argument("hardware: frame codec packet magic is invalid");
  }
  if (packet[4] != kVersion || packet[5] != 0) {
    throw std::invalid_argument("hardware: frame codec packet version or flags are unsupported");
  }
  if (read_u16(packet, 6) != expected_value_count) {
    throw std::invalid_argument("hardware: frame codec channel count differs from its contract");
  }
  if (packet_checksum(packet) != crc32(packet, packet.size() - kPacketCrcSize)) {
    throw std::invalid_argument("hardware: frame codec packet checksum mismatch");
  }

  Frame frame;
  frame.sequence = read_u64(packet, 8);
  const std::uint64_t timestamp_bits = read_u64(packet, 16);
  std::memcpy(&frame.timestamp_s, &timestamp_bits, sizeof(frame.timestamp_s));
  frame.values.resize(expected_value_count);
  for (std::size_t index = 0; index < expected_value_count; ++index) {
    const std::uint64_t bits = read_u64(packet, kPacketHeaderSize + index * sizeof(double));
    std::memcpy(&frame.values[index], &bits, sizeof(frame.values[index]));
  }
  if (!std::isfinite(frame.timestamp_s) || frame.timestamp_s < 0.0) {
    throw std::invalid_argument("hardware: frame codec decoded an invalid timestamp");
  }
  for (const double value : frame.values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("hardware: frame codec decoded a non-finite value");
    }
  }
  return frame;
}

std::string to_string(LinkState state) {
  switch (state) {
    case LinkState::Disconnected:
      return "disconnected";
    case LinkState::Ready:
      return "ready";
    case LinkState::Faulted:
      return "faulted";
  }
  return "unknown";
}

void validate_interface(const InterfaceSpec& specification) {
  if (specification.id.empty()) {
    throw std::invalid_argument("hardware: interface id is required");
  }
  if (!(specification.sample_period_s > 0.0) || !std::isfinite(specification.sample_period_s)) {
    throw std::invalid_argument("hardware: sample_period_s must be positive and finite");
  }
  validate_channels(specification.sensor_channels, "sensor");
  validate_channels(specification.actuator_channels, "actuator");
  std::unordered_set<std::string> all_names;
  for (const ChannelSpec& channel : specification.sensor_channels) {
    all_names.insert(channel.name);
  }
  for (const ChannelSpec& channel : specification.actuator_channels) {
    if (all_names.contains(channel.name)) {
      throw std::invalid_argument(
          "hardware: a sensor and actuator channel cannot share the same name");
    }
  }
}

void validate_frame(const Frame& frame,
                    const std::vector<ChannelSpec>& channels,
                    const char* direction) {
  if (!std::isfinite(frame.timestamp_s) || frame.timestamp_s < 0.0) {
    throw std::invalid_argument(std::string("hardware: ") + direction
                                + " frame timestamp must be finite and non-negative");
  }
  if (frame.values.size() != channels.size()) {
    throw std::invalid_argument(std::string("hardware: ") + direction
                                + " frame width does not match its channel contract");
  }
  for (double value : frame.values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(std::string("hardware: ") + direction
                                  + " frame contains a non-finite value");
    }
  }
}

ReplayTransport::ReplayTransport(std::vector<Frame> sensor_frames)
    : sensor_frames_(std::move(sensor_frames)) {}

void ReplayTransport::connect(const InterfaceSpec& specification) {
  validate_interface(specification);
  std::uint64_t previous_sequence = 0;
  double previous_timestamp = 0.0;
  bool has_previous = false;
  for (const Frame& frame : sensor_frames_) {
    validate_frame(frame, specification.sensor_channels, "sensor");
    validate_sequence(frame.sequence, previous_sequence, has_previous, "sensor");
    validate_timing(frame.timestamp_s,
                    previous_timestamp,
                    has_previous,
                    specification.sample_period_s,
                    "sensor");
    previous_sequence = frame.sequence;
    previous_timestamp = frame.timestamp_s;
    has_previous = true;
  }
  specification_ = specification;
  next_sensor_frame_ = 0;
  last_received_sequence_ = 0;
  last_sent_sequence_ = 0;
  sent_frames_.clear();
  state_ = LinkState::Ready;
}

void ReplayTransport::disconnect() noexcept {
  state_ = LinkState::Disconnected;
}

bool ReplayTransport::receive(Frame& frame) {
  if (state_ != LinkState::Ready) {
    throw std::logic_error("hardware: receive requires a ready link");
  }
  if (next_sensor_frame_ == sensor_frames_.size()) {
    return false;
  }
  frame = sensor_frames_[next_sensor_frame_++];
  validate_frame(frame, specification_.sensor_channels, "sensor");
  validate_sequence(frame.sequence, last_received_sequence_, next_sensor_frame_ > 1, "sensor");
  const double previous_timestamp =
      next_sensor_frame_ > 1 ? sensor_frames_[next_sensor_frame_ - 2].timestamp_s : 0.0;
  validate_timing(frame.timestamp_s,
                  previous_timestamp,
                  next_sensor_frame_ > 1,
                  specification_.sample_period_s,
                  "sensor");
  last_received_sequence_ = frame.sequence;
  return true;
}

void ReplayTransport::send(const Frame& frame) {
  if (state_ != LinkState::Ready) {
    throw std::logic_error("hardware: send requires a ready link");
  }
  validate_frame(frame, specification_.actuator_channels, "actuator");
  validate_sequence(frame.sequence, last_sent_sequence_, !sent_frames_.empty(), "actuator");
  const double previous_timestamp = sent_frames_.empty() ? 0.0 : sent_frames_.back().timestamp_s;
  validate_timing(frame.timestamp_s,
                  previous_timestamp,
                  !sent_frames_.empty(),
                  specification_.sample_period_s,
                  "actuator");
  last_sent_sequence_ = frame.sequence;
  sent_frames_.push_back(frame);
}

const InterfaceSpec& ReplayTransport::specification() const {
  if (state_ == LinkState::Disconnected && specification_.id.empty()) {
    throw std::logic_error("hardware: no interface has been connected");
  }
  return specification_;
}

void ArmingInterlock::arm(const std::string& operator_id, const std::string& confirmation) {
  if (operator_id.empty()) {
    throw std::invalid_argument("hardware: an operator identity is required to arm");
  }
  if (confirmation != "ARM") {
    throw std::invalid_argument(
        "hardware: arming requires the exact explicit confirmation token 'ARM'");
  }
  operator_id_ = operator_id;
  armed_ = true;
}

void ArmingInterlock::disarm() noexcept {
  armed_ = false;
  operator_id_.clear();
}

void ArmingInterlock::require_armed() const {
  if (!armed_) {
    throw std::logic_error(
        "hardware: actuator output is blocked until an operator explicitly arms the interlock");
  }
}

void GuardedTransport::connect(const InterfaceSpec& specification) {
  interlock_.disarm();
  try {
    transport_.connect(specification);
    if (transport_.state() != LinkState::Ready) {
      interlock_.disarm();
    }
  } catch (...) {
    interlock_.disarm();
    throw;
  }
}

void GuardedTransport::disconnect() noexcept {
  interlock_.disarm();
  transport_.disconnect();
}

bool GuardedTransport::receive(Frame& frame) {
  try {
    const bool received = transport_.receive(frame);
    if (transport_.state() != LinkState::Ready) {
      interlock_.disarm();
    }
    return received;
  } catch (...) {
    interlock_.disarm();
    throw;
  }
}

void GuardedTransport::send(const Frame& frame) {
  interlock_.require_armed();
  try {
    transport_.send(frame);
    if (transport_.state() != LinkState::Ready) {
      interlock_.disarm();
    }
  } catch (...) {
    interlock_.disarm();
    throw;
  }
}

}  // namespace galata::hardware
