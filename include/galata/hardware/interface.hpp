// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_HARDWARE_INTERFACE_HPP
#define GALATA_HARDWARE_INTERFACE_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace galata::hardware {

// A channel is part of the wire contract, not presentation metadata.  A
// target adapter must convert its native units and frame into this vocabulary
// before a control loop is allowed to consume it.
struct ChannelSpec {
  std::string name;
  std::string unit;
  std::string frame;
};

struct InterfaceSpec {
  std::string id;
  double sample_period_s = 0.0;
  std::vector<ChannelSpec> sensor_channels;
  std::vector<ChannelSpec> actuator_channels;
  // The transport never arms itself.  This flag records that an integration
  // requires an explicit operator and external-interlock step before output.
  bool external_arming_required = true;
};

// One timestamped, ordered exchange.  Values are in the order declared by the
// corresponding InterfaceSpec channel list.
struct Frame {
  std::uint64_t sequence = 0;
  double timestamp_s = 0.0;
  std::vector<double> values;
};

// Versioned, little-endian wire representation shared by target adapters.
// The codec is deliberately independent of serial, CAN and UDP APIs: those
// transports carry these bytes, while this layer owns framing, finite-value
// checks and corruption detection.  A target adapter must still provide its
// own bounded I/O, timeout, clock and failsafe policy.
class FrameCodec final {
 public:
  static constexpr std::uint8_t kVersion = 1;

  [[nodiscard]] static std::vector<std::uint8_t> encode(const Frame& frame,
                                                        std::size_t expected_value_count);
  [[nodiscard]] static Frame decode(const std::vector<std::uint8_t>& packet,
                                    std::size_t expected_value_count);
};

enum class LinkState {
  Disconnected,
  Ready,
  Faulted,
};

[[nodiscard]] std::string to_string(LinkState state);

// Throws std::invalid_argument when the contract is incomplete or ambiguous.
void validate_interface(const InterfaceSpec& specification);
void validate_frame(const Frame& frame,
                    const std::vector<ChannelSpec>& channels,
                    const char* direction);

// Hardware is deliberately an injection boundary.  The library owns no serial,
// CAN, UDP or vendor SDK implementation and therefore cannot open a real
// actuator link accidentally.  A target-specific adapter implements this
// interface after its timing, failure and safety behaviour has been reviewed.
class Transport {
 public:
  virtual ~Transport() = default;
  [[nodiscard]] virtual LinkState state() const noexcept = 0;
  virtual void connect(const InterfaceSpec& specification) = 0;
  virtual void disconnect() noexcept = 0;
  [[nodiscard]] virtual bool receive(Frame& frame) = 0;
  virtual void send(const Frame& frame) = 0;
};

// Deterministic record/replay transport for bench and software-in-the-loop
// work.  It stores outputs instead of sending them anywhere; it is not a
// hardware simulator and it cannot establish a target's timing or safety.
class ReplayTransport final : public Transport {
 public:
  explicit ReplayTransport(std::vector<Frame> sensor_frames = {});

  [[nodiscard]] LinkState state() const noexcept override {
    return state_;
  }

  void connect(const InterfaceSpec& specification) override;
  void disconnect() noexcept override;
  [[nodiscard]] bool receive(Frame& frame) override;
  void send(const Frame& frame) override;

  [[nodiscard]] const InterfaceSpec& specification() const;

  [[nodiscard]] const std::vector<Frame>& sent_frames() const noexcept {
    return sent_frames_;
  }

 private:
  std::vector<Frame> sensor_frames_;
  std::vector<Frame> sent_frames_;
  InterfaceSpec specification_;
  std::size_t next_sensor_frame_ = 0;
  std::uint64_t last_received_sequence_ = 0;
  std::uint64_t last_sent_sequence_ = 0;
  LinkState state_ = LinkState::Disconnected;
};

// The software gate is intentionally separate from Transport.  A connected
// link is not an armed link, and a replay transport is not a substitute for an
// external emergency stop or target-specific interlock.
class ArmingInterlock {
 public:
  void arm(const std::string& operator_id, const std::string& confirmation);
  void disarm() noexcept;

  [[nodiscard]] bool armed() const noexcept {
    return armed_;
  }

  [[nodiscard]] const std::string& operator_id() const noexcept {
    return operator_id_;
  }

  void require_armed() const;

 private:
  bool armed_ = false;
  std::string operator_id_;
};

// Safety decorator for any transport.  It makes the interlock part of the
// output path rather than a convention that each caller might forget: connect
// starts disarmed, every actuator frame requires an armed interlock, and
// disconnect disarms before dropping the link. Observing a non-ready state or
// an I/O exception also disarms: a caller must not be able to keep sending
// because it failed to notice that the underlying adapter lost its link.
class GuardedTransport final : public Transport {
 public:
  GuardedTransport(Transport& transport, ArmingInterlock& interlock)
      : transport_(transport), interlock_(interlock) {}

  [[nodiscard]] LinkState state() const noexcept override {
    const LinkState current = transport_.state();
    if (current != LinkState::Ready) {
      interlock_.disarm();
    }
    return current;
  }

  void connect(const InterfaceSpec& specification) override;
  void disconnect() noexcept override;

  [[nodiscard]] bool receive(Frame& frame) override;

  void send(const Frame& frame) override;

 private:
  Transport& transport_;
  ArmingInterlock& interlock_;
};

}  // namespace galata::hardware

#endif  // GALATA_HARDWARE_INTERFACE_HPP
