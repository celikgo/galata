// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_HARDWARE_SERIAL_TRANSPORT_HPP
#define GALATA_HARDWARE_SERIAL_TRANSPORT_HPP

#include "galata/hardware/interface.hpp"

#include <cstdint>
#include <string>

namespace galata::hardware {

// POSIX serial endpoint for a reviewed bench or flight-controller link. The
// adapter configures raw 8N1 framing; the Galata packet itself remains the
// versioned FrameCodec contract and is not a vendor bus protocol.
struct SerialEndpoint {
  std::string device;
  std::uint32_t baud_rate = 115200;
  std::uint32_t receive_timeout_ms = 100;
  std::uint32_t transmit_timeout_ms = 100;
};

// Bounded POSIX serial transport. Reads and writes are exact packet transfers
// with independent bounded receive and transmit deadlines. A short packet,
// corrupt packet, ordering/timing violation or device error faults the link;
// the caller must disconnect and execute its independent failsafe before
// reconnecting.
class SerialTransport final : public Transport {
 public:
  explicit SerialTransport(SerialEndpoint endpoint);
  ~SerialTransport() override;

  SerialTransport(const SerialTransport&) = delete;
  SerialTransport& operator=(const SerialTransport&) = delete;

  [[nodiscard]] LinkState state() const noexcept override {
    return state_;
  }

  void connect(const InterfaceSpec& specification) override;
  void disconnect() noexcept override;
  [[nodiscard]] bool receive(Frame& frame) override;
  void send(const Frame& frame) override;

 private:
  SerialEndpoint endpoint_;
  InterfaceSpec specification_;
  int descriptor_ = -1;
  std::uint64_t last_received_sequence_ = 0;
  std::uint64_t last_sent_sequence_ = 0;
  double last_received_timestamp_s_ = 0.0;
  double last_sent_timestamp_s_ = 0.0;
  bool has_received_ = false;
  bool has_sent_ = false;
  LinkState state_ = LinkState::Disconnected;
};

}  // namespace galata::hardware

#endif  // GALATA_HARDWARE_SERIAL_TRANSPORT_HPP
