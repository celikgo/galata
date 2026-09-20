// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_HARDWARE_UDP_TRANSPORT_HPP
#define GALATA_HARDWARE_UDP_TRANSPORT_HPP

#include "galata/hardware/interface.hpp"

#include <cstdint>
#include <string>

namespace galata::hardware {

// A connected UDP peer for a bench, simulator or flight-controller adapter.
// UDP carries the versioned FrameCodec packet; it does not provide encryption,
// authentication, delivery guarantees or a safety interlock.  A deployment
// must therefore put this transport behind GuardedTransport and provide the
// target's own link-loss/failsafe policy.
struct UdpEndpoint {
  std::string host;
  std::uint16_t port = 0;
  std::uint32_t receive_timeout_ms = 100;
  // Zero asks the OS for an ephemeral source port. A deployed flight link
  // should normally declare a fixed local port so the peer can send its first
  // sensor frame before receiving an actuator frame.
  std::uint16_t local_port = 0;
  std::uint32_t transmit_timeout_ms = 100;
};

// POSIX IPv4/IPv6 UDP transport. `receive` returns false only when its bounded
// receive timeout expires; `send` waits only up to the independent transmit
// timeout and uses non-blocking socket I/O. Malformed packets, out-of-order
// frames and socket failures fault the link and throw; the caller must
// disconnect and perform its external failsafe procedure before reconnecting.
class UdpTransport final : public Transport {
 public:
  explicit UdpTransport(UdpEndpoint endpoint);
  ~UdpTransport() override;

  UdpTransport(const UdpTransport&) = delete;
  UdpTransport& operator=(const UdpTransport&) = delete;

  [[nodiscard]] LinkState state() const noexcept override {
    return state_;
  }

  void connect(const InterfaceSpec& specification) override;
  void disconnect() noexcept override;
  [[nodiscard]] bool receive(Frame& frame) override;
  void send(const Frame& frame) override;

 private:
  UdpEndpoint endpoint_;
  InterfaceSpec specification_;
  int socket_ = -1;
  std::uint64_t last_received_sequence_ = 0;
  std::uint64_t last_sent_sequence_ = 0;
  double last_received_timestamp_s_ = 0.0;
  double last_sent_timestamp_s_ = 0.0;
  bool has_received_ = false;
  bool has_sent_ = false;
  LinkState state_ = LinkState::Disconnected;
};

}  // namespace galata::hardware

#endif  // GALATA_HARDWARE_UDP_TRANSPORT_HPP
