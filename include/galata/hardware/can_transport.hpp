// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_HARDWARE_CAN_TRANSPORT_HPP
#define GALATA_HARDWARE_CAN_TRANSPORT_HPP

#include "galata/hardware/interface.hpp"

#include <cstdint>
#include <string>

namespace galata::hardware {

// A Linux SocketCAN CAN-FD endpoint. The endpoint uses one extended CAN ID for
// sensor frames and one for actuator frames. CAN-FD is a transport adapter,
// not a flight-controller protocol: the target still owns bitrate, bus
// termination, watchdog and loss-of-link behaviour.
struct CanFdEndpoint {
  std::string interface_name;
  std::uint32_t receive_can_id = 0x120;
  std::uint32_t transmit_can_id = 0x121;
  std::uint32_t receive_timeout_ms = 100;
  std::uint32_t transmit_timeout_ms = 100;
};

// Linux SocketCAN CAN-FD transport for the versioned FrameCodec packet. One
// FrameCodec packet must fit in one CAN-FD frame; larger channel contracts are
// refused rather than fragmented without a reviewed protocol. Classic CAN is
// deliberately unsupported because its eight-byte payload cannot carry even a
// one-channel FrameCodec packet.
//
// On non-Linux platforms the type remains available for portable client code,
// but connect() throws a platform-support error. Use ReplayTransport or the
// serial/UDP adapters for local development on those platforms.
class CanFdTransport final : public Transport {
 public:
  explicit CanFdTransport(CanFdEndpoint endpoint);
  ~CanFdTransport() override;

  CanFdTransport(const CanFdTransport&) = delete;
  CanFdTransport& operator=(const CanFdTransport&) = delete;

  [[nodiscard]] LinkState state() const noexcept override {
    return state_;
  }

  void connect(const InterfaceSpec& specification) override;
  void disconnect() noexcept override;
  [[nodiscard]] bool receive(Frame& frame) override;
  void send(const Frame& frame) override;

 private:
  CanFdEndpoint endpoint_;
  InterfaceSpec specification_;
  int socket_ = -1;
#ifdef __linux__
  std::uint64_t last_received_sequence_ = 0;
  std::uint64_t last_sent_sequence_ = 0;
  double last_received_timestamp_s_ = 0.0;
  double last_sent_timestamp_s_ = 0.0;
  bool has_received_ = false;
  bool has_sent_ = false;
#endif
  LinkState state_ = LinkState::Disconnected;
};

}  // namespace galata::hardware

#endif  // GALATA_HARDWARE_CAN_TRANSPORT_HPP
