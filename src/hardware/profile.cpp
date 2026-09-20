// SPDX-License-Identifier: Apache-2.0
#include "galata/hardware/profile.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace galata::hardware {

void require_fixed_udp_local_port(const std::string& endpoint) {
  const std::size_t separator = endpoint.rfind('|');
  if (separator == std::string::npos || separator + 1U >= endpoint.size()) {
    throw std::invalid_argument(
        "hardware: UDP deployment endpoint must declare a fixed local port as host:port|local_port");
  }
  std::uint32_t port = 0;
  for (std::size_t index = separator + 1U; index < endpoint.size(); ++index) {
    const unsigned char character = static_cast<unsigned char>(endpoint[index]);
    if (character < static_cast<unsigned char>('0')
        || character > static_cast<unsigned char>('9')) {
      throw std::invalid_argument("hardware: UDP deployment local port must be numeric");
    }
    const std::uint32_t digit =
        static_cast<std::uint32_t>(character - static_cast<unsigned char>('0'));
    if (port > (std::numeric_limits<std::uint16_t>::max() - digit) / 10U) {
      throw std::invalid_argument(
          "hardware: UDP deployment local port is outside the 16-bit range");
    }
    port = port * 10U + digit;
  }
  if (port == 0U) {
    throw std::invalid_argument("hardware: UDP deployment local port must be non-zero");
  }
}

void validate_target_identity(const TargetIdentity& identity) {
  for (const auto& [value, field] : std::initializer_list<std::pair<const std::string*, const char*>>{
           {&identity.hardware_id, "hardware_id"},
           {&identity.flight_computer_id, "flight_computer_id"},
           {&identity.firmware_id, "firmware_id"},
           {&identity.emergency_stop_id, "emergency_stop_id"}}) {
    if (value->empty()) {
      throw std::invalid_argument("hardware: target " + std::string(field) + " is required");
    }
    for (const char character : *value) {
      const unsigned char byte = static_cast<unsigned char>(character);
      if (byte < 0x20U || byte == 0x7fU || character == '=') {
        throw std::invalid_argument(
            "hardware: target identity contains a control character or '='");
      }
    }
  }
}

void validate_transport_profile(const TransportProfile& profile, double sample_period_s) {
  if (profile.id.empty() || profile.endpoint.empty()) {
    throw std::invalid_argument("hardware: transport profile needs an id and endpoint identity");
  }
  for (const std::string* value : {&profile.id, &profile.transport, &profile.endpoint}) {
    for (const char character : *value) {
      const unsigned char byte = static_cast<unsigned char>(character);
      if (byte < 0x20U || byte == 0x7fU || character == '=') {
        throw std::invalid_argument(
            "hardware: transport profile text contains a control character or '='");
      }
    }
  }
  if (profile.transport != "replay" && profile.transport != "udp" && profile.transport != "serial"
      && profile.transport != "can_fd") {
    throw std::invalid_argument(
        "hardware: transport profile must use replay, udp, serial or can_fd");
  }
  if (profile.transport == "udp") {
    require_fixed_udp_local_port(profile.endpoint);
  }
  if (profile.receive_timeout_ms == 0 || profile.transmit_timeout_ms == 0) {
    throw std::invalid_argument("hardware: transport profile I/O timeouts must be non-zero");
  }
  if (!(sample_period_s > 0.0) || !std::isfinite(sample_period_s)) {
    throw std::invalid_argument("hardware: transport profile sample period must be positive");
  }
  if (!(profile.watchdog_timeout_s >= sample_period_s)
      || !std::isfinite(profile.watchdog_timeout_s)) {
    throw std::invalid_argument(
        "hardware: transport profile watchdog must be finite and at least one sample period");
  }
  if (!profile.emergency_stop_required) {
    throw std::invalid_argument(
        "hardware: a deployment profile must require an independent emergency stop");
  }
}

}  // namespace galata::hardware
