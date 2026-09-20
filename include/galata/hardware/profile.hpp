// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_HARDWARE_PROFILE_HPP
#define GALATA_HARDWARE_PROFILE_HPP

#include <cstdint>
#include <string>

namespace galata::hardware {

// Identity of the concrete target integration.  These fields are declarations
// that bind a handoff to one controlled hardware/firmware/safety configuration;
// they are not discovery and cannot substitute for an independent acceptance
// record.
struct TargetIdentity {
  std::string hardware_id;
  std::string flight_computer_id;
  std::string firmware_id;
  std::string emergency_stop_id;
};

// Throws when a target identity is absent or contains a manifest delimiter.
void validate_target_identity(const TargetIdentity& identity);

// A reviewed deployment description for the link carrying FrameCodec packets.
// This is configuration and traceability, not a vendor protocol or evidence
// that the named endpoint is present. Secrets and live handles must never be
// serialized here.
struct TransportProfile {
  std::string id;
  std::string transport;
  std::string endpoint;
  std::uint32_t receive_timeout_ms = 0;
  std::uint32_t transmit_timeout_ms = 0;
  double watchdog_timeout_s = 0.0;
  bool emergency_stop_required = true;
};

// Supported names intentionally match the concrete adapters. A profile is
// valid only with explicit I/O bounds and a watchdog no shorter than one
// declared sample period.
void validate_transport_profile(const TransportProfile& profile, double sample_period_s);

}  // namespace galata::hardware

#endif  // GALATA_HARDWARE_PROFILE_HPP
