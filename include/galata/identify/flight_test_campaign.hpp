// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "galata/identify/validate.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace galata::identify {

struct FlightTestCampaignFile {
  std::string role;
  std::string relative_path;
  std::string sha256;
};

// A deterministic, reviewable manifest for the controlled records required by
// the flight-test evidence gate. It is deliberately separate from a numerical
// validation result: the package verifier can check bytes and traceability,
// but it cannot decide whether a flight was safe, representative or accepted.
// The manifest's evidence_class is explicit: measured_flight is the only class
// eligible for the flight-test evidence gate; public_deidentified and
// synthetic_contract remain engineering references.
struct FlightTestCampaign {
  std::string manifest;
  std::string manifest_sha256;
  FlightTestEvidence evidence;
  std::vector<FlightTestCampaignFile> files;
};

[[nodiscard]] FlightTestCampaign parse_flight_test_campaign(const std::string& manifest);

// Verify every declared campaign file against the manifest. Relative paths
// must remain inside package_root, and neither the files nor their parent path
// components may be symlinks. The verifier checks completeness and byte
// identity only; it does not create flight-test, airworthiness or certification
// evidence.
[[nodiscard]] std::uintmax_t verify_flight_test_campaign(const FlightTestCampaign& campaign,
                                                         const std::filesystem::path& package_root);

}  // namespace galata::identify
