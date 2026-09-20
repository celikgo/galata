// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_ONBOARD_TARGET_EVIDENCE_HPP
#define GALATA_ONBOARD_TARGET_EVIDENCE_HPP

#include "galata/onboard/deployment.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace galata::onboard {

struct TargetEvidenceFile {
  std::string role;
  std::string relative_path;
  std::string sha256;

  bool operator==(const TargetEvidenceFile&) const = default;
};

struct TargetEvidenceSource {
  std::string role;
  std::filesystem::path source;
};

struct TargetEvidenceSpec {
  // Provenance class for the measurements. host_sil is a software-only
  // contract and therefore records physical-test and target-signing states as
  // not applicable; target_hil is evidence from the intended flight computer
  // in a controlled HIL setup, and flight_target is evidence from the target
  // in its intended installation. The class is traceability metadata, not an
  // approval decision.
  std::string evidence_class;
  double controller_worst_case_s = 0.0;
  double cycle_worst_case_s = 0.0;
  double watchdog_response_s = 0.0;
  bool emergency_stop_passed = false;
  bool loss_of_link_passed = false;
  bool hil_passed = false;
  bool signing_verified = false;
  std::vector<TargetEvidenceSource> files;
};

// Evidence produced by a target-integration programme.  The pass fields are
// assertions made by that programme and are not manufactured by Galata; the
// verifier binds their bytes and measured values to one deployment manifest.
struct TargetEvidencePackage {
  std::string manifest;
  std::string manifest_sha256;
  std::string evidence_class;
  hardware::TargetIdentity target_identity;
  std::string deployment_manifest_sha256;
  std::string interface_id;
  std::string transport_profile_id;
  double controller_worst_case_s = 0.0;
  double cycle_worst_case_s = 0.0;
  double watchdog_response_s = 0.0;
  std::vector<TargetEvidenceFile> files;
  std::string target_acceptance_state = "passed";
  std::string qualification_state = "not_qualified";
};

// Parse and self-check the deterministic target-evidence manifest.  This does
// not read evidence files or decide whether a test was representative.
[[nodiscard]] TargetEvidencePackage parse_target_evidence_package(const std::string& manifest);

// Assemble a new atomic target-evidence package from externally produced
// records. target_hil and flight_target require every pass flag from the
// responsible integration programme. host_sil rejects physical-test and
// target-signing claims; this function only copies, hashes and cross-checks
// the supplied records against the deployment contract.
[[nodiscard]] TargetEvidencePackage stage_target_evidence_package(
    const DeploymentPackage& deployment,
    const TargetEvidenceSpec& specification,
    const std::filesystem::path& destination);

// Bind the evidence claims to the exact deployment contract and reject timing
// or safety records that cannot satisfy that contract.
void verify_target_evidence_package(const TargetEvidencePackage& evidence,
                                    const DeploymentPackage& deployment);

// Re-hash every evidence file relative to package_root.  Paths must remain
// inside the package and must be regular non-symlink files.  Returns the total
// verified byte count.
[[nodiscard]] std::uintmax_t verify_target_evidence_files(
    const TargetEvidencePackage& evidence,
    const std::filesystem::path& package_root);

}  // namespace galata::onboard

#endif  // GALATA_ONBOARD_TARGET_EVIDENCE_HPP
