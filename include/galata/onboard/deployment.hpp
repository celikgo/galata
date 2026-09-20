// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_ONBOARD_DEPLOYMENT_HPP
#define GALATA_ONBOARD_DEPLOYMENT_HPP

#include "galata/hardware/interface.hpp"
#include "galata/hardware/profile.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace galata::onboard {

struct ArtifactReference {
  std::string role;
  std::string sha256;
};

// This is the boundary between an engineering study and a target-specific
// onboard programme.  It describes an immutable handoff, but deliberately
// contains no executable and makes no qualification claim.
struct DeploymentSpec {
  std::string target_platform;
  hardware::TargetIdentity target_identity;
  std::string model_description;
  std::string controller_description;
  std::string failsafe_action;
  // Maximum allowed synchronous controller callback time. The runtime detects
  // an overrun after the callback returns; a target watchdog remains required
  // to preempt a non-returning callback.
  double max_controller_time_s = 0.0;
  hardware::InterfaceSpec interface;
  hardware::TransportProfile transport_profile;
  std::vector<ArtifactReference> artifacts;
};

struct DeploymentPackage {
  std::string manifest;
  std::string manifest_sha256;
  std::string target_platform;
  hardware::TargetIdentity target_identity;
  hardware::InterfaceSpec interface;
  std::vector<ArtifactReference> artifacts;
  double max_controller_time_s = 0.0;
  hardware::TransportProfile transport_profile;
  bool contains_executable = false;
  std::string qualification_state = "not_qualified";
};

struct ArtifactFile {
  std::string role;
  std::filesystem::path source;
};

struct DeploymentReceipt {
  std::filesystem::path destination;
  std::string manifest_sha256;
  std::string runtime_sha256;
  std::vector<std::string> artifact_sha256;
  bool contains_executable = false;
  std::string qualification_state = "not_qualified";
};

// Throws when a target, interface, artifact identity or failsafe declaration is
// missing.  The returned package is a deterministic manifest-only handoff:
// target code, signing, bench evidence, flight evidence and approval remain
// responsibilities of the target integration programme.
[[nodiscard]] DeploymentPackage build_manifest_package(const DeploymentSpec& specification);

// Verify an exchanged package before an adapter consumes it. This checks the
// manifest identity and the non-qualified/non-executable safety markers; it
// does not approve a target or infer that any artifact hash is present on disk.
void verify_manifest_package(const DeploymentPackage& package);

// Parse a manifest-only handoff received from another process. The parser
// derives the package identity and artifact role/hash map from the manifest,
// then applies the same strict verification as verify_manifest_package.
[[nodiscard]] DeploymentPackage parse_manifest_package(const std::string& manifest);

// Atomically stage a verified manifest-only package and its referenced artifact
// files into a new directory. The destination must not already exist, source
// files must be regular non-symlink files whose SHA-256 matches the manifest,
// and the operation never creates an executable. This is a reviewable handoff
// staging step, not a flight-computer installer or a qualification decision.
[[nodiscard]] DeploymentReceipt stage_manifest_package(const DeploymentPackage& package,
                                                       const std::vector<ArtifactFile>& files,
                                                       const std::filesystem::path& destination);

// Stage a runnable POSIX handoff containing the Galata runtime executable and
// the same hash-verified model/controller artifacts. The manifest remains
// deliberately not_qualified; this operation proves package integrity and
// layout, not target timing, signing, HIL or airworthiness.
[[nodiscard]] DeploymentReceipt stage_runtime_package(const DeploymentPackage& package,
                                                      const ArtifactFile& runtime,
                                                      const std::vector<ArtifactFile>& files,
                                                      const std::filesystem::path& destination);

// Re-check a previously staged runtime package before an operator starts it.
// The returned receipt is still non-qualified even when every byte verifies.
[[nodiscard]] DeploymentReceipt verify_runtime_package(
    const std::filesystem::path& destination);

}  // namespace galata::onboard

#endif  // GALATA_ONBOARD_DEPLOYMENT_HPP
