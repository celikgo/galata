// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace galata::qualification {

// A bounded, byte-addressed dossier for an application-specific qualification
// review.  The verifier checks completeness and identity only.  It deliberately
// never promotes this record to an aircraft, product, or certification approval.
struct DossierFile {
  std::string role;
  std::string relative_path;
  std::string sha256;

  friend bool operator==(const DossierFile&, const DossierFile&) = default;
};

struct Dossier {
  std::string manifest;
  std::string manifest_sha256;
  std::string product_id;
  std::string product_version;
  std::string intended_use;
  std::string aircraft_id;
  std::string aircraft_configuration;
  std::string qualification_basis;
  std::string authority_id;
  std::vector<DossierFile> files;
};

[[nodiscard]] Dossier parse_dossier(const std::string& manifest);

// Verify every required dossier file against the manifest.  Relative paths
// must remain inside package_root, and neither files nor parent components may
// be symlinks.  A successful result is a complete, byte-consistent dossier;
// it is not a qualification, certification, airworthiness, or release claim.
[[nodiscard]] std::uintmax_t verify_dossier(const Dossier& dossier,
                                            const std::filesystem::path& package_root);

}  // namespace galata::qualification
