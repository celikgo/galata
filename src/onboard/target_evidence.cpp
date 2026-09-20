// SPDX-License-Identifier: Apache-2.0
#include "galata/onboard/target_evidence.hpp"

#include "galata/core/sha256.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cctype>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <unordered_set>

namespace galata::onboard {
namespace {

constexpr std::uintmax_t kMaximumEvidenceBytes = 256U * 1024U * 1024U;

bool is_sha256(const std::string& value) {
  if (value.size() != 64U) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isdigit(character) != 0 || (character >= 'a' && character <= 'f')
           || (character >= 'A' && character <= 'F');
  });
}

bool is_evidence_class(const std::string& value) {
  return value == "host_sil" || value == "target_hil" || value == "flight_target";
}

void validate_text(const std::string& value, const std::string& field) {
  if (value.empty()) {
    throw std::invalid_argument("onboard target evidence: " + field + " is empty");
  }
  for (const char raw_character : value) {
    const unsigned char character = static_cast<unsigned char>(raw_character);
    if (character < 0x20U || character == 0x7fU || raw_character == '=') {
      throw std::invalid_argument("onboard target evidence: " + field
                                  + " contains a control or separator character");
    }
  }
}

std::map<std::string, std::string> fields_from_manifest(const std::string& manifest) {
  std::map<std::string, std::string> fields;
  std::istringstream lines(manifest);
  std::string line;
  while (std::getline(lines, line)) {
    const std::size_t separator = line.find('=');
    if (separator == std::string::npos || separator == 0U) {
      throw std::invalid_argument("onboard target evidence: malformed manifest field");
    }
    const std::string key = line.substr(0, separator);
    const std::string value = line.substr(separator + 1U);
    validate_text(key, "manifest key");
    if (!value.empty()) {
      for (const char raw_character : value) {
        const unsigned char character = static_cast<unsigned char>(raw_character);
        if (character < 0x20U || character == 0x7fU) {
          throw std::invalid_argument("onboard target evidence: manifest value contains control "
                                      "characters");
        }
      }
    }
    if (!fields.emplace(key, value).second) {
      throw std::invalid_argument("onboard target evidence: duplicate manifest field '" + key
                                  + "'");
    }
  }
  return fields;
}

std::string required(const std::map<std::string, std::string>& fields, const std::string& key) {
  const auto found = fields.find(key);
  if (found == fields.end() || found->second.empty()) {
    throw std::invalid_argument("onboard target evidence: missing required field '" + key + "'");
  }
  return found->second;
}

double positive_number(const std::map<std::string, std::string>& fields,
                       const std::string& key) {
  const std::string text = required(fields, key);
  std::size_t consumed = 0U;
  double value = 0.0;
  try {
    value = std::stod(text, &consumed);
  } catch (const std::exception&) {
    throw std::invalid_argument("onboard target evidence: field '" + key
                                + "' must be a finite non-negative number");
  }
  if (consumed != text.size() || value < 0.0 || !std::isfinite(value)) {
    throw std::invalid_argument("onboard target evidence: field '" + key
                                + "' must be a finite non-negative number");
  }
  return value;
}

std::size_t index_from(const std::string& text, const std::string& key) {
  if (text.empty()) {
    throw std::invalid_argument("onboard target evidence: empty file index in '" + key + "'");
  }
  std::size_t value = 0U;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    throw std::invalid_argument("onboard target evidence: invalid file index in '" + key + "'");
  }
  return value;
}

bool safe_relative_path(const std::string& text) {
  const std::filesystem::path path(text);
  if (text.empty() || path.is_absolute() || path.lexically_normal() != path) {
    return false;
  }
  bool has_component = false;
  for (const auto& component : path) {
    has_component = true;
    if (component == "." || component == ".." || component.empty()) {
      return false;
    }
  }
  return has_component && path.begin()->string() == "evidence";
}

bool safe_role(const std::string& value) {
  if (value.empty()) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isalnum(character) != 0 || character == '_' || character == '-';
  });
}

std::map<std::size_t, std::map<std::string, std::string>> file_fields(
    const std::map<std::string, std::string>& fields) {
  std::map<std::size_t, std::map<std::string, std::string>> grouped;
  constexpr std::string_view prefix = "file.";
  for (const auto& [key, value] : fields) {
    if (!key.starts_with(prefix)) {
      continue;
    }
    const std::size_t separator = key.find('.', prefix.size());
    if (separator == std::string::npos || key.find('.', separator + 1U) != std::string::npos) {
      throw std::invalid_argument("onboard target evidence: malformed file field '" + key + "'");
    }
    const std::size_t index = index_from(key.substr(prefix.size(), separator - prefix.size()), key);
    const std::string field = key.substr(separator + 1U);
    if (field != "role" && field != "path" && field != "sha256") {
      throw std::invalid_argument("onboard target evidence: unknown file field '" + key + "'");
    }
    if (!grouped[index].emplace(field, value).second) {
      throw std::invalid_argument("onboard target evidence: duplicate file field '" + key + "'");
    }
  }
  return grouped;
}

bool same_target_identity(const hardware::TargetIdentity& left,
                          const hardware::TargetIdentity& right) {
  return left.hardware_id == right.hardware_id
         && left.flight_computer_id == right.flight_computer_id
         && left.firmware_id == right.firmware_id
         && left.emergency_stop_id == right.emergency_stop_id;
}

void reject_symlink_path(const std::filesystem::path& root,
                         const std::filesystem::path& relative) {
  const auto root_status = std::filesystem::symlink_status(root);
  if (!std::filesystem::is_directory(root_status) || std::filesystem::is_symlink(root_status)) {
    throw std::invalid_argument("onboard target evidence: package root must be a real directory");
  }
  std::filesystem::path current = root;
  for (const auto& component : relative) {
    current /= component;
    if (std::filesystem::is_symlink(std::filesystem::symlink_status(current))) {
      throw std::invalid_argument("onboard target evidence: symlink path component is refused: '"
                                  + current.string() + "'");
    }
  }
}

std::string read_file(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::invalid_argument("onboard target evidence: cannot open '" + path.string() + "'");
  }
  std::ostringstream bytes;
  bytes << input.rdbuf();
  if (!input.good() && !input.eof()) {
    throw std::runtime_error("onboard target evidence: failed to read '" + path.string() + "'");
  }
  return bytes.str();
}

void write_file(const std::filesystem::path& path, const std::string& bytes) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("onboard target evidence: cannot create '" + path.string() + "'");
  }
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  if (!output) {
    throw std::runtime_error("onboard target evidence: failed to write '" + path.string() + "'");
  }
}

void check_known_field(const std::string& key) {
  static const std::set<std::string> known = {
      "format",
      "evidence_class",
      "qualification_state",
      "target_acceptance_state",
      "target.hardware_id",
      "target.flight_computer_id",
      "target.firmware_id",
      "target.emergency_stop_id",
      "deployment_manifest_sha256",
      "interface.id",
      "hardware.profile_id",
      "measurement.controller_worst_case_s",
      "measurement.cycle_worst_case_s",
      "measurement.watchdog_response_s",
      "test.emergency_stop",
      "test.loss_of_link",
      "test.hil",
      "signing.state"};
  if (!known.contains(key) && !key.starts_with("file.")) {
    throw std::invalid_argument("onboard target evidence: unknown manifest field '" + key + "'");
  }
}

}  // namespace

TargetEvidencePackage parse_target_evidence_package(const std::string& manifest) {
  TargetEvidencePackage package;
  package.manifest = manifest;
  package.manifest_sha256 = core::sha256(manifest);
  const auto fields = fields_from_manifest(manifest);
  for (const auto& [key, unused] : fields) {
    static_cast<void>(unused);
    check_known_field(key);
  }
  if (required(fields, "format") != "galata-target-integration-evidence-v2"
      || required(fields, "qualification_state") != "not_qualified"
      || required(fields, "target_acceptance_state") != "passed") {
    throw std::invalid_argument(
        "onboard target evidence: format or safety state is not an accepted pass record");
  }
  package.qualification_state = required(fields, "qualification_state");
  package.target_acceptance_state = required(fields, "target_acceptance_state");
  package.evidence_class = required(fields, "evidence_class");
  if (!is_evidence_class(package.evidence_class)) {
    throw std::invalid_argument(
        "onboard target evidence: evidence_class must be host_sil, target_hil or flight_target");
  }
  package.target_identity = {required(fields, "target.hardware_id"),
                             required(fields, "target.flight_computer_id"),
                             required(fields, "target.firmware_id"),
                             required(fields, "target.emergency_stop_id")};
  hardware::validate_target_identity(package.target_identity);
  package.deployment_manifest_sha256 = required(fields, "deployment_manifest_sha256");
  if (!is_sha256(package.deployment_manifest_sha256)) {
    throw std::invalid_argument("onboard target evidence: deployment manifest hash is invalid");
  }
  package.interface_id = required(fields, "interface.id");
  package.transport_profile_id = required(fields, "hardware.profile_id");
  package.controller_worst_case_s = positive_number(fields, "measurement.controller_worst_case_s");
  package.cycle_worst_case_s = positive_number(fields, "measurement.cycle_worst_case_s");
  package.watchdog_response_s = positive_number(fields, "measurement.watchdog_response_s");
  const bool physical_tests_applicable = package.evidence_class != "host_sil";
  const std::string expected_test_state = physical_tests_applicable ? "passed" : "not_applicable";
  const std::string expected_signing_state = physical_tests_applicable ? "verified" : "not_applicable";
  if (required(fields, "test.emergency_stop") != expected_test_state
      || required(fields, "test.loss_of_link") != expected_test_state
      || required(fields, "test.hil") != expected_test_state
      || required(fields, "signing.state") != expected_signing_state) {
    throw std::invalid_argument(
        "onboard target evidence: required physical-test or signing state is not valid for the "
        "evidence class");
  }

  const auto grouped = file_fields(fields);
  constexpr std::array<std::string_view, 5> required_roles = {
      "timing_report", "hardware_hil_report", "failsafe_report", "signing_record",
      "target_configuration"};
  std::set<std::string> roles;
  for (std::size_t index = 0U; index < grouped.size(); ++index) {
    const auto found = grouped.find(index);
    if (found == grouped.end() || found->second.size() != 3U) {
      throw std::invalid_argument(
          "onboard target evidence: file indices must be contiguous and complete");
    }
    const auto& fields_for_file = found->second;
    const std::string role = fields_for_file.at("role");
    const std::string path = fields_for_file.at("path");
    const std::string sha256 = fields_for_file.at("sha256");
    if (!safe_role(role) || !roles.insert(role).second || !safe_relative_path(path)
        || !is_sha256(sha256)) {
      throw std::invalid_argument("onboard target evidence: invalid evidence file identity");
    }
    package.files.push_back({role, path, sha256});
  }
  if (roles.size() != required_roles.size()
      || !std::all_of(required_roles.begin(), required_roles.end(),
                      [&roles](const std::string_view role) { return roles.contains(std::string(role)); })) {
    throw std::invalid_argument(
        "onboard target evidence: all five target evidence roles are required");
  }
  return package;
}

void verify_target_evidence_package(const TargetEvidencePackage& evidence,
                                    const DeploymentPackage& deployment);
std::uintmax_t verify_target_evidence_files(const TargetEvidencePackage& evidence,
                                            const std::filesystem::path& package_root);

TargetEvidencePackage stage_target_evidence_package(
    const DeploymentPackage& deployment,
    const TargetEvidenceSpec& specification,
    const std::filesystem::path& destination) {
  verify_manifest_package(deployment);
  if (!is_evidence_class(specification.evidence_class)) {
    throw std::invalid_argument(
        "onboard target evidence: evidence_class must be host_sil, target_hil or flight_target");
  }
  const bool physical_tests_applicable = specification.evidence_class != "host_sil";
  if (physical_tests_applicable
      && (!specification.emergency_stop_passed || !specification.loss_of_link_passed
          || !specification.hil_passed || !specification.signing_verified)) {
    throw std::invalid_argument(
        "onboard target evidence: target_hil and flight_target require every physical-test and "
        "signing confirmation");
  }
  if (!physical_tests_applicable
      && (specification.emergency_stop_passed || specification.loss_of_link_passed
          || specification.hil_passed || specification.signing_verified)) {
    throw std::invalid_argument(
        "onboard target evidence: host_sil must not claim physical-test or target-signing "
        "passes");
  }
  for (const auto [value, field] :
       std::array<std::pair<double, const char*>, 3>{{
           {specification.controller_worst_case_s, "controller_worst_case_s"},
           {specification.cycle_worst_case_s, "cycle_worst_case_s"},
           {specification.watchdog_response_s, "watchdog_response_s"}}}) {
    if (value < 0.0 || !std::isfinite(value)) {
      throw std::invalid_argument("onboard target evidence: " + std::string(field)
                                  + " must be finite and non-negative");
    }
  }
  if (specification.controller_worst_case_s > deployment.max_controller_time_s
      || specification.cycle_worst_case_s > deployment.transport_profile.watchdog_timeout_s
      || specification.watchdog_response_s > deployment.transport_profile.watchdog_timeout_s
      || specification.cycle_worst_case_s < specification.controller_worst_case_s) {
    throw std::invalid_argument(
        "onboard target evidence: measured timing cannot satisfy the deployment contract");
  }

  constexpr std::array<std::string_view, 5> required_roles = {
      "timing_report", "hardware_hil_report", "failsafe_report", "signing_record",
      "target_configuration"};
  if (specification.files.size() != required_roles.size()) {
    throw std::invalid_argument(
        "onboard target evidence: exactly five evidence files are required");
  }
  std::map<std::string, std::filesystem::path> sources;
  for (const auto& source : specification.files) {
    if (!safe_role(source.role)
        || !sources.emplace(source.role, source.source).second
        || std::find(required_roles.begin(), required_roles.end(), source.role)
               == required_roles.end()) {
      throw std::invalid_argument("onboard target evidence: invalid or duplicate evidence role '"
                                  + source.role + "'");
    }
    const auto status = std::filesystem::symlink_status(source.source);
    if (std::filesystem::is_symlink(status) || !std::filesystem::is_regular_file(status)) {
      throw std::invalid_argument(
          "onboard target evidence: source must be a regular non-symlink file: '"
          + source.source.string() + "'");
    }
    std::error_code error;
    const std::uintmax_t size = std::filesystem::file_size(source.source, error);
    if (error || size > kMaximumEvidenceBytes) {
      throw std::invalid_argument("onboard target evidence: source is missing or exceeds 256 MiB: '"
                                  + source.source.string() + "'");
    }
  }
  for (const auto role : required_roles) {
    if (!sources.contains(std::string(role))) {
      throw std::invalid_argument("onboard target evidence: missing required role '"
                                  + std::string(role) + "'");
    }
  }
  if (destination.empty() || destination.filename() == "." || destination.filename() == ".."
      || std::filesystem::exists(std::filesystem::symlink_status(destination))) {
    throw std::invalid_argument(
        "onboard target evidence: destination must be a new named directory");
  }

  const std::filesystem::path parent = destination.parent_path().empty()
                                           ? std::filesystem::current_path()
                                           : destination.parent_path();
  std::filesystem::create_directories(parent);
  const std::filesystem::path staging = destination.string() + ".staging";
  if (std::filesystem::exists(std::filesystem::symlink_status(staging))) {
    throw std::invalid_argument("onboard target evidence: staging destination already exists");
  }

  std::error_code cleanup_error;
  try {
    std::filesystem::create_directories(staging / "evidence");
    std::vector<std::string> digests;
    digests.reserve(required_roles.size());
    for (const auto role : required_roles) {
      const std::filesystem::path target = staging / "evidence" / std::string(role);
      std::filesystem::copy_file(sources.at(std::string(role)), target,
                                 std::filesystem::copy_options::none);
      const std::string digest = core::sha256(read_file(target));
      if (digest != core::sha256(read_file(sources.at(std::string(role))))) {
        throw std::runtime_error("onboard target evidence: source changed while copying role '"
                                 + std::string(role) + "'");
      }
      digests.push_back(digest);
    }

    std::ostringstream manifest;
    manifest << "format=galata-target-integration-evidence-v2\n"
             << "qualification_state=not_qualified\n"
             << "target_acceptance_state=passed\n"
             << "evidence_class=" << specification.evidence_class << "\n"
             << "target.hardware_id=" << deployment.target_identity.hardware_id << "\n"
             << "target.flight_computer_id=" << deployment.target_identity.flight_computer_id
             << "\n"
             << "target.firmware_id=" << deployment.target_identity.firmware_id << "\n"
             << "target.emergency_stop_id=" << deployment.target_identity.emergency_stop_id
             << "\n"
             << "deployment_manifest_sha256=" << deployment.manifest_sha256 << "\n"
             << "interface.id=" << deployment.interface.id << "\n"
             << "hardware.profile_id=" << deployment.transport_profile.id << "\n"
             << "measurement.controller_worst_case_s="
             << specification.controller_worst_case_s << "\n"
             << "measurement.cycle_worst_case_s=" << specification.cycle_worst_case_s << "\n"
             << "measurement.watchdog_response_s=" << specification.watchdog_response_s << "\n"
             << "test.emergency_stop=" << (physical_tests_applicable ? "passed" : "not_applicable")
             << "\n"
             << "test.loss_of_link=" << (physical_tests_applicable ? "passed" : "not_applicable")
             << "\n"
             << "test.hil=" << (physical_tests_applicable ? "passed" : "not_applicable") << "\n"
             << "signing.state=" << (physical_tests_applicable ? "verified" : "not_applicable")
             << "\n";
    for (std::size_t index = 0U; index < required_roles.size(); ++index) {
      manifest << "file." << index << ".role=" << required_roles[index] << "\n"
               << "file." << index << ".path=evidence/" << required_roles[index] << "\n"
               << "file." << index << ".sha256=" << digests[index] << "\n";
    }
    write_file(staging / "target-evidence.manifest", manifest.str());
    write_file(staging / "target-evidence.manifest.sha256",
               core::sha256(manifest.str()) + "  target-evidence.manifest\n");
    const TargetEvidencePackage package = parse_target_evidence_package(manifest.str());
    verify_target_evidence_package(package, deployment);
    (void)verify_target_evidence_files(package, staging);
    std::filesystem::rename(staging, destination);
    return package;
  } catch (...) {
    std::filesystem::remove_all(staging, cleanup_error);
    throw;
  }
}

void verify_target_evidence_package(const TargetEvidencePackage& evidence,
                                    const DeploymentPackage& deployment) {
  const TargetEvidencePackage parsed = parse_target_evidence_package(evidence.manifest);
  if (parsed.manifest_sha256 != evidence.manifest_sha256
      || parsed.evidence_class != evidence.evidence_class
      || parsed.target_identity.hardware_id != evidence.target_identity.hardware_id
      || parsed.target_identity.flight_computer_id != evidence.target_identity.flight_computer_id
      || parsed.target_identity.firmware_id != evidence.target_identity.firmware_id
      || parsed.target_identity.emergency_stop_id != evidence.target_identity.emergency_stop_id
      || parsed.files != evidence.files
      || parsed.deployment_manifest_sha256 != evidence.deployment_manifest_sha256
      || parsed.interface_id != evidence.interface_id
      || parsed.transport_profile_id != evidence.transport_profile_id
      || parsed.controller_worst_case_s != evidence.controller_worst_case_s
      || parsed.cycle_worst_case_s != evidence.cycle_worst_case_s
      || parsed.watchdog_response_s != evidence.watchdog_response_s) {
    throw std::invalid_argument("onboard target evidence: parsed package does not match its object");
  }
  if (!same_target_identity(evidence.target_identity, deployment.target_identity)) {
    throw std::invalid_argument("onboard target evidence: target identity does not match deployment");
  }
  if (evidence.deployment_manifest_sha256 != deployment.manifest_sha256) {
    throw std::invalid_argument(
        "onboard target evidence: deployment manifest hash does not match evidence");
  }
  if (evidence.interface_id != deployment.interface.id
      || evidence.transport_profile_id != deployment.transport_profile.id) {
    throw std::invalid_argument(
        "onboard target evidence: interface or transport profile does not match deployment");
  }
  if (evidence.controller_worst_case_s > deployment.max_controller_time_s) {
    throw std::invalid_argument(
        "onboard target evidence: measured controller time exceeds deployment budget");
  }
  if (evidence.cycle_worst_case_s > deployment.transport_profile.watchdog_timeout_s
      || evidence.watchdog_response_s > deployment.transport_profile.watchdog_timeout_s) {
    throw std::invalid_argument(
        "onboard target evidence: measured timing exceeds the deployment watchdog");
  }
  if (evidence.cycle_worst_case_s < evidence.controller_worst_case_s) {
    throw std::invalid_argument(
        "onboard target evidence: cycle timing cannot be shorter than controller timing");
  }
}

std::uintmax_t verify_target_evidence_files(const TargetEvidencePackage& evidence,
                                            const std::filesystem::path& package_root) {
  reject_symlink_path(package_root, "evidence");
  const std::filesystem::path evidence_root = package_root / "evidence";
  if (!std::filesystem::is_directory(std::filesystem::symlink_status(evidence_root))) {
    throw std::invalid_argument("onboard target evidence: evidence directory is missing");
  }
  std::set<std::string> expected;
  std::uintmax_t total_bytes = 0U;
  for (const auto& file : evidence.files) {
    const std::filesystem::path relative(file.relative_path);
    reject_symlink_path(package_root, relative);
    const std::filesystem::path path = package_root / relative;
    const auto status = std::filesystem::symlink_status(path);
    if (std::filesystem::is_symlink(status) || !std::filesystem::is_regular_file(status)) {
      throw std::invalid_argument("onboard target evidence: evidence file is not a regular file: '"
                                  + path.string() + "'");
    }
    const std::uintmax_t size = std::filesystem::file_size(path);
    if (size > kMaximumEvidenceBytes) {
      throw std::invalid_argument("onboard target evidence: evidence file exceeds 256 MiB: '"
                                  + path.string() + "'");
    }
    if (core::sha256(read_file(path)) != file.sha256) {
      throw std::invalid_argument("onboard target evidence: evidence hash mismatch for role '"
                                  + file.role + "'");
    }
    total_bytes += size;
    expected.insert(file.relative_path);
  }
  std::set<std::string> actual;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(evidence_root)) {
    const auto entry_status = std::filesystem::symlink_status(entry.path());
    if (std::filesystem::is_symlink(entry_status)) {
      throw std::invalid_argument("onboard target evidence: evidence tree contains a symlink");
    }
    if (std::filesystem::is_directory(entry_status)) {
      continue;
    }
    if (!std::filesystem::is_regular_file(entry_status)) {
      throw std::invalid_argument("onboard target evidence: evidence tree contains a non-file");
    }
    actual.insert(std::filesystem::relative(entry.path(), package_root).generic_string());
  }
  if (actual != expected) {
    throw std::invalid_argument(
        "onboard target evidence: evidence directory contains an extra or missing file");
  }
  return total_bytes;
}

}  // namespace galata::onboard
