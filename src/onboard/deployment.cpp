// SPDX-License-Identifier: Apache-2.0
#include "galata/onboard/deployment.hpp"

#include "galata/core/sha256.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace galata::onboard {
namespace {

bool is_sha256(const std::string& value) {
  if (value.size() != 64) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isdigit(character) != 0 || (character >= 'a' && character <= 'f')
           || (character >= 'A' && character <= 'F');
  });
}

bool is_safe_role(const std::string& role) {
  if (role.empty() || role == "." || role == "..") {
    return false;
  }
  return std::all_of(role.begin(), role.end(), [](unsigned char character) {
    return std::isalnum(character) != 0 || character == '-' || character == '_' || character == '.';
  });
}

std::string read_bytes(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::invalid_argument("onboard: cannot open artifact '" + path.string() + "'");
  }
  std::ostringstream bytes;
  bytes << input.rdbuf();
  if (!input.good() && !input.eof()) {
    throw std::runtime_error("onboard: failed while reading artifact '" + path.string() + "'");
  }
  return bytes.str();
}

void write_bytes(const std::filesystem::path& path, const std::string& bytes) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("onboard: cannot create '" + path.string() + "'");
  }
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  if (!output) {
    throw std::runtime_error("onboard: failed while writing '" + path.string() + "'");
  }
}

void reject_symlink_components(const std::filesystem::path& root,
                               const std::filesystem::path& relative) {
  const std::filesystem::file_status root_status = std::filesystem::symlink_status(root);
  if (std::filesystem::is_symlink(root_status) || !std::filesystem::is_directory(root_status)) {
    throw std::invalid_argument("onboard: runtime package root must be a real directory: '"
                                + root.string() + "'");
  }
  std::filesystem::path current = root;
  for (const auto& component : relative) {
    current /= component;
    if (std::filesystem::is_symlink(std::filesystem::symlink_status(current))) {
      throw std::invalid_argument("onboard: runtime package contains a symlink path component: '"
                                  + current.string() + "'");
    }
  }
}

void validate_manifest_text(const std::string& value, const std::string& field) {
  for (const char raw_character : value) {
    const unsigned char character = static_cast<unsigned char>(raw_character);
    if (character < 0x20U || character == 0x7fU) {
      throw std::invalid_argument("onboard: " + field + " contains a control character");
    }
  }
}

std::map<std::string, std::string> manifest_fields(const std::string& manifest) {
  std::map<std::string, std::string> fields;
  std::istringstream lines(manifest);
  std::string line;
  while (std::getline(lines, line)) {
    const std::size_t separator = line.find('=');
    if (separator == std::string::npos || separator == 0) {
      throw std::invalid_argument("onboard: manifest contains a malformed field");
    }
    const std::string key = line.substr(0, separator);
    const std::string value = line.substr(separator + 1);
    validate_manifest_text(value, "manifest field " + key);
    if (!fields.emplace(key, value).second) {
      throw std::invalid_argument("onboard: manifest contains a duplicate field '" + key + "'");
    }
  }
  return fields;
}

std::map<std::string, std::string> manifest_artifacts(
    const std::map<std::string, std::string>& fields) {
  std::map<std::string, std::string> roles;
  std::map<std::string, std::string> hashes;
  const std::string prefix = "artifact.";
  for (const auto& [key, value] : fields) {
    if (!key.starts_with(prefix)) {
      continue;
    }
    const std::size_t field_separator = key.rfind('.');
    if (field_separator == std::string::npos || field_separator <= prefix.size()
        || (key.substr(field_separator + 1) != "role"
            && key.substr(field_separator + 1) != "sha256")) {
      throw std::invalid_argument("onboard: manifest contains a malformed artifact field");
    }
    const std::string index = key.substr(prefix.size(), field_separator - prefix.size());
    if (index.empty() || !std::all_of(index.begin(), index.end(), [](unsigned char character) {
          return std::isdigit(character) != 0;
        })) {
      throw std::invalid_argument("onboard: manifest contains a malformed artifact index");
    }
    const std::string field = key.substr(field_separator + 1);
    if (field == "role") {
      if (!is_safe_role(value)) {
        throw std::invalid_argument("onboard: manifest contains an invalid artifact role");
      }
      if (!roles.emplace(index, value).second) {
        throw std::invalid_argument("onboard: manifest contains a duplicate artifact role field");
      }
    } else {
      if (!is_sha256(value)) {
        throw std::invalid_argument("onboard: manifest contains an invalid artifact hash");
      }
      if (!hashes.emplace(index, value).second) {
        throw std::invalid_argument("onboard: manifest contains a duplicate artifact hash field");
      }
    }
  }
  if (roles.size() != hashes.size()) {
    throw std::invalid_argument("onboard: manifest artifact roles and hashes do not match");
  }
  std::map<std::string, std::string> artifacts;
  for (const auto& [index, role] : roles) {
    const auto hash = hashes.find(index);
    if (hash == hashes.end() || !artifacts.emplace(role, hash->second).second) {
      throw std::invalid_argument("onboard: manifest artifact identities are incomplete");
    }
  }
  return artifacts;
}

double positive_manifest_number(const std::map<std::string, std::string>& fields,
                                const std::string& key) {
  const auto found = fields.find(key);
  if (found == fields.end()) {
    throw std::invalid_argument("onboard: manifest is missing required field '" + key + "'");
  }
  std::size_t consumed = 0;
  double value = 0.0;
  try {
    value = std::stod(found->second, &consumed);
  } catch (const std::exception&) {
    throw std::invalid_argument("onboard: manifest field '" + key
                                + "' must be a positive finite number");
  }
  if (consumed != found->second.size() || !(value > 0.0) || !std::isfinite(value)) {
    throw std::invalid_argument("onboard: manifest field '" + key
                                + "' must be a positive finite number");
  }
  return value;
}

std::uint32_t positive_manifest_uint32(const std::map<std::string, std::string>& fields,
                                       const std::string& key) {
  const auto found = fields.find(key);
  if (found == fields.end()) {
    throw std::invalid_argument("onboard: manifest is missing required field '" + key + "'");
  }
  std::size_t consumed = 0;
  unsigned long long value = 0;
  try {
    value = std::stoull(found->second, &consumed);
  } catch (const std::exception&) {
    throw std::invalid_argument("onboard: manifest field '" + key + "' must be a positive uint32");
  }
  if (consumed != found->second.size() || value == 0
      || value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("onboard: manifest field '" + key + "' must be a positive uint32");
  }
  return static_cast<std::uint32_t>(value);
}

std::string required_manifest_text(const std::map<std::string, std::string>& fields,
                                   const std::string& key) {
  const auto found = fields.find(key);
  if (found == fields.end() || found->second.empty()) {
    throw std::invalid_argument("onboard: manifest is missing required field '" + key + "'");
  }
  return found->second;
}

std::size_t manifest_index(const std::string& text, const std::string& key) {
  if (text.empty()) {
    throw std::invalid_argument("onboard: manifest has an empty channel index in '" + key + "'");
  }
  std::size_t value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    throw std::invalid_argument("onboard: manifest has an invalid channel index in '" + key + "'");
  }
  return value;
}

std::vector<hardware::ChannelSpec> channels_from_fields(
    const std::map<std::string, std::string>& fields,
    const char* direction) {
  const std::string prefix = std::string("channel.") + direction + ".";
  std::map<std::size_t, hardware::ChannelSpec> channels;
  for (const auto& [key, value] : fields) {
    if (!key.starts_with(prefix)) {
      continue;
    }
    const std::string remainder = key.substr(prefix.size());
    const std::size_t separator = remainder.find('.');
    if (separator == std::string::npos || remainder.find('.', separator + 1) != std::string::npos) {
      throw std::invalid_argument("onboard: malformed channel field '" + key + "'");
    }
    const std::size_t index = manifest_index(remainder.substr(0, separator), key);
    const std::string field = remainder.substr(separator + 1);
    if (field != "name" && field != "unit" && field != "frame") {
      throw std::invalid_argument("onboard: unknown channel field '" + key + "'");
    }
    auto& channel = channels[index];
    if (field == "name") {
      if (!channel.name.empty()) {
        throw std::invalid_argument("onboard: duplicate channel field '" + key + "'");
      }
      channel.name = value;
    } else if (field == "unit") {
      if (!channel.unit.empty()) {
        throw std::invalid_argument("onboard: duplicate channel field '" + key + "'");
      }
      channel.unit = value;
    } else {
      if (!channel.frame.empty()) {
        throw std::invalid_argument("onboard: duplicate channel field '" + key + "'");
      }
      channel.frame = value;
    }
  }
  if (channels.empty()) {
    throw std::invalid_argument(std::string("onboard: manifest has no ") + direction + " channels");
  }
  std::vector<hardware::ChannelSpec> result;
  result.reserve(channels.size());
  for (std::size_t index = 0; index < channels.size(); ++index) {
    const auto found = channels.find(index);
    if (found == channels.end() || found->second.name.empty() || found->second.unit.empty()
        || found->second.frame.empty()) {
      throw std::invalid_argument(std::string("onboard: manifest has an incomplete ") + direction
                                  + " channel list");
    }
    result.push_back(found->second);
  }
  return result;
}

hardware::InterfaceSpec interface_from_fields(const std::map<std::string, std::string>& fields) {
  hardware::InterfaceSpec specification;
  specification.id = required_manifest_text(fields, "interface.id");
  specification.sample_period_s = positive_manifest_number(fields, "interface.sample_period_s");
  if (required_manifest_text(fields, "interface.external_arming_required") != "true") {
    throw std::invalid_argument("onboard: interface.external_arming_required must be true");
  }
  specification.external_arming_required = true;
  specification.sensor_channels = channels_from_fields(fields, "sensor");
  specification.actuator_channels = channels_from_fields(fields, "actuator");
  hardware::validate_interface(specification);
  return specification;
}

bool same_interface(const hardware::InterfaceSpec& left, const hardware::InterfaceSpec& right) {
  if (left.id != right.id || left.sample_period_s != right.sample_period_s
      || left.external_arming_required != right.external_arming_required
      || left.sensor_channels.size() != right.sensor_channels.size()
      || left.actuator_channels.size() != right.actuator_channels.size()) {
    return false;
  }
  const auto same_channels = [](const auto& left_channels, const auto& right_channels) {
    for (std::size_t index = 0; index < left_channels.size(); ++index) {
      if (left_channels[index].name != right_channels[index].name
          || left_channels[index].unit != right_channels[index].unit
          || left_channels[index].frame != right_channels[index].frame) {
        return false;
      }
    }
    return true;
  };
  return same_channels(left.sensor_channels, right.sensor_channels)
         && same_channels(left.actuator_channels, right.actuator_channels);
}

bool same_target_identity(const hardware::TargetIdentity& left,
                          const hardware::TargetIdentity& right) {
  return left.hardware_id == right.hardware_id
         && left.flight_computer_id == right.flight_computer_id
         && left.firmware_id == right.firmware_id
         && left.emergency_stop_id == right.emergency_stop_id;
}

hardware::TransportProfile transport_profile_from_fields(
    const std::map<std::string, std::string>& fields) {
  const auto required_text = [&fields](const std::string& key) {
    const auto found = fields.find(key);
    if (found == fields.end() || found->second.empty()) {
      throw std::invalid_argument("onboard: manifest is missing required field '" + key + "'");
    }
    return found->second;
  };
  hardware::TransportProfile profile;
  profile.id = required_text("hardware.profile_id");
  profile.transport = required_text("hardware.transport");
  profile.endpoint = required_text("hardware.endpoint");
  profile.receive_timeout_ms = positive_manifest_uint32(fields, "hardware.receive_timeout_ms");
  profile.transmit_timeout_ms = positive_manifest_uint32(fields, "hardware.transmit_timeout_ms");
  profile.watchdog_timeout_s = positive_manifest_number(fields, "hardware.watchdog_timeout_s");
  if (required_text("hardware.emergency_stop_required") != "true") {
    throw std::invalid_argument("onboard: hardware.emergency_stop_required must be true");
  }
  profile.emergency_stop_required = true;
  return profile;
}

bool same_transport_profile(const hardware::TransportProfile& left,
                            const hardware::TransportProfile& right) {
  return left.id == right.id && left.transport == right.transport && left.endpoint == right.endpoint
         && left.receive_timeout_ms == right.receive_timeout_ms
         && left.transmit_timeout_ms == right.transmit_timeout_ms
         && left.watchdog_timeout_s == right.watchdog_timeout_s
         && left.emergency_stop_required == right.emergency_stop_required;
}

void write_channel_list(std::ostringstream& manifest,
                        const char* direction,
                        const std::vector<hardware::ChannelSpec>& channels) {
  for (std::size_t index = 0; index < channels.size(); ++index) {
    const auto& channel = channels[index];
    validate_manifest_text(channel.name, std::string("channel ") + direction + ".name");
    validate_manifest_text(channel.unit, std::string("channel ") + direction + ".unit");
    validate_manifest_text(channel.frame, std::string("channel ") + direction + ".frame");
    manifest << "channel." << direction << "." << index << ".name=" << channel.name << "\n";
    manifest << "channel." << direction << "." << index << ".unit=" << channel.unit << "\n";
    manifest << "channel." << direction << "." << index << ".frame=" << channel.frame << "\n";
  }
}

}  // namespace

DeploymentPackage build_manifest_package(const DeploymentSpec& specification) {
  if (specification.target_platform.empty()) {
    throw std::invalid_argument("onboard: target_platform is required");
  }
  if (specification.model_description.empty()) {
    throw std::invalid_argument("onboard: model_description is required");
  }
  if (specification.controller_description.empty()) {
    throw std::invalid_argument("onboard: controller_description is required");
  }
  if (specification.failsafe_action.empty()) {
    throw std::invalid_argument(
        "onboard: failsafe_action is required; an integration cannot default its loss-of-link "
        "behaviour");
  }
  if (!(specification.max_controller_time_s > 0.0)
      || !std::isfinite(specification.max_controller_time_s)) {
    throw std::invalid_argument("onboard: max_controller_time_s must be positive and finite");
  }
  validate_manifest_text(specification.target_platform, "target_platform");
  hardware::validate_target_identity(specification.target_identity);
  validate_manifest_text(specification.model_description, "model_description");
  validate_manifest_text(specification.controller_description, "controller_description");
  validate_manifest_text(specification.failsafe_action, "failsafe_action");
  validate_manifest_text(specification.interface.id, "interface.id");
  hardware::validate_interface(specification.interface);
  hardware::validate_transport_profile(specification.transport_profile,
                                       specification.interface.sample_period_s);
  if (!specification.interface.external_arming_required) {
    throw std::invalid_argument(
        "onboard: external arming cannot be disabled by a manifest-only package");
  }

  std::unordered_set<std::string> roles;
  if (specification.artifacts.empty()) {
    throw std::invalid_argument("onboard: model and controller artifact identities are required");
  }
  for (const ArtifactReference& artifact : specification.artifacts) {
    if (!is_safe_role(artifact.role) || !is_sha256(artifact.sha256)) {
      throw std::invalid_argument(
          "onboard: every artifact needs a safe role and a 64-character SHA-256 identity");
    }
    validate_manifest_text(artifact.role, "artifact.role");
    if (!roles.insert(artifact.role).second) {
      throw std::invalid_argument("onboard: duplicate artifact role '" + artifact.role + "'");
    }
  }
  if (!roles.contains("model") || !roles.contains("controller")) {
    throw std::invalid_argument("onboard: artifact roles must include both model and controller");
  }

  std::vector<ArtifactReference> artifacts = specification.artifacts;
  std::sort(artifacts.begin(),
            artifacts.end(),
            [](const ArtifactReference& left, const ArtifactReference& right) {
              return left.role < right.role;
            });

  std::ostringstream manifest;
  manifest.imbue(std::locale::classic());
  manifest << std::setprecision(std::numeric_limits<double>::max_digits10);
  manifest << "format=galata-onboard-interface-manifest-v1\n";
  manifest << "qualification_state=not_qualified\n";
  manifest << "contains_executable=false\n";
  manifest << "target_platform=" << specification.target_platform << "\n";
  manifest << "target.hardware_id=" << specification.target_identity.hardware_id << "\n";
  manifest << "target.flight_computer_id=" << specification.target_identity.flight_computer_id
           << "\n";
  manifest << "target.firmware_id=" << specification.target_identity.firmware_id << "\n";
  manifest << "target.emergency_stop_id=" << specification.target_identity.emergency_stop_id
           << "\n";
  manifest << "model_description=" << specification.model_description << "\n";
  manifest << "controller_description=" << specification.controller_description << "\n";
  manifest << "failsafe_action=" << specification.failsafe_action << "\n";
  manifest << "runtime.max_controller_time_s=" << specification.max_controller_time_s << "\n";
  manifest << "interface.id=" << specification.interface.id << "\n";
  manifest << "interface.sample_period_s=" << specification.interface.sample_period_s << "\n";
  manifest << "interface.external_arming_required=true\n";
  manifest << "hardware.profile_id=" << specification.transport_profile.id << "\n";
  manifest << "hardware.transport=" << specification.transport_profile.transport << "\n";
  manifest << "hardware.endpoint=" << specification.transport_profile.endpoint << "\n";
  manifest << "hardware.receive_timeout_ms=" << specification.transport_profile.receive_timeout_ms
           << "\n";
  manifest << "hardware.transmit_timeout_ms=" << specification.transport_profile.transmit_timeout_ms
           << "\n";
  manifest << "hardware.watchdog_timeout_s=" << specification.transport_profile.watchdog_timeout_s
           << "\n";
  manifest << "hardware.emergency_stop_required=true\n";
  write_channel_list(manifest, "sensor", specification.interface.sensor_channels);
  write_channel_list(manifest, "actuator", specification.interface.actuator_channels);
  for (std::size_t index = 0; index < artifacts.size(); ++index) {
    const auto& artifact = artifacts[index];
    manifest << "artifact." << index << ".role=" << artifact.role << "\n";
    manifest << "artifact." << index << ".sha256=" << artifact.sha256 << "\n";
  }

  DeploymentPackage package;
  package.manifest = manifest.str();
  package.manifest_sha256 = core::sha256(package.manifest);
  package.target_platform = specification.target_platform;
  package.target_identity = specification.target_identity;
  package.interface = specification.interface;
  package.artifacts = std::move(artifacts);
  package.max_controller_time_s = specification.max_controller_time_s;
  package.transport_profile = specification.transport_profile;
  return package;
}

DeploymentPackage parse_manifest_package(const std::string& manifest) {
  DeploymentPackage package;
  package.manifest = manifest;
  package.manifest_sha256 = core::sha256(manifest);
  const auto fields = manifest_fields(manifest);
  package.target_platform = required_manifest_text(fields, "target_platform");
  package.target_identity = {
      required_manifest_text(fields, "target.hardware_id"),
      required_manifest_text(fields, "target.flight_computer_id"),
      required_manifest_text(fields, "target.firmware_id"),
      required_manifest_text(fields, "target.emergency_stop_id")};
  package.interface = interface_from_fields(fields);
  package.max_controller_time_s = positive_manifest_number(fields, "runtime.max_controller_time_s");
  package.transport_profile = transport_profile_from_fields(fields);
  if (const auto found = fields.find("qualification_state"); found != fields.end()) {
    package.qualification_state = found->second;
  } else {
    package.qualification_state.clear();
  }
  if (const auto found = fields.find("contains_executable"); found != fields.end()) {
    package.contains_executable = found->second == "true";
  }
  for (const auto& [role, sha256] : manifest_artifacts(fields)) {
    package.artifacts.push_back({role, sha256});
  }
  verify_manifest_package(package);
  return package;
}

void verify_manifest_package(const DeploymentPackage& package) {
  if (package.manifest.empty() || !is_sha256(package.manifest_sha256)
      || package.manifest_sha256 != core::sha256(package.manifest)) {
    throw std::invalid_argument("onboard: manifest package hash does not match its bytes");
  }
  if (package.contains_executable || package.qualification_state != "not_qualified") {
    throw std::invalid_argument(
        "onboard: a manifest-only package must remain non-executable and not_qualified");
  }
  if (package.manifest.find("qualification_state=not_qualified\n") == std::string::npos
      || package.manifest.find("contains_executable=false\n") == std::string::npos) {
    throw std::invalid_argument(
        "onboard: manifest package is missing its non-qualified safety markers");
  }
  const auto fields = manifest_fields(package.manifest);
  const auto require_nonempty = [&fields](const std::string& key) {
    const auto found = fields.find(key);
    if (found == fields.end() || found->second.empty()) {
      throw std::invalid_argument("onboard: manifest is missing required field '" + key + "'");
    }
  };
  const auto field_equals = [&fields](const std::string& key, const std::string& expected) {
    const auto found = fields.find(key);
    return found != fields.end() && found->second == expected;
  };
  if (!field_equals("format", "galata-onboard-interface-manifest-v1")
      || !field_equals("qualification_state", "not_qualified")
      || !field_equals("contains_executable", "false")
      || !field_equals("interface.external_arming_required", "true")) {
    throw std::invalid_argument("onboard: manifest has invalid safety or format markers");
  }
  for (const auto& key : {"target_platform",
                          "model_description",
                          "controller_description",
                          "failsafe_action",
                          "runtime.max_controller_time_s",
                          "interface.id",
                          "interface.sample_period_s",
                          "hardware.profile_id",
                          "hardware.transport",
                          "hardware.endpoint",
                          "hardware.receive_timeout_ms",
                          "hardware.transmit_timeout_ms",
                          "hardware.watchdog_timeout_s",
                          "hardware.emergency_stop_required"}) {
    require_nonempty(key);
  }
  const std::string declared_target_platform = required_manifest_text(fields, "target_platform");
  const hardware::TargetIdentity declared_target_identity{
      required_manifest_text(fields, "target.hardware_id"),
      required_manifest_text(fields, "target.flight_computer_id"),
      required_manifest_text(fields, "target.firmware_id"),
      required_manifest_text(fields, "target.emergency_stop_id")};
  hardware::validate_target_identity(declared_target_identity);
  const hardware::InterfaceSpec declared_interface = interface_from_fields(fields);
  if (package.target_platform != declared_target_platform) {
    throw std::invalid_argument("onboard: package target platform does not match its manifest");
  }
  if (!same_target_identity(package.target_identity, declared_target_identity)) {
    throw std::invalid_argument("onboard: package target identity does not match its manifest");
  }
  if (!same_interface(package.interface, declared_interface)) {
    throw std::invalid_argument("onboard: package interface does not match its manifest");
  }
  const double declared_controller_time =
      positive_manifest_number(fields, "runtime.max_controller_time_s");
  if (package.max_controller_time_s != declared_controller_time) {
    throw std::invalid_argument("onboard: package runtime budget does not match its manifest");
  }
  const double declared_sample_period =
      positive_manifest_number(fields, "interface.sample_period_s");
  const hardware::TransportProfile declared_transport_profile =
      transport_profile_from_fields(fields);
  hardware::validate_transport_profile(declared_transport_profile, declared_sample_period);
  if (!same_transport_profile(package.transport_profile, declared_transport_profile)) {
    throw std::invalid_argument("onboard: package transport profile does not match its manifest");
  }
  if (package.artifacts.empty()) {
    throw std::invalid_argument("onboard: manifest package has no artifact identities");
  }
  std::unordered_set<std::string> roles;
  std::map<std::string, std::string> expected_artifacts;
  for (const ArtifactReference& artifact : package.artifacts) {
    if (!is_safe_role(artifact.role) || !is_sha256(artifact.sha256)
        || !roles.insert(artifact.role).second) {
      throw std::invalid_argument("onboard: manifest package has invalid or duplicate artifacts");
    }
    expected_artifacts.emplace(artifact.role, artifact.sha256);
  }
  if (!roles.contains("model") || !roles.contains("controller")) {
    throw std::invalid_argument(
        "onboard: manifest package artifacts must include model and controller");
  }
  if (manifest_artifacts(fields) != expected_artifacts) {
    throw std::invalid_argument(
        "onboard: manifest artifact roles and hashes do not match the package");
  }
}

DeploymentReceipt stage_manifest_package(const DeploymentPackage& package,
                                         const std::vector<ArtifactFile>& files,
                                         const std::filesystem::path& destination) {
  verify_manifest_package(package);
  if (destination.empty() || destination.filename() == "." || destination.filename() == "..") {
    throw std::invalid_argument("onboard: deployment destination must be a named new directory");
  }
  if (std::filesystem::exists(destination)) {
    throw std::invalid_argument(
        "onboard: deployment destination already exists; refusing to overwrite it");
  }
  if (files.size() != package.artifacts.size()) {
    throw std::invalid_argument(
        "onboard: one source file is required for every manifest artifact identity");
  }

  std::unordered_set<std::string> supplied_roles;
  std::vector<std::pair<ArtifactReference, std::filesystem::path>> verified;
  verified.reserve(package.artifacts.size());
  for (const ArtifactReference& artifact : package.artifacts) {
    const auto found = std::find_if(files.begin(), files.end(), [&](const ArtifactFile& file) {
      return file.role == artifact.role;
    });
    if (found == files.end() || !supplied_roles.insert(found->role).second) {
      throw std::invalid_argument("onboard: source files do not match artifact roles");
    }
    if (!std::filesystem::is_regular_file(found->source)
        || std::filesystem::is_symlink(found->source)) {
      throw std::invalid_argument("onboard: artifact source must be a regular non-symlink file: '"
                                  + found->source.string() + "'");
    }
    const std::string bytes = read_bytes(found->source);
    if (core::sha256(bytes) != artifact.sha256) {
      throw std::invalid_argument("onboard: artifact hash does not match role '" + artifact.role
                                  + "'");
    }
    verified.emplace_back(artifact, found->source);
  }

  const std::filesystem::path parent = destination.parent_path().empty()
                                           ? std::filesystem::current_path()
                                           : destination.parent_path();
  std::filesystem::create_directories(parent);
  const std::filesystem::path staging =
      parent
      / (destination.filename().string() + ".staging-" + package.manifest_sha256.substr(0, 16));
  if (std::filesystem::exists(staging)) {
    throw std::invalid_argument("onboard: a previous staging directory already exists");
  }

  try {
    std::filesystem::create_directories(staging / "artifacts");
    write_bytes(staging / "onboard.manifest", package.manifest);
    write_bytes(staging / "onboard.manifest.sha256",
                package.manifest_sha256 + "  onboard.manifest\n");
    for (const auto& [artifact, source] : verified) {
      std::filesystem::copy_file(
          source, staging / "artifacts" / artifact.role, std::filesystem::copy_options::none);
      if (core::sha256(read_bytes(staging / "artifacts" / artifact.role)) != artifact.sha256) {
        throw std::runtime_error("onboard: staged artifact changed during publication for role '"
                                 + artifact.role + "'");
      }
    }
    if (core::sha256(read_bytes(staging / "onboard.manifest")) != package.manifest_sha256) {
      throw std::runtime_error("onboard: staged manifest changed during publication");
    }
    std::filesystem::rename(staging, destination);
  } catch (...) {
    std::error_code error;
    std::filesystem::remove_all(staging, error);
    throw;
  }

  DeploymentReceipt receipt;
  receipt.destination = destination;
  receipt.manifest_sha256 = package.manifest_sha256;
  receipt.contains_executable = false;
  receipt.qualification_state = "not_qualified";
  for (const ArtifactReference& artifact : package.artifacts) {
    receipt.artifact_sha256.push_back(artifact.sha256);
  }
  return receipt;
}

namespace {

void validate_runtime_source(const std::filesystem::path& source) {
  const std::filesystem::file_status status = std::filesystem::symlink_status(source);
  if (std::filesystem::is_symlink(status) || !std::filesystem::is_regular_file(status)) {
    throw std::invalid_argument("onboard: runtime must be a regular non-symlink file: '"
                                + source.string() + "'");
  }
  const auto permissions = status.permissions();
  const auto executable = std::filesystem::perms::owner_exec | std::filesystem::perms::group_exec
                          | std::filesystem::perms::others_exec;
  if ((permissions & executable) == std::filesystem::perms::none) {
    throw std::invalid_argument("onboard: runtime file is not executable: '" + source.string()
                                + "'");
  }
}

void write_runtime_receipt(const std::filesystem::path& path,
                           const DeploymentPackage& package,
                           const std::string& runtime_sha256) {
  std::ostringstream receipt;
  receipt << "format=galata-onboard-runtime-package-v1\n"
          << "qualification_state=not_qualified\n"
          << "contains_executable=true\n"
          << "manifest_sha256=" << package.manifest_sha256 << "\n"
          << "runtime_path=bin/galata\n"
          << "runtime_sha256=" << runtime_sha256 << "\n";
  for (std::size_t index = 0; index < package.artifacts.size(); ++index) {
    receipt << "artifact." << index << ".role=" << package.artifacts[index].role << "\n"
            << "artifact." << index << ".sha256=" << package.artifacts[index].sha256 << "\n";
  }
  write_bytes(path, receipt.str());
}

}  // namespace

DeploymentReceipt stage_runtime_package(const DeploymentPackage& package,
                                        const ArtifactFile& runtime,
                                        const std::vector<ArtifactFile>& files,
                                        const std::filesystem::path& destination) {
  verify_manifest_package(package);
  validate_runtime_source(runtime.source);
  if (destination.empty() || destination.filename() == "." || destination.filename() == "..") {
    throw std::invalid_argument("onboard: deployment destination must be a named new directory");
  }
  if (std::filesystem::exists(destination)) {
    throw std::invalid_argument(
        "onboard: deployment destination already exists; refusing to overwrite it");
  }
  if (files.size() != package.artifacts.size()) {
    throw std::invalid_argument(
        "onboard: one source file is required for every manifest artifact identity");
  }

  std::unordered_set<std::string> supplied_roles;
  std::vector<std::pair<ArtifactReference, std::filesystem::path>> verified;
  verified.reserve(package.artifacts.size());
  for (const ArtifactReference& artifact : package.artifacts) {
    const auto found = std::find_if(files.begin(), files.end(), [&](const ArtifactFile& file) {
      return file.role == artifact.role;
    });
    if (found == files.end() || !supplied_roles.insert(found->role).second) {
      throw std::invalid_argument("onboard: source files do not match artifact roles");
    }
    if (!std::filesystem::is_regular_file(found->source)
        || std::filesystem::is_symlink(found->source)) {
      throw std::invalid_argument("onboard: artifact source must be a regular non-symlink file: '"
                                  + found->source.string() + "'");
    }
    if (core::sha256(read_bytes(found->source)) != artifact.sha256) {
      throw std::invalid_argument("onboard: artifact hash does not match role '" + artifact.role
                                  + "'");
    }
    verified.emplace_back(artifact, found->source);
  }

  const std::string runtime_sha256 = core::sha256(read_bytes(runtime.source));
  const std::filesystem::path parent = destination.parent_path().empty()
                                           ? std::filesystem::current_path()
                                           : destination.parent_path();
  std::filesystem::create_directories(parent);
  const std::filesystem::path staging =
      parent
      / (destination.filename().string() + ".staging-" + package.manifest_sha256.substr(0, 16));
  if (std::filesystem::exists(staging)) {
    throw std::invalid_argument("onboard: a previous staging directory already exists");
  }

  try {
    std::filesystem::create_directories(staging / "artifacts");
    std::filesystem::create_directories(staging / "bin");
    write_bytes(staging / "onboard.manifest", package.manifest);
    write_bytes(staging / "onboard.manifest.sha256",
                package.manifest_sha256 + "  onboard.manifest\n");
    for (const auto& [artifact, source] : verified) {
      const auto target = staging / "artifacts" / artifact.role;
      std::filesystem::copy_file(source, target, std::filesystem::copy_options::none);
      if (core::sha256(read_bytes(target)) != artifact.sha256) {
        throw std::runtime_error("onboard: staged artifact changed during publication for role '"
                                 + artifact.role + "'");
      }
    }
    const auto runtime_target = staging / "bin" / "galata";
    std::filesystem::copy_file(runtime.source, runtime_target, std::filesystem::copy_options::none);
    if (core::sha256(read_bytes(runtime_target)) != runtime_sha256) {
      throw std::runtime_error("onboard: staged runtime changed during publication");
    }
    std::filesystem::permissions(runtime_target,
                                 std::filesystem::status(runtime.source).permissions(),
                                 std::filesystem::perm_options::replace);
    if (core::sha256(read_bytes(staging / "onboard.manifest")) != package.manifest_sha256) {
      throw std::runtime_error("onboard: staged manifest changed during publication");
    }
    write_runtime_receipt(staging / "deployment.receipt", package, runtime_sha256);
    std::filesystem::rename(staging, destination);
  } catch (...) {
    std::error_code error;
    std::filesystem::remove_all(staging, error);
    throw;
  }

  DeploymentReceipt receipt;
  receipt.destination = destination;
  receipt.manifest_sha256 = package.manifest_sha256;
  receipt.runtime_sha256 = runtime_sha256;
  receipt.contains_executable = true;
  receipt.qualification_state = "not_qualified";
  for (const ArtifactReference& artifact : package.artifacts) {
    receipt.artifact_sha256.push_back(artifact.sha256);
  }
  return receipt;
}

DeploymentReceipt verify_runtime_package(const std::filesystem::path& destination) {
  if (destination.empty() || std::filesystem::is_symlink(destination)
      || !std::filesystem::is_directory(destination)) {
    throw std::invalid_argument("onboard: runtime package must be a directory: '"
                                + destination.string() + "'");
  }
  const auto require_regular_file = [](const std::filesystem::path& path) {
    const auto status = std::filesystem::symlink_status(path);
    if (std::filesystem::is_symlink(status) || !std::filesystem::is_regular_file(status)) {
      throw std::invalid_argument(
          "onboard: runtime package file is not a regular non-symlink file: '" + path.string()
          + "'");
    }
  };
  reject_symlink_components(destination, "onboard.manifest");
  reject_symlink_components(destination, "onboard.manifest.sha256");
  reject_symlink_components(destination, "deployment.receipt");
  reject_symlink_components(destination, "bin/galata");
  require_regular_file(destination / "onboard.manifest");
  require_regular_file(destination / "onboard.manifest.sha256");
  require_regular_file(destination / "deployment.receipt");
  const auto package = parse_manifest_package(read_bytes(destination / "onboard.manifest"));
  if (read_bytes(destination / "onboard.manifest.sha256")
      != package.manifest_sha256 + "  onboard.manifest\n") {
    throw std::invalid_argument("onboard: staged manifest checksum file does not match its bytes");
  }
  const auto receipt_fields = manifest_fields(read_bytes(destination / "deployment.receipt"));
  const auto required = [&receipt_fields](const std::string& key) {
    const auto found = receipt_fields.find(key);
    if (found == receipt_fields.end() || found->second.empty()) {
      throw std::invalid_argument("onboard: runtime receipt is missing '" + key + "'");
    }
    return found->second;
  };
  if (required("format") != "galata-onboard-runtime-package-v1"
      || required("qualification_state") != "not_qualified"
      || required("contains_executable") != "true" || required("runtime_path") != "bin/galata"
      || required("manifest_sha256") != package.manifest_sha256) {
    throw std::invalid_argument("onboard: runtime receipt has invalid safety or identity fields");
  }

  const auto runtime_path = destination / "bin" / "galata";
  validate_runtime_source(runtime_path);
  const std::string runtime_sha256 = core::sha256(read_bytes(runtime_path));
  if (runtime_sha256 != required("runtime_sha256")) {
    throw std::invalid_argument("onboard: runtime receipt hash does not match bin/galata");
  }

  const auto receipt_artifacts = manifest_artifacts(receipt_fields);
  std::map<std::string, std::string> package_artifacts;
  for (const ArtifactReference& artifact : package.artifacts) {
    package_artifacts.emplace(artifact.role, artifact.sha256);
    const auto artifact_path = destination / "artifacts" / artifact.role;
    reject_symlink_components(destination, std::filesystem::path("artifacts") / artifact.role);
    require_regular_file(artifact_path);
    if (core::sha256(read_bytes(artifact_path)) != artifact.sha256) {
      throw std::invalid_argument("onboard: staged artifact does not match its manifest identity: '"
                                  + artifact.role + "'");
    }
  }
  if (receipt_artifacts != package_artifacts) {
    throw std::invalid_argument("onboard: runtime receipt artifacts do not match the manifest");
  }

  std::set<std::string> actual_files;
  std::filesystem::recursive_directory_iterator iterator(destination);
  const std::filesystem::recursive_directory_iterator end;
  for (; iterator != end; ++iterator) {
    const auto relative = iterator->path().lexically_relative(destination);
    const auto status = std::filesystem::symlink_status(iterator->path());
    if (std::filesystem::is_symlink(status)) {
      throw std::invalid_argument("onboard: runtime package contains a symlink: '"
                                  + iterator->path().string() + "'");
    }
    if (std::filesystem::is_directory(status)) {
      if (relative != std::filesystem::path("bin")
          && relative != std::filesystem::path("artifacts")) {
        throw std::invalid_argument("onboard: runtime package contains an unexpected directory: '"
                                    + relative.string() + "'");
      }
      continue;
    }
    if (!std::filesystem::is_regular_file(status)) {
      throw std::invalid_argument("onboard: runtime package contains a non-regular entry: '"
                                  + iterator->path().string() + "'");
    }
    actual_files.insert(relative.generic_string());
  }
  std::set<std::string> expected_files = {
      "onboard.manifest", "onboard.manifest.sha256", "deployment.receipt", "bin/galata"};
  for (const ArtifactReference& artifact : package.artifacts) {
    expected_files.insert((std::filesystem::path("artifacts") / artifact.role).generic_string());
  }
  if (actual_files != expected_files) {
    throw std::invalid_argument(
        "onboard: runtime package inventory contains unexpected or missing files");
  }

  DeploymentReceipt receipt;
  receipt.destination = destination;
  receipt.manifest_sha256 = package.manifest_sha256;
  receipt.runtime_sha256 = runtime_sha256;
  receipt.contains_executable = true;
  receipt.qualification_state = "not_qualified";
  for (const ArtifactReference& artifact : package.artifacts) {
    receipt.artifact_sha256.push_back(artifact.sha256);
  }
  return receipt;
}

}  // namespace galata::onboard
