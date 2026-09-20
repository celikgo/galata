// SPDX-License-Identifier: Apache-2.0
#include "galata/qualification/dossier.hpp"

#include "galata/core/sha256.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace galata::qualification {
namespace {

constexpr std::uintmax_t kMaximumDossierFileBytes = 256U * 1024U * 1024U;
const std::set<std::string> kRequiredRoles = {
    "requirements_matrix",
    "software_release",
    "verification_report",
    "flight_test_campaign",
    "hardware_hil_report",
    "safety_case",
    "independent_review",
    "authority_decision",
    "maintenance_plan",
};

bool is_sha256(const std::string& value) {
  if (value.size() != 64U) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isxdigit(character) != 0;
  });
}

void validate_text(const std::string& value, const std::string& field) {
  if (value.empty()) {
    throw std::invalid_argument("qualification dossier: " + field + " is empty");
  }
  for (const char raw_character : value) {
    const unsigned char character = static_cast<unsigned char>(raw_character);
    if (character < 0x20U || character == 0x7fU || character == '=') {
      throw std::invalid_argument("qualification dossier: " + field
                                  + " contains a control or separator character");
    }
  }
}

bool is_safe_relative_path(const std::string& value) {
  const std::filesystem::path path(value);
  if (value.empty() || path.empty() || path.is_absolute()) {
    return false;
  }
  for (const auto& component : path) {
    if (component == "." || component == ".." || component.empty()) {
      return false;
    }
  }
  return true;
}

std::size_t parse_index(const std::string& value) {
  if (value.empty() || !std::all_of(value.begin(), value.end(), [](unsigned char character) {
        return std::isdigit(character) != 0;
      })) {
    throw std::invalid_argument("qualification dossier: file index is not decimal");
  }
  std::size_t index = 0;
  const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), index);
  if (error != std::errc() || end != value.data() + value.size()) {
    throw std::invalid_argument("qualification dossier: file index is out of range");
  }
  return index;
}

std::string required_field(const std::map<std::string, std::string>& fields,
                           const std::string& key) {
  const auto found = fields.find(key);
  if (found == fields.end()) {
    throw std::invalid_argument("qualification dossier: missing required field '" + key + "'");
  }
  validate_text(found->second, key);
  return found->second;
}

std::string read_bounded_file(const std::filesystem::path& path, std::uintmax_t& byte_count) {
  const std::filesystem::file_status status = std::filesystem::symlink_status(path);
  if (std::filesystem::is_symlink(status) || !std::filesystem::is_regular_file(status)) {
    throw std::invalid_argument(
        "qualification dossier: evidence file must be a regular non-symlink file: "
        + path.string());
  }
  std::error_code error;
  const std::uintmax_t size = std::filesystem::file_size(path, error);
  if (error || size > kMaximumDossierFileBytes) {
    throw std::invalid_argument(
        "qualification dossier: evidence file is missing or exceeds the 256 MiB limit: "
        + path.string());
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::invalid_argument("qualification dossier: cannot open evidence file: "
                                + path.string());
  }
  std::string bytes(static_cast<std::size_t>(size), '\0');
  input.read(bytes.data(), static_cast<std::streamsize>(size));
  if (input.bad() || static_cast<std::uintmax_t>(input.gcount()) != size
      || input.peek() != std::char_traits<char>::eof()) {
    throw std::runtime_error("qualification dossier: evidence file changed or could not be read: "
                             + path.string());
  }
  byte_count = size;
  return bytes;
}

void reject_symlink_components(const std::filesystem::path& root,
                               const std::filesystem::path& relative) {
  const std::filesystem::file_status root_status = std::filesystem::symlink_status(root);
  if (std::filesystem::is_symlink(root_status) || !std::filesystem::is_directory(root_status)) {
    throw std::invalid_argument("qualification dossier: package root must be a real directory: "
                                + root.string());
  }
  std::filesystem::path current = root;
  for (const auto& component : relative) {
    current /= component;
    const std::filesystem::file_status status = std::filesystem::symlink_status(current);
    if (std::filesystem::is_symlink(status)) {
      throw std::invalid_argument("qualification dossier: symlink path component is refused: "
                                  + current.string());
    }
  }
}

}  // namespace

Dossier parse_dossier(const std::string& manifest) {
  Dossier dossier;
  dossier.manifest = manifest;
  dossier.manifest_sha256 = core::sha256(manifest);

  std::map<std::string, std::string> fields;
  std::map<std::size_t, std::map<std::string, std::string>> file_fields;
  std::istringstream lines(manifest);
  std::string line;
  while (std::getline(lines, line)) {
    const std::size_t separator = line.find('=');
    if (separator == std::string::npos || separator == 0U) {
      throw std::invalid_argument("qualification dossier: malformed manifest field");
    }
    const std::string key = line.substr(0, separator);
    const std::string value = line.substr(separator + 1U);
    validate_text(key, "manifest key");
    validate_text(value, "manifest field " + key);
    if (key.starts_with("file.")) {
      const std::size_t field_separator = key.find('.', 5U);
      if (field_separator == std::string::npos || field_separator == 5U) {
        throw std::invalid_argument("qualification dossier: malformed file field '" + key + "'");
      }
      const std::size_t index = parse_index(key.substr(5U, field_separator - 5U));
      const std::string field = key.substr(field_separator + 1U);
      if (field != "role" && field != "path" && field != "sha256") {
        throw std::invalid_argument("qualification dossier: unknown file field '" + key + "'");
      }
      if (!file_fields[index].emplace(field, value).second) {
        throw std::invalid_argument("qualification dossier: duplicate file field '" + key + "'");
      }
    } else if (!fields.emplace(key, value).second) {
      throw std::invalid_argument("qualification dossier: duplicate field '" + key + "'");
    }
  }

  const std::set<std::string> allowed_fields = {
      "format",
      "product_id",
      "product_version",
      "intended_use",
      "aircraft_id",
      "aircraft_configuration",
      "qualification_basis",
      "authority_id",
      "acceptance_state",
  };
  for (const auto& [key, value] : fields) {
    (void)value;
    if (allowed_fields.count(key) == 0U) {
      throw std::invalid_argument("qualification dossier: unknown field '" + key + "'");
    }
  }

  if (required_field(fields, "format") != "galata-qualification-evidence-v1") {
    throw std::invalid_argument("qualification dossier: unsupported format");
  }
  if (required_field(fields, "acceptance_state") != "not_qualified") {
    throw std::invalid_argument(
        "qualification dossier: acceptance_state must remain not_qualified until external "
        "acceptance is independently established");
  }
  dossier.product_id = required_field(fields, "product_id");
  dossier.product_version = required_field(fields, "product_version");
  dossier.intended_use = required_field(fields, "intended_use");
  dossier.aircraft_id = required_field(fields, "aircraft_id");
  dossier.aircraft_configuration = required_field(fields, "aircraft_configuration");
  dossier.qualification_basis = required_field(fields, "qualification_basis");
  dossier.authority_id = required_field(fields, "authority_id");

  if (file_fields.size() != kRequiredRoles.size()) {
    throw std::invalid_argument(
        "qualification dossier: exactly one file is required for every "
        "qualification evidence role");
  }
  std::set<std::string> roles;
  std::set<std::string> paths;
  for (std::size_t expected_index = 0; expected_index < file_fields.size(); ++expected_index) {
    const auto found = file_fields.find(expected_index);
    if (found == file_fields.end()) {
      throw std::invalid_argument("qualification dossier: file indices must be contiguous from 0");
    }
    const auto& fields_for_file = found->second;
    if (fields_for_file.size() != 3U || !fields_for_file.contains("role")
        || !fields_for_file.contains("path") || !fields_for_file.contains("sha256")) {
      throw std::invalid_argument("qualification dossier: every file needs role, path and sha256");
    }
    const std::string role = fields_for_file.at("role");
    const std::string path = fields_for_file.at("path");
    const std::string sha256 = fields_for_file.at("sha256");
    if (kRequiredRoles.count(role) == 0U || !roles.insert(role).second) {
      throw std::invalid_argument("qualification dossier: missing or duplicate required role '"
                                  + role + "'");
    }
    if (!is_safe_relative_path(path) || !paths.insert(path).second) {
      throw std::invalid_argument(
          "qualification dossier: file paths must be unique safe relative "
          "paths");
    }
    if (!is_sha256(sha256)) {
      throw std::invalid_argument("qualification dossier: invalid SHA-256 for role '" + role + "'");
    }
    dossier.files.push_back({role, path, sha256});
  }
  if (roles != kRequiredRoles) {
    throw std::invalid_argument(
        "qualification dossier: all required evidence roles must be present");
  }
  return dossier;
}

std::uintmax_t verify_dossier(const Dossier& dossier, const std::filesystem::path& package_root) {
  if (dossier.manifest.empty() || !is_sha256(dossier.manifest_sha256)
      || dossier.manifest_sha256 != core::sha256(dossier.manifest)) {
    throw std::invalid_argument("qualification dossier: manifest hash does not match its bytes");
  }
  const Dossier parsed = parse_dossier(dossier.manifest);
  if (parsed.manifest_sha256 != dossier.manifest_sha256 || parsed.files != dossier.files
      || parsed.product_id != dossier.product_id
      || parsed.product_version != dossier.product_version
      || parsed.intended_use != dossier.intended_use || parsed.aircraft_id != dossier.aircraft_id
      || parsed.aircraft_configuration != dossier.aircraft_configuration
      || parsed.qualification_basis != dossier.qualification_basis
      || parsed.authority_id != dossier.authority_id) {
    throw std::invalid_argument(
        "qualification dossier: parsed manifest does not match its package");
  }
  std::uintmax_t total_bytes = 0;
  for (const DossierFile& file : dossier.files) {
    const std::filesystem::path relative(file.relative_path);
    reject_symlink_components(package_root, relative);
    std::uintmax_t byte_count = 0;
    const std::string bytes = read_bounded_file(package_root / relative, byte_count);
    if (core::sha256(bytes) != file.sha256) {
      throw std::invalid_argument("qualification dossier: evidence hash does not match role '"
                                  + file.role + "'");
    }
    total_bytes += byte_count;
  }
  return total_bytes;
}

}  // namespace galata::qualification
