// SPDX-License-Identifier: Apache-2.0
#include "galata/identify/flight_test_campaign.hpp"

#include "galata/core/sha256.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace galata::identify {
namespace {

constexpr std::uintmax_t kMaximumCampaignFileBytes = 256U * 1024U * 1024U;

bool is_sha256(const std::string& value) {
  if (value.size() != 64U) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isxdigit(character) != 0;
  });
}

bool is_evidence_class(const std::string& value) {
  return value == "measured_flight" || value == "public_deidentified"
         || value == "synthetic_contract";
}

bool same_digest(const std::string& left, const std::string& right) {
  if (left.size() != right.size()) {
    return false;
  }
  return std::equal(
      left.begin(), left.end(), right.begin(), [](char left_character, char right_character) {
        return std::tolower(static_cast<unsigned char>(left_character))
               == std::tolower(static_cast<unsigned char>(right_character));
      });
}

void validate_text(const std::string& value, const std::string& field) {
  if (value.empty()) {
    throw std::invalid_argument("flight-test campaign: " + field + " is empty");
  }
  for (const char raw_character : value) {
    const unsigned char character = static_cast<unsigned char>(raw_character);
    if (character < 0x20U || character == 0x7fU) {
      throw std::invalid_argument("flight-test campaign: " + field
                                  + " contains a control character");
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
    throw std::invalid_argument("flight-test campaign: file index is not decimal");
  }
  std::size_t index = 0;
  const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), index);
  if (error != std::errc() || end != value.data() + value.size()) {
    throw std::invalid_argument("flight-test campaign: file index is out of range");
  }
  return index;
}

std::string required_field(const std::map<std::string, std::string>& fields,
                           const std::string& key) {
  const auto found = fields.find(key);
  if (found == fields.end()) {
    throw std::invalid_argument("flight-test campaign: missing required field '" + key + "'");
  }
  validate_text(found->second, key);
  return found->second;
}

std::string read_bounded_file(const std::filesystem::path& path, std::uintmax_t& byte_count) {
  const std::filesystem::file_status status = std::filesystem::symlink_status(path);
  if (std::filesystem::is_symlink(status) || !std::filesystem::is_regular_file(status)) {
    throw std::invalid_argument(
        "flight-test campaign: evidence file must be a regular "
        "non-symlink file: "
        + path.string());
  }
  std::error_code error;
  const std::uintmax_t size = std::filesystem::file_size(path, error);
  if (error || size > kMaximumCampaignFileBytes) {
    throw std::invalid_argument(
        "flight-test campaign: evidence file is missing or exceeds the "
        "256 MiB limit: "
        + path.string());
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::invalid_argument("flight-test campaign: cannot open evidence file: "
                                + path.string());
  }
  std::string bytes(static_cast<std::size_t>(size), '\0');
  input.read(bytes.data(), static_cast<std::streamsize>(size));
  if (input.bad() || static_cast<std::uintmax_t>(input.gcount()) != size
      || input.peek() != std::char_traits<char>::eof()) {
    throw std::runtime_error("flight-test campaign: evidence file changed or could not be read: "
                             + path.string());
  }
  byte_count = size;
  return bytes;
}

void reject_symlink_components(const std::filesystem::path& root,
                               const std::filesystem::path& relative) {
  const std::filesystem::file_status root_status = std::filesystem::symlink_status(root);
  if (std::filesystem::is_symlink(root_status) || !std::filesystem::is_directory(root_status)) {
    throw std::invalid_argument("flight-test campaign: package root must be a real directory: "
                                + root.string());
  }
  std::filesystem::path current = root;
  for (const auto& component : relative) {
    current /= component;
    const std::filesystem::file_status status = std::filesystem::symlink_status(current);
    if (std::filesystem::is_symlink(status)) {
      throw std::invalid_argument("flight-test campaign: symlink path component is refused: "
                                  + current.string());
    }
  }
}

}  // namespace

FlightTestCampaign parse_flight_test_campaign(const std::string& manifest) {
  FlightTestCampaign campaign;
  campaign.manifest = manifest;
  campaign.manifest_sha256 = core::sha256(manifest);

  std::map<std::string, std::string> fields;
  std::map<std::size_t, std::map<std::string, std::string>> file_fields;
  std::istringstream lines(manifest);
  std::string line;
  while (std::getline(lines, line)) {
    const std::size_t separator = line.find('=');
    if (separator == std::string::npos || separator == 0U) {
      throw std::invalid_argument("flight-test campaign: malformed manifest field");
    }
    const std::string key = line.substr(0, separator);
    const std::string value = line.substr(separator + 1U);
    validate_text(value, "manifest field " + key);
    if (key.starts_with("file.")) {
      const std::size_t field_separator = key.find('.', 5U);
      if (field_separator == std::string::npos || field_separator == 5U) {
        throw std::invalid_argument("flight-test campaign: malformed file field '" + key + "'");
      }
      const std::size_t index = parse_index(key.substr(5U, field_separator - 5U));
      const std::string field = key.substr(field_separator + 1U);
      if (field != "role" && field != "path" && field != "sha256") {
        throw std::invalid_argument("flight-test campaign: unknown file field '" + key + "'");
      }
      if (!file_fields[index].emplace(field, value).second) {
        throw std::invalid_argument("flight-test campaign: duplicate file field '" + key + "'");
      }
    } else if (!fields.emplace(key, value).second) {
      throw std::invalid_argument("flight-test campaign: duplicate field '" + key + "'");
    }
  }

  const std::set<std::string> allowed_fields = {"format",
                                                "evidence_class",
                                                "aircraft_id",
                                                "aircraft_configuration",
                                                "test_plan_id",
                                                "reviewer_id",
                                                "safety_review_complete"};
  for (const auto& [key, value] : fields) {
    (void)value;
    if (allowed_fields.count(key) == 0U) {
      throw std::invalid_argument("flight-test campaign: unknown field '" + key + "'");
    }
  }

  if (required_field(fields, "format") != "galata-flight-test-evidence-v2") {
    throw std::invalid_argument("flight-test campaign: unsupported format");
  }
  campaign.evidence.evidence_class = required_field(fields, "evidence_class");
  if (!is_evidence_class(campaign.evidence.evidence_class)) {
    throw std::invalid_argument(
        "flight-test campaign: evidence_class must be measured_flight, "
        "public_deidentified or synthetic_contract");
  }
  campaign.evidence.aircraft_id = required_field(fields, "aircraft_id");
  campaign.evidence.aircraft_configuration = required_field(fields, "aircraft_configuration");
  campaign.evidence.test_plan_id = required_field(fields, "test_plan_id");
  campaign.evidence.reviewer_id = required_field(fields, "reviewer_id");
  const std::string safety_review = required_field(fields, "safety_review_complete");
  if (safety_review != "true" && safety_review != "false") {
    throw std::invalid_argument(
        "flight-test campaign: safety_review_complete must be true or false");
  }
  campaign.evidence.safety_review_complete = safety_review == "true";

  const std::set<std::string> required_roles = {"test_plan",
                                                "flight_record",
                                                "calibration_manifest",
                                                "configuration_manifest",
                                                "reviewer_attestation"};
  if (file_fields.empty()) {
    throw std::invalid_argument("flight-test campaign: no evidence files were declared");
  }
  std::set<std::string> roles;
  for (std::size_t expected_index = 0; expected_index < file_fields.size(); ++expected_index) {
    const auto found = file_fields.find(expected_index);
    if (found == file_fields.end()) {
      throw std::invalid_argument("flight-test campaign: file indices must be contiguous from 0");
    }
    const auto& fields_for_file = found->second;
    const auto get_file_field = [&](const std::string& name) {
      const auto field = fields_for_file.find(name);
      if (field == fields_for_file.end()) {
        throw std::invalid_argument("flight-test campaign: evidence file is missing " + name);
      }
      return field->second;
    };
    const std::string role = get_file_field("role");
    const std::string path = get_file_field("path");
    const std::string sha256 = get_file_field("sha256");
    if (!roles.insert(role).second || required_roles.count(role) == 0U) {
      throw std::invalid_argument(
          "flight-test campaign: evidence roles must be the five "
          "required unique roles");
    }
    if (!is_safe_relative_path(path)) {
      throw std::invalid_argument(
          "flight-test campaign: evidence path must be relative and "
          "must not contain '.' or '..': "
          + path);
    }
    if (!is_sha256(sha256)) {
      throw std::invalid_argument(
          "flight-test campaign: evidence file has an invalid SHA-256 "
          "digest");
    }
    campaign.files.push_back({role, path, sha256});
  }
  if (roles != required_roles) {
    throw std::invalid_argument(
        "flight-test campaign: all five required evidence roles must be "
        "present exactly once");
  }

  const auto digest_for_role = [&](const std::string& role) -> const std::string& {
    const auto found = std::find_if(
        campaign.files.begin(), campaign.files.end(), [&](const FlightTestCampaignFile& file) {
          return file.role == role;
        });
    return found->sha256;
  };
  campaign.evidence.calibration_manifest_sha256 = digest_for_role("calibration_manifest");
  campaign.evidence.configuration_manifest_sha256 = digest_for_role("configuration_manifest");
  campaign.evidence.reviewer_attestation_sha256 = digest_for_role("reviewer_attestation");
  return campaign;
}

std::uintmax_t verify_flight_test_campaign(const FlightTestCampaign& campaign,
                                           const std::filesystem::path& package_root) {
  if (campaign.manifest.empty()
      || !same_digest(campaign.manifest_sha256, core::sha256(campaign.manifest))) {
    throw std::invalid_argument("flight-test campaign: manifest hash does not match its bytes");
  }
  if (!campaign.evidence.safety_review_complete) {
    throw std::invalid_argument(
        "flight-test campaign: safety_review_complete must be true for a verified campaign");
  }
  if (campaign.files.size() != 5U) {
    throw std::invalid_argument(
        "flight-test campaign: exactly five required evidence files are expected");
  }

  std::uintmax_t total_bytes = 0U;
  for (const FlightTestCampaignFile& file : campaign.files) {
    const std::filesystem::path relative(file.relative_path);
    reject_symlink_components(package_root, relative);
    const std::filesystem::path path = package_root / relative;
    std::uintmax_t byte_count = 0U;
    const std::string bytes = read_bounded_file(path, byte_count);
    const std::string actual_sha256 = core::sha256(bytes);
    if (!same_digest(actual_sha256, file.sha256)) {
      throw std::invalid_argument("flight-test campaign: SHA-256 mismatch for role '" + file.role
                                  + "' at " + file.relative_path);
    }
    total_bytes += byte_count;
  }
  return total_bytes;
}

}  // namespace galata::identify
