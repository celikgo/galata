// SPDX-License-Identifier: Apache-2.0
//
// The galata command-line interface.
//
// Everything the desktop application will do, this does first, through the same
// pipeline document. That ordering is deliberate: a capability reachable only
// through a graphical surface is a capability that cannot be run in CI, cannot
// be scripted, and cannot be reproduced by a reader of a paper.

#include "galata/core/sha256.hpp"
#include "galata/hardware/can_transport.hpp"
#include "galata/hardware/serial_transport.hpp"
#include "galata/hardware/udp_transport.hpp"
#include "galata/identify/flight_test_campaign.hpp"
#include "galata/onboard/controller_plugin.hpp"
#include "galata/onboard/deployment.hpp"
#include "galata/onboard/runtime.hpp"
#include "galata/onboard/target_evidence.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/pipeline.hpp"
#include "galata/pipeline/registry.hpp"
#include "galata/qualification/dossier.hpp"
#include "galata/version.hpp"

#include "project.hpp"
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <any>
#include <atomic>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace {

int print_usage(std::ostream& out) {
  out << "galata — flight dynamics, control-law design and simulation\n"
         "\n"
         "usage:\n"
         "  galata run <pipeline.yaml> [--output-dir <dir>] [--overwrite] [--json]\n"
         "  galata capabilities [--markdown|--json]\n"
         "  galata onboard verify <manifest>\n"
         "  galata onboard stage <manifest> <new-directory> --artifact <role=path>...\n"
         "  galata onboard deploy <manifest> <new-directory> --runtime <path>\n"
         "      --artifact <role=path>...\n"
         "  galata onboard verify-deployment <directory>\n"
         "  galata onboard target create <new-directory> <deployment.manifest>\n"
         "      --evidence-class <host_sil|target_hil|flight_target>\n"
         "      --controller-worst-case-s <s> --cycle-worst-case-s <s>\n"
         "      --watchdog-response-s <s> --emergency-stop-passed true\n"
         "      --loss-of-link-passed true --hil-passed true --signing-verified true\n"
         "      --file <role=path>...\n"
         "  galata onboard target verify <deployment.manifest> <target-evidence.manifest>\n"
         "  galata onboard run <manifest> --model <path> --controller <path> --operator <id>\n"
         "      --confirm ARM --cycles <count> [--linger-ms <count>]\n"
         "  galata onboard self-test\n"
         "  galata flighttest create <new-directory> --aircraft-id <id>\n"
         "      --configuration <id> --evidence-class <class> --test-plan-id <id>\n"
         "      --reviewer-id <id>\n"
         "      --safety-review-complete true --file <role=path>...\n"
         "  galata flighttest verify <campaign.manifest>\n"
         "  galata flighttest validate <campaign.manifest> <study.yaml>\n"
         "      --output-dir <dir> [--receipt <path>] [--overwrite]\n"
         "  galata qualification create <new-directory> --product-id <id>\n"
         "      --product-version <version> --intended-use <use> --aircraft-id <id>\n"
         "      --configuration <id> --qualification-basis <basis> --authority-id <id>\n"
         "      --file <role=path>...\n"
         "  galata qualification verify <dossier.manifest>\n"
         "  galata qualification verify-chain <dossier.manifest>\n"
         "      --flighttest <campaign.manifest> --flight-validation-receipt <receipt.json>\n"
         "      --deployment <deployment.manifest> --deployment-dir <runtime-package>\n"
         "      --target-evidence <target-evidence.manifest>\n"
         "  galata project <create|inspect|save|run|export|verify> <directory> [options]\n"
         "  galata project import-linear <new-directory> <study.yaml>\n"
         "  galata project revisions <directory>\n"
         "  galata project revision <directory> <revision>\n"
         "  galata project restore <directory> <revision> --expected-revision <current>\n"
         "  galata --version\n"
         "  galata --help\n"
         "\n"
         "  run           execute a pipeline, streaming each stage as it completes\n"
         "  capabilities  list what this build can do, and how far each has been checked\n"
         "\n"
         "Relative paths inside a pipeline resolve against the pipeline file's own\n"
         "directory, so a study is runnable from anywhere. Outputs go to the pipeline's\n"
         "directory unless --output-dir says otherwise; missing directories are created.\n"
         "Output paths must stay inside that directory, without symlinks or '..'.\n"
         "Existing reports are refused unless --overwrite is supplied. Each successful\n"
         "run writes an immutable run-<SHA256>.json manifest with input snapshots,\n"
         "output digests and build provenance.\n";
  return 0;
}

std::string json_quote(const std::string& text) {
  std::ostringstream result;
  result << '"';
  for (const char character : text) {
    if (character == '\\' || character == '"') {
      result << '\\';
    }
    result << character;
  }
  result << '"';
  return result.str();
}

int print_version() {
  // Single source of version truth: this string comes from the VERSION file by
  // way of the generated build_config.hpp. See ADR-0005.
  std::cout << galata::build_identification() << "\n";
  return 0;
}

// Markdown, for the README's status table.
//
// Generated rather than written by hand, and checked by CI, because charter
// rule 2 says nothing is documented before it works — and a hand-maintained
// capability table is exactly the thing that quietly starts claiming more than
// the code does.
int list_capabilities_markdown() {
  std::cout << "| Capability | What it does | Produces | State |\n";
  std::cout << "|---|---|---|---|\n";
  for (const galata::pipeline::Capability* capability :
       galata::pipeline::builtin_registry().all()) {
    std::cout << "| `" << capability->id << "` | " << capability->summary << " | `"
              << capability->produces << "` | " << galata::pipeline::to_string(capability->state)
              << " |\n";
  }
  return 0;
}

int list_capabilities() {
  const galata::pipeline::Registry& registry = galata::pipeline::builtin_registry();
  const std::vector<const galata::pipeline::Capability*> all = registry.all();

  std::cout << "galata " << galata::version_string() << " provides " << all.size()
            << " capabilities.\n\n";
  for (const galata::pipeline::Capability* capability : all) {
    std::cout << "  " << capability->id << "\n";
    std::cout << "      " << capability->summary << "\n";
    std::cout << "      produces: " << capability->produces
              << "   state: " << galata::pipeline::to_string(capability->state) << "\n\n";
  }
  std::cout << "\"implemented, unvalidated\" means the capability works and is tested, but\n"
               "its output has not been compared against a published reference. See\n"
               "docs/VERIFICATION.md for what has.\n";
  return 0;
}

int list_capabilities_json() {
  const auto all = galata::pipeline::builtin_registry().all();
  std::cout << "{\"capabilities\":[";
  for (std::size_t i = 0; i < all.size(); ++i) {
    const auto* capability = all[i];
    std::cout << (i == 0 ? "" : ",") << "{\"id\":" << json_quote(capability->id)
              << ",\"summary\":" << json_quote(capability->summary)
              << ",\"produces\":" << json_quote(capability->produces)
              << ",\"state\":" << json_quote(galata::pipeline::to_string(capability->state))
              << ",\"inputs\":[";
    for (std::size_t j = 0; j < capability->input_keys.size(); ++j) {
      std::cout << (j == 0 ? "" : ",") << json_quote(capability->input_keys[j]);
    }
    std::cout << "]}";
  }
  std::cout << "]}\n";
  return 0;
}

int run_pipeline_command(const std::vector<std::string>& arguments) {
  if (arguments.empty()) {
    std::cerr << "galata run: no pipeline file given\n";
    return 2;
  }

  std::string pipeline_path = arguments[0];
  std::string output_directory;
  galata::pipeline::RunOptions options;
  bool machine_readable = false;

  for (std::size_t i = 1; i < arguments.size(); ++i) {
    if (arguments[i] == "--output-dir") {
      if (i + 1 >= arguments.size()) {
        std::cerr << "galata run: --output-dir needs a directory\n";
        return 2;
      }
      output_directory = arguments[++i];
    } else if (arguments[i] == "--overwrite") {
      options.overwrite = true;
    } else if (arguments[i] == "--json") {
      machine_readable = true;
    } else {
      std::cerr << "galata run: unrecognised argument '" << arguments[i] << "'\n";
      return 2;
    }
  }

  try {
    const std::filesystem::path path(pipeline_path);
    if (!std::filesystem::exists(path)) {
      std::cerr << "galata run: no such file: " << pipeline_path << "\n";
      return 1;
    }
    const std::string base_directory =
        path.has_parent_path() ? path.parent_path().string() : std::string(".");
    if (output_directory.empty()) {
      output_directory = base_directory;
    }

    const galata::pipeline::Pipeline pipeline = galata::pipeline::load_pipeline(pipeline_path);
    if (!machine_readable) {
      std::cout << "galata " << galata::version_string() << " — running " << pipeline_path << " ("
                << pipeline.stages.size() << " stages)\n\n";
    }

    // One line per stage, printed when the stage finishes.
    //
    // Not a carriage-return spinner: this output is redirected to a file or a
    // CI log at least as often as it is watched in a terminal, and a \r that
    // nothing consumes leaves both halves of every line in the transcript.
    const auto progress = [machine_readable](const std::string& stage_id,
                                             const std::string& capability,
                                             bool finished,
                                             const std::string& summary) {
      if (!finished) {
        return;
      }
      if (!machine_readable) {
        std::cout << "  " << stage_id << "  [" << capability << "]  " << summary << "\n"
                  << std::flush;
      }
    };

    const galata::pipeline::RunResult result =
        galata::pipeline::run_pipeline(pipeline,
                                       galata::pipeline::builtin_registry(),
                                       base_directory,
                                       output_directory,
                                       progress,
                                       options);

    if (machine_readable) {
      std::cout << "{\"status\":\"completed\",\"stage_count\":" << result.stages.size()
                << ",\"manifest_path\":" << json_quote(result.manifest_path) << "}\n";
    } else {
      std::cout << "\n" << result.stages.size() << " stages completed.\n";
    }
    if (!machine_readable && !result.manifest_path.empty()) {
      std::cout << "Run manifest: " << result.manifest_path << "\n";
    }
    return 0;
  } catch (const std::exception& error) {
    // The message already names the stage and capability; adding a prefix here
    // would only push the useful part further right.
    std::cerr << "\ngalata: " << error.what() << "\n";
    return 1;
  }
}

constexpr std::uintmax_t kOnboardManifestBytes = 2U * 1024U * 1024U;

std::string read_bounded_manifest(const std::filesystem::path& path) {
  const std::filesystem::file_status status = std::filesystem::symlink_status(path);
  if (std::filesystem::is_symlink(status) || !std::filesystem::is_regular_file(status)) {
    throw std::invalid_argument("manifest must be a regular non-symlink file: " + path.string());
  }
  std::error_code error;
  const std::uintmax_t size = std::filesystem::file_size(path, error);
  if (error || size > kOnboardManifestBytes) {
    throw std::invalid_argument("manifest is missing or exceeds the 2 MiB limit: " + path.string());
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::invalid_argument("cannot open manifest: " + path.string());
  }
  std::string bytes(static_cast<std::size_t>(size), '\0');
  input.read(bytes.data(), static_cast<std::streamsize>(size));
  if (input.bad() || static_cast<std::uintmax_t>(input.gcount()) != size
      || input.peek() != std::char_traits<char>::eof()) {
    throw std::runtime_error("manifest changed or could not be read: " + path.string());
  }
  return bytes;
}

std::string sha256_regular_file(const std::filesystem::path& path) {
  const std::filesystem::file_status status = std::filesystem::symlink_status(path);
  if (std::filesystem::is_symlink(status) || !std::filesystem::is_regular_file(status)) {
    throw std::invalid_argument("artifact must be a regular non-symlink file: " + path.string());
  }
  std::error_code error;
  constexpr std::uintmax_t kMaximumArtifactBytes = 256U * 1024U * 1024U;
  const std::uintmax_t size = std::filesystem::file_size(path, error);
  if (error || size > kMaximumArtifactBytes) {
    throw std::invalid_argument("artifact is missing or exceeds the 256 MiB limit: "
                                + path.string());
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::invalid_argument("cannot open artifact: " + path.string());
  }
  std::string bytes(static_cast<std::size_t>(size), '\0');
  input.read(bytes.data(), static_cast<std::streamsize>(size));
  if (input.bad() || static_cast<std::uintmax_t>(input.gcount()) != size
      || input.peek() != std::char_traits<char>::eof()) {
    throw std::runtime_error("artifact changed or could not be read: " + path.string());
  }
  return galata::core::sha256(bytes);
}

galata::onboard::ArtifactFile onboard_artifact_argument(const std::string& argument) {
  const std::size_t separator = argument.find('=');
  if (separator == std::string::npos || separator == 0 || separator + 1 >= argument.size()) {
    throw std::invalid_argument("--artifact expects role=path");
  }
  return {argument.substr(0, separator), argument.substr(separator + 1)};
}

struct FlightTestSourceArgument {
  std::string role;
  std::filesystem::path path;
};

FlightTestSourceArgument flight_test_source_argument(const std::string& argument) {
  const std::size_t separator = argument.find('=');
  if (separator == std::string::npos || separator == 0U || separator + 1U >= argument.size()) {
    throw std::invalid_argument("flighttest: --file expects role=path");
  }
  return {argument.substr(0, separator), argument.substr(separator + 1U)};
}

void validate_campaign_text(const std::string& value, const char* field) {
  if (value.empty()) {
    throw std::invalid_argument(std::string("flighttest: ") + field + " is required");
  }
  for (const char character : value) {
    const unsigned char code = static_cast<unsigned char>(character);
    if (code < 0x20U || code == 0x7fU) {
      throw std::invalid_argument(std::string("flighttest: ") + field
                                  + " contains a control character");
    }
  }
}

void validate_campaign_evidence_class(const std::string& value) {
  if (value != "measured_flight" && value != "public_deidentified"
      && value != "synthetic_contract") {
    throw std::invalid_argument(
        "flighttest: --evidence-class must be measured_flight, public_deidentified or "
        "synthetic_contract");
  }
}

int flight_test_create_command(const std::vector<std::string>& arguments) {
  if (arguments.size() < 2U || arguments[0] != "create") {
    std::cerr << "usage: galata flighttest create <new-directory> --aircraft-id <id>"
                 " --configuration <id> --evidence-class <class> --test-plan-id <id>"
                 " --reviewer-id <id>"
                 " --safety-review-complete true --file <role=path>...\n";
    return 2;
  }

  try {
    const std::filesystem::path destination(arguments[1]);
    std::string aircraft_id;
    std::string aircraft_configuration;
    std::string evidence_class;
    std::string test_plan_id;
    std::string reviewer_id;
    std::string safety_review_complete;
    std::vector<FlightTestSourceArgument> sources;
    for (std::size_t index = 2U; index < arguments.size();) {
      const auto read_value = [&](const char* option) {
        if (index + 1U >= arguments.size()) {
          throw std::invalid_argument(std::string("flighttest: ") + option + " requires a value");
        }
        return arguments[index + 1U];
      };
      if (arguments[index] == "--aircraft-id") {
        aircraft_id = read_value("--aircraft-id");
        index += 2U;
      } else if (arguments[index] == "--configuration") {
        aircraft_configuration = read_value("--configuration");
        index += 2U;
      } else if (arguments[index] == "--evidence-class") {
        evidence_class = read_value("--evidence-class");
        index += 2U;
      } else if (arguments[index] == "--test-plan-id") {
        test_plan_id = read_value("--test-plan-id");
        index += 2U;
      } else if (arguments[index] == "--reviewer-id") {
        reviewer_id = read_value("--reviewer-id");
        index += 2U;
      } else if (arguments[index] == "--safety-review-complete") {
        safety_review_complete = read_value("--safety-review-complete");
        index += 2U;
      } else if (arguments[index] == "--file") {
        sources.push_back(flight_test_source_argument(read_value("--file")));
        index += 2U;
      } else {
        throw std::invalid_argument("flighttest: unrecognised or incomplete option '"
                                    + arguments[index] + "'");
      }
    }
    validate_campaign_text(aircraft_id, "aircraft_id");
    validate_campaign_text(aircraft_configuration, "aircraft_configuration");
    validate_campaign_text(evidence_class, "evidence_class");
    validate_campaign_evidence_class(evidence_class);
    validate_campaign_text(test_plan_id, "test_plan_id");
    validate_campaign_text(reviewer_id, "reviewer_id");
    if (safety_review_complete != "true") {
      throw std::invalid_argument(
          "flighttest: --safety-review-complete must be explicitly set to true");
    }

    const std::vector<std::string> required_roles = {"test_plan",
                                                     "flight_record",
                                                     "calibration_manifest",
                                                     "configuration_manifest",
                                                     "reviewer_attestation"};
    if (sources.size() != required_roles.size()) {
      throw std::invalid_argument(
          "flighttest: exactly one --file is required for each of the five required roles");
    }
    std::set<std::string> seen_roles;
    for (const auto& source : sources) {
      if (std::find(required_roles.begin(), required_roles.end(), source.role)
          == required_roles.end()) {
        throw std::invalid_argument("flighttest: unknown evidence role '" + source.role + "'");
      }
      if (!seen_roles.insert(source.role).second) {
        throw std::invalid_argument("flighttest: evidence role '" + source.role
                                    + "' was supplied more than once");
      }
    }
    if (seen_roles.size() != required_roles.size()) {
      throw std::invalid_argument("flighttest: all five required evidence roles are required");
    }

    const std::filesystem::file_status destination_status =
        std::filesystem::symlink_status(destination);
    if (std::filesystem::exists(destination_status)) {
      throw std::invalid_argument("flighttest: destination already exists: "
                                  + destination.string());
    }
    const std::filesystem::path parent =
        destination.has_parent_path() ? destination.parent_path() : std::filesystem::path(".");
    std::filesystem::create_directories(parent);
    const std::filesystem::path staging = destination.string() + ".staging";
    if (std::filesystem::exists(std::filesystem::symlink_status(staging))) {
      throw std::invalid_argument("flighttest: staging destination already exists: "
                                  + staging.string());
    }

    std::error_code cleanup_error;
    try {
      std::filesystem::create_directories(staging / "evidence");
      std::vector<std::string> digests;
      digests.reserve(sources.size());
      for (const auto& source : sources) {
        const std::string digest = sha256_regular_file(source.path);
        const std::filesystem::path copied = staging / "evidence" / source.role;
        std::filesystem::copy_file(source.path, copied);
        if (sha256_regular_file(copied) != digest) {
          throw std::runtime_error("flighttest: source changed while copying role '" + source.role
                                   + "'");
        }
        digests.push_back(digest);
      }

      std::ostringstream manifest;
      manifest << "format=galata-flight-test-evidence-v2\n"
               << "aircraft_id=" << aircraft_id << "\n"
               << "aircraft_configuration=" << aircraft_configuration << "\n"
               << "evidence_class=" << evidence_class << "\n"
               << "test_plan_id=" << test_plan_id << "\n"
               << "reviewer_id=" << reviewer_id << "\n"
               << "safety_review_complete=true\n";
      for (std::size_t index = 0U; index < sources.size(); ++index) {
        manifest << "file." << index << ".role=" << sources[index].role << "\n"
                 << "file." << index << ".path=evidence/" << sources[index].role << "\n"
                 << "file." << index << ".sha256=" << digests[index] << "\n";
      }
      {
        std::ofstream output(staging / "campaign.manifest", std::ios::binary);
        if (!output) {
          throw std::runtime_error("flighttest: cannot write staged campaign manifest");
        }
        output << manifest.str();
        if (!output) {
          throw std::runtime_error("flighttest: failed while writing staged campaign manifest");
        }
      }

      const auto campaign = galata::identify::parse_flight_test_campaign(manifest.str());
      const std::uintmax_t total_bytes =
          galata::identify::verify_flight_test_campaign(campaign, staging);
      std::filesystem::rename(staging, destination);
      std::cout << "{\"schema\":\"galata.flight-test-evidence-package.v2\","
                   "\"status\":\"staged\",\"destination\":"
                << json_quote(destination.string())
                << ",\"manifest\":" << json_quote((destination / "campaign.manifest").string())
                << ",\"manifest_sha256\":" << json_quote(campaign.manifest_sha256)
                << ",\"evidence_class\":" << json_quote(campaign.evidence.evidence_class)
                << ",\"file_count\":" << campaign.files.size() << ",\"total_bytes\":" << total_bytes
                << ",\"evidence_references_verified\":true,"
                   "\"qualification_state\":\"not_qualified\","
                   "\"airworthiness_claim\":false,\"certification_claim\":false}\n";
      return 0;
    } catch (...) {
      std::filesystem::remove_all(staging, cleanup_error);
      throw;
    }
  } catch (const std::exception& error) {
    std::cerr << "galata flighttest create: " << error.what() << "\n";
    return 1;
  }
}

int flight_test_verify_command(const std::vector<std::string>& arguments) {
  if (arguments.size() != 2U || arguments[0] != "verify") {
    std::cerr << "usage: galata flighttest verify <campaign.manifest>\n";
    return 2;
  }

  try {
    const std::filesystem::path manifest_path(arguments[1]);
    const auto campaign =
        galata::identify::parse_flight_test_campaign(read_bounded_manifest(manifest_path));
    const std::filesystem::path package_root =
        manifest_path.has_parent_path() ? manifest_path.parent_path() : std::filesystem::path(".");
    const std::uintmax_t total_bytes =
        galata::identify::verify_flight_test_campaign(campaign, package_root);
    std::cout << "{\"schema\":\"galata.flight-test-evidence-verification.v2\","
                 "\"status\":\"verified\",\"manifest_sha256\":"
              << json_quote(campaign.manifest_sha256)
              << ",\"aircraft_id\":" << json_quote(campaign.evidence.aircraft_id)
              << ",\"aircraft_configuration\":"
              << json_quote(campaign.evidence.aircraft_configuration)
              << ",\"evidence_class\":" << json_quote(campaign.evidence.evidence_class)
              << ",\"test_plan_id\":" << json_quote(campaign.evidence.test_plan_id)
              << ",\"reviewer_id\":" << json_quote(campaign.evidence.reviewer_id)
              << ",\"safety_review_complete\":true,\"file_count\":" << campaign.files.size()
              << ",\"file_roles\":[";
    for (std::size_t index = 0U; index < campaign.files.size(); ++index) {
      if (index != 0U) {
        std::cout << ',';
      }
      std::cout << json_quote(campaign.files[index].role);
    }
    std::cout << "],\"total_bytes\":" << total_bytes
              << ",\"evidence_references_verified\":true,"
                 "\"qualification_state\":\"not_qualified\","
                 "\"airworthiness_claim\":false,\"certification_claim\":false}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "galata flighttest: " << error.what() << "\n";
    return 1;
  }
}

bool path_matches(const std::filesystem::path& left, const std::filesystem::path& right) {
  std::error_code left_error;
  std::error_code right_error;
  const std::filesystem::path left_canonical = std::filesystem::weakly_canonical(left, left_error);
  const std::filesystem::path right_canonical =
      std::filesystem::weakly_canonical(right, right_error);
  return !left_error && !right_error && left_canonical == right_canonical;
}

bool value_binds_campaign_manifest(const galata::pipeline::ValuePtr& value,
                                   const std::string& base_directory,
                                   const std::filesystem::path& campaign_manifest) {
  if (!value) {
    return false;
  }
  if (value->kind() == galata::pipeline::Value::Kind::Map) {
    for (const auto& [key, child] : value->as_map()) {
      if (key == "campaign_manifest" && child->kind() == galata::pipeline::Value::Kind::String) {
        const std::filesystem::path declared(child->as_string());
        const std::filesystem::path resolved =
            declared.is_absolute() ? declared : std::filesystem::path(base_directory) / declared;
        if (path_matches(resolved, campaign_manifest)) {
          return true;
        }
      }
      if (value_binds_campaign_manifest(child, base_directory, campaign_manifest)) {
        return true;
      }
    }
  } else if (value->kind() == galata::pipeline::Value::Kind::List) {
    for (const auto& child : value->as_list()) {
      if (value_binds_campaign_manifest(child, base_directory, campaign_manifest)) {
        return true;
      }
    }
  }
  return false;
}

void write_validation_receipt(const std::filesystem::path& path,
                              const std::string& bytes,
                              bool overwrite) {
  const std::filesystem::file_status status = std::filesystem::symlink_status(path);
  if (std::filesystem::is_symlink(status)) {
    throw std::invalid_argument("flighttest validate: receipt path must not be a symlink");
  }
  if (std::filesystem::exists(status) && !overwrite) {
    throw std::invalid_argument("flighttest validate: receipt already exists: " + path.string());
  }
  if (path.has_parent_path()) {
    std::error_code error;
    std::filesystem::create_directories(path.parent_path(), error);
    if (error) {
      throw std::runtime_error("flighttest validate: cannot create receipt directory: "
                               + path.parent_path().string());
    }
  }
  const std::filesystem::path temporary = path.string() + ".tmp";
  if (std::filesystem::exists(std::filesystem::symlink_status(temporary))) {
    throw std::invalid_argument("flighttest validate: temporary receipt already exists: "
                                + temporary.string());
  }
  {
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output) {
      throw std::runtime_error("flighttest validate: cannot create receipt: " + temporary.string());
    }
    output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    if (!output) {
      std::filesystem::remove(temporary);
      throw std::runtime_error("flighttest validate: failed while writing receipt");
    }
  }
  if (overwrite) {
    std::error_code error;
    std::filesystem::remove(path, error);
    if (error) {
      std::filesystem::remove(temporary);
      throw std::runtime_error("flighttest validate: cannot replace receipt: " + path.string());
    }
  }
  std::error_code error;
  std::filesystem::rename(temporary, path, error);
  if (error) {
    std::filesystem::remove(temporary);
    throw std::runtime_error("flighttest validate: cannot publish receipt: " + path.string());
  }
}

int flight_test_validate_command(const std::vector<std::string>& arguments) {
  if (arguments.size() < 3U || arguments[0] != "validate") {
    std::cerr << "usage: galata flighttest validate <campaign.manifest> <study.yaml>"
                 " --output-dir <dir> [--receipt <path>] [--overwrite]\n";
    return 2;
  }

  try {
    const std::filesystem::path campaign_manifest(arguments[1]);
    const std::filesystem::path study_path(arguments[2]);
    std::string output_directory;
    std::filesystem::path receipt_path;
    bool overwrite = false;
    for (std::size_t index = 3U; index < arguments.size();) {
      const std::string& option = arguments[index];
      if (option == "--output-dir" || option == "--receipt") {
        if (index + 1U >= arguments.size() || arguments[index + 1U].empty()) {
          throw std::invalid_argument("flighttest validate: " + option + " needs a path");
        }
        if (option == "--output-dir") {
          output_directory = arguments[index + 1U];
        } else {
          receipt_path = arguments[index + 1U];
        }
        index += 2U;
      } else if (option == "--overwrite") {
        overwrite = true;
        ++index;
      } else {
        throw std::invalid_argument("flighttest validate: unrecognised argument '" + option + "'");
      }
    }
    if (output_directory.empty()) {
      throw std::invalid_argument("flighttest validate: --output-dir is required");
    }
    if (receipt_path.empty()) {
      receipt_path =
          std::filesystem::path(output_directory) / "flight-test-validation-receipt.json";
    }
    if (!std::filesystem::is_regular_file(std::filesystem::symlink_status(study_path))) {
      throw std::invalid_argument("flighttest validate: study is not a regular file: "
                                  + study_path.string());
    }

    const auto campaign =
        galata::identify::parse_flight_test_campaign(read_bounded_manifest(campaign_manifest));
    const std::filesystem::path campaign_root = campaign_manifest.has_parent_path()
                                                    ? campaign_manifest.parent_path()
                                                    : std::filesystem::path(".");
    (void)galata::identify::verify_flight_test_campaign(campaign, campaign_root);

    const galata::pipeline::Pipeline pipeline =
        galata::pipeline::load_pipeline(study_path.string());
    const std::string base_directory =
        study_path.has_parent_path() ? study_path.parent_path().string() : std::string(".");
    const bool campaign_bound = std::any_of(
        pipeline.stages.begin(), pipeline.stages.end(), [&](const galata::pipeline::Stage& stage) {
          return value_binds_campaign_manifest(stage.input, base_directory, campaign_manifest);
        });

    galata::pipeline::RunOptions options;
    options.overwrite = overwrite;
    const auto progress = [](const std::string&, const std::string&, bool, const std::string&) {};
    const galata::pipeline::RunResult run =
        galata::pipeline::run_pipeline(pipeline,
                                       galata::pipeline::builtin_registry(),
                                       base_directory,
                                       output_directory,
                                       progress,
                                       options);

    const galata::pipeline::ValidationArtifact* validation = nullptr;
    bool report_has_flight_gate = false;
    std::string report_path;
    for (const auto& stage : run.stages) {
      if (stage.capability == "identify.validate.vehicle") {
        validation = &stage.artifact.payload_as<galata::pipeline::ValidationArtifact>("validation");
      }
      if (stage.artifact.kind == "report") {
        const auto* path = std::any_cast<std::string>(&stage.artifact.payload);
        if (path != nullptr) {
          report_path = *path;
          const std::string report = read_bounded_manifest(report_path);
          report_has_flight_gate = report.find("Flight-test evidence gate:") != std::string::npos;
        }
      }
    }

    std::string numerical_gate = "not_reported";
    std::string flight_gate = "not_reported";
    std::vector<std::string> reasons;
    if (!campaign_bound) {
      reasons.push_back(
          "study does not bind the supplied campaign manifest in acceptance.campaign_manifest");
    }
    if (validation == nullptr) {
      reasons.push_back("study did not execute identify.validate.vehicle");
    } else {
      if (validation->gate) {
        numerical_gate = galata::identify::to_string(validation->gate->status);
      } else {
        reasons.push_back("validation stage did not declare a numerical acceptance gate");
      }
      if (validation->flight_test_gate) {
        flight_gate = galata::identify::to_string(validation->flight_test_gate->status);
        if (validation->flight_test_gate->campaign_manifest_sha256 != campaign.manifest_sha256) {
          reasons.push_back(
              "validation gate campaign manifest digest does not match the verified package");
        }
      } else {
        reasons.push_back("validation stage did not declare a flight-test evidence gate");
      }
    }
    if (report_path.empty() || !report_has_flight_gate) {
      reasons.push_back("study did not produce a report retaining the flight-test evidence gate");
    }
    if (numerical_gate != "pass") {
      reasons.push_back("numerical acceptance gate is " + numerical_gate);
    }
    if (flight_gate != "pass") {
      reasons.push_back("flight-test evidence gate is " + flight_gate);
    }
    if (campaign.evidence.evidence_class != "measured_flight") {
      reasons.push_back(
          "only measured_flight provenance can produce a production validation receipt");
    }
    const bool gate_passed = reasons.empty();
    const std::string status = gate_passed ? "gate_passed" : "not_ready";
    std::ostringstream receipt;
    receipt << "{\n  \"schema\":\"galata.flight-test-validation-receipt.v1\",\n"
            << "  \"status\":" << json_quote(status) << ",\n"
            << "  \"campaign_manifest\":" << json_quote(campaign_manifest.string()) << ",\n"
            << "  \"campaign_manifest_sha256\":" << json_quote(campaign.manifest_sha256) << ",\n"
            << "  \"evidence_class\":" << json_quote(campaign.evidence.evidence_class) << ",\n"
            << "  \"study\":" << json_quote(study_path.string()) << ",\n"
            << "  \"run_manifest\":" << json_quote(run.manifest_path) << ",\n"
            << "  \"campaign_binding_verified\":" << (campaign_bound ? "true" : "false") << ",\n"
            << "  \"run_inputs_verified\":true,\n"
            << "  \"report\":" << json_quote(report_path) << ",\n"
            << "  \"numerical_acceptance_gate\":" << json_quote(numerical_gate) << ",\n"
            << "  \"flight_test_evidence_gate\":" << json_quote(flight_gate) << ",\n"
            << "  \"reasons\":[";
    for (std::size_t index = 0U; index < reasons.size(); ++index) {
      if (index != 0U) {
        receipt << ',';
      }
      receipt << json_quote(reasons[index]);
    }
    receipt << "],\n  \"qualification_state\":\"not_qualified\",\n"
            << "  \"airworthiness_claim\":false,\n  \"certification_claim\":false,\n"
            << "  \"external_authority_acceptance_required\":true\n}\n";
    write_validation_receipt(receipt_path, receipt.str(), overwrite);
    std::cout << "{\"schema\":\"galata.flight-test-validation-receipt.v1\",\"status\":"
              << json_quote(status) << ",\"receipt\":" << json_quote(receipt_path.string())
              << ",\"campaign_manifest_sha256\":" << json_quote(campaign.manifest_sha256)
              << ",\"numerical_acceptance_gate\":" << json_quote(numerical_gate)
              << ",\"flight_test_evidence_gate\":" << json_quote(flight_gate)
              << ",\"qualification_state\":\"not_qualified\",\"airworthiness_claim\":false,"
                 "\"certification_claim\":false}\n";
    return gate_passed ? 0 : 2;
  } catch (const std::exception& error) {
    std::cerr << "galata flighttest validate: " << error.what() << "\n";
    return 1;
  }
}

int flight_test_command(const std::vector<std::string>& arguments) {
  if (!arguments.empty() && arguments[0] == "create") {
    return flight_test_create_command(arguments);
  }
  if (!arguments.empty() && arguments[0] == "validate") {
    return flight_test_validate_command(arguments);
  }
  return flight_test_verify_command(arguments);
}

int qualification_create_command(const std::vector<std::string>& arguments) {
  if (arguments.size() < 2U || arguments[0] != "create") {
    std::cerr << "usage: galata qualification create <new-directory> --product-id <id>"
                 " --product-version <version> --intended-use <use> --aircraft-id <id>"
                 " --configuration <id> --qualification-basis <basis> --authority-id <id>"
                 " --file <role=path>...\n";
    return 2;
  }

  try {
    const std::filesystem::path destination(arguments[1]);
    std::string product_id;
    std::string product_version;
    std::string intended_use;
    std::string aircraft_id;
    std::string aircraft_configuration;
    std::string qualification_basis;
    std::string authority_id;
    std::vector<FlightTestSourceArgument> sources;
    for (std::size_t index = 2U; index < arguments.size();) {
      const auto read_value = [&](const char* option) {
        if (index + 1U >= arguments.size()) {
          throw std::invalid_argument(std::string("qualification: ") + option
                                      + " requires a value");
        }
        return arguments[index + 1U];
      };
      if (arguments[index] == "--product-id") {
        product_id = read_value("--product-id");
        index += 2U;
      } else if (arguments[index] == "--product-version") {
        product_version = read_value("--product-version");
        index += 2U;
      } else if (arguments[index] == "--intended-use") {
        intended_use = read_value("--intended-use");
        index += 2U;
      } else if (arguments[index] == "--aircraft-id") {
        aircraft_id = read_value("--aircraft-id");
        index += 2U;
      } else if (arguments[index] == "--configuration") {
        aircraft_configuration = read_value("--configuration");
        index += 2U;
      } else if (arguments[index] == "--qualification-basis") {
        qualification_basis = read_value("--qualification-basis");
        index += 2U;
      } else if (arguments[index] == "--authority-id") {
        authority_id = read_value("--authority-id");
        index += 2U;
      } else if (arguments[index] == "--file") {
        sources.push_back(flight_test_source_argument(read_value("--file")));
        index += 2U;
      } else {
        throw std::invalid_argument("qualification: unrecognised or incomplete option '"
                                    + arguments[index] + "'");
      }
    }
    const auto validate_text = [](const std::string& value, const char* field) {
      if (value.empty()) {
        throw std::invalid_argument(std::string("qualification: ") + field + " is required");
      }
      for (const char character : value) {
        const unsigned char code = static_cast<unsigned char>(character);
        if (code < 0x20U || code == 0x7fU || character == '=') {
          throw std::invalid_argument(std::string("qualification: ") + field
                                      + " contains a control or separator character");
        }
      }
    };
    validate_text(product_id, "product_id");
    validate_text(product_version, "product_version");
    validate_text(intended_use, "intended_use");
    validate_text(aircraft_id, "aircraft_id");
    validate_text(aircraft_configuration, "aircraft_configuration");
    validate_text(qualification_basis, "qualification_basis");
    validate_text(authority_id, "authority_id");

    const std::vector<std::string> required_roles = {"requirements_matrix",
                                                     "software_release",
                                                     "verification_report",
                                                     "flight_test_campaign",
                                                     "hardware_hil_report",
                                                     "safety_case",
                                                     "independent_review",
                                                     "authority_decision",
                                                     "maintenance_plan"};
    if (sources.size() != required_roles.size()) {
      throw std::invalid_argument(
          "qualification: exactly one --file is required for each of the nine required roles");
    }
    std::set<std::string> seen_roles;
    for (const auto& source : sources) {
      if (std::find(required_roles.begin(), required_roles.end(), source.role)
          == required_roles.end()) {
        throw std::invalid_argument("qualification: unknown evidence role '" + source.role + "'");
      }
      if (!seen_roles.insert(source.role).second) {
        throw std::invalid_argument("qualification: evidence role '" + source.role
                                    + "' was supplied more than once");
      }
    }
    if (seen_roles.size() != required_roles.size()) {
      throw std::invalid_argument("qualification: all nine required evidence roles are required");
    }

    const std::filesystem::file_status destination_status =
        std::filesystem::symlink_status(destination);
    if (std::filesystem::exists(destination_status)) {
      throw std::invalid_argument("qualification: destination already exists: "
                                  + destination.string());
    }
    const std::filesystem::path parent =
        destination.has_parent_path() ? destination.parent_path() : std::filesystem::path(".");
    std::filesystem::create_directories(parent);
    const std::filesystem::path staging = destination.string() + ".staging";
    if (std::filesystem::exists(std::filesystem::symlink_status(staging))) {
      throw std::invalid_argument("qualification: staging destination already exists: "
                                  + staging.string());
    }

    std::error_code cleanup_error;
    try {
      std::filesystem::create_directories(staging / "evidence");
      std::vector<std::string> digests;
      digests.reserve(sources.size());
      for (const auto& source : sources) {
        const std::string digest = sha256_regular_file(source.path);
        const std::filesystem::path copied = staging / "evidence" / source.role;
        std::filesystem::copy_file(source.path, copied);
        if (sha256_regular_file(copied) != digest) {
          throw std::runtime_error("qualification: source changed while copying role '"
                                   + source.role + "'");
        }
        digests.push_back(digest);
      }

      std::ostringstream manifest;
      manifest << "format=galata-qualification-evidence-v1\n"
               << "product_id=" << product_id << "\n"
               << "product_version=" << product_version << "\n"
               << "intended_use=" << intended_use << "\n"
               << "aircraft_id=" << aircraft_id << "\n"
               << "aircraft_configuration=" << aircraft_configuration << "\n"
               << "qualification_basis=" << qualification_basis << "\n"
               << "authority_id=" << authority_id << "\n"
               << "acceptance_state=not_qualified\n";
      for (std::size_t index = 0U; index < sources.size(); ++index) {
        manifest << "file." << index << ".role=" << sources[index].role << "\n"
                 << "file." << index << ".path=evidence/" << sources[index].role << "\n"
                 << "file." << index << ".sha256=" << digests[index] << "\n";
      }
      {
        std::ofstream output(staging / "qualification.manifest", std::ios::binary);
        if (!output) {
          throw std::runtime_error("qualification: cannot write staged dossier manifest");
        }
        output << manifest.str();
        if (!output) {
          throw std::runtime_error("qualification: failed while writing staged dossier manifest");
        }
      }

      const auto dossier = galata::qualification::parse_dossier(manifest.str());
      const std::uintmax_t total_bytes = galata::qualification::verify_dossier(dossier, staging);
      std::filesystem::rename(staging, destination);
      std::cout << "{\"schema\":\"galata.qualification-evidence-package.v1\","
                   "\"status\":\"staged\",\"destination\":"
                << json_quote(destination.string())
                << ",\"manifest\":" << json_quote((destination / "qualification.manifest").string())
                << ",\"manifest_sha256\":" << json_quote(dossier.manifest_sha256)
                << ",\"file_count\":" << dossier.files.size() << ",\"total_bytes\":" << total_bytes
                << ",\"evidence_references_verified\":true,"
                   "\"qualification_state\":\"not_qualified\","
                   "\"airworthiness_claim\":false,\"certification_claim\":false}\n";
      return 0;
    } catch (...) {
      std::filesystem::remove_all(staging, cleanup_error);
      throw;
    }
  } catch (const std::exception& error) {
    std::cerr << "galata qualification create: " << error.what() << "\n";
    return 1;
  }
}

int qualification_command(const std::vector<std::string>& arguments) {
  if (!arguments.empty() && arguments[0] == "create") {
    return qualification_create_command(arguments);
  }
  if (!arguments.empty() && arguments[0] == "verify-chain") {
    if (arguments.size() < 2U) {
      std::cerr << "usage: galata qualification verify-chain <dossier.manifest>"
                   " --flighttest <campaign.manifest>"
                   " --flight-validation-receipt <receipt.json>"
                   " --deployment <deployment.manifest>"
                   " --deployment-dir <runtime-package>"
                   " --target-evidence <target-evidence.manifest>\n";
      return 2;
    }

    try {
      const std::filesystem::path dossier_path(arguments[1]);
      std::filesystem::path flight_test_path;
      std::filesystem::path flight_validation_receipt_path;
      std::filesystem::path deployment_path;
      std::filesystem::path deployment_directory;
      std::filesystem::path target_evidence_path;
      for (std::size_t index = 2U; index < arguments.size();) {
        if (index + 1U >= arguments.size()) {
          throw std::invalid_argument("qualification verify-chain: option requires a value");
        }
        const std::filesystem::path value(arguments[index + 1U]);
        if (arguments[index] == "--flighttest") {
          if (!flight_test_path.empty()) {
            throw std::invalid_argument(
                "qualification verify-chain: --flighttest was supplied more than once");
          }
          flight_test_path = value;
        } else if (arguments[index] == "--deployment") {
          if (!deployment_path.empty()) {
            throw std::invalid_argument(
                "qualification verify-chain: --deployment was supplied more than once");
          }
          deployment_path = value;
        } else if (arguments[index] == "--deployment-dir") {
          if (!deployment_directory.empty()) {
            throw std::invalid_argument(
                "qualification verify-chain: --deployment-dir was supplied more than once");
          }
          deployment_directory = value;
        } else if (arguments[index] == "--flight-validation-receipt") {
          if (!flight_validation_receipt_path.empty()) {
            throw std::invalid_argument(
                "qualification verify-chain: --flight-validation-receipt was supplied more than "
                "once");
          }
          flight_validation_receipt_path = value;
        } else if (arguments[index] == "--target-evidence") {
          if (!target_evidence_path.empty()) {
            throw std::invalid_argument(
                "qualification verify-chain: --target-evidence was supplied more than once");
          }
          target_evidence_path = value;
        } else {
          throw std::invalid_argument("qualification verify-chain: unrecognised option '"
                                      + arguments[index] + "'");
        }
        index += 2U;
      }
      if (flight_test_path.empty() || flight_validation_receipt_path.empty()
          || deployment_path.empty() || deployment_directory.empty()
          || target_evidence_path.empty()) {
        throw std::invalid_argument(
            "qualification verify-chain: --flighttest, --flight-validation-receipt, "
            "--deployment, --deployment-dir and --target-evidence are all required");
      }

      const auto dossier =
          galata::qualification::parse_dossier(read_bounded_manifest(dossier_path));
      const std::filesystem::path dossier_root =
          dossier_path.has_parent_path() ? dossier_path.parent_path() : std::filesystem::path(".");
      const std::uintmax_t dossier_bytes =
          galata::qualification::verify_dossier(dossier, dossier_root);

      const auto campaign =
          galata::identify::parse_flight_test_campaign(read_bounded_manifest(flight_test_path));
      const std::filesystem::path campaign_root = flight_test_path.has_parent_path()
                                                      ? flight_test_path.parent_path()
                                                      : std::filesystem::path(".");
      const std::uintmax_t campaign_bytes =
          galata::identify::verify_flight_test_campaign(campaign, campaign_root);

      const YAML::Node validation_receipt =
          YAML::Load(read_bounded_manifest(flight_validation_receipt_path));
      const auto receipt_string = [&](const char* key) {
        if (!validation_receipt[key] || !validation_receipt[key].IsScalar()) {
          return std::string();
        }
        return validation_receipt[key].as<std::string>();
      };
      const auto receipt_bool = [&](const char* key) {
        return validation_receipt[key] && validation_receipt[key].IsScalar()
               && validation_receipt[key].as<bool>();
      };
      const bool flight_validation_receipt_verified =
          receipt_string("schema") == "galata.flight-test-validation-receipt.v1"
          && receipt_string("status") == "gate_passed"
          && receipt_string("campaign_manifest_sha256") == campaign.manifest_sha256
          && receipt_string("evidence_class") == "measured_flight"
          && receipt_string("numerical_acceptance_gate") == "pass"
          && receipt_string("flight_test_evidence_gate") == "pass"
          && receipt_bool("campaign_binding_verified") && receipt_bool("run_inputs_verified")
          && validation_receipt["campaign_manifest"]
          && validation_receipt["campaign_manifest"].IsScalar()
          && path_matches(validation_receipt["campaign_manifest"].as<std::string>(),
                          flight_test_path);

      const auto deployment =
          galata::onboard::parse_manifest_package(read_bounded_manifest(deployment_path));
      const auto deployment_runtime = galata::onboard::verify_runtime_package(deployment_directory);
      if (deployment_runtime.manifest_sha256 != deployment.manifest_sha256) {
        throw std::invalid_argument(
            "qualification verify-chain: deployed runtime package is bound to a different "
            "deployment manifest");
      }
      const auto target_evidence = galata::onboard::parse_target_evidence_package(
          read_bounded_manifest(target_evidence_path));
      galata::onboard::verify_target_evidence_package(target_evidence, deployment);
      const std::filesystem::path target_root = target_evidence_path.has_parent_path()
                                                    ? target_evidence_path.parent_path()
                                                    : std::filesystem::path(".");
      const std::uintmax_t target_bytes =
          galata::onboard::verify_target_evidence_files(target_evidence, target_root);

      const auto dossier_role_digest = [&](const std::string& role) -> const std::string& {
        const auto found = std::find_if(
            dossier.files.begin(),
            dossier.files.end(),
            [&](const galata::qualification::DossierFile& file) { return file.role == role; });
        if (found == dossier.files.end()) {
          throw std::invalid_argument("qualification verify-chain: dossier is missing role '" + role
                                      + "'");
        }
        return found->sha256;
      };
      if (sha256_regular_file(flight_test_path) != dossier_role_digest("flight_test_campaign")) {
        throw std::invalid_argument(
            "qualification verify-chain: dossier flight_test_campaign digest does not link to "
            "the supplied campaign manifest");
      }
      if (sha256_regular_file(target_evidence_path) != dossier_role_digest("hardware_hil_report")) {
        throw std::invalid_argument(
            "qualification verify-chain: dossier hardware_hil_report digest does not link to "
            "the supplied target-evidence manifest");
      }
      if (dossier.aircraft_id != campaign.evidence.aircraft_id
          || dossier.aircraft_configuration != campaign.evidence.aircraft_configuration) {
        throw std::invalid_argument(
            "qualification verify-chain: dossier aircraft identity does not match the flight "
            "test campaign");
      }

      // Traceability and qualification readiness are deliberately separate
      // states.  A verified manifest can still contain only synthetic or
      // host-SIL evidence, and a non-pending authority id is only a declared
      // routing value until the responsible authority's record is reviewed.
      const bool flight_test_eligibility = campaign.evidence.evidence_class == "measured_flight"
                                           && campaign.evidence.safety_review_complete
                                           && flight_validation_receipt_verified;
      const bool target_hardware_eligibility = target_evidence.evidence_class == "target_hil"
                                               || target_evidence.evidence_class == "flight_target";
      const bool authority_decision_eligibility =
          !dossier.authority_id.empty() && dossier.authority_id != "external-authority-pending";
      const bool qualification_eligibility = flight_test_eligibility && target_hardware_eligibility
                                             && authority_decision_eligibility
                                             && deployment_runtime.contains_executable;
      std::vector<std::string> eligibility_reasons;
      if (!flight_test_eligibility) {
        eligibility_reasons.emplace_back(
            "flight_test_requires_measured_flight_evidence_and_completed_safety_review");
        if (!flight_validation_receipt_verified) {
          eligibility_reasons.emplace_back(
              "flight_validation_receipt_must_be_gate_passed_and_bound_to_the_campaign");
        }
      }
      if (!target_hardware_eligibility) {
        eligibility_reasons.emplace_back(
            "target_hardware_requires_target_hil_or_flight_target_evidence");
      }
      if (!authority_decision_eligibility) {
        eligibility_reasons.emplace_back("external_authority_decision_is_not_identified");
      }

      const std::uintmax_t total_bytes = dossier_bytes + campaign_bytes + target_bytes;
      std::cout
          << "{\"schema\":\"galata.qualification-chain-verification.v1\","
             "\"status\":\"verified\",\"dossier_manifest_sha256\":"
          << json_quote(dossier.manifest_sha256)
          << ",\"flight_test_manifest_sha256\":" << json_quote(campaign.manifest_sha256)
          << ",\"target_evidence_manifest_sha256\":" << json_quote(target_evidence.manifest_sha256)
          << ",\"deployment_manifest_sha256\":" << json_quote(deployment.manifest_sha256)
          << ",\"deployment_runtime_sha256\":" << json_quote(deployment_runtime.runtime_sha256)
          << ",\"deployment_runtime_verified\":true"
          << ",\"flight_validation_receipt_sha256\":"
          << json_quote(sha256_regular_file(flight_validation_receipt_path))
          << ",\"aircraft_id\":" << json_quote(dossier.aircraft_id)
          << ",\"aircraft_configuration\":" << json_quote(dossier.aircraft_configuration)
          << ",\"dossier_evidence_verified\":true"
             ",\"flight_test_evidence_verified\":true"
             ",\"flight_validation_receipt_verified\":"
          << (flight_validation_receipt_verified ? "true" : "false")
          << ",\"target_evidence_verified\":true"
          << ",\"traceability_links_verified\":true"
          << ",\"flight_test_eligibility\":" << (flight_test_eligibility ? "true" : "false")
          << ",\"target_hardware_eligibility\":" << (target_hardware_eligibility ? "true" : "false")
          << ",\"authority_decision_eligibility\":"
          << (authority_decision_eligibility ? "true" : "false")
          << ",\"qualification_eligibility\":"
          << json_quote(qualification_eligibility ? "eligible_for_authority_review" : "not_ready")
          << ",\"qualification_eligibility_reasons\":[";
      for (std::size_t index = 0U; index < eligibility_reasons.size(); ++index) {
        if (index != 0U) {
          std::cout << ",";
        }
        std::cout << json_quote(eligibility_reasons[index]);
      }
      std::cout << "]"
                   ",\"total_bytes\":"
                << total_bytes
                << ",\"qualification_state\":\"not_qualified\""
                   ",\"external_authority_acceptance_required\":true"
                   ",\"airworthiness_claim\":false,\"certification_claim\":false}\n";
      return 0;
    } catch (const std::exception& error) {
      std::cerr << "galata qualification verify-chain: " << error.what() << "\n";
      return 1;
    }
  }
  if (arguments.size() != 2U || arguments[0] != "verify") {
    std::cerr << "usage: galata qualification verify <dossier.manifest>\n";
    return 2;
  }

  try {
    const std::filesystem::path manifest_path(arguments[1]);
    const auto dossier = galata::qualification::parse_dossier(read_bounded_manifest(manifest_path));
    const std::filesystem::path package_root =
        manifest_path.has_parent_path() ? manifest_path.parent_path() : std::filesystem::path(".");
    const std::uintmax_t total_bytes = galata::qualification::verify_dossier(dossier, package_root);
    std::cout << "{\"schema\":\"galata.qualification-evidence-verification.v1\","
                 "\"status\":\"verified\",\"manifest_sha256\":"
              << json_quote(dossier.manifest_sha256)
              << ",\"product_id\":" << json_quote(dossier.product_id)
              << ",\"product_version\":" << json_quote(dossier.product_version)
              << ",\"intended_use\":" << json_quote(dossier.intended_use)
              << ",\"aircraft_id\":" << json_quote(dossier.aircraft_id)
              << ",\"aircraft_configuration\":" << json_quote(dossier.aircraft_configuration)
              << ",\"qualification_basis\":" << json_quote(dossier.qualification_basis)
              << ",\"authority_id\":" << json_quote(dossier.authority_id)
              << ",\"file_count\":" << dossier.files.size() << ",\"file_roles\":[";
    for (std::size_t index = 0; index < dossier.files.size(); ++index) {
      if (index != 0U) {
        std::cout << ',';
      }
      std::cout << json_quote(dossier.files[index].role);
    }
    std::cout << "],\"total_bytes\":" << total_bytes
              << ",\"evidence_package_complete\":true"
                 ",\"qualification_state\":\"not_qualified\""
                 ",\"external_authority_acceptance_required\":true"
                 ",\"airworthiness_claim\":false,\"certification_claim\":false}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "galata qualification: " << error.what() << "\n";
    return 1;
  }
}

std::uint64_t parse_positive_uint64(const std::string& text, const char* option) {
  if (text.empty()) {
    throw std::invalid_argument(std::string(option) + " requires a positive integer");
  }
  std::uint64_t value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || value == 0U) {
    throw std::invalid_argument(std::string(option) + " requires a positive integer");
  }
  return value;
}

std::uint32_t parse_uint32(const std::string& text, const char* field) {
  if (text.empty()) {
    throw std::invalid_argument(std::string(field) + " must be a non-zero integer");
  }
  unsigned long long value = 0;
  std::size_t consumed = 0;
  try {
    value = std::stoull(text, &consumed, 0);
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string(field) + " must be a non-zero integer");
  }
  if (consumed != text.size() || value == 0U || value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument(std::string(field) + " must be a non-zero integer");
  }
  return static_cast<std::uint32_t>(value);
}

std::vector<std::string> split_endpoint(const std::string& endpoint, char separator) {
  std::vector<std::string> parts;
  std::size_t start = 0;
  while (start <= endpoint.size()) {
    const std::size_t end = endpoint.find(separator, start);
    parts.push_back(endpoint.substr(start, end == std::string::npos ? end : end - start));
    if (end == std::string::npos) {
      break;
    }
    start = end + 1U;
  }
  return parts;
}

galata::hardware::SerialEndpoint serial_endpoint(const std::string& endpoint,
                                                 std::uint32_t receive_timeout_ms,
                                                 std::uint32_t transmit_timeout_ms) {
  const std::size_t separator = endpoint.rfind('|');
  const std::string device =
      separator == std::string::npos ? endpoint : endpoint.substr(0, separator);
  const std::uint32_t baud = separator == std::string::npos
                                 ? 115200U
                                 : parse_uint32(endpoint.substr(separator + 1U), "serial baud");
  if (device.empty()) {
    throw std::invalid_argument("onboard: serial endpoint has an empty device path");
  }
  return {device, baud, receive_timeout_ms, transmit_timeout_ms};
}

galata::hardware::UdpEndpoint udp_endpoint(const std::string& endpoint,
                                           std::uint32_t receive_timeout_ms,
                                           std::uint32_t transmit_timeout_ms) {
  const std::size_t local_separator = endpoint.rfind('|');
  const std::string remote_endpoint =
      local_separator == std::string::npos ? endpoint : endpoint.substr(0, local_separator);
  const std::uint32_t local_port =
      local_separator == std::string::npos
          ? 0U
          : parse_uint32(endpoint.substr(local_separator + 1U), "UDP local port");
  if (local_port > std::numeric_limits<std::uint16_t>::max()) {
    throw std::invalid_argument("onboard: UDP local port is outside the 16-bit range");
  }

  std::string host;
  std::string port_text;
  if (!remote_endpoint.empty() && remote_endpoint.front() == '[') {
    const std::size_t closing = remote_endpoint.find(']');
    if (closing == std::string::npos || closing + 2U > remote_endpoint.size()
        || remote_endpoint[closing + 1U] != ':') {
      throw std::invalid_argument("onboard: UDP IPv6 endpoint must use [host]:port[|local_port]");
    }
    host = remote_endpoint.substr(1U, closing - 1U);
    port_text = remote_endpoint.substr(closing + 2U);
  } else {
    const std::size_t separator = remote_endpoint.rfind(':');
    if (separator == std::string::npos || remote_endpoint.find(':') != separator) {
      throw std::invalid_argument(
          "onboard: UDP endpoint must use host:port[|local_port] or [IPv6]:port[|local_port]");
    }
    host = remote_endpoint.substr(0, separator);
    port_text = remote_endpoint.substr(separator + 1U);
  }
  const std::uint32_t port = parse_uint32(port_text, "UDP port");
  if (port > std::numeric_limits<std::uint16_t>::max()) {
    throw std::invalid_argument("onboard: UDP port is outside the 16-bit range");
  }
  return {host,
          static_cast<std::uint16_t>(port),
          receive_timeout_ms,
          static_cast<std::uint16_t>(local_port),
          transmit_timeout_ms};
}

galata::hardware::CanFdEndpoint can_fd_endpoint(const std::string& endpoint,
                                                const galata::hardware::TransportProfile& profile) {
  const std::vector<std::string> parts = split_endpoint(endpoint, ',');
  if (parts.size() != 3U || parts[0].empty()) {
    throw std::invalid_argument(
        "onboard: CAN-FD endpoint must use interface,receive_id,transmit_id");
  }
  return {parts[0],
          parse_uint32(parts[1], "CAN-FD receive ID"),
          parse_uint32(parts[2], "CAN-FD transmit ID"),
          profile.receive_timeout_ms,
          profile.transmit_timeout_ms};
}

std::unique_ptr<galata::hardware::Transport> live_transport(
    const galata::hardware::TransportProfile& profile) {
  if (profile.transport == "serial") {
    return std::make_unique<galata::hardware::SerialTransport>(
        serial_endpoint(profile.endpoint, profile.receive_timeout_ms, profile.transmit_timeout_ms));
  }
  if (profile.transport == "udp") {
    return std::make_unique<galata::hardware::UdpTransport>(
        udp_endpoint(profile.endpoint, profile.receive_timeout_ms, profile.transmit_timeout_ms));
  }
  if (profile.transport == "can_fd") {
    return std::make_unique<galata::hardware::CanFdTransport>(
        can_fd_endpoint(profile.endpoint, profile));
  }
  throw std::invalid_argument(
      "onboard: onboard run requires serial, udp or can_fd; replay is SIL-only");
}

std::atomic_bool onboard_stop_requested = false;

void request_onboard_stop(int) {
  onboard_stop_requested.store(true, std::memory_order_relaxed);
}

int onboard_run_command(const std::vector<std::string>& arguments) {
  if (arguments.size() < 2U) {
    std::cerr << "usage: galata onboard run <manifest> --model <path> --controller <path> "
                 "--operator <id> "
                 "--confirm ARM --cycles <count>\n";
    return 2;
  }
  try {
    const std::filesystem::path manifest_path(arguments[1]);
    const auto package =
        galata::onboard::parse_manifest_package(read_bounded_manifest(manifest_path));
    std::filesystem::path model_path;
    std::filesystem::path controller_path;
    std::string operator_id;
    std::string confirmation;
    std::uint64_t cycles = 0;
    std::uint64_t linger_ms = 0;
    bool cycles_supplied = false;
    for (std::size_t index = 2; index < arguments.size();) {
      if (arguments[index] == "--model" && index + 1U < arguments.size()) {
        model_path = arguments[index + 1U];
        index += 2U;
      } else if (arguments[index] == "--controller" && index + 1U < arguments.size()) {
        controller_path = arguments[index + 1U];
        index += 2U;
      } else if (arguments[index] == "--operator" && index + 1U < arguments.size()) {
        operator_id = arguments[index + 1U];
        index += 2U;
      } else if (arguments[index] == "--confirm" && index + 1U < arguments.size()) {
        confirmation = arguments[index + 1U];
        index += 2U;
      } else if (arguments[index] == "--cycles" && index + 1U < arguments.size()) {
        cycles = parse_positive_uint64(arguments[index + 1U], "--cycles");
        cycles_supplied = true;
        index += 2U;
      } else if (arguments[index] == "--linger-ms" && index + 1U < arguments.size()) {
        linger_ms = parse_positive_uint64(arguments[index + 1U], "--linger-ms");
        if (linger_ms > 60000U) {
          throw std::invalid_argument("--linger-ms must not exceed 60000 milliseconds");
        }
        index += 2U;
      } else if (arguments[index] == "--until-signal" && index + 1U == arguments.size()) {
        cycles = 0;
        cycles_supplied = true;
        index += 1U;
      } else {
        throw std::invalid_argument("onboard run: unrecognised or incomplete option '"
                                    + arguments[index] + "'");
      }
    }
    if (model_path.empty() || controller_path.empty() || operator_id.empty() || confirmation.empty()
        || !cycles_supplied) {
      throw std::invalid_argument(
          "onboard run: --model, --controller, --operator, --confirm and "
          "--cycles/--until-signal are required");
    }
    if (confirmation != "ARM") {
      throw std::invalid_argument("onboard run: --confirm must be exactly ARM");
    }

    const auto expected_artifact = [&package](const std::string& role) -> std::string {
      const auto found = std::find_if(package.artifacts.begin(),
                                      package.artifacts.end(),
                                      [&role](const galata::onboard::ArtifactReference& artifact) {
                                        return artifact.role == role;
                                      });
      if (found == package.artifacts.end()) {
        throw std::invalid_argument("onboard run: manifest has no '" + role + "' artifact");
      }
      return found->sha256;
    };
    if (sha256_regular_file(model_path) != expected_artifact("model")) {
      throw std::invalid_argument("onboard run: model artifact hash does not match the manifest");
    }
    if (sha256_regular_file(controller_path) != expected_artifact("controller")) {
      throw std::invalid_argument(
          "onboard run: controller artifact hash does not match the manifest");
    }

    auto transport = live_transport(package.transport_profile);
    galata::onboard::ControllerPlugin controller(
        controller_path, model_path, package.interface, package.manifest);
    galata::hardware::ArmingInterlock interlock;
    galata::onboard::Runtime runtime(*transport,
                                     interlock,
                                     {package.interface,
                                      package.max_controller_time_s,
                                      package.transport_profile.watchdog_timeout_s});

    const auto previous_int = std::signal(SIGINT, request_onboard_stop);
    const auto previous_term = std::signal(SIGTERM, request_onboard_stop);
    onboard_stop_requested.store(false, std::memory_order_relaxed);
    try {
      runtime.connect();
      runtime.arm(operator_id, confirmation);
      while (!onboard_stop_requested.load(std::memory_order_relaxed)
             && (cycles == 0U || runtime.completed_cycles() < cycles)) {
        if (!runtime.step([&controller](const galata::hardware::Frame& sensor) {
              return controller.step(sensor);
            })) {
          throw std::runtime_error("onboard run: sensor receive timed out");
        }
      }
      if (linger_ms != 0U) {
        std::this_thread::sleep_for(std::chrono::milliseconds(linger_ms));
      }
      const bool stopped_by_signal = onboard_stop_requested.load(std::memory_order_relaxed);
      runtime.stop();
      std::signal(SIGINT, previous_int);
      std::signal(SIGTERM, previous_term);
      std::cout << "{\"schema\":\"galata.onboard-run.v1\",\"status\":"
                << json_quote(stopped_by_signal ? "stopped" : "completed")
                << ",\"completed_cycles\":" << runtime.completed_cycles()
                << ",\"observed_cycles\":" << runtime.metrics().observed_cycles
                << ",\"observed_controller_worst_case_s\":"
                << runtime.metrics().controller_worst_case_s
                << ",\"observed_cycle_worst_case_s\":" << runtime.metrics().cycle_worst_case_s
                << ",\"transport\":" << json_quote(package.transport_profile.transport)
                << ",\"target_platform\":" << json_quote(package.target_platform)
                << ",\"qualification_state\":\"not_qualified\","
                   "\"target_timing_claim\":false,\"airworthiness_claim\":false}\n";
      return 0;
    } catch (...) {
      runtime.stop();
      std::signal(SIGINT, previous_int);
      std::signal(SIGTERM, previous_term);
      throw;
    }
  } catch (const std::exception& error) {
    std::cerr << "galata onboard run: " << error.what() << "\n";
    return 1;
  }
}

int onboard_self_test_command() {
  try {
    galata::hardware::InterfaceSpec interface;
    interface.id = "galata-replay-self-test-v1";
    interface.sample_period_s = 0.01;
    interface.sensor_channels = {{"airspeed_m_s", "m/s", "body"}};
    interface.actuator_channels = {{"elevator_rad", "rad", "body"}};

    using galata::hardware::Frame;
    galata::hardware::ReplayTransport transport(
        {Frame{0, 0.00, {25.0}}, Frame{1, 0.01, {25.1}}, Frame{2, 0.02, {25.2}}});
    galata::hardware::ArmingInterlock interlock;
    galata::onboard::Runtime runtime(transport, interlock, {interface, 0.01, 0.05});
    runtime.connect();
    runtime.arm("galata-self-test", "ARM");
    const auto controller = [](const Frame& sensor) {
      return Frame{sensor.sequence, sensor.timestamp_s, {sensor.values.front() * 0.001}};
    };
    for (int cycle = 0; cycle < 3; ++cycle) {
      if (!runtime.step(controller)) {
        throw std::runtime_error("onboard self-test did not publish a complete cycle");
      }
    }
    if (transport.sent_frames().size() != 3U || runtime.completed_cycles() != 3U) {
      throw std::runtime_error("onboard self-test published an unexpected cycle count");
    }
    runtime.stop();
    std::cout << "{\"schema\":\"galata.onboard-self-test.v1\",\"status\":\"passed\","
                 "\"completed_cycles\":3,\"transport\":\"replay\","
                 "\"observed_cycles\":"
              << runtime.metrics().observed_cycles << ",\"observed_controller_worst_case_s\":"
              << runtime.metrics().controller_worst_case_s
              << ",\"observed_cycle_worst_case_s\":" << runtime.metrics().cycle_worst_case_s
              << ",\"target_executable\":false,\"qualification_state\":\"not_qualified\","
                 "\"hardware_timing_claim\":false,\"airworthiness_claim\":false}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "galata onboard self-test: " << error.what() << "\n";
    return 1;
  }
}

int onboard_target_verify_command(const std::vector<std::string>& arguments) {
  if (arguments.size() != 4U || arguments[0] != "target" || arguments[1] != "verify") {
    std::cerr << "usage: galata onboard target verify <deployment.manifest> "
                 "<target-evidence.manifest>\n";
    return 2;
  }
  try {
    const std::filesystem::path deployment_path(arguments[2]);
    const std::filesystem::path evidence_path(arguments[3]);
    const auto deployment =
        galata::onboard::parse_manifest_package(read_bounded_manifest(deployment_path));
    const auto evidence =
        galata::onboard::parse_target_evidence_package(read_bounded_manifest(evidence_path));
    galata::onboard::verify_target_evidence_package(evidence, deployment);
    const std::filesystem::path package_root =
        evidence_path.has_parent_path() ? evidence_path.parent_path() : std::filesystem::path(".");
    const std::uintmax_t total_bytes =
        galata::onboard::verify_target_evidence_files(evidence, package_root);
    std::cout << "{\"schema\":\"galata.onboard-target-evidence-verification.v2\","
                 "\"status\":\"verified\",\"manifest_sha256\":"
              << json_quote(evidence.manifest_sha256) << ",\"deployment_manifest_sha256\":"
              << json_quote(evidence.deployment_manifest_sha256)
              << ",\"evidence_class\":" << json_quote(evidence.evidence_class)
              << ",\"target_hardware_id\":" << json_quote(evidence.target_identity.hardware_id)
              << ",\"flight_computer_id\":"
              << json_quote(evidence.target_identity.flight_computer_id)
              << ",\"firmware_id\":" << json_quote(evidence.target_identity.firmware_id)
              << ",\"physical_tests_applicable\":"
              << (evidence.evidence_class == "host_sil" ? "false" : "true")
              << ",\"controller_worst_case_s\":" << evidence.controller_worst_case_s
              << ",\"cycle_worst_case_s\":" << evidence.cycle_worst_case_s
              << ",\"watchdog_response_s\":" << evidence.watchdog_response_s
              << ",\"file_count\":" << evidence.files.size() << ",\"total_bytes\":" << total_bytes
              << ",\"target_acceptance_state\":\"passed\","
                 "\"qualification_state\":\"not_qualified\","
                 "\"airworthiness_claim\":false,\"certification_claim\":false}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "galata onboard target verify: " << error.what() << "\n";
    return 1;
  }
}

double parse_nonnegative_double(const std::string& text, const char* option) {
  std::size_t consumed = 0U;
  double value = 0.0;
  try {
    value = std::stod(text, &consumed);
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string(option) + " requires a finite non-negative number");
  }
  if (consumed != text.size() || value < 0.0 || !std::isfinite(value)) {
    throw std::invalid_argument(std::string(option) + " requires a finite non-negative number");
  }
  return value;
}

galata::onboard::TargetEvidenceSource target_evidence_source_argument(const std::string& argument) {
  const std::size_t separator = argument.find('=');
  if (separator == std::string::npos || separator == 0U || separator + 1U >= argument.size()) {
    throw std::invalid_argument("onboard target create: --file expects role=path");
  }
  return {argument.substr(0, separator), argument.substr(separator + 1U)};
}

int onboard_target_create_command(const std::vector<std::string>& arguments) {
  if (arguments.size() < 4U || arguments[0] != "target" || arguments[1] != "create") {
    std::cerr << "usage: galata onboard target create <new-directory> <deployment.manifest> "
                 "--evidence-class <host_sil|target_hil|flight_target> "
                 "--controller-worst-case-s <s> --cycle-worst-case-s <s> "
                 "--watchdog-response-s <s> [--emergency-stop-passed true "
                 "--loss-of-link-passed true --hil-passed true --signing-verified true] "
                 "--file <role=path>...\n"
                 "host_sil omits the four physical-test/signing options; target_hil and "
                 "flight_target require them.\n";
    return 2;
  }
  try {
    galata::onboard::TargetEvidenceSpec specification;
    bool evidence_class_supplied = false;
    bool controller_supplied = false;
    bool cycle_supplied = false;
    bool watchdog_supplied = false;
    bool emergency_stop_supplied = false;
    bool loss_of_link_supplied = false;
    bool hil_supplied = false;
    bool signing_supplied = false;
    const auto require_value = [&arguments](std::size_t index, const char* option) {
      if (index + 1U >= arguments.size()) {
        throw std::invalid_argument(std::string("onboard target create: ") + option
                                    + " requires a value");
      }
      return arguments[index + 1U];
    };
    for (std::size_t index = 4U; index < arguments.size();) {
      if (arguments[index] == "--evidence-class") {
        specification.evidence_class = require_value(index, "--evidence-class");
        evidence_class_supplied = true;
        index += 2U;
      } else if (arguments[index] == "--controller-worst-case-s") {
        specification.controller_worst_case_s = parse_nonnegative_double(
            require_value(index, "--controller-worst-case-s"), "--controller-worst-case-s");
        controller_supplied = true;
        index += 2U;
      } else if (arguments[index] == "--cycle-worst-case-s") {
        specification.cycle_worst_case_s = parse_nonnegative_double(
            require_value(index, "--cycle-worst-case-s"), "--cycle-worst-case-s");
        cycle_supplied = true;
        index += 2U;
      } else if (arguments[index] == "--watchdog-response-s") {
        specification.watchdog_response_s = parse_nonnegative_double(
            require_value(index, "--watchdog-response-s"), "--watchdog-response-s");
        watchdog_supplied = true;
        index += 2U;
      } else if (arguments[index] == "--emergency-stop-passed") {
        specification.emergency_stop_passed =
            require_value(index, "--emergency-stop-passed") == "true";
        emergency_stop_supplied = true;
        index += 2U;
      } else if (arguments[index] == "--loss-of-link-passed") {
        specification.loss_of_link_passed = require_value(index, "--loss-of-link-passed") == "true";
        loss_of_link_supplied = true;
        index += 2U;
      } else if (arguments[index] == "--hil-passed") {
        specification.hil_passed = require_value(index, "--hil-passed") == "true";
        hil_supplied = true;
        index += 2U;
      } else if (arguments[index] == "--signing-verified") {
        specification.signing_verified = require_value(index, "--signing-verified") == "true";
        signing_supplied = true;
        index += 2U;
      } else if (arguments[index] == "--file") {
        specification.files.push_back(
            target_evidence_source_argument(require_value(index, "--file")));
        index += 2U;
      } else {
        throw std::invalid_argument("onboard target create: unrecognised or incomplete option '"
                                    + arguments[index] + "'");
      }
    }
    if (!evidence_class_supplied || !controller_supplied || !cycle_supplied || !watchdog_supplied
        || specification.files.size() != 5U) {
      throw std::invalid_argument(
          "onboard target create: evidence class, timing values and exactly five --file "
          "arguments are required");
    }
    const bool physical_confirmation_supplied =
        emergency_stop_supplied || loss_of_link_supplied || hil_supplied || signing_supplied;
    if (specification.evidence_class == "host_sil" && physical_confirmation_supplied) {
      throw std::invalid_argument(
          "onboard target create: host_sil omits physical-test and target-signing confirmations; "
          "those states are recorded as not_applicable");
    }
    if ((specification.evidence_class == "target_hil"
         || specification.evidence_class == "flight_target")
        && (!emergency_stop_supplied || !loss_of_link_supplied || !hil_supplied || !signing_supplied
            || !specification.emergency_stop_passed || !specification.loss_of_link_passed
            || !specification.hil_passed || !specification.signing_verified)) {
      throw std::invalid_argument(
          "onboard target create: target_hil and flight_target require every explicit pass "
          "confirmation to be true");
    }
    const auto deployment =
        galata::onboard::parse_manifest_package(read_bounded_manifest(arguments[3]));
    const std::filesystem::path destination(arguments[2]);
    const auto package =
        galata::onboard::stage_target_evidence_package(deployment, specification, destination);
    const std::uintmax_t total_bytes =
        galata::onboard::verify_target_evidence_files(package, destination);
    std::cout << "{\"schema\":\"galata.onboard-target-evidence-package.v2\","
                 "\"status\":\"staged\",\"destination\":"
              << json_quote(destination.string())
              << ",\"manifest\":" << json_quote((destination / "target-evidence.manifest").string())
              << ",\"manifest_sha256\":" << json_quote(package.manifest_sha256)
              << ",\"evidence_class\":" << json_quote(package.evidence_class)
              << ",\"physical_tests_applicable\":"
              << (package.evidence_class == "host_sil" ? "false" : "true")
              << ",\"file_count\":" << package.files.size() << ",\"total_bytes\":" << total_bytes
              << ",\"target_acceptance_state\":\"passed\","
                 "\"qualification_state\":\"not_qualified\","
                 "\"evidence_references_verified\":true}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "galata onboard target create: " << error.what() << "\n";
    return 1;
  }
}

int onboard_command(const std::vector<std::string>& arguments) {
  if (!arguments.empty() && arguments[0] == "target") {
    if (arguments.size() > 1U && arguments[1] == "create") {
      return onboard_target_create_command(arguments);
    }
    return onboard_target_verify_command(arguments);
  }
  if (!arguments.empty() && arguments[0] == "run") {
    return onboard_run_command(arguments);
  }
  if (arguments.size() == 1 && arguments[0] == "self-test") {
    return onboard_self_test_command();
  }
  if (arguments.size() == 2 && arguments[0] == "verify-deployment") {
    try {
      const auto receipt = galata::onboard::verify_runtime_package(arguments[1]);
      std::cout << "{\"schema\":\"galata.onboard-deployment-verification.v1\","
                   "\"status\":\"verified\",\"destination\":"
                << json_quote(receipt.destination.string())
                << ",\"manifest_sha256\":" << json_quote(receipt.manifest_sha256)
                << ",\"runtime_sha256\":" << json_quote(receipt.runtime_sha256)
                << ",\"artifact_count\":" << receipt.artifact_sha256.size()
                << ",\"contains_executable\":true,"
                   "\"qualification_state\":\"not_qualified\","
                   "\"target_timing_claim\":false,\"airworthiness_claim\":false}\n";
      return 0;
    } catch (const std::exception& error) {
      std::cerr << "galata onboard verify-deployment: " << error.what() << "\n";
      return 1;
    }
  }
  if (arguments.size() < 2) {
    std::cerr << "usage: galata onboard verify <manifest>\n"
                 "       galata onboard stage <manifest> <new-directory>"
                 " --artifact <role=path>...\n"
                 "       galata onboard deploy <manifest> <new-directory> --runtime <path>"
                 " --artifact <role=path>...\n"
                 "       galata onboard verify-deployment <directory>\n"
                 "       galata onboard run <manifest> --model <path> --controller <path>"
                 " --operator <id> --confirm ARM --cycles <count> [--linger-ms <count>]\n"
                 "       galata onboard self-test\n";
    return 2;
  }

  try {
    const std::string& operation = arguments[0];
    const auto package = galata::onboard::parse_manifest_package(
        read_bounded_manifest(std::filesystem::path(arguments[1])));
    if (operation == "verify") {
      if (arguments.size() != 2) {
        std::cerr << "galata onboard verify: expected exactly one manifest path\n";
        return 2;
      }
      std::cout << "{\"schema\":\"galata.onboard-verification.v1\",\"status\":\"verified\","
                   "\"manifest_sha256\":"
                << json_quote(package.manifest_sha256) << ",\"target_identity\":{\"hardware_id\":"
                << json_quote(package.target_identity.hardware_id) << ",\"flight_computer_id\":"
                << json_quote(package.target_identity.flight_computer_id)
                << ",\"firmware_id\":" << json_quote(package.target_identity.firmware_id)
                << ",\"emergency_stop_id\":"
                << json_quote(package.target_identity.emergency_stop_id)
                << "},\"artifact_count\":" << package.artifacts.size() << ",\"artifact_roles\":[";
      for (std::size_t index = 0; index < package.artifacts.size(); ++index) {
        if (index != 0) {
          std::cout << ',';
        }
        std::cout << json_quote(package.artifacts[index].role);
      }
      std::cout << "]"
                << ",\"max_controller_time_s\":" << package.max_controller_time_s
                << ",\"transport_profile\":{\"id\":" << json_quote(package.transport_profile.id)
                << ",\"transport\":" << json_quote(package.transport_profile.transport)
                << ",\"endpoint\":" << json_quote(package.transport_profile.endpoint)
                << ",\"receive_timeout_ms\":" << package.transport_profile.receive_timeout_ms
                << ",\"transmit_timeout_ms\":" << package.transport_profile.transmit_timeout_ms
                << ",\"watchdog_timeout_s\":" << package.transport_profile.watchdog_timeout_s
                << ",\"emergency_stop_required\":true}"
                << ",\"contains_executable\":false,"
                   "\"qualification_state\":\"not_qualified\"}\n";
      return 0;
    }

    if (operation == "deploy") {
      if (arguments.size() < 6U) {
        throw std::invalid_argument(
            "usage: galata onboard deploy <manifest> <new-directory> --runtime <path> "
            "--artifact <role=path>...");
      }
      std::filesystem::path runtime_path;
      std::vector<galata::onboard::ArtifactFile> files;
      for (std::size_t index = 3; index < arguments.size();) {
        if (arguments[index] == "--runtime" && index + 1U < arguments.size()) {
          if (!runtime_path.empty()) {
            throw std::invalid_argument("galata onboard deploy: --runtime may appear once");
          }
          runtime_path = arguments[index + 1U];
          index += 2U;
        } else if (arguments[index] == "--artifact" && index + 1U < arguments.size()) {
          files.push_back(onboard_artifact_argument(arguments[index + 1U]));
          index += 2U;
        } else {
          throw std::invalid_argument(
              "galata onboard deploy: expected --runtime and repeated --artifact arguments");
        }
      }
      if (runtime_path.empty()) {
        throw std::invalid_argument("galata onboard deploy: --runtime is required");
      }
      const auto receipt = galata::onboard::stage_runtime_package(
          package, {"runtime", runtime_path}, files, std::filesystem::path(arguments[2]));
      std::cout << "{\"schema\":\"galata.onboard-deployment.v1\","
                   "\"status\":\"staged\",\"destination\":"
                << json_quote(receipt.destination.string())
                << ",\"manifest_sha256\":" << json_quote(receipt.manifest_sha256)
                << ",\"runtime_sha256\":" << json_quote(receipt.runtime_sha256)
                << ",\"artifact_count\":" << receipt.artifact_sha256.size()
                << ",\"contains_executable\":true,"
                   "\"qualification_state\":\"not_qualified\","
                   "\"target_timing_claim\":false,\"airworthiness_claim\":false}\n";
      return 0;
    }

    if (operation != "stage" || arguments.size() < 5) {
      std::cerr << "usage: galata onboard verify <manifest>\n"
                   "       galata onboard stage <manifest> <new-directory>"
                   " --artifact <role=path>...\n"
                   "       galata onboard deploy <manifest> <new-directory> --runtime <path>"
                   " --artifact <role=path>...\n"
                   "       galata onboard verify-deployment <directory>\n"
                   "       galata onboard run <manifest> --model <path> --controller <path>"
                   " --operator <id> --confirm ARM --cycles <count> [--linger-ms <count>]\n"
                   "       galata onboard self-test\n";
      return 2;
    }
    std::vector<galata::onboard::ArtifactFile> files;
    for (std::size_t index = 3; index < arguments.size(); index += 2) {
      if (arguments[index] != "--artifact" || index + 1 >= arguments.size()) {
        std::cerr << "galata onboard stage: expected repeated --artifact role=path arguments\n";
        return 2;
      }
      files.push_back(onboard_artifact_argument(arguments[index + 1]));
    }
    const auto receipt = galata::onboard::stage_manifest_package(
        package, files, std::filesystem::path(arguments[2]));
    std::cout << "{\"schema\":\"galata.onboard-stage.v1\",\"status\":\"staged\","
                 "\"destination\":"
              << json_quote(receipt.destination.string())
              << ",\"manifest_sha256\":" << json_quote(receipt.manifest_sha256)
              << ",\"artifact_count\":" << receipt.artifact_sha256.size()
              << ",\"max_controller_time_s\":" << package.max_controller_time_s
              << ",\"transport_profile\":{\"id\":" << json_quote(package.transport_profile.id)
              << ",\"transport\":" << json_quote(package.transport_profile.transport)
              << ",\"endpoint\":" << json_quote(package.transport_profile.endpoint)
              << ",\"receive_timeout_ms\":" << package.transport_profile.receive_timeout_ms
              << ",\"transmit_timeout_ms\":" << package.transport_profile.transmit_timeout_ms
              << ",\"watchdog_timeout_s\":" << package.transport_profile.watchdog_timeout_s
              << ",\"emergency_stop_required\":true}"
              << ",\"contains_executable\":false,\"qualification_state\":\"not_qualified\"}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "galata onboard: " << error.what() << "\n";
    return 1;
  }
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string> arguments(argv + 1, argv + argc);
  if (arguments.empty()) {
    return print_usage(std::cout);
  }

  const std::string& command = arguments[0];
  if (command == "--help" || command == "-h" || command == "help") {
    return print_usage(std::cout);
  }
  if (command == "--version" || command == "-v" || command == "version") {
    return print_version();
  }
  if (command == "capabilities") {
    if (arguments.size() == 2 && arguments[1] == "--markdown") {
      return list_capabilities_markdown();
    }
    if (arguments.size() == 2 && arguments[1] == "--json") {
      return list_capabilities_json();
    }
    if (arguments.size() == 1) {
      return list_capabilities();
    }
    std::cerr << "galata capabilities: expected no arguments, --markdown or --json\n";
    return 2;
  }
  if (command == "run") {
    return run_pipeline_command(std::vector<std::string>(arguments.begin() + 1, arguments.end()));
  }
  if (command == "onboard") {
    return onboard_command(std::vector<std::string>(arguments.begin() + 1, arguments.end()));
  }
  if (command == "flighttest") {
    return flight_test_command(std::vector<std::string>(arguments.begin() + 1, arguments.end()));
  }
  if (command == "qualification") {
    return qualification_command(std::vector<std::string>(arguments.begin() + 1, arguments.end()));
  }
  if (command == "project") {
    return project_command(std::vector<std::string>(arguments.begin() + 1, arguments.end()));
  }

  std::cerr << "galata: unrecognised command '" << command << "'\n\n";
  (void)print_usage(std::cerr);
  return 2;
}
