// SPDX-License-Identifier: Apache-2.0
#include "provenance.hpp"

#include "galata/linearize/finite_difference.hpp"
#include "galata/version.hpp"

#include "galata_pipeline_provenance.hpp"

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>

namespace galata::pipeline {
namespace {
std::string hex_bytes(std::string_view bytes) {
  constexpr char hex[] = "0123456789abcdef";
  std::string out;
  out.reserve(bytes.size() * 2);
  for (const char raw : bytes) {
    const auto value = static_cast<unsigned char>(raw);
    out += hex[value >> 4U];
    out += hex[value & 15U];
  }
  return out;
}

void value_json(std::ostream& out, const Value& value) {
  switch (value.kind()) {
    case Value::Kind::Null:
      out << "null";
      break;
    case Value::Kind::Bool:
      out << (value.as_bool() ? "true" : "false");
      break;
    case Value::Kind::Number:
      out << std::setprecision(std::numeric_limits<double>::max_digits10) << value.as_number();
      break;
    case Value::Kind::String:
      out << json_quote(value.as_string());
      break;
    case Value::Kind::StageReference:
      out << "{\"from\":" << json_quote(value.as_stage_reference()) << '}';
      break;
    case Value::Kind::List: {
      out << '[';
      bool first = true;
      for (const auto& item : value.as_list()) {
        if (!first) {
          out << ',';
        }
        first = false;
        value_json(out, *item);
      }
      out << ']';
      break;
    }
    case Value::Kind::Map: {
      out << '{';
      bool first = true;
      for (const auto& [key, item] : value.as_map()) {
        if (!first) {
          out << ',';
        }
        first = false;
        out << json_quote(key) << ':';
        value_json(out, *item);
      }
      out << '}';
      break;
    }
  }
}

void file_json(std::ostream& out, const FileRecord& file, bool include_bytes) {
  out << "{\"path\":" << json_quote(file.path) << ",\"sha256\":" << json_quote(file.sha256)
      << ",\"size_bytes\":" << file.bytes.size();
  if (include_bytes) {
    out << ",\"bytes_hex\":" << json_quote(hex_bytes(file.bytes));
  }
  out << '}';
}

void matrix_json(std::ostream& out, const Eigen::MatrixXd& matrix) {
  out << '[';
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    out << (row == 0 ? "[" : ",[");
    for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
      if (col != 0) {
        out << ',';
      }
      out << matrix(row, col);
    }
    out << ']';
  }
  out << ']';
}

void linearization_json(std::ostream& out, const linearize::Linearisation& record) {
  out << std::setprecision(std::numeric_limits<double>::max_digits10)
      << "{\"chart_conditioning\":" << record.chart_conditioning
      << ",\"minimum_chart_conditioning\":" << linearize::kMinimumChartConditioning
      << ",\"equilibrium_residual\":" << record.trim_residual_norm
      << ",\"equilibrium_tolerance\":" << record.trim_residual_tolerance
      << ",\"worst_relative_truncation\":" << record.worst_relative_truncation
      << ",\"truncation_estimated\":" << (record.a_truncation.size() > 0 ? "true" : "false")
      << ",\"neglected_coupling\":" << record.neglected_coupling
      << ",\"altitude_m\":" << record.trim_altitude_m
      << ",\"airspeed_m_s\":" << record.trim_airspeed_m_s
      << ",\"alpha_rad\":" << record.trim_alpha_rad
      << ",\"delta_isa_k\":" << record.trim_delta_isa_k;
  out << ",\"state_names\":[";
  for (std::size_t index = 0; index < record.state_names.size(); ++index) {
    out << (index == 0 ? "" : ",") << json_quote(record.state_names[index]);
  }
  out << "],\"input_names\":[";
  for (std::size_t index = 0; index < record.input_names.size(); ++index) {
    out << (index == 0 ? "" : ",") << json_quote(record.input_names[index]);
  }
  out << "],\"a\":";
  matrix_json(out, record.a);
  out << ",\"b\":";
  matrix_json(out, record.b);
  out << ",\"c\":";
  matrix_json(out, record.c);
  out << ",\"d\":";
  matrix_json(out, record.d);
  out << ",\"state_steps\":";
  matrix_json(out, record.state_steps);
  out << ",\"control_steps\":";
  matrix_json(out, record.control_steps);
  out << ",\"a_truncation\":";
  matrix_json(out, record.a_truncation);
  out << ",\"b_truncation\":";
  matrix_json(out, record.b_truncation);
  out << '}';
}
}  // namespace

void validate_linearization_evidence(const linearize::Linearisation& record) {
  auto system = record.to_linear_system("source evidence", "");
  system.c = record.c;
  system.d = record.d;
  system.validate();
  const bool finite = record.state_steps.allFinite() && record.control_steps.allFinite()
                      && record.a_truncation.allFinite() && record.b_truncation.allFinite();
  if (!finite) {
    throw std::invalid_argument("linearization evidence: nonfinite steps or truncation matrix");
  }
  for (const double value : {record.chart_conditioning,
                             record.trim_residual_norm,
                             record.trim_residual_tolerance,
                             record.worst_relative_truncation,
                             record.neglected_coupling,
                             record.trim_altitude_m,
                             record.trim_delta_isa_k,
                             record.trim_airspeed_m_s,
                             record.trim_alpha_rad}) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("linearization evidence: nonfinite scalar diagnostic");
    }
  }
  if (record.b.rows() != record.a.rows()
      || static_cast<Eigen::Index>(record.input_names.size()) != record.b.cols()
      || record.state_steps.size() != record.a.rows()
      || record.control_steps.size() != record.b.cols() || (record.state_steps.array() <= 0.0).any()
      || (record.control_steps.array() <= 0.0).any()
      || (record.a_truncation.size() > 0
          && (record.a_truncation.rows() != record.a.rows()
              || record.a_truncation.cols() != record.a.cols()))
      || (record.b_truncation.size() > 0
          && (record.b_truncation.rows() != record.b.rows()
              || record.b_truncation.cols() != record.b.cols()))) {
    throw std::invalid_argument("linearization evidence: invalid steps or truncation shape");
  }
  if (record.chart_conditioning < linearize::kMinimumChartConditioning
      || record.chart_conditioning > 1.0 || record.trim_residual_norm < 0.0
      || record.trim_residual_tolerance <= 0.0
      || record.trim_residual_norm > record.trim_residual_tolerance
      || record.worst_relative_truncation < 0.0 || record.neglected_coupling < 0.0
      || record.trim_airspeed_m_s <= 0.0) {
    throw std::invalid_argument("linearization evidence: source diagnostics outside their domain");
  }
}

std::string write_run_manifest(const Pipeline& pipeline,
                               const RunResult& result,
                               RunFiles& files,
                               const RunOptions& options,
                               const std::string& input_directory,
                               const FileRecord& executable,
                               const RuntimeIdentity& runtime) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "{\n  \"schema\":\"galata.run.v1\",\n  \"status\":\"completed\",\n"
      << "  \"build\":{\"identification\":" << json_quote(galata::build_identification())
      << ",\"source_commit\":" << json_quote(GALATA_SOURCE_COMMIT)
      << ",\"source_status\":" << json_quote(GALATA_SOURCE_STATUS)
      << ",\"source_tree_sha256\":" << json_quote(GALATA_SOURCE_TREE_SHA256)
      << ",\"configuration_sha256\":" << json_quote(GALATA_BUILD_CONFIGURATION_SHA256)
      << ",\"configuration\":" << GALATA_BUILD_CONFIGURATION_JSON
      << ",\"dependency_manifest_sha256\":" << json_quote(GALATA_DEPENDENCY_MANIFEST_SHA256)
      << ",\"dependencies\":{\"Eigen\":" << json_quote(GALATA_EIGEN_VERSION)
      << ",\"yaml-cpp\":" << json_quote(GALATA_YAML_VERSION) << "}},\n"
      << "  \"executable\":{\"path\":" << json_quote(executable.path)
      << ",\"sha256\":" << json_quote(executable.sha256)
      << ",\"size_bytes\":" << executable.bytes.size() << "},\n";
  out << "  \"runtime\":{\"os_identity\":" << json_quote(runtime.os_identity)
      << ",\"scope\":\"loader inventory and on-disk identity; not process-memory attestation\""
      << ",\"modules\":[";
  bool first_module = true;
  for (const auto& module : runtime.modules) {
    out << (first_module ? "" : ",") << "{\"path\":" << json_quote(module.path)
        << ",\"storage\":" << json_quote(module.storage)
        << ",\"loader_identity\":" << json_quote(module.loader_identity)
        << ",\"sha256\":" << (module.sha256.empty() ? "null" : json_quote(module.sha256))
        << ",\"size_bytes\":"
        << (module.storage == "file" ? std::to_string(module.size_bytes) : "null") << '}';
    first_module = false;
  }
  out << "]},\n  \"output_directory\":" << json_quote(files.output_directory()) << ",\n"
      << "  \"input_directory\":"
      << json_quote(
             std::filesystem::weakly_canonical(input_directory.empty() ? "." : input_directory)
                 .string())
      << ",\n"
      << "  \"overwrite_requested\":" << (options.overwrite ? "true" : "false") << ",\n"
      << "  \"study\":";
  file_json(out,
            FileRecord{pipeline.source_name, sha256(pipeline.source_bytes), pipeline.source_bytes},
            true);
  out << ",\n  \"stages\":[";
  bool first = true;
  for (const auto& stage : pipeline.stages) {
    if (!first) {
      out << ',';
    }
    first = false;
    out << "{\"id\":" << json_quote(stage.id) << ",\"capability\":" << json_quote(stage.capability)
        << ",\"input\":";
    value_json(out, *stage.input);
    out << '}';
  }
  out << "],\n  \"execution_order\":[";
  first = true;
  for (const auto& stage : result.stages) {
    if (!first) {
      out << ',';
    }
    first = false;
    out << json_quote(stage.stage_id);
  }
  out << "],\n  \"linearization_evidence\":{";
  std::map<std::string, std::shared_ptr<const linearize::Linearisation>> linearizations;
  for (const auto& stage : result.stages) {
    linearizations.insert(stage.artifact.linearization_evidence.begin(),
                          stage.artifact.linearization_evidence.end());
  }
  first = true;
  for (const auto& [source, evidence] : linearizations) {
    out << (first ? "" : ",") << json_quote(source) << ':';
    linearization_json(out, *evidence);
    first = false;
  }
  out << "},\n  \"stage_linearization_sources\":{";
  first = true;
  for (const auto& stage : result.stages) {
    out << (first ? "" : ",") << json_quote(stage.stage_id) << ":[";
    bool first_source = true;
    for (const auto& [source, evidence] : stage.artifact.linearization_evidence) {
      (void)evidence;
      out << (first_source ? "" : ",") << json_quote(source);
      first_source = false;
    }
    out << ']';
    first = false;
  }
  out << "},\n  \"inputs\":[";
  first = true;
  for (const auto& [path, record] : files.inputs()) {
    (void)path;
    if (!first) {
      out << ',';
    }
    first = false;
    file_json(out, record, true);
  }
  out << "],\n  \"outputs\":[";
  first = true;
  for (const auto& [path, record] : files.outputs()) {
    (void)path;
    if (!first) {
      out << ',';
    }
    first = false;
    file_json(out, record, false);
  }
  out << "]\n}\n";
  const std::string bytes = out.str();
  const std::string name = "run-" + sha256(bytes) + ".json";
  files.write_output(name, bytes, true);
  return files.output_path(name);
}
}  // namespace galata::pipeline
