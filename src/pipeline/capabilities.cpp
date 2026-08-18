// SPDX-License-Identifier: Apache-2.0
//
// The capabilities compiled into this build.
//
// Each one declares its own honest state. `galata capabilities` prints them,
// and the README's status table is generated from the same source, which is
// what stops the documentation from claiming more than the code does
// (charter rule 2).

#include "galata/analyze/modes.hpp"
#include "galata/model/linear_system.hpp"
#include "galata/pipeline/registry.hpp"
#include "galata/version.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace galata::pipeline {
namespace {

std::string format(double value, int precision = 4) {
  if (std::isnan(value)) {
    return "—";
  }
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

// --- model.linear.statespace ----------------------------------------------

Artifact load_state_space(const StageContext& context) {
  const std::string path = context.resolve_input_path(context.input->string_at("path"));
  const model::LinearSystem system = model::load_linear_system(path);

  std::ostringstream summary;
  summary << system.state_count() << " states";
  if (system.input_count() > 0) {
    summary << ", " << system.input_count() << " inputs";
  }
  if (!system.description.empty()) {
    summary << " — " << system.description;
  }

  Artifact artifact;
  artifact.kind = "linear_system";
  artifact.summary = summary.str();
  artifact.payload = system;
  return artifact;
}

// --- analyze.modes ---------------------------------------------------------

struct ModalTable {
  analyze::ModalDecomposition decomposition;
  std::string system_description;
  std::string system_citation;
};

Artifact analyze_modes_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("system");
  const auto& system = upstream.payload_as<model::LinearSystem>("linear_system");

  const bool classify = context.input->bool_at("classify", true);
  analyze::StateRoles roles;
  if (classify) {
    roles = analyze::StateRoles::from_names(system.state_names);
  }

  ModalTable table;
  table.decomposition = analyze::analyze_modes(system.a, system.state_names, roles);
  table.system_description = system.description;
  table.system_citation = system.citation;

  std::ostringstream summary;
  summary << table.decomposition.modes.size() << " modes";
  int labelled = 0;
  for (const auto& mode : table.decomposition.modes) {
    if (mode.label != analyze::ModeLabel::Unclassified) {
      ++labelled;
    }
  }
  if (classify) {
    summary << ", " << labelled << " classified";
  }
  if (!table.decomposition.participation_is_meaningful) {
    summary << " (eigenvectors ill-conditioned: participation not reported)";
  }

  Artifact artifact;
  artifact.kind = "modal_table";
  artifact.summary = summary.str();
  artifact.payload = table;
  return artifact;
}

// --- report.markdown -------------------------------------------------------

void write_modal_section(std::ostream& out, const ModalTable& table) {
  if (!table.system_description.empty()) {
    out << "**Model.** " << table.system_description << "\n\n";
  }
  if (!table.system_citation.empty()) {
    out << "**Source.** " << table.system_citation << "\n\n";
  }

  out << "| Mode | Eigenvalue (1/s) | omega_n (rad/s) | zeta | Period (s) | "
         "T to half (s) | T to double (s) | Evidence |\n";
  out << "|---|---|---|---|---|---|---|---|\n";

  for (const auto& mode : table.decomposition.modes) {
    std::ostringstream eigenvalue;
    eigenvalue << format(mode.eigenvalue.real());
    if (mode.is_oscillatory) {
      eigenvalue << " ± " << format(mode.eigenvalue.imag()) << "j";
    }

    out << "| " << analyze::to_string(mode.label) << " | " << eigenvalue.str() << " | "
        << format(mode.natural_frequency_rad_s) << " | " << format(mode.damping_ratio) << " | "
        << format(mode.period_s, 3) << " | " << format(mode.time_to_half_amplitude_s, 3) << " | "
        << format(mode.time_to_double_amplitude_s, 3) << " | "
        << (mode.label_reason.empty() ? "—" : mode.label_reason) << " |\n";
  }

  out << "\nA dash means the quantity is not defined for that mode: a real root has no\n"
         "period, and a mode has either a time to half amplitude or a time to double,\n"
         "never both.\n\n";

  if (table.decomposition.participation_is_meaningful) {
    out << "### Participation factors\n\n";
    out << "| Mode |";
    for (const std::string& name : table.decomposition.state_names) {
      out << " " << name << " |";
    }
    out << "\n|---|";
    for (std::size_t i = 0; i < table.decomposition.state_names.size(); ++i) {
      out << "---|";
    }
    out << "\n";
    for (const auto& mode : table.decomposition.modes) {
      out << "| " << analyze::to_string(mode.label) << " |";
      for (const double value : mode.participation) {
        out << " " << format(value, 3) << " |";
      }
      out << "\n";
    }
    out << "\nParticipation is normalised to sum to one across states. It is the measure\n"
           "the classification rests on, so a label whose evidence looks thin here is a\n"
           "label to distrust.\n\n";
    out << "Eigenvector matrix condition number: "
        << format(table.decomposition.eigenvector_condition_number, 2) << ".\n\n";
  } else {
    out << "**Participation factors are not reported.** The eigenvector matrix has a\n"
           "condition number of "
        << format(table.decomposition.eigenvector_condition_number, 2)
        << ", which means the eigenvectors are nearly linearly\n"
           "dependent and participation factors would be numerical noise. The\n"
           "eigenvalues above are still well defined.\n\n";
  }
}

Artifact write_markdown_report(const StageContext& context) {
  const std::string relative = context.input->string_at("path");
  const std::string path = context.resolve_output_path(relative);
  const std::string title = context.input->string_at("title", "galata report");

  const ValuePtr sections = context.input->get("sections");
  if (!sections || sections->kind() != Value::Kind::List) {
    throw std::runtime_error("'sections' must be a list of stage references");
  }

  std::ostringstream out;
  out << "# " << title << "\n\n";

  for (const ValuePtr& section : sections->as_list()) {
    if (section->kind() != Value::Kind::StageReference) {
      throw std::runtime_error("each entry of 'sections' must be {from: stage_id}");
    }
    const std::string& stage_id = section->as_stage_reference();
    const auto found = context.upstream.find(stage_id);
    if (found == context.upstream.end()) {
      throw std::runtime_error("section refers to stage '" + stage_id
                               + "', which produced no output");
    }
    const Artifact& artifact = found->second;

    out << "## " << stage_id << "\n\n";
    if (artifact.kind == "modal_table") {
      write_modal_section(out, std::any_cast<const ModalTable&>(artifact.payload));
    } else {
      // Reporting an artefact kind this writer does not understand is a gap in
      // the writer, and it says so rather than silently omitting the section.
      out << "*No Markdown writer exists for an artefact of kind `" << artifact.kind
          << "`, so this section is empty. That is a missing feature, not an empty result.*\n\n";
    }
    out << "_Produced by `" << artifact.produced_by_capability << "`._\n\n";
  }

  // Charter rule 9: no number reaches the user without provenance.
  out << "---\n\n";
  out << "Generated by `" << galata::build_identification() << "`.\n";

  std::ofstream file(path);
  if (!file) {
    throw std::runtime_error("cannot write report to '" + path + "'");
  }
  file << out.str();
  if (!file) {
    throw std::runtime_error("failed while writing report to '" + path + "'");
  }

  Artifact artifact;
  artifact.kind = "report";
  artifact.summary = "wrote " + relative;
  artifact.payload = path;
  return artifact;
}

Registry build_registry() {
  Registry registry;

  registry.add(
      Capability{"model.linear.statespace",
                 "Load a linear state-space model (A, B, state and input names) from a YAML file",
                 "linear_system",
                 Capability::State::ImplementedUnvalidated,
                 load_state_space});

  registry.add(Capability{
      "analyze.modes",
      "Eigenvalues, modal metrics and participation factors, with the classical aircraft "
      "modes classified by participation",
      "modal_table",
      Capability::State::Implemented,
      analyze_modes_capability});

  registry.add(Capability{"report.markdown",
                          "Write a Markdown report from upstream results",
                          "report",
                          Capability::State::ImplementedUnvalidated,
                          write_markdown_report});

  return registry;
}

}  // namespace

const Registry& builtin_registry() {
  static const Registry registry = build_registry();
  return registry;
}

}  // namespace galata::pipeline
