// SPDX-License-Identifier: Apache-2.0
//
// The capabilities compiled into this build.
//
// Each one declares its own honest state. `galata capabilities` prints them,
// and the README's status table is generated from the same source, which is
// what stops the documentation from claiming more than the code does
// (charter rule 2).

#include "galata/analyze/disk_margin.hpp"
#include "galata/analyze/frequency_response.hpp"
#include "galata/analyze/margins.hpp"
#include "galata/analyze/modes.hpp"
#include "galata/analyze/sensitivity.hpp"
#include "galata/analyze/singular_values.hpp"
#include "galata/linearize/finite_difference.hpp"
#include "galata/model/aircraft.hpp"
#include "galata/model/linear_system.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/registry.hpp"
#include "galata/trim/level.hpp"
#include "galata/units.hpp"
#include "galata/version.hpp"

#include "html_report.hpp"

#include <algorithm>
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
  const model::LinearSystem system =
      model::parse_linear_system(context.read_input(context.input->string_at("path")), path);

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

// --- model.aircraft.derivatives --------------------------------------------

Artifact load_aircraft_model(const StageContext& context) {
  const std::string path = context.resolve_input_path(context.input->string_at("path"));
  const model::Aircraft aircraft =
      model::parse_aircraft(context.read_input(context.input->string_at("path")), path);

  std::ostringstream summary;
  summary << "derivative-buildup model";
  if (!aircraft.description.empty()) {
    summary << " — " << aircraft.description;
  }

  Artifact artifact;
  artifact.kind = "aircraft";
  artifact.summary = summary.str();
  artifact.payload = aircraft;
  return artifact;
}

// --- model.quadrotor -------------------------------------------------------

Artifact load_quadrotor_model(const StageContext& context) {
  const std::string path = context.resolve_input_path(context.input->string_at("path"));
  const model::Quadrotor quadrotor =
      model::parse_quadrotor(context.read_input(context.input->string_at("path")), path);

  std::ostringstream summary;
  summary << quadrotor.rotor_count() << " rotors, " << quadrotor.extended_state_size() << " states";
  if (!quadrotor.has_battery()) {
    summary << ", fixed voltage";
  }
  if (!quadrotor.description.empty()) {
    summary << " — " << quadrotor.description;
  }

  Artifact artifact;
  artifact.kind = "quadrotor";
  artifact.summary = summary.str();
  artifact.payload = quadrotor;
  return artifact;
}

// --- trim.level ------------------------------------------------------------

Artifact trim_level_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("aircraft");
  const auto& aircraft = upstream.payload_as<model::Aircraft>("aircraft");

  trim::LevelTrimRequest request;
  // Units are named in every key, per ADR-0003: a pipeline never carries a
  // bare "altitude" whose unit the reader has to guess.
  request.altitude_m = context.input->number_at("altitude_m");
  const ValuePtr airspeed = context.input->get("airspeed_m_s");
  const ValuePtr mach = context.input->get("mach");
  if (airspeed) {
    request.airspeed_m_s = airspeed->as_number();
  }
  if (mach) {
    request.mach = mach->as_number();
  }
  request.flight_path_angle_rad =
      units::degrees_to_radians(context.input->number_at("flight_path_angle_deg", 0.0));
  request.delta_isa_k = context.input->number_at("delta_isa_k", 0.0);
  request.residual_tolerance = context.input->number_at("tolerance", 1e-10);

  TrimArtifact trimmed{trim::trim_level(aircraft, request), aircraft};

  std::ostringstream summary;
  summary << std::fixed << std::setprecision(3) << "alpha "
          << units::radians_to_degrees(trimmed.point.alpha_rad) << " deg, elevator "
          << units::radians_to_degrees(trimmed.point.controls.elevator_rad) << " deg, thrust "
          << std::setprecision(0) << trimmed.point.controls.thrust_n << " N; residual "
          << std::scientific << std::setprecision(1) << trimmed.point.residual_norm;

  Artifact artifact;
  artifact.kind = "trim_point";
  artifact.summary = summary.str();
  artifact.payload = trimmed;
  return artifact;
}

// --- linearize.finitediff --------------------------------------------------

Artifact linearize_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("trim_point");
  const auto& trimmed = upstream.payload_as<TrimArtifact>("trim_point");

  linearize::LinearisationOptions options;
  const std::string axes = context.input->string_at("axes", "all");
  if (axes == "longitudinal") {
    options.state_subset = linearize::longitudinal_states();
  } else if (axes == "lateral") {
    options.state_subset = linearize::lateral_states();
  } else if (axes != "all") {
    throw std::runtime_error("'axes' is '" + axes
                             + "'; it must be 'longitudinal', 'lateral' or 'all'");
  }
  options.report_truncation_error = context.input->bool_at("report_truncation_error", true);
  options.equilibrium_tolerance = context.input->number_at("equilibrium_tolerance", 1e-10);

  const linearize::Linearisation linearisation =
      linearize::linearize_finite_difference(trimmed.aircraft, trimmed.point, options);

  std::ostringstream description;
  description << trimmed.aircraft.description << " — linearised about "
              << units::metres_to_feet(linearisation.trim_altitude_m) << " ft, "
              << units::radians_to_degrees(linearisation.trim_alpha_rad) << " deg alpha";

  std::ostringstream summary;
  summary << linearisation.a.rows() << " states, " << linearisation.b.cols() << " inputs (" << axes
          << "); chart conditioning " << linearisation.chart_conditioning
          << ", actual equilibrium residual " << std::scientific << std::setprecision(1)
          << linearisation.trim_residual_norm;
  if (options.report_truncation_error) {
    summary << ", worst truncation " << linearisation.worst_relative_truncation;
  } else {
    summary << ", truncation not estimated";
  }
  if (linearisation.neglected_coupling > 1e-6) {
    summary << ", neglected coupling " << linearisation.neglected_coupling;
  }

  Artifact artifact;
  artifact.kind = "linear_system";
  artifact.summary = summary.str();
  artifact.payload = linearisation.to_linear_system(description.str(), trimmed.aircraft.citation);
  artifact.linearization_evidence.emplace(
      context.stage_id, std::make_shared<const linearize::Linearisation>(linearisation));
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

void write_trim_section(std::ostream& out, const TrimArtifact& trimmed) {
  const trim::TrimPoint& point = trimmed.point;
  out << "| Quantity | Value |\n|---|---|\n";
  out << "| Altitude | " << format(units::metres_to_feet(-point.state.position_ned_m.z()), 0)
      << " ft |\n";
  out << "| Airspeed | " << format(point.airspeed_m_s, 3) << " m/s |\n";
  out << "| Mach | " << format(point.mach, 4) << " |\n";
  out << "| Dynamic pressure | " << format(point.dynamic_pressure_pa, 1) << " Pa |\n";
  out << "| Angle of attack | " << format(units::radians_to_degrees(point.alpha_rad), 4)
      << " deg |\n";
  out << "| Flight-path angle | "
      << format(units::radians_to_degrees(point.flight_path_angle_rad), 4) << " deg |\n";
  out << "| Elevator | " << format(units::radians_to_degrees(point.controls.elevator_rad), 4)
      << " deg |\n";
  out << "| Thrust | " << format(point.controls.thrust_n, 1) << " N |\n";
  out << "| Trim lift coefficient | " << format(point.lift_coefficient, 5) << " |\n";
  out << "\n**Evidence.** Residual norm " << format(point.residual_norm, 12)
      << " (m/s^2 and rad/s^2); Jacobian condition number "
      << format(point.jacobian_condition_number, 1) << ".\n\n";
  out << "A trim is only as good as its residual, so the residual is reported rather\n"
         "than asserted. The solver refuses to return an answer at all when it is above\n"
         "tolerance: a linearisation about a point that is not an equilibrium produces a\n"
         "state-space model that is plausible and wrong.\n\n";
  if (point.envelope.outside_advisory_envelope) {
    out << "> **Outside the model's advisory envelope.** The angle of attack is "
        << format(units::radians_to_degrees(point.envelope.alpha_departure_rad), 2)
        << " deg from the reference condition this derivative set was built about. A\n"
           "> first-order model returns a confident answer at any angle of attack it is\n"
           "> asked for, including ones where it has no stall and no business being used.\n\n";
  }
}

void write_linear_system_section(std::ostream& out, const model::LinearSystem& system) {
  if (!system.description.empty()) {
    out << "**Model.** " << system.description << "\n\n";
  }
  if (!system.units.empty()) {
    out << "**Units.** " << system.units << "\n\n";
  }
  out << "State matrix A, rows and columns in the order ";
  for (std::size_t i = 0; i < system.state_names.size(); ++i) {
    out << (i == 0 ? "" : ", ") << "`" << system.state_names[i] << "`";
  }
  out << ":\n\n```\n";
  for (Eigen::Index i = 0; i < system.a.rows(); ++i) {
    for (Eigen::Index j = 0; j < system.a.cols(); ++j) {
      out << "  " << format(system.a(i, j), 6);
    }
    out << "\n";
  }
  out << "```\n\n";
}

// --- analyze.freqresp ------------------------------------------------------

struct FrequencyResponseArtifact {
  analyze::FrequencyResponse response;
  std::string system_description;
  std::string system_citation;
  int input_index;
  int output_index;
};

// Resolve a name or an index against a list of names. Names are preferred in
// pipeline files because an index silently means something different the
// moment a model gains a state.
int resolve_channel(const ValuePtr& input,
                    const std::string& key,
                    const std::vector<std::string>& names,
                    const char* what) {
  const ValuePtr entry = input->get(key);
  if (!entry) {
    return 0;
  }
  if (entry->kind() == Value::Kind::Number) {
    const int index = input->integer_at(key, 0);
    if (index < 0 || index >= static_cast<int>(names.size())) {
      std::ostringstream message;
      message << what << " index " << index << " is out of range; the model has " << names.size();
      throw std::runtime_error(message.str());
    }
    return index;
  }
  const std::string& wanted = entry->as_string();
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (names[index] == wanted) {
      return static_cast<int>(index);
    }
  }
  std::ostringstream message;
  message << what << " '" << wanted << "' is not in the model. Available: ";
  for (std::size_t index = 0; index < names.size(); ++index) {
    message << (index == 0 ? "" : ", ") << names[index];
  }
  throw std::runtime_error(message.str());
}

analyze::MarginOptions read_margin_options(const StageContext& context) {
  analyze::MarginOptions options;
  options.start_rad_s = context.input->number_at("from_rad_s", options.start_rad_s);
  options.stop_rad_s = context.input->number_at("to_rad_s", options.stop_rad_s);
  options.grid_points = context.input->integer_at("points", options.grid_points);
  return options;
}

Artifact frequency_response_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("system");
  const auto& system = upstream.payload_as<model::LinearSystem>("linear_system");
  if (system.input_count() == 0) {
    throw std::runtime_error(
        "analyze.freqresp: the model has no inputs, so it has no transfer "
        "function");
  }

  const int input_index = resolve_channel(context.input, "input", system.input_names, "input");
  const int output_index =
      resolve_channel(context.input, "output", system.output_labels(), "output");

  const double from = context.input->number_at("from_rad_s", 1.0e-3);
  const double to = context.input->number_at("to_rad_s", 1.0e3);
  const int points = context.input->integer_at("points", 500);
  const bool refine = context.input->bool_at("refine_near_modes", true);

  const std::vector<double> grid = refine
                                       ? analyze::grid_refined_for_modes(system.a, from, to, points)
                                       : analyze::logarithmic_grid(from, to, points);

  FrequencyResponseArtifact result;
  result.response = analyze::single_loop_response(system, input_index, output_index, grid);
  result.system_description = system.description;
  result.system_citation = system.citation;
  result.input_index = input_index;
  result.output_index = output_index;

  std::ostringstream summary;
  summary << grid.size() << " frequencies from " << format(from, 4) << " to " << format(to, 1)
          << " rad/s, " << system.input_names.at(static_cast<std::size_t>(input_index)) << " to "
          << system.output_labels().at(static_cast<std::size_t>(output_index));

  Artifact artifact;
  artifact.kind = "frequency_response";
  artifact.summary = summary.str();
  artifact.payload = result;
  return artifact;
}

// --- analyze.margins -------------------------------------------------------

struct MarginArtifact {
  analyze::StabilityMargins margins;
  std::string loop_name;
  std::string system_description;
};

Artifact margins_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("system");
  const auto& system = upstream.payload_as<model::LinearSystem>("linear_system");
  if (system.input_count() == 0) {
    throw std::runtime_error("analyze.margins: the model has no inputs, so it has no loop");
  }

  const int input_index = resolve_channel(context.input, "input", system.input_names, "input");
  const int output_index =
      resolve_channel(context.input, "output", system.output_labels(), "output");

  MarginArtifact result;
  result.margins =
      analyze::stability_margins(system, input_index, output_index, read_margin_options(context));
  result.loop_name = system.input_names.at(static_cast<std::size_t>(input_index)) + " to "
                     + system.output_labels().at(static_cast<std::size_t>(output_index));
  result.system_description = system.description;

  std::ostringstream summary;
  if (result.margins.has_gain_margin) {
    summary << "GM " << format(result.margins.gain_margin_db, 2) << " dB";
  } else {
    summary << "GM not found in searched band";
  }
  summary << ", ";
  if (result.margins.has_phase_margin) {
    summary << "PM " << format(units::radians_to_degrees(result.margins.phase_margin_rad), 2)
            << " deg";
  } else {
    summary << "PM not found in searched band";
  }

  Artifact artifact;
  artifact.kind = "stability_margins";
  artifact.summary = summary.str();
  artifact.payload = result;
  return artifact;
}

// --- analyze.diskmargin ----------------------------------------------------

struct DiskMarginArtifact {
  analyze::DiskMargin margin;
  std::string loop_name;
  std::string system_description;
};

Artifact disk_margin_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("system");
  const auto& system = upstream.payload_as<model::LinearSystem>("linear_system");
  if (system.input_count() == 0) {
    throw std::runtime_error("analyze.diskmargin: the model has no inputs, so it has no loop");
  }

  const int input_index = resolve_channel(context.input, "input", system.input_names, "input");
  const int output_index =
      resolve_channel(context.input, "output", system.output_labels(), "output");
  const double skew = context.input->number_at("skew", 0.0);

  DiskMarginArtifact result;
  result.margin =
      analyze::disk_margin(system, input_index, output_index, skew, read_margin_options(context));
  result.loop_name = system.input_names.at(static_cast<std::size_t>(input_index)) + " to "
                     + system.output_labels().at(static_cast<std::size_t>(output_index));
  result.system_description = system.description;

  std::ostringstream summary;
  summary << "alpha " << format(result.margin.alpha, 4) << " at skew " << format(skew, 2)
          << ", peak at " << format(result.margin.critical_frequency_rad_s, 4) << " rad/s";

  Artifact artifact;
  artifact.kind = "disk_margin";
  artifact.summary = summary.str();
  artifact.payload = result;
  return artifact;
}

// --- analyze.sigma ---------------------------------------------------------

struct SigmaArtifact {
  analyze::SingularValueResponse response;
  std::string system_description;
};

Artifact sigma_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("system");
  const auto& system = upstream.payload_as<model::LinearSystem>("linear_system");
  if (system.input_count() == 0) {
    throw std::runtime_error(
        "analyze.sigma: the model has no inputs, so it has no transfer "
        "matrix");
  }

  const double from = context.input->number_at("from_rad_s", 1.0e-3);
  const double to = context.input->number_at("to_rad_s", 1.0e3);
  const int points = context.input->integer_at("points", 500);
  const bool refine = context.input->bool_at("refine_near_modes", true);
  const std::vector<double> grid = refine
                                       ? analyze::grid_refined_for_modes(system.a, from, to, points)
                                       : analyze::logarithmic_grid(from, to, points);

  SigmaArtifact result;
  result.response = analyze::singular_values(system, grid);
  result.system_description = system.description;

  std::ostringstream summary;
  summary << result.response.channel_count() << " principal gains, peak "
          << format(result.response.peak_gain, 4) << " at "
          << format(result.response.peak_frequency_rad_s, 4) << " rad/s";

  Artifact artifact;
  artifact.kind = "singular_values";
  artifact.summary = summary.str();
  artifact.payload = result;
  return artifact;
}

// --- analyze.sensitivity ---------------------------------------------------

struct SensitivityArtifact {
  analyze::SensitivityPeaks peaks;
  std::string system_description;
};

Artifact sensitivity_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("system");
  const auto& system = upstream.payload_as<model::LinearSystem>("linear_system");

  SensitivityArtifact result;
  result.peaks = analyze::sensitivity_peaks(system, read_margin_options(context));
  result.system_description = system.description;

  std::ostringstream summary;
  summary << "M_S " << format(result.peaks.sensitivity_peak, 4) << " at "
          << format(result.peaks.sensitivity_peak_frequency_rad_s, 4) << " rad/s, M_T "
          << format(result.peaks.complementary_peak, 4) << " at "
          << format(result.peaks.complementary_peak_frequency_rad_s, 4) << " rad/s";

  Artifact artifact;
  artifact.kind = "sensitivity_peaks";
  artifact.summary = summary.str();
  artifact.payload = result;
  return artifact;
}

// --- Markdown writers for the frequency-domain artefacts --------------------

std::string frequency_or_dash(bool present, double value) {
  return present ? format(value, 4) : "—";
}

void write_frequency_response_section(std::ostream& out,
                                      const FrequencyResponseArtifact& artifact) {
  const analyze::FrequencyResponse& response = artifact.response;
  if (!artifact.system_description.empty()) {
    out << "**Model.** " << artifact.system_description << "\n\n";
  }
  out << "**Loop.** `" << response.input_names.front() << "` to `" << response.output_names.front()
      << "`\n\n";
  out << "**Grid.** " << response.frequencies_rad_s.size() << " frequencies from "
      << format(response.frequencies_rad_s.front(), 5) << " to "
      << format(response.frequencies_rad_s.back(), 2) << " rad/s.\n\n";

  // A Bode table, decade by decade. The whole grid would be hundreds of rows
  // and no reader would look at it; a decade sample is a plot until there is a
  // plot.
  const std::vector<double> magnitude_db = response.magnitude_db();
  const std::vector<double> phase_rad = response.phase_rad();
  out << "| w (rad/s) | \\|G\\| (dB) | phase (deg) |\n";
  out << "| ---: | ---: | ---: |\n";
  const std::size_t stride = std::max<std::size_t>(1, response.frequencies_rad_s.size() / 20);
  for (std::size_t index = 0; index < response.frequencies_rad_s.size(); index += stride) {
    out << "| " << format(response.frequencies_rad_s[index], 5) << " | "
        << format(magnitude_db[index], 3) << " | "
        << format(units::radians_to_degrees(phase_rad[index]), 3) << " |\n";
  }
  out << "\n*Sampled every " << stride << " points of the grid.*\n\n";
}

void write_margins_section(std::ostream& out, const MarginArtifact& artifact) {
  const analyze::StabilityMargins& margins = artifact.margins;
  if (!artifact.system_description.empty()) {
    out << "**Model.** " << artifact.system_description << "\n\n";
  }
  out << "**Loop.** " << artifact.loop_name << "\n\n";
  if (!margins.nominal_stability_checked) {
    out << "**Nominal closed-loop stability.** Not checked. Delay boundaries assume "
           "the nominal loop is stable.\n\n";
  } else if (margins.nominal_closed_loop_stable) {
    out << "**Nominal closed-loop stability.** Checked: strictly stable.\n\n";
  } else {
    out << "**Nominal closed-loop stability.** Checked: unstable or marginal. No positive "
           "delay tolerance is established; crossover values alone do not establish "
           "stability.\n\n";
  }

  if (!margins.nominal_stability_diagnostic.empty()) {
    out << "**Stability assessment.** " << margins.nominal_stability_diagnostic << "\n\n";
  }
  out << "| margin | value | at (rad/s) |\n";
  out << "| --- | ---: | ---: |\n";
  out << "| Gain | "
      << (margins.has_gain_margin
              ? format(margins.gain_margin, 4) + " (" + format(margins.gain_margin_db, 2) + " dB)"
              : std::string("not found in searched band"))
      << " | " << frequency_or_dash(margins.has_gain_margin, margins.gain_margin_frequency_rad_s)
      << " |\n";
  out << "| Phase | "
      << (margins.has_phase_margin
              ? format(units::radians_to_degrees(margins.phase_margin_rad), 3) + " deg"
              : std::string("not found in searched band"))
      << " | " << frequency_or_dash(margins.has_phase_margin, margins.phase_margin_frequency_rad_s)
      << " |\n";
  out << "| Delay | "
      << (margins.has_delay_margin ? format(margins.delay_margin_s, 5) + " s" : std::string("none"))
      << " | " << frequency_or_dash(margins.has_delay_margin, margins.delay_margin_frequency_rad_s)
      << " |\n\n";

  // Every crossover, not only the governing one. A loop that crosses unity
  // three times has three phase margins, and a report that showed one would be
  // hiding the other two.
  if (margins.gain_crossings.size() > 1 || margins.phase_crossings.size() > 1) {
    out << "All crossovers:\n\n";
    out << "| kind | w (rad/s) | margin |\n| --- | ---: | ---: |\n";
    for (const auto& crossing : margins.gain_crossings) {
      out << "| \\|L\\| = 1 | " << format(crossing.frequency_rad_s, 5) << " | "
          << format(units::radians_to_degrees(crossing.phase_margin_rad), 3) << " deg |\n";
    }
    for (const auto& crossing : margins.phase_crossings) {
      out << "| phase = -180 | " << format(crossing.frequency_rad_s, 5) << " | "
          << format(crossing.gain_margin_db, 3) << " dB |\n";
    }
    out << "\n";
  }

  out << "*Searched " << format(margins.searched_from_rad_s, 5) << " to "
      << format(margins.searched_to_rad_s, 2) << " rad/s over " << margins.grid_points
      << " points. A crossover narrower than that spacing would not be found.*\n\n";
}

void write_disk_margin_section(std::ostream& out, const DiskMarginArtifact& artifact) {
  const analyze::DiskMargin& margin = artifact.margin;
  if (!artifact.system_description.empty()) {
    out << "**Model.** " << artifact.system_description << "\n\n";
  }
  out << "**Loop.** " << artifact.loop_name << " — skew sigma = " << format(margin.skew, 2)
      << (margin.skew == 0.0 ? " (symmetric)" : "") << "\n\n";

  out << "| quantity | value |\n| --- | ---: |\n";
  out << "| Disk margin alpha | " << format(margin.alpha, 5) << " |\n";
  out << "| Peak of \\|S + (sigma-1)/2\\| | " << format(margin.peak_gain, 5) << " |\n";
  out << "| Critical frequency | " << format(margin.critical_frequency_rad_s, 5) << " rad/s |\n";
  out << "| Estimated gain range | "
      << (margin.gain_variation_is_bounded
              ? format(margin.gain_variation_min, 4) + " to " + format(margin.gain_variation_max, 4)
                    + " (" + format(margin.gain_variation_min_db, 2) + " to "
                    + format(margin.gain_variation_max_db, 2) + " dB)"
              : format(margin.gain_variation_min, 4) + " upwards, unbounded")
      << " |\n";
  out << "| Estimated phase range | "
      << (margin.phase_variation_is_bounded
              ? "+/- " + format(units::radians_to_degrees(margin.phase_variation_rad), 3) + " deg"
              : std::string("any phase"))
      << " |\n\n";

  out << "A candidate boundary perturbation from the sampled peak: f = "
      << format(margin.destabilising_perturbation.real(), 4) << " "
      << (margin.destabilising_perturbation.imag() < 0.0 ? "-" : "+") << " "
      << format(std::abs(margin.destabilising_perturbation.imag()), 4) << "j, which places a "
      << "candidate closed-loop pole near s = j" << format(margin.critical_frequency_rad_s, 4)
      << ".\n\n";

  out << "**These sampled ranges are estimates, not guaranteed tolerances.** A missed peak "
         "can make the disk and its gain/phase ranges too large. Use analyze.robust_bounds "
         "for a separate numerical bound assessment.\n\n";
  out << "*The peak was found by searching " << format(margin.searched_from_rad_s, 5) << " to "
      << format(margin.searched_to_rad_s, 2) << " rad/s over " << margin.grid_points
      << " points and refining. A grid maximum understates the true peak, so this alpha is an "
         "upper bound on the true disk margin — the error is optimistic. Treat a marginal "
         "result as marginal.*\n\n";
}

void write_sigma_section(std::ostream& out, const SigmaArtifact& artifact) {
  const analyze::SingularValueResponse& response = artifact.response;
  if (!artifact.system_description.empty()) {
    out << "**Model.** " << artifact.system_description << "\n\n";
  }
  out << "**Channels.** " << response.channel_count() << " principal gains ("
      << response.output_names.size() << " outputs, " << response.input_names.size()
      << " inputs).\n\n";
  out << "**Peak gain.** " << format(response.peak_gain, 5) << " at "
      << format(response.peak_frequency_rad_s, 5) << " rad/s.\n\n";

  const std::vector<double> largest = response.largest();
  const std::vector<double> smallest = response.smallest();
  const std::vector<double> condition = response.condition_number();

  out << "| w (rad/s) | sigma_max | sigma_min | condition |\n";
  out << "| ---: | ---: | ---: | ---: |\n";
  const std::size_t stride = std::max<std::size_t>(1, response.frequencies_rad_s.size() / 20);
  for (std::size_t index = 0; index < response.frequencies_rad_s.size(); index += stride) {
    out << "| " << format(response.frequencies_rad_s[index], 5) << " | "
        << format(largest[index], 5) << " | " << format(smallest[index], 5) << " | "
        << (std::isinf(condition[index]) ? std::string("infinite") : format(condition[index], 3))
        << " |\n";
  }
  out << "\n*Sampled every " << stride
      << " points of the grid. The peak is a grid maximum "
         "and therefore a LOWER bound on the H-infinity norm.*\n\n";
}

void write_sensitivity_section(std::ostream& out, const SensitivityArtifact& artifact) {
  const analyze::SensitivityPeaks& peaks = artifact.peaks;
  if (!artifact.system_description.empty()) {
    out << "**Model.** " << artifact.system_description << "\n\n";
  }
  out << "| peak | value | at (rad/s) |\n| --- | ---: | ---: |\n";
  out << "| Sensitivity M_S | " << format(peaks.sensitivity_peak, 5) << " | "
      << format(peaks.sensitivity_peak_frequency_rad_s, 5) << " |\n";
  out << "| Complementary M_T | " << format(peaks.complementary_peak, 5) << " | "
      << format(peaks.complementary_peak_frequency_rad_s, 5) << " |\n\n";

  if (peaks.is_single_loop) {
    out << "For this single loop, M_S is the reciprocal of the shortest distance from the "
           "Nyquist curve to the critical point -1. The sampled distance estimate is "
        << format(1.0 / peaks.sensitivity_peak, 5) << ".\n\n";
  } else {
    out << "For this multi-loop system, the reciprocal of M_S estimates the smallest "
           "singular value of I+L over the searched frequencies: "
        << format(1.0 / peaks.sensitivity_peak, 5)
        << ". This is not a distance from a scalar Nyquist curve. Per-channel margins "
           "do not establish simultaneous multi-loop robustness.\n\n";
  }

  out << "**No guaranteed margins are derived from sampled peaks.** A sampled maximum "
         "is a lower estimate of the norm; the margin formulas require an upper norm bound. "
         "Use analyze.robust_bounds for a separate numerical bound assessment.\n\n";
  if (!peaks.is_single_loop) {
    out << "*The classical sensitivity-margin formulas are stated for SINGLE-LOOP systems; "
           "they do not establish simultaneous multi-loop robustness.*\n\n";
  }

  out << "*Both peaks are grid maxima and therefore LOWER bounds on the true H-infinity norms; "
         "for M_S that error makes the loop look more robust than it is. Searched "
      << format(peaks.searched_from_rad_s, 5) << " to " << format(peaks.searched_to_rad_s, 2)
      << " rad/s over " << peaks.grid_points << " points.*\n\n";
}

void write_linearization_evidence(std::ostream& out, const Artifact& artifact) {
  for (const auto& [source, evidence] : artifact.linearization_evidence) {
    out << "**Upstream linearization evidence — " << source
        << ".** "
           "These diagnostics describe the source Jacobian, not an error bound for this "
           "downstream result. Full step vectors and truncation matrices are retained in the "
           "run manifest.\n\n";
    out << std::scientific << std::setprecision(6)
        << "| Source diagnostic | Value |\n| --- | ---: |\n"
        << "| Euler chart conditioning | " << evidence->chart_conditioning << " |\n"
        << "| Minimum supported chart conditioning | " << linearize::kMinimumChartConditioning
        << " |\n"
        << "| Recomputed equilibrium residual (m/s^2, rad/s^2 norm) | "
        << evidence->trim_residual_norm << " |\n"
        << "| Equilibrium acceptance budget (same norm) | " << evidence->trim_residual_tolerance
        << " |\n"
        << "| Neglected coupling ratio | " << evidence->neglected_coupling << " |\n";
    if (evidence->a_truncation.size() > 0) {
      out << "| Worst relative Richardson truncation estimate | "
          << evidence->worst_relative_truncation << " |\n";
    } else {
      out << "| Richardson truncation estimate | not requested |\n";
    }
    out << "\n" << std::defaultfloat;
  }
}

std::string markdown_document(const StageContext& context) {
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
    if (artifact.kind == "aircraft") {
      const auto& aircraft = artifact.payload_as<model::Aircraft>("aircraft");
      out << "**Model.** " << aircraft.description << "\n\n"
          << "**Source.** " << aircraft.citation << "\n\n"
          << "| Property | Value |\n|---|---:|\n"
          << "| Mass (kg) | " << format(aircraft.mass.mass_kg, 6) << " |\n"
          << "| Wing area (m^2) | " << format(aircraft.geometry.wing_area_m2, 6) << " |\n"
          << "| Wing span (m) | " << format(aircraft.geometry.wing_span_m, 6) << " |\n"
          << "| Reference alpha (rad) | " << format(aircraft.aero.reference_alpha_rad, 6) << " |\n"
          << "| Reference Mach | " << format(aircraft.aero.reference_mach, 6) << " |\n\n"
          << "Local derivative-buildup model. The run manifest retains the exact coefficient "
             "input.\n\n";
    } else if (artifact.kind == "report") {
      out << "Written report: `" << artifact.payload_as<std::string>("report") << "`.\n\n";
    } else if (artifact.kind == "modal_table") {
      write_modal_section(out, std::any_cast<const ModalTable&>(artifact.payload));
    } else if (artifact.kind == "trim_point") {
      write_trim_section(out, std::any_cast<const TrimArtifact&>(artifact.payload));
    } else if (artifact.kind == "linear_system") {
      write_linear_system_section(out, std::any_cast<const model::LinearSystem&>(artifact.payload));
    } else if (artifact.kind == "frequency_response") {
      write_frequency_response_section(
          out, std::any_cast<const FrequencyResponseArtifact&>(artifact.payload));
    } else if (artifact.kind == "stability_margins") {
      write_margins_section(out, std::any_cast<const MarginArtifact&>(artifact.payload));
    } else if (artifact.kind == "singular_values") {
      write_sigma_section(out, std::any_cast<const SigmaArtifact&>(artifact.payload));
    } else if (artifact.kind == "sensitivity_peaks") {
      write_sensitivity_section(out, std::any_cast<const SensitivityArtifact&>(artifact.payload));
    } else if (artifact.kind == "disk_margin") {
      write_disk_margin_section(out, std::any_cast<const DiskMarginArtifact&>(artifact.payload));
    } else if (!write_design_section(out, artifact)) {
      throw std::runtime_error("no report writer exists for artifact kind '" + artifact.kind + "'");
    }
    write_linearization_evidence(out, artifact);
    out << "_Produced by `" << artifact.produced_by_capability << "`._\n\n";
  }

  // Charter rule 9: no number reaches the user without provenance.
  out << "---\n\n";
  out << "Generated by `" << galata::build_identification() << "`.\n";

  return out.str();
}

Artifact write_report(const StageContext& context, bool html) {
  const std::string relative = context.input->string_at("path");
  const auto markdown = markdown_document(context);
  std::vector<TimeSeriesChart> charts;
  if (html) {
    for (const auto& section : context.input->get("sections")->as_list()) {
      const auto& stage_id = section->as_stage_reference();
      for (auto chart : design_charts(context.upstream.at(stage_id))) {
        chart.title = stage_id + " — " + chart.title;
        charts.push_back(std::move(chart));
      }
    }
  }
  const auto bytes =
      html ? standalone_html(context.input->string_at("title", "galata report"), markdown, charts)
           : markdown;
  context.write_output(relative, bytes);
  Artifact artifact;
  artifact.kind = "report";
  artifact.summary = "wrote " + relative;
  artifact.payload = context.resolve_output_path(relative);
  return artifact;
}

Artifact write_markdown_report(const StageContext& context) {
  return write_report(context, false);
}

Artifact write_html_report(const StageContext& context) {
  return write_report(context, true);
}

Registry build_registry() {
  Registry registry;

  registry.add(
      Capability{"model.linear.statespace",
                 "Load a linear state-space model (A, B, state and input names) from a YAML file",
                 "linear_system",
                 Capability::State::ImplementedUnvalidated,
                 load_state_space,
                 {"path"},
                 {"path"}});

  registry.add(Capability{
      "analyze.modes",
      "Eigenvalues, modal metrics and participation factors, with the classical aircraft "
      "modes classified by participation",
      "modal_table",
      Capability::State::Implemented,
      analyze_modes_capability,
      {"system", "classify"}});

  registry.add(
      Capability{"model.aircraft.derivatives",
                 "Load a nonlinear aircraft model built from a non-dimensional derivative set",
                 "aircraft",
                 Capability::State::Implemented,
                 load_aircraft_model,
                 {"path"},
                 {"path"}});

  // ImplementedUnvalidated, deliberately. The validation cases behind this model
  // are exact invariants of its own equations and a cross-check against an
  // independent implementation; neither is a published reference, and RFC-0002's
  // acceptance section records why none anchors it.
  registry.add(Capability{"model.quadrotor",
                          "Load a nonlinear multirotor plant — rotors with first-order speed lag, "
                          "per-axis drag and an optional battery",
                          "quadrotor",
                          Capability::State::ImplementedUnvalidated,
                          load_quadrotor_model,
                          {"path"},
                          {"path"}});

  registry.add(Capability{
      "trim.level",
      "Solve straight-line trim — wings level, no sideslip — for angle of attack, elevator "
      "and thrust, by Newton on a square residual",
      "trim_point",
      Capability::State::Implemented,
      trim_level_capability,
      {"aircraft",
       "altitude_m",
       "airspeed_m_s",
       "mach",
       "flight_path_angle_deg",
       "delta_isa_k",
       "tolerance"}});

  registry.add(
      Capability{"linearize.finitediff",
                 "Linearise about a trim point by central differences, with a Richardson "
                 "truncation-error estimate per entry",
                 "linear_system",
                 Capability::State::Implemented,
                 linearize_capability,
                 {"trim_point", "axes", "report_truncation_error", "equilibrium_tolerance"}});

  registry.add(Capability{
      "analyze.freqresp",
      "Frequency response of one loop of a linear model, evaluated by Hessenberg solves "
      "with the grid refined around the system's own lightly damped modes",
      "frequency_response",
      Capability::State::Implemented,
      frequency_response_capability,
      {"system", "input", "output", "from_rad_s", "to_rad_s", "points", "refine_near_modes"}});

  registry.add(
      Capability{"analyze.margins",
                 "Gain, phase and delay margins of one loop, with every crossover reported and the "
                 "frequency at which each occurs",
                 "stability_margins",
                 Capability::State::Implemented,
                 margins_capability,
                 {"system", "input", "output", "from_rad_s", "to_rad_s", "points"}});

  registry.add(Capability{
      "analyze.diskmargin",
      "Disk margin of one loop — robustness to simultaneous gain and phase variation — with "
      "estimated gain and phase ranges and a candidate boundary perturbation",
      "disk_margin",
      Capability::State::Implemented,
      disk_margin_capability,
      {"system", "input", "output", "skew", "from_rad_s", "to_rad_s", "points"}});

  registry.add(Capability{
      "analyze.sigma",
      "Singular values of a MIMO transfer matrix over frequency — the principal gains, their "
      "spread, and the peak gain",
      "singular_values",
      Capability::State::Implemented,
      sigma_capability,
      {"system", "from_rad_s", "to_rad_s", "points", "refine_near_modes"}});

  registry.add(Capability{
      "analyze.sensitivity",
      "Sensitivity and complementary sensitivity peaks M_S and M_T of a loop closed with "
      "negative unit feedback, and the frequencies at which they occur",
      "sensitivity_peaks",
      Capability::State::Implemented,
      sensitivity_capability,
      {"system", "from_rad_s", "to_rad_s", "points"}});

  registry.add(Capability{"report.markdown",
                          "Write a Markdown report from upstream results",
                          "report",
                          Capability::State::ImplementedUnvalidated,
                          write_markdown_report,
                          {"path", "title", "sections"},
                          {},
                          {"path"}});

  registry.add(
      Capability{"report.html",
                 "Write a self-contained HTML report with readable tables and no remote resources",
                 "report",
                 Capability::State::ImplementedUnvalidated,
                 write_html_report,
                 {"path", "title", "sections"},
                 {},
                 {"path"}});

  register_data_capabilities(registry);
  register_design_capabilities(registry);
  register_model_capabilities(registry);
  register_quadrotor_capabilities(registry);
  return registry;
}

}  // namespace

const Registry& builtin_registry() {
  static const Registry registry = build_registry();
  return registry;
}

}  // namespace galata::pipeline
