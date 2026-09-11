// SPDX-License-Identifier: Apache-2.0
// File-driven control design and simulation adapters. All numeric algorithms
// live in synth/sim; this layer validates wiring, preserves names and renders.
#include "galata/analyze/gramians.hpp"
#include "galata/analyze/hinfinity.hpp"
#include "galata/identify/validate.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/charts.hpp"
#include "galata/sim/linear.hpp"
#include "galata/sim/nonlinear.hpp"
#include "galata/synth/control.hpp"

#include "input_schedule_parse.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

namespace galata::pipeline {
namespace {

ValuePtr required(const ValuePtr& input, const std::string& key) {
  auto value = input->get(key);
  if (!value) {
    throw std::invalid_argument("missing required input '" + key + "'");
  }
  return value;
}

Eigen::MatrixXd matrix(const ValuePtr& value, const std::string& name) {
  const auto& rows = value->as_list();
  if (rows.empty() || rows.size() > 256) {
    throw std::invalid_argument(name + ": expected 1..256 matrix rows");
  }
  const auto columns = rows.front()->as_list().size();
  if (columns == 0 || columns > 256) {
    throw std::invalid_argument(name + ": expected 1..256 matrix columns");
  }
  Eigen::MatrixXd result(static_cast<Eigen::Index>(rows.size()),
                         static_cast<Eigen::Index>(columns));
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const auto& row = rows[i]->as_list();
    if (row.size() != columns) {
      throw std::invalid_argument(name + ": ragged matrix");
    }
    for (std::size_t j = 0; j < columns; ++j) {
      result(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = row[j]->as_number();
    }
  }
  if (!result.allFinite()) {
    throw std::invalid_argument(name + ": finite entries required");
  }
  return result;
}

Eigen::VectorXd vector(const ValuePtr& value, Eigen::Index size, const std::string& name) {
  if (!value) {
    return Eigen::VectorXd::Zero(size);
  }
  const auto& entries = value->as_list();
  if (entries.size() != static_cast<std::size_t>(size)) {
    throw std::invalid_argument(name + ": length does not match model");
  }
  Eigen::VectorXd result(size);
  for (Eigen::Index i = 0; i < size; ++i) {
    result(i) = entries[static_cast<std::size_t>(i)]->as_number();
  }
  if (!result.allFinite()) {
    throw std::invalid_argument(name + ": finite entries required");
  }
  return result;
}

void keys(const ValuePtr& value, const std::vector<std::string>& allowed, const std::string& name) {
  for (const auto& [key, entry] : value->as_map()) {
    (void)entry;
    if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
      throw std::invalid_argument(name + ": unknown key '" + key + "'");
    }
  }
}

// A string literal default materialises a temporary at every call site, and GCC's
// -Wdangling-reference cannot then prove the returned reference points into the
// upstream artifact rather than into that temporary. Naming the default removes
// the temporary, which settles the question rather than suppressing it.
const std::string kDefaultSystemKey = "system";

const model::LinearSystem& system_at(const StageContext& context,
                                     const std::string& key = kDefaultSystemKey) {
  return context.upstream_at(key).payload_as<model::LinearSystem>("linear_system");
}

Artifact system_artifact(model::LinearSystem system) {
  Artifact result;
  result.kind = "linear_system";
  result.summary = std::to_string(system.state_count()) + " states, "
                   + std::to_string(system.input_count()) + " inputs, "
                   + std::to_string(system.output_count()) + " outputs";
  result.payload = std::move(system);
  return result;
}

std::vector<Eigen::Index> channels(const ValuePtr& value, const std::vector<std::string>& names) {
  std::vector<Eigen::Index> indices;
  if (!value) {
    for (std::size_t i = 0; i < names.size(); ++i) {
      indices.push_back(static_cast<Eigen::Index>(i));
    }
    return indices;
  }
  std::set<std::string> seen;
  for (const auto& item : value->as_list()) {
    const auto name = item->as_string();
    const auto found = std::find(names.begin(), names.end(), name);
    if (found == names.end() || !seen.insert(name).second) {
      throw std::invalid_argument("unknown or duplicated channel '" + name + "'");
    }
    indices.push_back(std::distance(names.begin(), found));
  }
  if (indices.empty()) {
    throw std::invalid_argument("channel selection must not be empty");
  }
  return indices;
}

Artifact select_channels(const StageContext& context) {
  const auto& original = system_at(context);
  const auto output_labels = original.output_labels();
  const auto inputs = channels(context.input->get("inputs"), original.input_names);
  const auto outputs = channels(context.input->get("outputs"), output_labels);
  model::LinearSystem selected = original;
  selected.b.resize(original.state_count(), static_cast<Eigen::Index>(inputs.size()));
  selected.c.resize(static_cast<Eigen::Index>(outputs.size()), original.state_count());
  selected.d.resize(static_cast<Eigen::Index>(outputs.size()),
                    static_cast<Eigen::Index>(inputs.size()));
  selected.input_names.clear();
  selected.output_names.clear();
  const auto c = original.output_matrix(), d = original.feedthrough_matrix();
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    selected.b.col(static_cast<Eigen::Index>(i)) = original.b.col(inputs[i]);
    selected.input_names.push_back(original.input_names[static_cast<std::size_t>(inputs[i])]);
  }
  for (std::size_t i = 0; i < outputs.size(); ++i) {
    selected.c.row(static_cast<Eigen::Index>(i)) = c.row(outputs[i]);
    selected.output_names.push_back(output_labels[static_cast<std::size_t>(outputs[i])]);
    for (std::size_t j = 0; j < inputs.size(); ++j) {
      selected.d(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) =
          d(outputs[i], inputs[j]);
    }
  }
  selected.validate();
  return system_artifact(std::move(selected));
}

Artifact care(const StageContext& context) {
  const auto a = matrix(required(context.input, "a"), "a");
  const auto b = matrix(required(context.input, "b"), "b");
  const auto q = matrix(required(context.input, "q"), "q");
  const auto r = matrix(required(context.input, "r"), "r");
  const auto n = context.input->get("n");
  auto solved = synth::solve_care(a, b, q, r, n ? matrix(n, "n") : Eigen::MatrixXd{});
  Artifact result;
  result.kind = "care_solution";
  std::ostringstream summary;
  summary << a.rows() << " states; stabilising CARE residual " << std::scientific
          << solved.relative_residual;
  result.summary = summary.str();
  result.payload = std::move(solved);
  return result;
}

Artifact lqr(const StageContext& context) {
  if (context.input->string_at("break_at") != "plant_input") {
    throw std::invalid_argument(
        "synth.lqr: break_at must be plant_input; margins apply at this loop break");
  }
  const auto q = matrix(required(context.input, "q"), "q");
  const auto r = matrix(required(context.input, "r"), "r");
  const auto n = context.input->get("n");
  auto design = synth::design_lqr(system_at(context), q, r, n ? matrix(n, "n") : Eigen::MatrixXd{});
  Artifact result;
  result.kind = "control_law";
  std::ostringstream summary;
  summary << design.riccati.k.rows() << " controls, " << design.riccati.k.cols()
          << " states; CARE residual " << std::scientific << design.riccati.relative_residual;
  result.summary = summary.str();
  result.payload = std::move(design);
  return result;
}

// --- analyze.gramians ------------------------------------------------------
//
// RFC-0002 asked for this so that "an exported model's defective integrator
// chains and any unobservable direction are reported rather than discovered
// from a failed synthesis". The horizon is required rather than defaulted: see
// the header for why a default would put a number nobody chose into a figure a
// reader quotes.
Artifact gramians(const StageContext& context) {
  const model::LinearSystem system = system_at(context);
  analyze::GramianOptions options;
  options.horizon_s = context.input->number_at("horizon_s");
  options.steps = context.input->integer_at("steps", 400);
  options.rank_tolerance = context.input->number_at("rank_tolerance", 1e-9);
  auto analysis = analyze::analyse_gramians(system, options);

  std::ostringstream summary;
  summary << "reachable " << analysis.reachability.rank << "/" << analysis.reachability.state_count
          << ", observable " << analysis.observability.rank << "/"
          << analysis.observability.state_count << " at a relative floor of " << std::scientific
          << std::setprecision(1) << options.rank_tolerance;
  summary.unsetf(std::ios::floatfield);
  // The one thing a reader most needs and would otherwise assume: these are
  // finite-horizon Gramians, and for this class of model the infinite-horizon
  // ones do not exist at all.
  summary << "; finite-horizon Gramians over " << std::fixed << std::setprecision(2)
          << analysis.horizon_s << " s";
  summary.unsetf(std::ios::floatfield);
  if (!analysis.spectrum_is_strictly_stable) {
    summary << " (the infinite-horizon Gramians DO NOT EXIST for this model — rightmost "
               "eigenvalue real part "
            << std::scientific << std::setprecision(3) << analysis.rightmost_eigenvalue_real_part
            << ")";
    summary.unsetf(std::ios::floatfield);
  }
  if (!analysis.reachability.missing_directions.empty()
      || !analysis.observability.missing_directions.empty()) {
    summary << "; "
            << analysis.reachability.missing_directions.size()
                   + analysis.observability.missing_directions.size()
            << " direction(s) named in the report";
  }

  Artifact artifact;
  artifact.kind = "gramians";
  artifact.summary = summary.str();
  artifact.payload = std::move(analysis);
  return artifact;
}

Artifact control_system(const StageContext& context) {
  const auto& law = context.upstream_at("law").payload_as<synth::LqrDesign>("control_law");
  const auto use = context.input->string_at("use");
  if (use == "closed_loop") {
    return system_artifact(law.closed_loop);
  }
  if (use == "broken_loop") {
    return system_artifact(law.broken_loop);
  }
  // The loop-at-a-time reading, which is the one a frequency-domain margin can
  // be computed from on a plant that needs all its channels. See
  // `synth::single_loop_others_closed` for why `broken_loop` cannot be handed
  // to a SISO margin routine on such a plant, and for what a set of these
  // figures does NOT bound.
  if (use == "single_loop") {
    const ValuePtr channel = context.input->get("channel");
    if (!channel) {
      throw std::invalid_argument(
          "model.control_system: `use: single_loop` needs `channel`, naming which plant input "
          "the loop is broken at. There is one such loop per input and they are different "
          "loops with different margins; picking one for the caller would be choosing which "
          "number to report");
    }
    const std::vector<std::string>& inputs = law.plant.input_names;
    int index = -1;
    if (channel->kind() == Value::Kind::Number) {
      index = static_cast<int>(channel->as_number());
    } else {
      const std::string name = channel->as_string();
      for (std::size_t i = 0; i < inputs.size(); ++i) {
        if (inputs[i] == name) {
          index = static_cast<int>(i);
          break;
        }
      }
      if (index < 0) {
        std::ostringstream message;
        message << "model.control_system: the plant has no input '" << name << "'. It has: ";
        for (std::size_t i = 0; i < inputs.size(); ++i) {
          message << (i == 0 ? "" : ", ") << inputs[i];
        }
        throw std::invalid_argument(message.str());
      }
    }
    return system_artifact(synth::single_loop_others_closed(law, index));
  }
  throw std::invalid_argument(
      "model.control_system: use must be closed_loop, broken_loop or single_loop");
}

analyze::HinfinityOptions norm_options(const ValuePtr& input) {
  analyze::HinfinityOptions options;
  if (input->get("relative_tolerance")) {
    options.relative_tolerance = input->number_at("relative_tolerance");
  }
  if (input->get("absolute_tolerance")) {
    options.absolute_tolerance = input->number_at("absolute_tolerance");
  }
  return options;
}

Artifact hinfinity(const StageContext& context) {
  auto norm = analyze::hinfinity_norm(system_at(context), norm_options(context.input));
  if (!norm.numerically_reliable || !norm.tolerance_met) {
    throw std::runtime_error("analyze.hinfnorm: " + norm.diagnostic);
  }
  Artifact artifact;
  artifact.kind = "hinfinity_norm";
  std::ostringstream summary;
  summary << "H-infinity norm in [" << std::setprecision(10) << norm.lower_bound << ", "
          << norm.upper_bound << "]; " << norm.diagnostic;
  artifact.summary = summary.str();
  artifact.payload = std::move(norm);
  return artifact;
}

void norm_table(std::ostream& out, const analyze::HinfinityNorm& norm) {
  out << std::setprecision(17) << "| Numerical evidence | Value |\n|---|---|\n"
      << "| Norm lower bound | " << norm.lower_bound << " |\n"
      << "| Norm upper bound | " << norm.upper_bound << " |\n"
      << "| DC gain | " << norm.dc_gain << " |\n"
      << "| Infinite-frequency feedthrough gain | " << norm.feedthrough_gain << " |\n"
      << "| Relative bracket width | " << norm.relative_gap << " |\n"
      << "| Hamiltonian evaluations | " << norm.hamiltonian_evaluations << " |\n\n"
      << norm.diagnostic << "\n\n";
}

struct RobustBounds {
  analyze::SensitivityNormBounds sensitivity;
  analyze::DiskMarginBounds disk;
  bool has_disk = false;
};

Artifact robust_bounds(const StageContext& context) {
  const auto& loop = system_at(context);
  const auto options = norm_options(context.input);
  RobustBounds bounds;
  bounds.sensitivity = analyze::sensitivity_norm_bounds(loop, options);
  const auto check = [](const analyze::HinfinityNorm& norm) {
    if (!norm.numerically_reliable || !norm.tolerance_met) {
      throw std::runtime_error("analyze.robust_bounds: " + norm.diagnostic);
    }
  };
  check(bounds.sensitivity.sensitivity);
  check(bounds.sensitivity.complementary);
  bounds.has_disk = loop.input_count() == 1;
  if (bounds.has_disk) {
    const double skew = context.input->get("skew") ? context.input->number_at("skew") : 0.0;
    bounds.disk = analyze::disk_margin_bounds(loop, skew, options);
    check(bounds.disk.shifted_sensitivity);
  } else if (context.input->get("skew")) {
    throw std::invalid_argument("analyze.robust_bounds: disk skew applies only to a SISO loop");
  }
  Artifact artifact;
  artifact.kind = "robust_bounds";
  artifact.summary = "Internally stable negative-feedback loop; Hamiltonian S/T norm bounds"
                     + std::string(bounds.has_disk ? " and SISO disk bounds" : "");
  artifact.payload = std::move(bounds);
  return artifact;
}

struct LinearRun {
  model::LinearSystem system;
  numerics::Trajectory trajectory;
  // The constant input, or a declared history's value at t = 0.
  Eigen::VectorXd input;
  // Under a declared history, the input IN FORCE at each recorded sample —
  // right-continuous at events, the value the next step integrates under. Empty
  // for a constant run, whose every sample is `input`.
  std::vector<Eigen::VectorXd> input_samples;
  std::string history;  // how the history was declared; empty for a constant run
  std::vector<int> event_steps;

  [[nodiscard]] const Eigen::VectorXd& input_at(std::size_t sample) const {
    return input_samples.empty() ? input : input_samples[sample];
  }
};

// A constant input or a declared history, never both: a run given both would
// leave it unsaid which one drove it. The history's schema is `sim.plant`'s, so
// one schedule means the same thing on the linear and the nonlinear path; its
// semantics are in include/galata/sim/linear.hpp.
Artifact linear_simulation(const StageContext& context) {
  LinearRun run;
  run.system = system_at(context);
  const bool has_constant = context.input->get("constant_input") != nullptr;
  const bool has_history = context.input->get("input_schedule") != nullptr;
  if (has_constant && has_history) {
    throw std::invalid_argument(
        "sim.linear: give `constant_input` or `input_schedule`, not both. A run given both "
        "leaves it unsaid which input drove it");
  }
  const Eigen::VectorXd initial =
      vector(context.input->get("initial_state"), run.system.state_count(), "initial_state");
  const double step_s = context.input->number_at("step_s");
  const int steps = context.input->integer_at("steps", 0);
  const int stride = context.input->integer_at("sample_stride", 1);
  std::ostringstream summary;
  if (has_history) {
    const ValuePtr declared = context.input->get("input_schedule");
    const sim::InputSchedule schedule =
        schedule_at(context, "input_schedule", static_cast<int>(run.system.input_count()));
    sim::ScheduledLinearRun scheduled =
        sim::simulate_linear(run.system, initial, schedule, step_s, steps, stride);
    run.trajectory = std::move(scheduled.trajectory);
    run.input_samples = std::move(scheduled.input_samples);
    run.input = run.input_samples.front();
    run.event_steps = std::move(scheduled.event_steps);
    std::ostringstream history;
    history << declared->string_at("hold") << " hold, extrapolation "
            << declared->string_at("extrapolation") << ", "
            << declared->get("samples")->as_list().size() << " sample(s), "
            << run.event_steps.size() << " event(s) inside the run";
    run.history = history.str();
    summary << run.trajectory.states.size() << " samples; declared input history (" << run.history
            << ")";
  } else {
    run.input =
        vector(context.input->get("constant_input"), run.system.input_count(), "constant_input");
    run.trajectory = sim::simulate_linear(run.system, initial, run.input, step_s, steps, stride);
    summary << run.trajectory.states.size() << " samples; completed linear response";
  }
  Artifact result;
  result.kind = "linear_trajectory";
  result.summary = summary.str();
  result.payload = std::move(run);
  return result;
}

Artifact nonlinear_simulation(const StageContext& context) {
  const auto& trimmed = context.upstream_at("trim_point").payload_as<TrimArtifact>("trim_point");
  sim::NonlinearRequest request;
  request.step_s = context.input->number_at("step_s");
  request.step_count = context.input->integer_at("steps", 0);
  request.sample_stride = context.input->integer_at("sample_stride", 1);
  // The CLI never opts out of the model envelope. An incomplete study must not
  // reach a successful report; programmatic experiments may explicitly opt out.
  request.stop_outside_envelope = true;
  const auto actuators = required(context.input, "actuators");
  const std::vector<std::string> names = {"elevator", "aileron", "rudder", "thrust"};
  keys(actuators, names, "actuators");
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto specification = required(actuators, names[i]);
    const std::string unit = i == 3 ? "n" : "rad";
    keys(specification,
         {"minimum_" + unit, "maximum_" + unit, "rate_limit_" + unit + "_s", "time_constant_s"},
         names[i]);
    request.actuators[i] = {specification->number_at("minimum_" + unit),
                            specification->number_at("maximum_" + unit),
                            specification->number_at("rate_limit_" + unit + "_s"),
                            specification->number_at("time_constant_s")};
  }
  if (context.input->get("law")) {
    const auto& law = context.upstream_at("law").payload_as<synth::LqrDesign>("control_law");
    request.feedback = {law.riccati.k, law.plant.state_names, law.plant.input_names};
  }
  if (const auto initial = context.input->get("initial_perturbation")) {
    request.initial_perturbation.resize(static_cast<Eigen::Index>(initial->as_map().size()));
    Eigen::Index i = 0;
    for (const auto& [name, value] : initial->as_map()) {
      request.perturbation_state_names.push_back(name);
      request.initial_perturbation(i++) = value->as_number();
    }
  }
  if (const auto commands = context.input->get("command_increment")) {
    keys(commands, {"elevator_rad", "aileron_rad", "rudder_rad", "thrust_n"}, "command_increment");
    request.command_increment.elevator_rad = commands->number_at("elevator_rad", 0);
    request.command_increment.aileron_rad = commands->number_at("aileron_rad", 0);
    request.command_increment.rudder_rad = commands->number_at("rudder_rad", 0);
    request.command_increment.thrust_n = commands->number_at("thrust_n", 0);
  }
  auto run = sim::simulate_nonlinear(trimmed.aircraft, trimmed.point, request);
  if (!run.completed) {
    throw std::runtime_error("nonlinear simulation incomplete after "
                             + std::to_string(run.completed_steps)
                             + " steps: " + run.termination_reason);
  }
  Artifact result;
  result.kind = "nonlinear_trajectory";
  result.summary = std::to_string(run.samples.size()) + " samples; completed within model envelope";
  result.payload = std::move(run);
  return result;
}

void matrix_table(std::ostream& out,
                  const std::string& name,
                  const Eigen::MatrixXd& value,
                  const std::vector<std::string>& rows = {},
                  const std::vector<std::string>& columns = {}) {
  out << "**" << name << "**\n\n| row |";
  for (Eigen::Index j = 0; j < value.cols(); ++j) {
    out << ' ' << (columns.empty() ? std::to_string(j) : columns[static_cast<std::size_t>(j)])
        << " |";
  }
  out << "\n|---|";
  for (Eigen::Index j = 0; j < value.cols(); ++j) {
    out << "---:|";
  }
  out << '\n';
  for (Eigen::Index i = 0; i < value.rows(); ++i) {
    out << "| " << (rows.empty() ? std::to_string(i) : rows[static_cast<std::size_t>(i)]) << " |";
    for (Eigen::Index j = 0; j < value.cols(); ++j) {
      out << ' ' << std::setprecision(12) << value(i, j) << " |";
    }
    out << '\n';
  }
  out << '\n';
}

void care_evidence(std::ostream& out, const synth::CareSolution& solution) {
  out << "| Numerical check | Value |\n|---|---:|\n"
      << std::scientific << std::setprecision(6) << "| Relative CARE residual | "
      << solution.relative_residual << " |\n"
      << "| Residual acceptance budget | " << solution.residual_budget << " |\n"
      << "| Symmetry defect | " << solution.symmetry_defect << " |\n"
      << "| Stable-subspace condition | " << solution.subspace_condition << " |\n"
      << "| Hamiltonian distance from imaginary axis (1/s) | " << solution.hamiltonian_separation
      << " |\n\n"
      << std::defaultfloat;
}

std::string csv_label(const std::string& label) {
  std::string out = "\"";
  for (char c : label) {
    if (c == '"') {
      out += '"';
    }
    out += c;
  }
  return out + '"';
}

Artifact csv(const StageContext& context) {
  const auto& source = context.upstream_at("trajectory");
  std::ostringstream out;
  out << std::setprecision(17);
  if (source.kind == "linear_trajectory") {
    const auto& run = source.payload_as<LinearRun>("linear_trajectory");
    out << "time_s";
    for (const auto& name : run.system.state_names) {
      out << ',' << csv_label("state:" + name);
    }
    for (const auto& name : run.system.output_labels()) {
      out << ',' << csv_label("output:" + name);
    }
    // Under a declared history the input changes, so it is a column: a reader
    // reconstructing an output from the states needs the D u that produced it.
    // A constant run keeps the header it always had.
    const bool history = !run.input_samples.empty();
    if (history) {
      for (const auto& name : run.system.input_names) {
        out << ',' << csv_label("input:" + name);
      }
    }
    out << '\n';
    for (std::size_t i = 0; i < run.trajectory.states.size(); ++i) {
      out << run.trajectory.times_s[i];
      const auto& x = run.trajectory.states[i];
      const Eigen::VectorXd& u = run.input_at(i);
      const Eigen::VectorXd y =
          run.system.output_matrix() * x + run.system.feedthrough_matrix() * u;
      for (Eigen::Index j = 0; j < x.size(); ++j) {
        out << ',' << x(j);
      }
      for (Eigen::Index j = 0; j < y.size(); ++j) {
        out << ',' << y(j);
      }
      if (history) {
        for (Eigen::Index j = 0; j < u.size(); ++j) {
          out << ',' << u(j);
        }
      }
      out << '\n';
    }
  } else if (source.kind == "nonlinear_trajectory") {
    const auto& run = source.payload_as<sim::NonlinearResult>("nonlinear_trajectory");
    out << "time_s,p_n_m,p_e_m,p_d_m,u_m_s,v_m_s,w_m_s,qw,qx,qy,qz,p_rad_s,q_rad_s,r_rad_s,"
           "elevator_rad,aileron_rad,rudder_rad,thrust_n\n";
    for (const auto& sample : run.samples) {
      out << sample.time_s;
      const auto state = sample.state.to_vector();
      const auto controls = sample.controls.to_vector();
      for (Eigen::Index j = 0; j < state.size(); ++j) {
        out << ',' << state(j);
      }
      for (Eigen::Index j = 0; j < controls.size(); ++j) {
        out << ',' << controls(j);
      }
      out << '\n';
    }
  } else if (source.kind == "sampled_trajectory") {
    // The plant rows, plus what the controller asked for beside what it got. A
    // reader comparing the two sees the authority the law wanted and did not
    // have; a file with only the applied command cannot show that.
    const auto& sampled = source.payload_as<SampledRun>("sampled_trajectory");
    const PlantRun& plant = sampled.plant;
    out << "time_s";
    for (const auto& name : plant.state_names) {
      out << ',' << csv_label("state:" + name);
    }
    for (Eigen::Index j = 0; j < plant.command_rad_s.size(); ++j) {
      const std::string index = std::to_string(j);
      out << ',' << csv_label("requested:omega_" + index + "_rad_s");
      out << ',' << csv_label("applied:omega_" + index + "_rad_s");
    }
    out << ',' << csv_label("wind:north_m_s") << ',' << csv_label("wind:east_m_s") << ','
        << csv_label("wind:down_m_s");
    // A discrete design's prediction of its own loop, beside what the plant did,
    // in the same chart coordinates and at the same ticks. Both columns, because
    // the discrepancy the report summarises is only checkable from the pair.
    const LinearPredictionRecord& prediction = sampled.control.prediction;
    if (prediction.available) {
      for (const std::string& name : prediction.chart_names) {
        out << ',' << csv_label("chart:" + name) << ',' << csv_label("predicted:" + name);
      }
    }
    out << '\n';
    for (std::size_t i = 0; i < plant.trajectory.states.size(); ++i) {
      out << plant.trajectory.times_s[i];
      const auto& x = plant.trajectory.states[i];
      for (Eigen::Index j = 0; j < x.size(); ++j) {
        out << ',' << x(j);
      }
      // The final sample is the state after the last hold; it has no tick of
      // its own, so it repeats the last tick's commands rather than inventing
      // one that was never issued.
      const std::size_t tick = std::min(i, sampled.control.requested_rad_s.size() - 1);
      for (Eigen::Index j = 0; j < plant.command_rad_s.size(); ++j) {
        out << ',' << sampled.control.requested_rad_s[tick](j);
        out << ',' << sampled.control.applied_rad_s[tick](j);
      }
      const Eigen::Vector3d& wind_now = plant.wind_samples_ned_m_s[i];
      out << ',' << wind_now.x() << ',' << wind_now.y() << ',' << wind_now.z();
      if (prediction.available) {
        const Eigen::VectorXd& measured = prediction.measured_chart[i];
        const Eigen::VectorXd& predicted = prediction.predicted_chart[i];
        for (Eigen::Index j = 0; j < measured.size(); ++j) {
          out << ',' << measured(j) << ',' << predicted(j);
        }
      }
      out << '\n';
    }
  } else if (source.kind == "plant_trajectory") {
    // The columns are the model's, so they are read from the run rather than
    // written down here: a six-rotor vehicle and a four-rotor one with a battery
    // have different widths, and a fixed header would be wrong for both.
    const auto& run = source.payload_as<PlantRun>("plant_trajectory");
    out << "time_s";
    for (const auto& name : run.state_names) {
      out << ',' << csv_label("state:" + name);
    }
    for (Eigen::Index j = 0; j < run.command_rad_s.size(); ++j) {
      out << ',' << csv_label("command:omega_command_" + std::to_string(j) + "_rad_s");
    }
    out << ',' << csv_label("wind:north_m_s") << ',' << csv_label("wind:east_m_s") << ','
        << csv_label("wind:down_m_s") << '\n';
    for (std::size_t i = 0; i < run.trajectory.states.size(); ++i) {
      out << run.trajectory.times_s[i];
      const auto& x = run.trajectory.states[i];
      for (Eigen::Index j = 0; j < x.size(); ++j) {
        out << ',' << x(j);
      }
      // The command and the wind are constant over this run and are repeated on
      // every row rather than left to a header comment, so one file is one
      // complete record of what was integrated. `sim.linear` writes its constant
      // input into the outputs for the same reason.
      // Per sample, because a declared history makes the configured constant a
      // half-truth: a reader who combines the recorded air-relative velocity
      // with the wrong wind reconstructs a ground velocity that never happened.
      const Eigen::VectorXd& command_now = run.command_samples_rad_s[i];
      const Eigen::Vector3d& wind_now = run.wind_samples_ned_m_s[i];
      for (Eigen::Index j = 0; j < command_now.size(); ++j) {
        out << ',' << command_now(j);
      }
      out << ',' << wind_now.x() << ',' << wind_now.y() << ',' << wind_now.z() << '\n';
    }
  } else {
    throw std::invalid_argument(
        "report.csv requires a linear, nonlinear, plant or sampled trajectory");
  }
  const auto path = context.input->string_at("path");
  context.write_output(path, out.str());
  Artifact result;
  result.kind = "report";
  result.summary = "wrote " + path;
  result.payload = context.resolve_output_path(path);
  return result;
}
}  // namespace

void register_design_capabilities(Registry& registry) {
  using State = Capability::State;
  registry.add({"analyze.hinfnorm",
                "Bound a stable continuous-time H-infinity norm using Hamiltonian level tests",
                "hinfinity_norm",
                State::ImplementedUnvalidated,
                hinfinity,
                {"system", "relative_tolerance", "absolute_tolerance"}});
  registry.add({"analyze.robust_bounds",
                "Bound S/T norms and SISO disk size for an internally stable feedback loop",
                "robust_bounds",
                State::ImplementedUnvalidated,
                robust_bounds,
                {"system", "skew", "relative_tolerance", "absolute_tolerance"}});
  registry.add(
      {"synth.care",
       "Solve a continuous-time algebraic Riccati equation with residual and stability checks",
       "care_solution",
       State::Implemented,
       care,
       {"a", "b", "q", "r", "n"}});
  registry.add(
      {"synth.lqr",
       "Design continuous full-state feedback and retain the weights and numerical evidence",
       "control_law",
       State::ImplementedUnvalidated,
       lqr,
       {"system", "q", "r", "n", "break_at"}});
  registry.add({"synth.pid",
                "Realise explicitly supplied PID gains with a mandatory derivative filter",
                "linear_system",
                State::ImplementedUnvalidated,
                [](const StageContext& c) {
                  return system_artifact(
                      synth::filtered_pid(c.input->number_at("kp"),
                                          c.input->number_at("ki"),
                                          c.input->number_at("kd"),
                                          c.input->number_at("derivative_filter_s")));
                },
                {"kp", "ki", "kd", "derivative_filter_s"}});
  registry.add({"model.channels",
                "Select named inputs and outputs while retaining all internal states",
                "linear_system",
                State::ImplementedUnvalidated,
                select_channels,
                {"system", "inputs", "outputs"}});
  registry.add({"analyze.gramians",
                "Reachability and observability of a linear model for a declared input and "
                "output set — the subspace ranks, the directions that fall outside them by "
                "state name, and finite-horizon Gramians over a declared horizon",
                "gramians",
                State::ImplementedUnvalidated,
                gramians,
                {"system", "horizon_s", "steps", "rank_tolerance"}});
  registry.add({"model.control_system",
                "Extract an LQR design's closed loop, its plant-input return ratio, or the "
                "single loop at one input with the other loops still closed",
                "linear_system",
                State::ImplementedUnvalidated,
                control_system,
                {"law", "use", "channel"}});
  registry.add({"model.series",
                "Cascade two state-space systems in declared channel order",
                "linear_system",
                State::ImplementedUnvalidated,
                [](const StageContext& c) {
                  return system_artifact(
                      synth::series(system_at(c, "first"), system_at(c, "second")));
                },
                {"first", "second"}});
  registry.add({"model.feedback",
                "Close a square state-space loop with negative identity feedback",
                "linear_system",
                State::ImplementedUnvalidated,
                [](const StageContext& c) {
                  return system_artifact(synth::negative_feedback(system_at(c)));
                },
                {"system"}});
  registry.add({"sim.linear",
                "Integrate a continuous linear model with fixed-step RK4 under a constant input or "
                "a declared input history with a stated hold and extrapolation",
                "linear_trajectory",
                State::ImplementedUnvalidated,
                linear_simulation,
                {"system",
                 "step_s",
                 "steps",
                 "sample_stride",
                 "initial_state",
                 "constant_input",
                 "input_schedule"}});
  registry.add(
      {"sim.nonlinear",
       "Simulate a local aircraft model with bounded actuators and optional full-state feedback",
       "nonlinear_trajectory",
       State::ImplementedUnvalidated,
       nonlinear_simulation,
       {"trim_point",
        "law",
        "step_s",
        "steps",
        "sample_stride",
        "actuators",
        "initial_perturbation",
        "command_increment"}});
  registry.add({"report.csv",
                "Export a computed linear or nonlinear time history with named columns",
                "report",
                State::ImplementedUnvalidated,
                csv,
                {"trajectory", "path"},
                {},
                {"path"}});
}

std::vector<TimeSeriesChart> design_charts(const Artifact& artifact) {
  std::vector<TimeSeriesChart> charts;
  if (artifact.kind == "linear_trajectory") {
    const auto& run = artifact.payload_as<LinearRun>("linear_trajectory");
    const auto labels = run.system.output_labels();
    // Keep the page manageable; the CSV carries every declared channel.
    const auto count = std::min<Eigen::Index>(run.system.output_count(), 8);
    for (Eigen::Index index = 0; index < count; ++index) {
      TimeSeriesChart chart;
      const auto& label = labels[static_cast<std::size_t>(index)];
      chart.title = "Linear response: " + label;
      chart.y_label = label + " (declared model units)";
      chart.times_s = run.trajectory.times_s;
      chart.series.push_back({label, {}});
      for (std::size_t sample = 0; sample < run.trajectory.states.size(); ++sample) {
        const Eigen::VectorXd output = run.system.output_matrix() * run.trajectory.states[sample]
                                       + run.system.feedthrough_matrix() * run.input_at(sample);
        chart.series.front().values.push_back(output(index));
      }
      charts.push_back(std::move(chart));
    }
  } else if (artifact.kind == "nonlinear_trajectory") {
    const auto& run = artifact.payload_as<sim::NonlinearResult>("nonlinear_trajectory");
    charts = {{"Nonlinear body velocity", "m/s", {}, {{"u", {}}, {"v", {}}, {"w", {}}}},
              {"Nonlinear Euler attitude", "rad", {}, {{"roll", {}}, {"pitch", {}}, {"yaw", {}}}},
              {"Nonlinear body angular rates", "rad/s", {}, {{"p", {}}, {"q", {}}, {"r", {}}}},
              {"Actual actuator deflections",
               "rad",
               {},
               {{"elevator", {}}, {"aileron", {}}, {"rudder", {}}}},
              {"Actual thrust", "N", {}, {{"thrust", {}}}}};
    for (const auto& sample : run.samples) {
      for (auto& chart : charts) {
        chart.times_s.push_back(sample.time_s);
      }
      const auto euler = core::euler_from_quaternion(sample.state.attitude_body_to_ned);
      const Eigen::Vector3d angles(euler.roll_rad, euler.pitch_rad, euler.yaw_rad);
      const auto controls = sample.controls.to_vector();
      for (std::size_t i = 0; i < 3; ++i) {
        const auto index = static_cast<Eigen::Index>(i);
        charts[0].series[i].values.push_back(sample.state.velocity_body_m_s(index));
        charts[1].series[i].values.push_back(angles(index));
        charts[2].series[i].values.push_back(sample.state.angular_rate_body_rad_s(index));
        charts[3].series[i].values.push_back(controls(index));
      }
      charts[4].series.front().values.push_back(sample.controls.thrust_n);
    }
  }
  return charts;
}

bool write_design_section(std::ostream& out, const Artifact& artifact) {
  // A fitted plant reaching a report must arrive with the account of how it was
  // fitted, and a loaded one must arrive saying it was loaded. A report that
  // showed only the coefficients would be a table of numbers with no way to tell
  // a measurement from an estimate.
  if (artifact.kind == "quadrotor") {
    const auto& subject = artifact.payload_as<QuadrotorArtifact>("quadrotor");
    if (!subject.identity.is_fitted() || !subject.identity.fit) {
      out << "Loaded from `" << subject.identity.path << "` (sha256 " << subject.identity.sha256
          << "). No parameter was fitted.\n\n";
      return true;
    }
    const FittedModelProvenance& fit = *subject.identity.fit;
    out << "### Identified parameters\n\n";
    out << "| Parameter | Unit | Value | Standard error | Bounds | At bound |\n";
    out << "|---|---|---|---|---|---|\n";
    for (const FittedParameter& parameter : fit.fitted) {
      out << "| `" << parameter.path << "` | " << parameter.unit << " | " << parameter.value
          << " | ";
      if (parameter.standard_error_is_estimable) {
        out << parameter.standard_error;
      } else {
        out << "none estimable";
      }
      out << " | [" << parameter.lower << ", " << parameter.upper << "] | "
          << (parameter.at_bound ? "**yes**" : "no") << " |\n";
    }
    out << "\n"
        << fit.preserved_parameter_paths.size()
        << " further parameter(s) were NOT fitted and carry the base model's values "
           "unchanged.\n\n";
    out << "**Fitted against** `" << fit.estimation_record_path << "` (sha256 "
        << fit.estimation_record_sha256 << "), " << fit.estimation_sample_count
        << " sample(s) over [" << fit.estimation_first_sample_s << ", "
        << fit.estimation_last_sample_s << "] s, from base model `" << fit.base_model_path
        << "` (sha256 " << fit.base_model_sha256 << ").\n\n";
    out << "| Diagnostic | Value |\n|---|---|\n"
        << "| Objective, start → final (scaled) | " << fit.initial_objective << " → "
        << fit.objective << " |\n"
        << "| Objective improved | " << (fit.objective_improved ? "yes" : "**no**") << " |\n"
        << "| Residual RMS (scaled) | " << fit.residual_rms << " |\n"
        << "| Residuals | " << fit.residual_count << " |\n"
        << "| Iterations declared / run | " << fit.iterations_declared << " / "
        << fit.iterations_run << " |\n"
        << "| Stop reason | " << fit.stop_reason << " |\n"
        << "| Accepted steps, last at iteration | " << fit.accepted_steps << ", "
        << fit.last_accepted_iteration << " |\n"
        << "| First-order measure, ‖Jᵀr‖∞ at the final point | " << fit.gradient_infinity_norm
        << " |\n"
        << "| The same over each parameter's declared range | "
        << fit.gradient_over_bound_span_infinity_norm << " |\n"
        << "| Sensitivity condition number | " << fit.jacobian_condition_number << " |\n\n";
    out << "The table above answers five separate questions and no sixth one. That the routine "
           "ran is not that the objective improved; that the objective improved is not that the "
           "fit converged; that it converged is not that the parameters are identifiable; and "
           "none of them is whether the fit is accurate enough for a use. There is deliberately "
           "no convergence verdict here.\n\n";
    out << "_Objective:_ " << fit.objective_definition << "\n\n";
    out << "_Uncertainty:_ "
        << (fit.uncertainty_is_estimable ? fit.uncertainty_assumptions
                                         : "none reported — " + fit.uncertainty_assumptions)
        << "\n\n";
    out << "A completed fit is not a validation, and these coefficients are not measured "
           "aircraft data. Whether the residual is small enough for any use is an engineering "
           "judgement nothing here makes.\n\n";
    return true;
  }
  if (artifact.kind == "validation") {
    const auto& validation = artifact.payload_as<ValidationArtifact>("validation");
    const identify::ValidationResult& result = validation.result;
    // The label first and in words. It is the difference between a diagnostic
    // and a claim, and a reader skimming a report must meet it before the
    // numbers rather than after them.
    out << "**Record separation: " << identify::to_string(result.separation) << "** — "
        << result.separation_basis << "\n\n";
    if (result.caller_declaration_was_contradicted) {
      out << "> The study declared these records to hold different data and a check "
             "contradicted it. The numbers below are a diagnostic, not a validation.\n\n";
    }
    out << "| Output | State | RMSE | Max abs error | Mean error | Fit fraction | Residual "
           "lag-1 autocorrelation |\n";
    out << "|---|---|---|---|---|---|---|\n";
    for (const identify::ValidationOutput& output : result.outputs) {
      out << "| `" << output.channel << "` | `" << output.state_name << "` | " << output.rmse
          << " | " << output.max_absolute_error << " | " << output.mean_error << " | ";
      if (output.fit_fraction_is_defined) {
        out << output.fit_fraction;
      } else {
        out << "undefined — " << output.undefined_reason;
      }
      out << " | ";
      if (output.autocorrelation_is_defined) {
        out << output.residual_lag_one_autocorrelation;
      } else {
        out << "undefined";
      }
      out << " |\n";
    }
    out << "\nThe label above is about SAMPLE separation. It bounds what the fit could have "
           "seen; it is not a claim of statistical independence, and two windows of one flight "
           "share the aircraft, the trim, the air mass and every unmodelled effect that "
           "persists across the cut.\n\n";
    out << "Scored over " << result.sample_count << " sample(s). Validation record sha256 "
        << result.validation_record_sha256 << "; estimation record sha256 "
        << result.estimation_record_sha256 << ".\n\n";
    out << "_Assumptions:_ " << result.assumptions << "\n\n";
    return true;
  }
  if (artifact.kind == "gramians") {
    const auto& analysis = artifact.payload_as<analyze::GramianAnalysis>("gramians");
    const auto subspace =
        [&out](const char* title, const char* verb, const analyze::SubspaceAnalysis& s) {
          out << "### " << title << "\n\n";
          out << "| Quantity | Value |\n|---|---|\n"
              << "| Rank | " << s.rank << " of " << s.state_count << " |\n"
              << "| Condition number of the retained directions | " << s.retained_condition_number
              << " |\n\n";
          if (s.missing_directions.empty()) {
            out << "Every direction in the state space can be " << verb << ".\n\n";
            return;
          }
          out << s.missing_directions.size() << " direction(s) cannot be " << verb
              << ". Each is a unit vector in the model's own state coordinates; the states listed "
                 "are those carrying more than a one-percent share of it.\n\n";
          out << "| Direction | Dominant states (share) |\n|---|---|\n";
          for (std::size_t k = 0; k < s.missing_directions.size(); ++k) {
            out << "| " << (k + 1) << " | ";
            const auto& names = s.missing_directions[k].dominant_states;
            for (std::size_t i = 0; i < names.size(); ++i) {
              out << (i == 0 ? "" : ", ") << "`" << names[i] << "`";
            }
            out << " |\n";
          }
          out << "\n";
        };
    subspace("Reachability", "moved by the declared inputs", analysis.reachability);
    subspace("Observability", "seen by the declared outputs", analysis.observability);

    out << "### Finite-horizon Gramians\n\n";
    out << "| Quantity | Controllability | Observability |\n|---|---|---|\n"
        << "| Largest eigenvalue | "
        << (analysis.controllability_eigenvalues.empty()
                ? 0.0
                : analysis.controllability_eigenvalues.front())
        << " | "
        << (analysis.observability_eigenvalues.empty() ? 0.0
                                                       : analysis.observability_eigenvalues.front())
        << " |\n"
        << "| Smallest eigenvalue | "
        << (analysis.controllability_eigenvalues.empty()
                ? 0.0
                : analysis.controllability_eigenvalues.back())
        << " | "
        << (analysis.observability_eigenvalues.empty() ? 0.0
                                                       : analysis.observability_eigenvalues.back())
        << " |\n"
        << "| Condition number | " << analysis.controllability_condition_number << " | "
        << analysis.observability_condition_number << " |\n\n";
    out << "Integrated over " << analysis.horizon_s << " s in " << analysis.steps
        << " fixed RK4 steps. **These are not the infinite-horizon Gramians**";
    if (analysis.spectrum_is_strictly_stable) {
      out << ", which do exist for this model — its rightmost eigenvalue has real part "
          << analysis.rightmost_eigenvalue_real_part << " — but are not computed here.\n\n";
    } else {
      out << ", and for this model they do not exist at all: the rightmost eigenvalue has real "
             "part "
          << analysis.rightmost_eigenvalue_real_part
          << ", so the T to infinity limit diverges and the matrix a Lyapunov solve would "
             "return for it would not be a Gramian of anything.\n\n";
    }
    out << "_Assumptions:_ " << analysis.assumptions << "\n\n";
    return true;
  }
  if (artifact.kind == "robust_bounds") {
    const auto& bounds = artifact.payload_as<RobustBounds>("robust_bounds");
    out << "### Sensitivity S = (I + L)^-1\n\n";
    norm_table(out, bounds.sensitivity.sensitivity);
    out << "### Complementary sensitivity T = I - S\n\n";
    norm_table(out, bounds.sensitivity.complementary);
    if (bounds.has_disk) {
      out << "### SISO disk size\n\n"
          << "| Quantity | Value |\n|---|---|\n"
          << "| Skew | " << bounds.disk.skew << " |\n"
          << "| Conservative alpha lower bound | " << bounds.disk.alpha_lower << " |\n"
          << "| Alpha upper bound | " << bounds.disk.alpha_upper << " |\n\n"
          << bounds.disk.diagnostic << "\n\n";
    }
    out << "These bounds use the declared loop break and channel scaling. The numerical "
           "upper norm bound supplies the conservative disk lower bound. They do not "
           "include actuator limits, sampling, model uncertainty or a flight-data comparison.\n\n";
    return true;
  }
  if (artifact.kind == "hinfinity_norm") {
    norm_table(out, artifact.payload_as<analyze::HinfinityNorm>("hinfinity_norm"));
    return true;
  }
  if (artifact.kind == "care_solution") {
    const auto& solution = artifact.payload_as<synth::CareSolution>("care_solution");
    matrix_table(out, "Riccati solution X", solution.x);
    matrix_table(out, "State feedback K", solution.k);
    care_evidence(out, solution);
    return true;
  }
  if (artifact.kind == "control_law") {
    const auto& law = artifact.payload_as<synth::LqrDesign>("control_law");
    out << "Continuous full-state feedback: delta u = -K delta x. Loop break: plant input.\n\n";
    matrix_table(out,
                 "K (input per state unit)",
                 law.riccati.k,
                 law.plant.input_names,
                 law.plant.state_names);
    matrix_table(out, "Q", law.q, law.plant.state_names, law.plant.state_names);
    matrix_table(out, "R", law.r, law.plant.input_names, law.plant.input_names);
    matrix_table(out, "N", law.n, law.plant.state_names, law.plant.input_names);
    care_evidence(out, law.riccati);
    out << "This design assumes exact state feedback. Actuator constraints, estimation errors and "
           "the aircraft operating envelope require separate verification.\n\n";
    return true;
  }
  if (artifact.kind == "linear_trajectory") {
    const auto& run = artifact.payload_as<LinearRun>("linear_trajectory");
    out << "Completed " << run.trajectory.step_count << " steps at " << run.trajectory.step_s
        << " s. Fixed-step RK4; compare with a smaller step to assess integration error.\n\n";
    if (!run.history.empty()) {
      out << "Driven by a declared input history: " << run.history
          << ". Timestamps are seconds from the start of the run; each recorded input is the "
             "value in force after any event at that instant, and report.csv carries it per "
             "sample.\n\n";
    }
    matrix_table(out, "Final state", run.trajectory.states.back(), run.system.state_names);
    return true;
  }
  // A sampled run's report is about the CONTROLLER as much as the trajectory:
  // what the law asked for, how much of it the vehicle was allowed, and at what
  // rate and delay it asked. A report showing only the final state would
  // describe a flight without saying it was flown by a loop that samples.
  if (artifact.kind == "sampled_trajectory") {
    const auto& sampled = artifact.payload_as<SampledRun>("sampled_trajectory");
    const SampledControlRecord& control = sampled.control;
    out << "| Quantity | Value |\n|---|---|\n"
        << "| Controller period | " << control.controller_period_s << " s |\n"
        << "| Ticks executed | " << control.tick_times_s.size() << " |\n"
        << "| Delay | " << control.delay_periods << " period(s) |\n"
        << "| Integration step | " << sampled.plant.step_s << " s |\n"
        << "| Integration steps | " << sampled.plant.step_count << " |\n"
        << "| Reference translates with the trim velocity | "
        << (control.reference_follows_trim_velocity ? "yes" : "no") << " |\n"
        << "| Ticks at which saturation changed the command | " << control.saturated_tick_count
        << " |\n"
        << "| Worst single-channel saturation residual | "
        << control.worst_saturation_residual_rad_s << " rad/s |\n";
    const bool discrete = control.law_time_domain == "discrete_design";
    out << "| Law | "
        << (discrete ? "discrete design, executed at the period it was designed for"
                     : "continuous design, executed at a rate (emulation)")
        << " |\n"
        << "| Design sample time | ";
    if (discrete) {
      out << control.design_sample_time_s << " s";
    } else {
      out << "none — a continuous design has no sample time";
    }
    out << " |\n| Hold | " << control.hold << " |\n\n";
    if (control.saturated_tick_count > 0) {
      out << "The law asked for authority it did not get at " << control.saturated_tick_count
          << " tick(s). The residual above is the honest measure of how much: a run reported "
             "only through its applied commands would not show it.\n\n";
    }
    if (discrete) {
      const LinearPredictionRecord& prediction = control.prediction;
      out << "### The discrete design's own prediction\n\n";
      if (!prediction.available) {
        out << "No comparison was made: " << prediction.unavailable_reason << ".\n\n";
      } else {
        out << "| Quantity | Value |\n|---|---|\n"
            << "| Worst discrepancy over the prediction's peak, in the design's cost-to-go norm "
               "| ";
        if (prediction.relative_discrepancy_defined) {
          out << prediction.relative_discrepancy;
        } else {
          out << "undefined — the prediction is identically zero, because the run started at "
                 "the reference";
        }
        out << " |\n| At tick | " << prediction.worst_tick << " |\n"
            << "| Smallest over largest eigenvalue of the cost-to-go X | "
            << prediction.cost_to_go_eigenvalue_ratio << " |\n"
            << "| Declared small-perturbation budget | ";
        if (prediction.budget_declared) {
          // An undefined comparison is neither inside nor outside a budget. A
          // run that starts at the reference predicts nothing but zero, and
          // printing OUTSIDE beside "undefined" would contradict the row above.
          out << prediction.budget << " |\n| Verdict | "
              << (!prediction.relative_discrepancy_defined
                      ? "none — the comparison is undefined, so the budget can be neither met nor "
                        "missed"
                  : prediction.within_budget ? "within the declared budget"
                                             : "**OUTSIDE the declared budget**");
        } else {
          out << "none declared; reported without a verdict";
        }
        out << " |\n\n";
        if (prediction.premise_violated_by_saturation) {
          out << "**The run saturated, and the prediction has no actuator limits.** The "
                 "comparison above is outside the premise it rests on.\n\n";
        }
        out << "| Chart coordinate | Peak of the prediction | Worst absolute discrepancy |\n"
               "|---|---:|---:|\n";
        for (std::size_t j = 0; j < prediction.chart_names.size(); ++j) {
          const auto index = static_cast<Eigen::Index>(j);
          double peak = 0.0;
          double miss = 0.0;
          for (std::size_t k = 0; k < prediction.predicted_chart.size(); ++k) {
            peak = std::fmax(peak, std::fabs(prediction.predicted_chart[k](index)));
            miss = std::fmax(miss,
                             std::fabs(prediction.measured_chart[k](index)
                                       - prediction.predicted_chart[k](index)));
          }
          out << "| " << prediction.chart_names[j] << " | " << peak << " | " << miss << " |\n";
        }
        out << "\nThe prediction is the design's own discrete model, iterated with the same "
               "gain, the same whole-period delay and the same hold from the same initial chart "
               "state. It has no nonlinearity and no actuator limits, so the discrepancy is the "
               "linearisation's error plus whatever else the run did that the design did not "
               "model. The chart coordinates mix units, which is why the headline uses the "
               "design's own cost-to-go norm and the table above shows each coordinate in its "
               "own unit.\n\n";
      }
    }
    matrix_table(
        out, "Final state", sampled.plant.trajectory.states.back(), sampled.plant.state_names);
    if (discrete) {
      out << "This is the NONLINEAR plant, flown by a law designed in discrete time for this "
             "period and this hold. A Riccati solution whose closed loop lies inside the unit "
             "circle and a run that converged are both statements about the nominal loop: "
             "neither is a gain, phase, delay or disk margin of the sampled loop, and the "
             "sampled loop's own robustness is a separate question no capability here answers. "
             "The design modelled no delay, so the "
          << control.delay_periods
          << " period(s) of delay this run applied are a plant it did not see. Use report.csv "
             "for the requested, applied, measured and predicted values at every tick.\n\n";
    } else {
      out << "This is the NONLINEAR plant, executed with zero-order hold and a whole-period "
             "delay. Gain and phase margins computed from the continuous linearisation describe "
             "the continuous loop and not this one; the sampled loop's own robustness is a "
             "separate question no capability here answers. Use report.csv for the requested "
             "and applied commands at every tick.\n\n";
    }
    return true;
  }
  if (artifact.kind == "nonlinear_trajectory") {
    const auto& run = artifact.payload_as<sim::NonlinearResult>("nonlinear_trajectory");
    out << "Completed " << run.completed_steps << " steps at " << run.step_s
        << " s within the local model envelope.\n\n"
        << "| Envelope measure | Maximum departure |\n|---|---:|\n| Angle of attack (rad) | "
        << run.max_alpha_departure_rad << " |\n| Mach | " << run.max_mach_departure << " |\n\n"
        << "| Actuator | Position-limited steps | Rate-limited steps |\n|---|---:|---:|\n";
    const std::vector<std::string> names = {"elevator", "aileron", "rudder", "thrust"};
    for (std::size_t i = 0; i < names.size(); ++i) {
      out << "| " << names[i] << " | " << run.position_limited_steps[i] << " | "
          << run.rate_limited_steps[i] << " |\n";
    }
    out << "\nFlat, nonrotating Earth; still air; rigid airframe; local coefficient model; "
           "continuous full-state feedback when configured. Actuator limits are study inputs, not "
           "verified aircraft specifications. Use report.csv for the computed time history and "
           "repeat with smaller steps for convergence.\n\n";
    return true;
  }
  return false;
}
}  // namespace galata::pipeline
