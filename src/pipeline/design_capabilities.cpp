// SPDX-License-Identifier: Apache-2.0
// File-driven control design and simulation adapters. All numeric algorithms
// live in synth/sim; this layer validates wiring, preserves names and renders.
#include "galata/analyze/hinfinity.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/charts.hpp"
#include "galata/sim/linear.hpp"
#include "galata/sim/nonlinear.hpp"
#include "galata/synth/control.hpp"

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

Artifact control_system(const StageContext& context) {
  const auto& law = context.upstream_at("law").payload_as<synth::LqrDesign>("control_law");
  const auto use = context.input->string_at("use");
  if (use == "closed_loop") {
    return system_artifact(law.closed_loop);
  }
  if (use == "broken_loop") {
    return system_artifact(law.broken_loop);
  }
  throw std::invalid_argument("model.control_system: use must be closed_loop or broken_loop");
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
  Eigen::VectorXd input;
};

Artifact linear_simulation(const StageContext& context) {
  LinearRun run;
  run.system = system_at(context);
  run.input =
      vector(context.input->get("constant_input"), run.system.input_count(), "constant_input");
  run.trajectory = sim::simulate_linear(
      run.system,
      vector(context.input->get("initial_state"), run.system.state_count(), "initial_state"),
      run.input,
      context.input->number_at("step_s"),
      context.input->integer_at("steps", 0),
      context.input->integer_at("sample_stride", 1));
  Artifact result;
  result.kind = "linear_trajectory";
  result.summary =
      std::to_string(run.trajectory.states.size()) + " samples; completed linear response";
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
    out << '\n';
    for (std::size_t i = 0; i < run.trajectory.states.size(); ++i) {
      out << run.trajectory.times_s[i];
      const auto& x = run.trajectory.states[i];
      const Eigen::VectorXd y =
          run.system.output_matrix() * x + run.system.feedthrough_matrix() * run.input;
      for (Eigen::Index j = 0; j < x.size(); ++j) {
        out << ',' << x(j);
      }
      for (Eigen::Index j = 0; j < y.size(); ++j) {
        out << ',' << y(j);
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
      for (Eigen::Index j = 0; j < run.command_rad_s.size(); ++j) {
        out << ',' << run.command_rad_s(j);
      }
      out << ',' << run.wind_ned_m_s.x() << ',' << run.wind_ned_m_s.y() << ','
          << run.wind_ned_m_s.z() << '\n';
    }
  } else {
    throw std::invalid_argument("report.csv requires a linear, nonlinear or plant trajectory");
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
  registry.add({"model.control_system",
                "Extract the closed loop or plant-input return ratio of an LQR design",
                "linear_system",
                State::ImplementedUnvalidated,
                control_system,
                {"law", "use"}});
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
                "Integrate a continuous linear model with a constant input and fixed-step RK4",
                "linear_trajectory",
                State::ImplementedUnvalidated,
                linear_simulation,
                {"system", "step_s", "steps", "sample_stride", "initial_state", "constant_input"}});
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
      for (const auto& state : run.trajectory.states) {
        const Eigen::VectorXd output =
            run.system.output_matrix() * state + run.system.feedthrough_matrix() * run.input;
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
    matrix_table(out, "Final state", run.trajectory.states.back(), run.system.state_names);
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
