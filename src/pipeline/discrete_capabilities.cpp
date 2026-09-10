// SPDX-License-Identifier: Apache-2.0
//
// Study adapters for discrete-time models and sampled synthesis: the library
// half of F14 — `model::discretize_zoh`, `synth::solve_dare` and
// `synth::design_sampled_lqr` — made reachable from a study file.
//
// The numerics live in galata::model and galata::synth. What lives here is the
// schema, the evidence, and THE TIME-DOMAIN BOUNDARY, which every capability
// below enforces a piece of.
//
// A continuous model is a `linear_system` and a discrete one is a
// `discrete_linear_system`. They are different artefact kinds rather than one
// kind with a flag, so a stage wired to the wrong one fails by name before any
// number is computed — F14's "mixed time domains are rejected without an
// explicit adapter". There is exactly one adapter, `model.discretize`, from
// continuous to discrete under a declared hold. There is none in the other
// direction: recovering a continuous model from a sampled one is a different
// problem with its own non-uniqueness, and a capability that pretended
// otherwise would be choosing a branch of a matrix logarithm for the caller.
//
// Every continuous capability downstream — `analyze.modes`, `analyze.margins`,
// `model.control_system`, `sim.linear` — reads its upstream through
// `payload_as<model::LinearSystem>("linear_system")` or the `control_law`
// equivalent, so a discrete artefact reaching one of them is refused by the
// kind check with both kinds named. That is the refusal, not an accident of
// it, and `DiscreteWorkflow` holds it.
//
// WHAT THIS IS NOT. No capability here computes a margin of the sampled loop —
// gain, phase, delay or disk — and none is implied. A DARE solution whose
// closed loop has every eigenvalue inside the unit circle says the NOMINAL
// sampled loop converges; it says nothing about how much gain, phase or delay
// that loop tolerates, which needs its own machinery and a decision about which
// of several inequivalent sampled-margin definitions is meant. No delay is
// modelled in any design here, and no actuator limit.

#include "galata/model/discrete_system.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/files.hpp"
#include "galata/synth/discrete_control.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <ostream>
#include <sstream>
#include <stdexcept>

namespace galata::pipeline {
namespace {

// --- schema helpers --------------------------------------------------------

Eigen::MatrixXd matrix_value(const ValuePtr& value,
                             const std::string& routine,
                             const std::string& key) {
  const auto& rows = value->as_list();
  if (rows.empty() || rows.size() > 256) {
    throw std::invalid_argument(routine + ": `" + key + "` must have 1..256 rows");
  }
  const auto columns = rows.front()->as_list().size();
  if (columns == 0 || columns > 256) {
    throw std::invalid_argument(routine + ": `" + key + "` must have 1..256 columns");
  }
  Eigen::MatrixXd result(static_cast<Eigen::Index>(rows.size()),
                         static_cast<Eigen::Index>(columns));
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const auto& row = rows[i]->as_list();
    if (row.size() != columns) {
      throw std::invalid_argument(routine + ": `" + key + "` is ragged");
    }
    for (std::size_t j = 0; j < columns; ++j) {
      result(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = row[j]->as_number();
    }
  }
  if (!result.allFinite()) {
    throw std::invalid_argument(routine + ": `" + key + "` must have finite entries");
  }
  return result;
}

Eigen::MatrixXd required_matrix(const StageContext& context,
                                const std::string& routine,
                                const std::string& key) {
  const ValuePtr value = context.input->get(key);
  if (!value || value->kind() == Value::Kind::Null) {
    throw std::invalid_argument(routine + ": `" + key + "` is required");
  }
  return matrix_value(value, routine, key);
}

Eigen::MatrixXd optional_matrix(const StageContext& context,
                                const std::string& routine,
                                const std::string& key) {
  const ValuePtr value = context.input->get(key);
  if (!value || value->kind() == Value::Kind::Null) {
    return {};
  }
  return matrix_value(value, routine, key);
}

// The weights are checked against the PLANT here, by name and count, before the
// library sees them. The library refuses a mismatch too, but in terms of n and m;
// a study writer who miscounted sixteen states deserves to be told sixteen.
void require_weight_shapes(const std::string& routine,
                           const Eigen::MatrixXd& q,
                           const Eigen::MatrixXd& r,
                           const Eigen::MatrixXd& n,
                           Eigen::Index states,
                           Eigen::Index inputs) {
  const auto describe =
      [&](const char* key, const Eigen::MatrixXd& value, Eigen::Index rows, Eigen::Index columns) {
        if (value.rows() != rows || value.cols() != columns) {
          std::ostringstream message;
          message << routine << ": `" << key << "` is " << value.rows() << "x" << value.cols()
                  << "; the plant has " << states << " state(s) and " << inputs
                  << " input(s), so it must be " << rows << "x" << columns;
          throw std::invalid_argument(message.str());
        }
      };
  describe("q", q, states, states);
  describe("r", r, inputs, inputs);
  if (n.size() != 0) {
    describe("n", n, states, inputs);
  }
}

// Required, with no default. Every discrete matrix belongs to one sample time,
// and a model whose sample time nobody chose cannot be compared with anything,
// executed at a rate, or turned back into a frequency.
double required_sample_time(const StageContext& context, const std::string& routine) {
  const ValuePtr value = context.input->get("sample_time_s");
  if (!value || value->kind() == Value::Kind::Null) {
    throw std::invalid_argument(
        routine
        + ": `sample_time_s` is required. There is no default: every discrete matrix belongs to "
          "one sample time, and one nobody chose cannot be compared with anything, executed at "
          "a rate, or turned back into a frequency");
  }
  const double sample_time_s = value->as_number();
  if (!(sample_time_s > 0.0) || !std::isfinite(sample_time_s)) {
    std::ostringstream message;
    message << routine << ": `sample_time_s` is " << sample_time_s
            << ", which is not a positive finite number of seconds";
    throw std::invalid_argument(message.str());
  }
  return sample_time_s;
}

// Required, with no default, and only one value is implemented. The hold is the
// assumption that makes the discretisation exact — constant across the
// interval, stepping at the tick — so a study that does not state it has not
// said what its discrete model is a model of.
void require_zero_order_hold(const StageContext& context, const std::string& routine) {
  const ValuePtr value = context.input->get("hold");
  if (!value || value->kind() == Value::Kind::Null) {
    throw std::invalid_argument(
        routine
        + ": `hold` is required and must be `zero_order`. There is no default, because the hold "
          "is the assumption that makes the discretisation exact, and a study that does not "
          "state it has not said what its discrete model is a model of");
  }
  const std::string name = value->as_string();
  if (name != "zero_order") {
    throw std::invalid_argument(
        routine + ": `hold: " + name
        + "` is not implemented. `zero_order` is the only hold galata discretises under; a "
          "first-order hold, a Tustin or a matched-pole mapping is a different discretisation "
          "with different assumptions, and is refused rather than approximated by this one");
  }
}

// `const char*` rather than `const std::string&`: a string temporary bound at a
// call site would leave GCC's -Wdangling-reference unable to prove that the
// returned reference points into the upstream artefact.
const model::LinearSystem& continuous_system(const StageContext& context,
                                             const char* routine,
                                             const char* remedy) {
  const Artifact& upstream = context.upstream_at("system");
  if (upstream.kind == "discrete_linear_system") {
    throw std::invalid_argument(std::string(routine)
                                + ": `system` is a DISCRETE-time model, and this capability "
                                  "takes a continuous one. "
                                + remedy);
  }
  return upstream.payload_as<model::LinearSystem>("linear_system");
}

std::vector<std::string> generated_names(const char* stem, Eigen::Index count) {
  std::vector<std::string> names;
  for (Eigen::Index i = 0; i < count; ++i) {
    names.push_back(stem + std::to_string(i));
  }
  return names;
}

// --- model.discretize ------------------------------------------------------
//
// The one explicit adapter between the two time domains. The hold and the
// sample time are both required: the first is what makes the result exact and
// the second is what every matrix in it belongs to.

Artifact discretize_capability(const StageContext& context) {
  const model::LinearSystem& system = continuous_system(
      context,
      "model.discretize",
      "Re-discretising a discrete model at another rate is not an adapter galata has — it would "
      "need the continuous model the first discretisation threw away. Discretise the continuous "
      "model at the rate you need.");
  const double sample_time_s = required_sample_time(context, "model.discretize");
  require_zero_order_hold(context, "model.discretize");

  model::Discretisation discretisation = model::discretize_zoh(system, sample_time_s);
  const model::DiscretisationEvidence& evidence = discretisation.evidence;

  std::ostringstream summary;
  summary << discretisation.system.state_count() << " states, "
          << discretisation.system.input_count() << " inputs at " << std::scientific
          << std::setprecision(3) << sample_time_s << " s, zero-order hold; spectral radius "
          << std::fixed << std::setprecision(6) << evidence.spectral_radius
          << "; fastest continuous mode " << std::setprecision(1) << evidence.fastest_mode_rad_s
          << " rad/s against Nyquist " << evidence.nyquist_rad_s << " rad/s";

  Artifact artifact;
  artifact.kind = "discrete_linear_system";
  artifact.summary = summary.str();
  artifact.payload = std::move(discretisation);
  return artifact;
}

// --- synth.dare ------------------------------------------------------------
//
// Two ways in, exactly one of which must be taken. A `discrete_linear_system`
// carries its own sample time and names. Bare matrices carry neither, so the
// sample time is then REQUIRED: a matrix pair without one is not a discrete
// model, and the solution would belong to no rate. A continuous model is
// refused by name, with the adapter that would make it admissible.

Artifact dare_capability(const StageContext& context) {
  const bool from_model = context.input->get("system") != nullptr;
  const bool from_matrices =
      context.input->get("a") != nullptr || context.input->get("b") != nullptr;
  if (from_model == from_matrices) {
    throw std::invalid_argument(
        "synth.dare: give exactly one of `system` (a discrete_linear_system, which carries its "
        "own sample time) or `a` and `b` with `sample_time_s`. Both together leaves it unsaid "
        "which plant the equation is posed on; neither leaves no plant");
  }

  DareArtifact result;
  Eigen::MatrixXd a;
  Eigen::MatrixXd b;
  if (from_model) {
    if (context.input->get("sample_time_s") != nullptr) {
      throw std::invalid_argument(
          "synth.dare: `sample_time_s` was given beside `system`, whose model already carries "
          "one. Two sample times for one equation is a contradiction waiting to happen, so the "
          "second is refused rather than compared with the first");
    }
    const Artifact& upstream = context.upstream_at("system");
    if (upstream.kind == "linear_system") {
      throw std::invalid_argument(
          "synth.dare: `system` is a CONTINUOUS-time model. The discrete Riccati equation is "
          "posed on x[k+1] = A x[k] + B u[k], and the A of x_dot = A x + B u in its place would "
          "solve an equation about a different system and return a number that means nothing. "
          "Discretise it first with `model.discretize`, the explicit adapter, or solve the "
          "continuous equation with `synth.care`");
    }
    const auto& discretisation =
        upstream.payload_as<model::Discretisation>("discrete_linear_system");
    a = discretisation.system.a;
    b = discretisation.system.b;
    result.sample_time_s = discretisation.system.sample_time_s;
    result.state_names = discretisation.system.state_names;
    result.input_names = discretisation.system.input_names;
    result.from_discrete_model = true;
  } else {
    a = required_matrix(context, "synth.dare", "a");
    b = required_matrix(context, "synth.dare", "b");
    model::DiscreteLinearSystem declared;
    declared.a = a;
    declared.b = b;
    declared.sample_time_s = required_sample_time(context, "synth.dare");
    declared.state_names = generated_names("state_", a.rows());
    declared.input_names = generated_names("input_", b.cols());
    declared.validate();
    result.sample_time_s = declared.sample_time_s;
    result.state_names = declared.state_names;
    result.input_names = declared.input_names;
  }

  result.q = required_matrix(context, "synth.dare", "q");
  result.r = required_matrix(context, "synth.dare", "r");
  result.n = optional_matrix(context, "synth.dare", "n");
  require_weight_shapes("synth.dare", result.q, result.r, result.n, a.rows(), b.cols());
  if (result.n.size() == 0) {
    result.n = Eigen::MatrixXd::Zero(a.rows(), b.cols());
  }
  result.solution = synth::solve_dare(a, b, result.q, result.r, result.n);

  std::ostringstream summary;
  summary << a.rows() << " states, " << b.cols() << " inputs at " << std::scientific
          << std::setprecision(3) << result.sample_time_s << " s; DARE residual "
          << std::setprecision(1) << result.solution.relative_residual << " (budget "
          << result.solution.residual_budget << "); closed-loop spectral radius " << std::fixed
          << std::setprecision(6) << result.solution.spectral_radius;

  Artifact artifact;
  artifact.kind = "dare_solution";
  artifact.summary = summary.str();
  artifact.payload = std::move(result);
  return artifact;
}

// --- synth.sampled_lqr ------------------------------------------------------
//
// The whole sampled path, in the order it must happen: the plant AND the
// continuous cost are discretised under the same declared hold at the same
// sample time, and only then is the discrete Riccati equation solved. The cost
// discretisation produces a state-input cross term N even where the declared
// continuous cost had none; it is carried into the solver, into the artefact,
// into the report and into the evidence file, because dropping it anywhere
// would describe a different objective from the one that was minimised.

std::ostringstream evidence_stream() {
  std::ostringstream out;
  // ADR-0004: the classic locale, and the shortest precision that survives a
  // text round trip.
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  return out;
}

void write_yaml_matrix(std::ostream& out,
                       const std::string& indent,
                       const std::string& key,
                       const Eigen::MatrixXd& value) {
  out << indent << key << ":";
  if (value.size() == 0) {
    out << " []\n";
    return;
  }
  out << "\n";
  for (Eigen::Index i = 0; i < value.rows(); ++i) {
    out << indent << "  - [";
    for (Eigen::Index j = 0; j < value.cols(); ++j) {
      out << (j == 0 ? "" : ", ") << value(i, j);
    }
    out << "]\n";
  }
}

// A double-quoted YAML scalar. Citations quote titles, and an unescaped quote
// inside one ends the scalar early and leaves a file no reader can parse.
std::string quoted(const std::string& text) {
  std::string out = "\"";
  for (const char character : text) {
    if (character == '"' || character == '\\') {
      out += '\\';
    }
    out += static_cast<unsigned char>(character) < 0x20 ? ' ' : character;
  }
  return out + "\"";
}

void write_yaml_names(std::ostream& out,
                      const std::string& indent,
                      const std::string& key,
                      const std::vector<std::string>& names) {
  out << indent << key << ": [";
  for (std::size_t i = 0; i < names.size(); ++i) {
    out << (i == 0 ? "" : ", ") << quoted(names[i]);
  }
  out << "]\n";
}

void write_yaml_eigenvalues(std::ostream& out,
                            const std::string& indent,
                            const std::string& key,
                            const std::vector<std::complex<double>>& values) {
  out << indent << key << ":   # [real, imaginary], dimensionless\n";
  for (const std::complex<double>& value : values) {
    out << indent << "  - [" << value.real() << ", " << value.imag() << "]\n";
  }
}

std::string sampled_design_evidence(const synth::SampledLqrDesign& design,
                                    const model::LinearSystem& plant,
                                    const Artifact& upstream,
                                    const std::string& stage_id) {
  const model::DiscreteLinearSystem& discrete = design.discretisation.system;
  const model::DiscretisationEvidence& evidence = design.discretisation.evidence;
  const synth::DareSolution& riccati = design.riccati;

  std::ostringstream out = evidence_stream();
  out << "schema: galata.sampled-lqr.v1\n";
  out << "stage: " << quoted(stage_id) << "\n";
  out << "# Everything a reader needs to check this sampled design without re-running it:\n";
  out << "# what was declared, what was discretised, what was solved, and every check the\n";
  out << "# solution passed. Not a study input; no capability reads this back.\n";

  out << "conventions:\n";
  out << "  cost: \"J = sum over k of x[k]' Qd x[k] + 2 x[k]' Nd u[k] + u[k]' Rd u[k], the "
         "EXACT integral of the continuous cost over each held interval\"\n";
  out << "  gain: \"u[k] = -K x[k] with K = (Rd + Bd' X Bd)^-1 (Bd' X Ad + Nd'); negative "
         "feedback, deviation coordinates about the plant's operating point\"\n";
  out << "  residual: \"|| X - Ad' X Ad - Qd + (Ad' X Bd + Nd) K ||_F over the sum of the "
         "norms of its terms, of the equation AS POSED with its cross term; a solution outside "
         "residual_budget is refused, never returned\"\n";
  out << "  stability: \"every eigenvalue of Ad - Bd K strictly inside the unit circle, and "
         "the symplectic spectrum off it; either failure is a refusal\"\n";

  out << "sample_time_s: " << design.sample_time_s << "\n";
  out << "sample_rate_hz: " << discrete.sample_rate_hz() << "\n";
  out << "hold: \"" << model::to_string(discrete.hold) << "\"\n";
  out << "plant:\n";
  out << "  description: " << quoted(plant.description) << "\n";
  out << "  citation: " << quoted(plant.citation) << "\n";
  write_yaml_names(out, "  ", "state_names", discrete.state_names);
  write_yaml_names(out, "  ", "input_names", discrete.input_names);
  // The Jacobians this plant was linearised from, by the stage that took them.
  // The records themselves are in the run manifest; naming them here ties this
  // file to them.
  out << "  upstream_linearization_evidence: [";
  bool first = true;
  for (const auto& [source, record] : upstream.linearization_evidence) {
    (void)record;
    out << (first ? "" : ", ") << quoted(source);
    first = false;
  }
  out << "]\n";

  out << "declared_continuous_cost:   # per second\n";
  write_yaml_matrix(out, "  ", "q", design.continuous_q);
  write_yaml_matrix(out, "  ", "r", design.continuous_r);
  write_yaml_matrix(out, "  ", "n", design.continuous_n);

  out << "discretised_cost:   # per sample; the cross term n is the hold's own and is RETAINED\n";
  write_yaml_matrix(out, "  ", "q", design.cost.q);
  write_yaml_matrix(out, "  ", "r", design.cost.r);
  write_yaml_matrix(out, "  ", "n", design.cost.n);
  out << "  symmetry_defect: " << design.cost.symmetry_defect << "\n";
  out << "  minimum_block_eigenvalue: " << design.cost.minimum_block_eigenvalue << "\n";
  out << "  exponential_error_bound: " << design.cost.exponential_error_bound << "\n";
  out << "  exponential_squarings: " << design.cost.exponential_squarings << "\n";

  out << "discrete_plant:\n";
  write_yaml_matrix(out, "  ", "a", discrete.a);
  write_yaml_matrix(out, "  ", "b", discrete.b);
  out << "  spectral_radius: " << evidence.spectral_radius << "\n";
  out << "  fastest_continuous_mode_rad_s: " << evidence.fastest_mode_rad_s << "\n";
  out << "  nyquist_rad_s: " << evidence.nyquist_rad_s << "\n";
  out << "  exponential_one_norm: " << evidence.exponential_one_norm << "\n";
  out << "  exponential_pade_order: " << evidence.exponential_pade_order << "\n";
  out << "  exponential_squarings: " << evidence.exponential_squarings << "\n";
  out << "  exponential_error_bound: " << evidence.exponential_error_bound << "\n";

  out << "riccati:\n";
  write_yaml_matrix(out, "  ", "x", riccati.x);
  write_yaml_matrix(out, "  ", "k", riccati.k);
  out << "  relative_residual: " << riccati.relative_residual << "\n";
  out << "  residual_budget: " << riccati.residual_budget << "\n";
  out << "  symmetry_defect: " << riccati.symmetry_defect << "\n";
  out << "  subspace_condition: " << riccati.subspace_condition << "\n";
  out << "  symplectic_separation: " << riccati.symplectic_separation << "\n";
  out << "  transition_condition: " << riccati.transition_condition << "\n";
  out << "  spectral_radius: " << riccati.spectral_radius << "\n";
  write_yaml_eigenvalues(out, "  ", "closed_loop_eigenvalues", riccati.closed_loop_eigenvalues);

  out << "not_established:\n";
  out << "  - \"A margin of the sampled loop. No gain, phase, delay or disk margin was computed; "
         "a spectral radius below one is nominal convergence, not robustness.\"\n";
  out << "  - \"Tolerance of delay. The design modelled none; a delay the loop is executed "
         "with is a plant this design did not see.\"\n";
  out << "  - \"Actuator limits. None were modelled.\"\n";
  out << "  - \"Validity at any other sample time. Every matrix above belongs to this one.\"\n";
  return out.str();
}

Artifact sampled_lqr_capability(const StageContext& context) {
  const model::LinearSystem& plant = continuous_system(
      context,
      "synth.sampled_lqr",
      "It designs from the continuous plant because the exact cost of a held interval depends "
      "on how the continuous state moves THROUGH the interval, which a discrete model has "
      "already thrown away. Wire the continuous linearisation here; to solve a problem already "
      "posed in discrete time with per-sample weights, use `synth.dare`.");
  const double sample_time_s = required_sample_time(context, "synth.sampled_lqr");
  require_zero_order_hold(context, "synth.sampled_lqr");
  const Eigen::MatrixXd q = required_matrix(context, "synth.sampled_lqr", "q");
  const Eigen::MatrixXd r = required_matrix(context, "synth.sampled_lqr", "r");
  const Eigen::MatrixXd n = optional_matrix(context, "synth.sampled_lqr", "n");
  require_weight_shapes("synth.sampled_lqr", q, r, n, plant.state_count(), plant.input_count());

  synth::SampledLqrDesign design = synth::design_sampled_lqr(plant, q, r, n, sample_time_s);

  // THE EVIDENCE FILE IS REQUIRED, for the reason `model.quadrotor.export`
  // requires its own. A discrete gain is sixteen numbers per actuator that look
  // the same whatever cost, hold and sample time produced them; the record of
  // those — and of the cross term the hold added — is what makes the gain
  // checkable by someone who was not in the room.
  context.write_output(
      context.input->string_at("evidence_path"),
      sampled_design_evidence(design, plant, context.upstream_at("system"), context.stage_id));

  std::ostringstream summary;
  summary << design.riccati.k.rows() << " controls, " << design.riccati.k.cols() << " states at "
          << std::scientific << std::setprecision(3) << sample_time_s
          << " s, zero-order hold; DARE residual " << std::setprecision(1)
          << design.riccati.relative_residual << " (budget " << design.riccati.residual_budget
          << "); closed-loop spectral radius " << std::fixed << std::setprecision(6)
          << design.riccati.spectral_radius << "; discretised cross term retained, norm "
          << std::scientific << std::setprecision(2) << design.cost.n.norm();

  Artifact artifact;
  artifact.kind = "sampled_control_law";
  artifact.summary = summary.str();
  artifact.payload = std::move(design);
  return artifact;
}

// --- report sections -------------------------------------------------------

void matrix_table(std::ostream& out,
                  const std::string& name,
                  const Eigen::MatrixXd& value,
                  const std::vector<std::string>& rows,
                  const std::vector<std::string>& columns) {
  out << "**" << name << "**\n\n| row |";
  for (Eigen::Index j = 0; j < value.cols(); ++j) {
    out << ' ' << columns[static_cast<std::size_t>(j)] << " |";
  }
  out << "\n|---|";
  for (Eigen::Index j = 0; j < value.cols(); ++j) {
    out << "---:|";
  }
  out << '\n';
  for (Eigen::Index i = 0; i < value.rows(); ++i) {
    out << "| " << rows[static_cast<std::size_t>(i)] << " |";
    for (Eigen::Index j = 0; j < value.cols(); ++j) {
      out << ' ' << std::setprecision(12) << value(i, j) << " |";
    }
    out << '\n';
  }
  out << '\n' << std::defaultfloat;
}

void discretisation_table(std::ostream& out, const model::Discretisation& discretisation) {
  const model::DiscretisationEvidence& evidence = discretisation.evidence;
  out << "| Quantity | Value |\n|---|---|\n"
      << "| Sample time | " << evidence.sample_time_s << " s ("
      << discretisation.system.sample_rate_hz() << " Hz) |\n"
      << "| Hold | " << model::to_string(evidence.hold) << " |\n"
      << "| Discrete spectral radius (reported, not gated) | " << evidence.spectral_radius << " |\n"
      << "| Fastest continuous mode | " << evidence.fastest_mode_rad_s << " rad/s |\n"
      << "| Nyquist angular frequency | " << evidence.nyquist_rad_s << " rad/s |\n"
      << "| Matrix exponential: one-norm, Pade order, squarings | " << evidence.exponential_one_norm
      << ", " << evidence.exponential_pade_order << ", " << evidence.exponential_squarings << " |\n"
      << "| Declared backward error bound of the exponential | " << evidence.exponential_error_bound
      << " |\n\n";
  out << "_Assumptions:_ " << evidence.assumptions << "\n\n";
}

void riccati_table(std::ostream& out, const synth::DareSolution& solution) {
  out << "| Numerical check | Value |\n|---|---:|\n"
      << std::scientific << std::setprecision(6)
      << "| Relative DARE residual, cross term included | " << solution.relative_residual << " |\n"
      << "| Residual acceptance budget | " << solution.residual_budget << " |\n"
      << "| Symmetry defect | " << solution.symmetry_defect << " |\n"
      << "| Deflating-subspace condition | " << solution.subspace_condition << " |\n"
      << "| Symplectic spectrum's distance from the unit circle | "
      << solution.symplectic_separation << " |\n"
      << "| Condition of A - B R^-1 N' | " << solution.transition_condition << " |\n"
      << "| Closed-loop spectral radius (strictly below 1 or refused) | "
      << solution.spectral_radius << " |\n\n"
      << std::defaultfloat;
}

void conventions(std::ostream& out) {
  out << "**Conventions.** The cost is J = sum over k of x[k]' Q x[k] + 2 x[k]' N u[k] + u[k]' "
         "R u[k], with weights per sample. The gain is applied as u[k] = -K x[k], with K = (R + "
         "B' X B)^-1 (B' X A + N'). The residual is that of X = A' X A + Q - (A' X B + N) K — "
         "the equation as posed, cross term included — relative to the sum of the norms of its "
         "terms, and a solution outside its budget is refused rather than returned. Every "
         "closed-loop eigenvalue of A - B K must lie strictly inside the unit circle and the "
         "symplectic spectrum must stay off it; either failure is a refusal.\n\n";
  out << "A stabilising solution says the NOMINAL sampled loop converges. It is not a gain, "
         "phase, delay or disk margin of that loop, and no capability here computes one.\n\n";
}

}  // namespace

bool write_discrete_section(std::ostream& out, const Artifact& artifact) {
  if (artifact.kind == "discrete_linear_system") {
    const auto& discretisation =
        artifact.payload_as<model::Discretisation>("discrete_linear_system");
    out << "Discrete-time model x[k+1] = A x[k] + B u[k], " << discretisation.system.state_count()
        << " states and " << discretisation.system.input_count()
        << " inputs, exact under the stated hold.\n\n";
    discretisation_table(out, discretisation);
    out << "This describes the state AT THE TICKS and nothing between them, includes no "
           "computational or transport delay, and does not filter: a continuous mode above the "
           "Nyquist frequency appears here as a slower one.\n\n";
    return true;
  }
  if (artifact.kind == "dare_solution") {
    const auto& dare = artifact.payload_as<DareArtifact>("dare_solution");
    out << "Discrete algebraic Riccati equation at a sample time of " << dare.sample_time_s
        << " s, posed on "
        << (dare.from_discrete_model ? "a discrete_linear_system"
                                     : "matrices declared in the study with their sample time")
        << ".\n\n";
    matrix_table(out, "Riccati solution X", dare.solution.x, dare.state_names, dare.state_names);
    matrix_table(
        out, "Gain K (input per state unit)", dare.solution.k, dare.input_names, dare.state_names);
    riccati_table(out, dare.solution);
    conventions(out);
    return true;
  }
  if (artifact.kind == "sampled_control_law") {
    const auto& design = artifact.payload_as<synth::SampledLqrDesign>("sampled_control_law");
    const model::DiscreteLinearSystem& plant = design.discretisation.system;
    out << "Sampled full-state feedback designed for a sample time of " << design.sample_time_s
        << " s (" << plant.sample_rate_hz()
        << " Hz) under a zero-order hold: the plant and the declared continuous cost were both "
           "discretised under that hold, and the discrete Riccati equation was solved for the "
           "discretised problem.\n\n";
    matrix_table(out,
                 "Gain K (input per state unit)",
                 design.riccati.k,
                 plant.input_names,
                 plant.state_names);
    out << "The discretised cost carries a state-input cross term N that the hold produces "
        << (design.continuous_n.norm() == 0.0 ? "even though the declared continuous cost had none"
                                              : "on top of the declared continuous cross term")
        << ". It was solved with, and it is shown here because dropping it would describe a "
           "different objective from the one that was minimised.\n\n";
    matrix_table(out,
                 "Discretised cross term N (per sample)",
                 design.cost.n,
                 plant.state_names,
                 plant.input_names);
    matrix_table(out,
                 "Discretised input weight R (per sample)",
                 design.cost.r,
                 plant.input_names,
                 plant.input_names);
    out << "The discretised state weight, the declared continuous weights, the discrete plant "
           "and the Riccati solution X are in the artefact and in the evidence file this "
           "stage is required to write.\n\n";
    discretisation_table(out, design.discretisation);
    riccati_table(out, design.riccati);
    conventions(out);
    out << "No delay and no actuator limit were modelled in this design. A loop executed with "
           "either is running on a plant the design did not see, and `sim.sampled` records "
           "which it was flown with.\n\n";
    return true;
  }
  return false;
}

void register_discrete_capabilities(Registry& registry) {
  // ImplementedUnvalidated throughout. The references behind these are closed
  // forms, Riccati value iteration and hand-integrated interval costs — every one
  // independent of the routine it checks, and none of them a published DARE
  // benchmark. See `sampled.discrete_references` in the case registry.
  registry.add(Capability{
      "model.discretize",
      "Discretise a continuous linear model exactly under a declared zero-order hold at a "
      "declared sample time, reporting the fastest mode against the Nyquist frequency",
      "discrete_linear_system",
      Capability::State::ImplementedUnvalidated,
      discretize_capability,
      {"system", "sample_time_s", "hold"}});

  registry.add(Capability{
      "synth.dare",
      "Solve a discrete-time algebraic Riccati equation, cross term included, refusing a "
      "residual over budget or a closed loop not strictly inside the unit circle",
      "dare_solution",
      Capability::State::ImplementedUnvalidated,
      dare_capability,
      {"system", "a", "b", "sample_time_s", "q", "r", "n"}});

  registry.add(Capability{
      "synth.sampled_lqr",
      "Design sampled full-state feedback: discretise the plant and the continuous cost under "
      "one hold, retain the cost's state-input cross term, and solve the discrete Riccati "
      "equation",
      "sampled_control_law",
      Capability::State::ImplementedUnvalidated,
      sampled_lqr_capability,
      {"system", "sample_time_s", "hold", "q", "r", "n", "evidence_path"},
      {},
      {"evidence_path"}});
}

}  // namespace galata::pipeline
