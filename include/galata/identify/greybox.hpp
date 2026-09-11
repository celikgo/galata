// SPDX-License-Identifier: Apache-2.0
//
// Grey-box identification: fitting declared parameters of the nonlinear plant
// to a measured record.
//
// GREY, not black. The model is `model::Quadrotor` — the same plant `sim.plant`
// integrates, not a surrogate and not a second implementation — and what is
// fitted is a DECLARED SUBSET of its parameters. Nothing is discovered: a
// parameter the study did not name is held at the value the model file gave it.
//
// THE OBJECTIVE, stated because every other number here is relative to it. The
// model is simulated from a declared initial state with the record's own inputs
// held zero-order between samples, its declared outputs are compared with the
// record's, and the sum of squared residuals — each divided by that channel's
// declared scale — is minimised. The scale is the caller's: without it a
// position error in metres and a rate error in radians per second are added
// together as though a metre and a radian per second were the same size, which
// is a weighting decision made by accident.
//
// FIXED ITERATIONS, NEVER A TOLERANCE. ADR-0004 requires the same bits every
// run, and an optimiser that stops when it is "close enough" stops after a
// different number of steps on a different machine. This runs a declared count
// and reports what it reached.
//
// AND FIVE THINGS THAT ARE NOT THE SAME, kept apart because conflating them is
// how an identification run produces a confident wrong answer. An earlier
// version of this header listed three and the result carried a single
// `optimiser_finished` flag that was assigned `true` unconditionally — which,
// with a declared iteration count, is a field that can never be false and
// therefore reads as a check that passed. It has been removed rather than
// documented, and the five questions below are answered separately:
//
//   1. EXECUTION COMPLETED — the routine ran the work it was asked for.
//      `iterations_declared` and `iterations_run`. With a fixed count these are
//      equal by construction; both are reported so the invariant is visible
//      rather than assumed, and so a future stopping rule cannot quietly make
//      them differ without a reader seeing it.
//   2. WHY IT STOPPED — `stop_reason`, which has exactly one value.
//      `DeclaredIterationsCompleted` is the only way out of the loop, and the
//      enum exists to say that in the result rather than in a comment. A
//      tolerance-based reason is not missing; it is forbidden.
//   3. THE OBJECTIVE IMPROVED — `initial_objective` against `objective`, with
//      `objective_improved` and `accepted_steps`. A run whose objective never
//      improved returned its own starting point, and that is a finding rather
//      than a refusal.
//   4. CONVERGENCE — `convergence`, which is EVIDENCE and not a verdict. There
//      is deliberately no `converged` boolean: first-order optimality is a
//      matter of degree against a scale only the caller knows, and a flag would
//      be this code making an engineering judgement it is not entitled to. What
//      is reported is the first-order measure at the FINAL point, the size of
//      the last step, and which iteration last improved anything.
//   5. THE PARAMETERS ARE IDENTIFIABLE — the data actually constrains each one,
//      separately. A direction the Jacobian cannot see is a parameter the run
//      did not measure, however small the residual became. This one is a
//      REFUSAL rather than a field: see `identifiability_ratio`.
//
// None of the five is the sixth question, THE FIT IS ACCEPTABLE — an engineering
// judgement about whether the residual is small enough for the use. This code
// does not make it and does not imply it. A returned result does not mean the
// fit converged; it means the routine ran.
#pragma once

#include "galata/model/quadrotor.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace galata::data {
struct Record;
}  // namespace galata::data

namespace galata::identify {

// One parameter to fit, addressed by a path into the model.
//
// Supported paths, and nothing else — an unrecognised one is refused rather
// than ignored, because a parameter silently not fitted is a run that reports
// success having estimated less than it was asked to:
//
//   mass.mass_kg
//   rotors[i].thrust_coefficient_n_s2
//   rotors[i].torque_coefficient_n_m_s2
//   rotors[i].speed_time_constant_s
//   drag.linear_n_s_m[k]
//   drag.quadratic_n_s2_m2[k]
//   drag.angular_n_m_s[k]
struct Parameter {
  std::string path;
  double lower = 0.0;
  double upper = 0.0;  // upper must exceed lower
  double initial = 0.0;
};

// Which recorded channel corresponds to which extended-state component.
struct OutputMatch {
  std::string channel;     // in the record
  std::string state_name;  // from Quadrotor::extended_state_names()
  // Divides this channel's residual. The caller's weighting decision, made on
  // purpose rather than by unit accident.
  double scale = 1.0;
};

struct GreyboxRequest {
  std::vector<Parameter> parameters;
  std::vector<OutputMatch> outputs;
  // Channels holding the commanded rotor speeds, in rotor order.
  std::vector<std::string> command_channels;
  Eigen::VectorXd initial_extended_state;
  double step_s = 0.0;
  int iterations = 40;
  // A direction of the Jacobian whose singular value is below this fraction of
  // the largest is treated as unmeasured, and the run is refused rather than
  // reporting a number for it.
  double identifiability_ratio = 1e-6;
};

// The only way this routine leaves its loop. One value, deliberately: a
// tolerance-based exit would stop after a different number of steps on a
// different machine, which ADR-0004 forbids. The enum exists so the result
// STATES the stopping rule instead of a reader having to know it.
enum class StopReason {
  DeclaredIterationsCompleted,
};

[[nodiscard]] std::string to_string(StopReason reason);

// EVIDENCE ABOUT CONVERGENCE, AND NOT A VERDICT ABOUT IT.
//
// There is no `converged` flag here on purpose. First-order optimality is a
// matter of degree against a scale only the caller knows — what counts as a
// small gradient depends on what the parameters mean and what the fit is for —
// so a boolean would be this code making a judgement it is not entitled to, and
// a caller would read it as one.
struct ConvergenceEvidence {
  // The norm of the last ACCEPTED step, or zero when the last iteration's trial
  // point was rejected. Zero therefore means "the last thing tried did not
  // help", which is not the same as "there was nothing left to try".
  double last_step_norm = 0.0;
  // The last iteration that improved the objective, or -1 when none did. A run
  // whose last improvement was at iteration 2 of 40 spent 38 iterations
  // confirming it was stuck, which is worth seeing.
  int last_accepted_iteration = -1;

  // FIRST-ORDER OPTIMALITY AT THE FINAL POINT: the infinity norm of J^T r,
  // where J is recomputed AT the point being reported rather than reused from
  // the last iteration's start. A stationary point of the least-squares
  // objective has J^T r = 0; how near zero is near enough is the caller's
  // question. Units are mixed across parameters, so this figure is not
  // comparable between two parameters of different kinds.
  double gradient_infinity_norm = 0.0;
  // The same measure made dimensionless and comparable, by scaling each
  // parameter's gradient component by that parameter's own declared bound span:
  // the first-order change in the objective from moving that parameter across
  // the whole range the study declared admissible. A small value here means the
  // objective is flat against the caller's OWN notion of how far the parameter
  // could move, which is the closest thing to a scale-free convergence
  // indication this routine can honestly report.
  double gradient_over_bound_span_infinity_norm = 0.0;
};

struct GreyboxResult {
  std::vector<std::string> names;
  Eigen::VectorXd value;
  Eigen::VectorXd standard_error;  // meaningful only when uncertainty_is_estimable

  // 1. Execution completed.
  int iterations_declared = 0;
  int iterations_run = 0;
  // 2. Why it stopped.
  StopReason stop_reason = StopReason::DeclaredIterationsCompleted;
  // 3. The objective improved.
  double objective = 0.0;          // final sum of squared scaled residuals
  double initial_objective = 0.0;  // at the declared starting point, for comparison
  bool objective_improved = false;
  int accepted_steps = 0;     // iterations whose trial point beat the incumbent
  double residual_rms = 0.0;  // in scaled units
  int residual_count = 0;
  // 4. Convergence evidence, which is not a convergence claim.
  ConvergenceEvidence convergence;
  // 5. Identifiability. The condition number and the ratio the run was held to,
  //    both at the final point. A fit that failed this test does not return at
  //    all — the refusal is the report — so these describe a fit that passed it.
  double jacobian_condition_number = 0.0;
  double identifiability_ratio = 0.0;

  bool uncertainty_is_estimable = false;
  std::vector<bool> at_bound;  // a parameter resting on a bound is not an interior estimate
  std::string note;

  // THE FIT'S PRODUCT, and the reason it is here rather than left to the caller
  // to rebuild. `value` is a vector of numbers whose meaning is a list of path
  // strings; turning that back into a plant means re-implementing the path
  // resolution this file's own `resolve` already does, and a second
  // implementation of it is a second place for a parameter to land somewhere
  // other than where the optimiser thought it did. So the routine that moved
  // the parameters returns what it moved them into.
  //
  // It is the base model with the DECLARED parameters replaced and nothing
  // else touched: every parameter the study did not name is the value the base
  // model file gave it, byte for byte. `fitted_parameter_paths()` is the
  // complete list of what changed, so what did not is the complement — that
  // is what makes "unfitted parameters are preserved" a checkable statement
  // rather than a promise.
  model::Quadrotor fitted_model;

  [[nodiscard]] const std::vector<std::string>& fitted_parameter_paths() const noexcept {
    return names;
  }
};

// Refuses: an unrecognised parameter path; an initial value outside its own
// bounds; bounds that do not bracket; a record missing a declared channel; a
// model whose parameters go invalid during the search; and a parameter
// direction the data does not constrain.
[[nodiscard]] GreyboxResult fit_greybox(const model::Quadrotor& model,
                                        const data::Record& record,
                                        const GreyboxRequest& request);

}  // namespace galata::identify
