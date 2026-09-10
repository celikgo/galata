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
// and reports what it reached. `converged` is a statement about the last step's
// size, not a licence the optimiser granted itself.
//
// AND THREE THINGS THAT ARE NOT THE SAME, kept apart because conflating them is
// how an identification run produces a confident wrong answer:
//
//   the OPTIMISER FINISHED — it ran its iterations and the objective stopped
//     moving. Says nothing about whether the parameters are meaningful.
//   the PARAMETERS ARE IDENTIFIABLE — the data actually constrains each one,
//     separately. A direction the Jacobian cannot see is a parameter the run
//     did not measure, however small the residual became.
//   the FIT IS ACCEPTABLE — an engineering judgement about whether the residual
//     is small enough for the use. This code does not make it and does not
//     imply it.
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

struct GreyboxResult {
  std::vector<std::string> names;
  Eigen::VectorXd value;
  Eigen::VectorXd standard_error;  // meaningful only when uncertainty_is_estimable
  double objective = 0.0;          // final sum of squared scaled residuals
  double residual_rms = 0.0;       // in scaled units
  int iterations = 0;              // as declared; not a count of what was needed
  int residual_count = 0;
  double last_step_norm = 0.0;
  bool optimiser_finished = false;
  bool uncertainty_is_estimable = false;
  double jacobian_condition_number = 0.0;
  std::vector<bool> at_bound;  // a parameter resting on a bound is not an interior estimate
  std::string note;
};

// Refuses: an unrecognised parameter path; an initial value outside its own
// bounds; bounds that do not bracket; a record missing a declared channel; a
// model whose parameters go invalid during the search; and a parameter
// direction the data does not constrain.
[[nodiscard]] GreyboxResult fit_greybox(const model::Quadrotor& model,
                                        const data::Record& record,
                                        const GreyboxRequest& request);

}  // namespace galata::identify
