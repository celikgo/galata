// SPDX-License-Identifier: Apache-2.0
//
// Reachability and observability of a linear model: which directions in the
// state space a declared input set can move, which a declared output set can
// see, and how strongly over a declared horizon.
//
// Reference:
//   R. E. Kalman, "Mathematical description of linear dynamical systems",
//   SIAM J. Control 1(2), pp. 152-192, 1963 — the reachable and unobservable
//   subspaces, and the Gramians that measure them.
//   T. Kailath, "Linear Systems", Prentice-Hall, 1980, chapters 2 and 6 — the
//   staircase construction this file uses instead of the Krylov matrix.
//   G. H. Golub and C. F. Van Loan, "Matrix Computations", 4th ed., 2013,
//   sections 5.4 and 8.6 — orthogonal bases by SVD, and numerical rank.
//
// WHY THIS EXISTS. RFC-0002 asked for it in one sentence: "so that an exported
// model's defective integrator chains and any unobservable direction are
// reported rather than discovered from a failed synthesis". A synthesis that
// fails on an unreachable direction fails with a Riccati diagnostic, several
// stages after the fact, and a caller then has to work backwards to which
// coordinate it was. This answers the question directly, before any design.
//
// ===========================================================================
// THE INFINITE-HORIZON GRAMIAN USUALLY DOES NOT EXIST FOR THIS PLANT
// ===========================================================================
//
// The textbook controllability Gramian is the T -> infinity limit of the
// integral below, and that limit exists only when every eigenvalue of A has a
// strictly negative real part. A multirotor linearised at hover has six
// eigenvalues AT the origin — three because nothing reads position and three
// because nothing reads attitude at a level hover — so the limit diverges and
// the matrix a Lyapunov solve would return for it is not a Gramian of anything.
//
// So the integral is taken over a DECLARED FINITE HORIZON, which is a different
// quantity and is labelled as one. It answers "how far can these inputs move
// each direction in T seconds", which is the engineering question anyway, and it
// exists for every A. `spectrum_is_strictly_stable` says whether the T ->
// infinity limit would exist at all, so a reader knows whether the finite
// horizon is a truncation of something or the only thing available.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * NOT A PROOF OF RANK. Numerical rank over floating point is a judgement
//   against a threshold, and the threshold is declared rather than universal: a
//   singular value below `rank_tolerance` times the largest is treated as zero.
//   A direction reachable only through a gain of 1e-12 is unreachable for every
//   engineering purpose and is reported as such; that is a choice, and a caller
//   who disagrees changes the tolerance and gets a different rank.
// * NOT SCALE-FREE, and the Gramian's eigenvalues are the place this bites
//   hardest. A rotor speed is several hundred radians per second and a position
//   is of order one metre, so an unscaled Gramian's spectrum spans many orders
//   of magnitude for reasons that are entirely about units. Its condition
//   number is reported so that a reader does not mistake unit spread for a
//   near-unreachable direction. The RANK results do not have this problem: they
//   are taken from an orthogonal staircase whose tolerance is relative.
// * NOT A BALANCED REALISATION and not a model-reduction tool. It reports; it
//   does not transform the model, and it computes no Hankel singular values.
// * NOT A STATEMENT ABOUT THE NONLINEAR PLANT. Reachability of a linearisation
//   is reachability of the linearisation, valid where the linearisation is.
// * NOT A STATEMENT ABOUT AN AIRCRAFT'S SENSORS. Observability here is a
//   property of the DECLARED OUTPUT SET of the model handed in. A model whose
//   outputs omit a heading reference is unobservable in yaw whether or not the
//   airframe carries a magnetometer, and the right reading of such a result is
//   "this observation model cannot see that direction" — never "the vehicle
//   cannot". Making the model see it means adding the observation, which is a
//   model extension and not a correction to this analysis.
// * NOT A CONTROLLABILITY GUARANTEE UNDER LIMITS. Every actuator here is
//   unbounded. A direction this reports as reachable may be reachable only with
//   a rotor speed no motor can produce, and nothing in these figures says so.
#ifndef GALATA_ANALYZE_GRAMIANS_HPP
#define GALATA_ANALYZE_GRAMIANS_HPP

#include "galata/model/linear_system.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace galata::analyze {

struct GramianOptions {
  // The horizon the Gramians are integrated over. Required and never defaulted:
  // the answer depends on it, and a default would put a number nobody chose
  // into a figure a reader will quote.
  double horizon_s = 0.0;
  // Fixed integration steps over that horizon, per ADR-0004. Not a tolerance,
  // and not adaptive: the same bits every run.
  int steps = 400;
  // Relative singular-value floor for the numerical rank decisions. Declared,
  // for the reason the header gives.
  double rank_tolerance = 1e-9;
};

// One direction the inputs cannot move, or the outputs cannot see, described in
// the terms a reader can act on: which states it is made of.
struct StateDirection {
  // The unit vector itself, in the model's own state coordinates.
  Eigen::VectorXd coordinates;
  // The states carrying the largest share of it, most first, as
  // "name (share)" where the share is the squared component. Only the states
  // above a one-percent share are listed: a list of sixteen names in which
  // fifteen are noise is not a legible answer.
  std::vector<std::string> dominant_states;
};

struct SubspaceAnalysis {
  int state_count = 0;
  // The dimension of the reachable (or observable) subspace, at the declared
  // tolerance.
  int rank = 0;
  // Singular values of the staircase basis at the step that decided the rank,
  // largest first, so a reader can see how close the decision was.
  std::vector<double> singular_values;
  // The ratio of largest to smallest RETAINED singular value. A large value
  // means the rank decision was marginal.
  double retained_condition_number = 0.0;
  // Empty when the rank is full. Otherwise one entry per missing dimension.
  std::vector<StateDirection> missing_directions;
};

struct GramianAnalysis {
  // Whether the T -> infinity Gramians exist at all. False for any model with
  // an eigenvalue on or right of the imaginary axis, which includes every
  // multirotor hover linearisation this repository produces.
  bool spectrum_is_strictly_stable = false;
  // The largest real part in the spectrum, so a reader can see how far from
  // strictly stable the model is rather than only that it is.
  double rightmost_eigenvalue_real_part = 0.0;

  SubspaceAnalysis reachability;
  SubspaceAnalysis observability;

  // The finite-horizon Gramians over `horizon_s`, symmetric and positive
  // semi-definite by construction. Kept whole: a caller wanting a specific
  // direction's energy needs the matrix, not a summary of it.
  Eigen::MatrixXd controllability;
  Eigen::MatrixXd observability_gramian;
  // Eigenvalues of each, largest first. Read the header's scaling warning
  // before drawing a conclusion from the smallest.
  std::vector<double> controllability_eigenvalues;
  std::vector<double> observability_eigenvalues;
  double controllability_condition_number = 0.0;
  double observability_condition_number = 0.0;

  double horizon_s = 0.0;
  int steps = 0;
  double rank_tolerance = 0.0;
  // The two sentences a report quotes: what was computed, and what it does not
  // establish.
  std::string assumptions;
};

// Refuses: a non-finite or malformed system; a horizon that is not positive and
// finite; a step count below one; a rank tolerance outside (0, 1).
[[nodiscard]] GramianAnalysis analyse_gramians(const model::LinearSystem& system,
                                               const GramianOptions& options);

}  // namespace galata::analyze

#endif  // GALATA_ANALYZE_GRAMIANS_HPP
