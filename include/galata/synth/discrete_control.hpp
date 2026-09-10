// SPDX-License-Identifier: Apache-2.0
//
// Sampled quadratic state feedback: the discrete algebraic Riccati equation,
// the exact discretisation of a continuous quadratic cost under a hold, and the
// design that puts the two together.
//
// References:
//   T. Pappas, A. J. Laub and N. R. Sandell, "On the Numerical Solution of the
//   Discrete-Time Algebraic Riccati Equation", IEEE TAC 25(4), 1980,
//   pp. 631-641 — the symplectic pencil and the deflating subspace this file
//   takes the stabilising solution from.
//   A. J. Laub, "A Schur Method for Solving Algebraic Riccati Equations", IEEE
//   TAC 24(6), 1979, pp. 913-921 — the ordered-Schur construction, which the
//   continuous solver beside this one already uses.
//   C. F. Van Loan, "Computing Integrals Involving the Matrix Exponential",
//   IEEE TAC 23(3), 1978, pp. 395-404 — the block-matrix exponential that makes
//   the cost discretisation exact rather than a rectangle rule.
//   B. D. O. Anderson and J. B. Moore, "Optimal Control: Linear Quadratic
//   Methods", 1990, chapters 3 and 6 — the discrete-time optimality conditions
//   and the detectability requirement.
//   K. J. Astrom and B. Wittenmark, "Computer-Controlled Systems", 3rd ed.,
//   1997, chapter 11.
//
// ===========================================================================
// WHY THE COST HAS TO BE DISCRETISED AND NOT JUST REUSED
// ===========================================================================
//
// A continuous cost integral(x'Qx + u'Ru) dt is not the same objective as the
// sum(x'Qx + u'Ru) a naive port would minimise, and the difference is not a
// scale factor. Holding u constant across an interval makes x move THROUGH the
// interval, so the true cost of that interval depends on the state at its start
// AND on the input that acted during it — a state-input CROSS TERM that the
// continuous problem may not have had at all.
//
// So `discretize_cost` returns an N even when it was handed none, and the DARE
// below takes one. Dropping it minimises a different functional and returns a
// gain that is optimal for nothing in particular; multiplying Q by the sample
// time instead is a first-order approximation of the integral whose error is
// unstated. Both are common, and neither is what F14 asked for: "Preserve the
// cost discretisation and input-hold assumptions in the design."
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * NOT A SAMPLED-LOOP MARGIN, and it computes none. A stabilising discrete
//   gain with every closed-loop eigenvalue inside the unit circle says the
//   nominal sampled loop converges. It says nothing about gain, phase, delay or
//   simultaneous robustness of that loop, which needs its own machinery. Do not
//   read `spectral_radius` as a robustness figure.
// * NOT A DELAY-AWARE DESIGN. The hold is modelled; computational and transport
//   delay are not. A design produced here and then executed with a delay is
//   being run on a plant this design did not see.
// * NOT AN ESTIMATOR, and full state feedback is assumed exactly as in the
//   continuous solver. Sampling does not make measured states any more
//   available than they were.
// * NOT A CONSTRAINED DESIGN. Actuator limits are absent. A gain that saturates
//   every actuator on the first tick satisfies every check here.
// * NOT VALID ACROSS SAMPLE RATES. Every matrix and every eigenvalue below
//   belongs to ONE sample time. Comparing a discrete pole from one rate against
//   one from another is comparing two different coordinate systems.
#ifndef GALATA_SYNTH_DISCRETE_CONTROL_HPP
#define GALATA_SYNTH_DISCRETE_CONTROL_HPP

#include "galata/model/discrete_system.hpp"
#include "galata/model/linear_system.hpp"

#include <complex>
#include <string>
#include <vector>

namespace galata::synth {

struct DareSolution {
  Eigen::MatrixXd x;  // cost / (state_i state_j)
  Eigen::MatrixXd k;  // input_i / state_j
  // Backward error of the DISCRETE Riccati equation as it was posed, cross term
  // included, against a conditioning-scaled budget no caller can widen.
  double relative_residual = 0.0;  // dimensionless
  double residual_budget = 0.0;    // dimensionless
  double symmetry_defect = 0.0;    // dimensionless
  double subspace_condition = 0.0; // dimensionless
  // How far the symplectic spectrum stays from the UNIT CIRCLE — the discrete
  // analogue of the Hamiltonian's distance from the imaginary axis. A spectrum
  // touching the circle has no separated stabilising solution.
  double symplectic_separation = 0.0;  // dimensionless
  // Condition number of the state transition the symplectic form must invert.
  // Reported because the construction needs it invertible and a caller whose
  // problem is near the boundary deserves to know before reading the gain.
  double transition_condition = 0.0;  // dimensionless
  // max |lambda| of A - B K. Strictly below 1 for a stabilising solution.
  double spectral_radius = 0.0;  // dimensionless
  std::vector<std::complex<double>> closed_loop_eigenvalues;  // dimensionless
};

// Q and R must be symmetric, R positive definite, and the block cost
// [Q N; N' R] positive semidefinite. Empty N means zero. A is NOT required to
// be invertible as a model, but the symplectic construction inverts
// A - B R^-1 N', and a numerically singular one is refused by name rather than
// producing an uncertified gain.
//
// Throws on invalid input, on a spectrum that touches the unit circle, on an
// ill-conditioned deflating subspace, and on a residual outside the budget.
[[nodiscard]] DareSolution solve_dare(const Eigen::MatrixXd& a,
                                      const Eigen::MatrixXd& b,
                                      const Eigen::MatrixXd& q,
                                      const Eigen::MatrixXd& r,
                                      const Eigen::MatrixXd& n = {});

struct DiscreteCost {
  Eigen::MatrixXd q;  // state-cost weights per SAMPLE, mixed units
  Eigen::MatrixXd r;  // input-cost weights per sample, mixed units
  // The state-input cross term the hold produces. Nonzero in general even when
  // the continuous cost had none; see the header's note.
  Eigen::MatrixXd n;
  double sample_time_s = 0.0;  // s
  // Symmetry defect of the block cost before it was symmetrised, as a
  // dimensionless ratio. Reported rather than silently cleaned.
  double symmetry_defect = 0.0;
  // The smallest eigenvalue of the discretised block cost, scaled by its norm.
  // A materially negative value means the integral lost definiteness to
  // roundoff, which is refused.
  double minimum_block_eigenvalue = 0.0;  // dimensionless
  double exponential_error_bound = 0.0;   // dimensionless, declared
  int exponential_squarings = 0;
  std::string assumptions;
};

// The exact cost of one interval under a zero-order hold, by the exponential of
// Van Loan's block matrix. Refuses a non-positive or non-finite sample time, a
// system with no inputs, and a block cost that is not positive semidefinite.
[[nodiscard]] DiscreteCost discretize_cost(const model::LinearSystem& plant,
                                           const Eigen::MatrixXd& q,
                                           const Eigen::MatrixXd& r,
                                           const Eigen::MatrixXd& n,
                                           double sample_time_s);

struct SampledLqrDesign {
  DareSolution riccati;
  model::Discretisation discretisation;  // the discrete plant, and how it was made
  DiscreteCost cost;                     // the discretised objective
  // x[k+1] = (A - B K) x[k], carrying the plant's names and sample time.
  model::DiscreteLinearSystem closed_loop;
  double sample_time_s = 0.0;  // s
  // The continuous weights the caller declared, kept so a report can show what
  // was asked for beside what was solved.
  Eigen::MatrixXd continuous_q;
  Eigen::MatrixXd continuous_r;
  Eigen::MatrixXd continuous_n;
};

// THE WHOLE SAMPLED PATH, in the order it must happen: discretise the plant
// under the declared hold, discretise the cost under the same hold, then solve
// the discrete Riccati equation for the discretised problem. Designing on the
// continuous problem and discretising the gain afterwards is a different and
// weaker operation, and is not what this does.
//
// Refuses everything `discretize_zoh`, `discretize_cost` and `solve_dare`
// refuse, each by its own message.
[[nodiscard]] SampledLqrDesign design_sampled_lqr(const model::LinearSystem& plant,
                                                  const Eigen::MatrixXd& q,
                                                  const Eigen::MatrixXd& r,
                                                  const Eigen::MatrixXd& n,
                                                  double sample_time_s);

}  // namespace galata::synth

#endif  // GALATA_SYNTH_DISCRETE_CONTROL_HPP
