// SPDX-License-Identifier: Apache-2.0
//
// A discrete-time linear state-space model, and the zero-order-hold
// discretisation that produces one from a continuous model.
//
//   x[k+1] = A x[k] + B u[k]
//   y[k]   = C x[k] + D u[k]
//
// References:
//   C. F. Van Loan, "Computing Integrals Involving the Matrix Exponential",
//   IEEE Trans. Automatic Control 23(3), 1978, pp. 395-404 — the block-matrix
//   exponential this file uses to get A and B together and exactly.
//   K. J. Astrom and B. Wittenmark, "Computer-Controlled Systems: Theory and
//   Design", 3rd ed., 1997, chapters 2-3 — the sampled-system model, and what
//   the hold assumption does and does not cover.
//   G. F. Franklin, J. D. Powell and M. L. Workman, "Digital Control of
//   Dynamic Systems", 3rd ed., 1998, chapter 4.
//
// ===========================================================================
// WHY THIS IS ITS OWN TYPE AND NOT A FLAG ON `LinearSystem`
// ===========================================================================
//
// `model::LinearSystem` means x_dot = A x + B u. Its A has units of 1/time and
// its eigenvalues are compared against the imaginary axis. A discrete model's A
// is dimensionless and its eigenvalues are compared against the UNIT CIRCLE.
// The two are not interchangeable, and almost every routine in this repository
// that takes a `LinearSystem` would accept a discrete one, run, and return a
// number that means nothing — a "settling time" in samples reported as seconds,
// a Hurwitz test on a matrix for which Hurwitz is the wrong question.
//
// A bool on the existing struct would make that mistake possible everywhere and
// catchable nowhere. A separate type makes it a COMPILE ERROR for a C++ caller
// and a named refusal at the pipeline boundary, which is what F14's "mixed time
// domains are rejected without an explicit adapter" asks for. The adapter is
// explicit: `discretize_zoh` in one direction, and nothing in the other, since
// recovering a continuous model from a sampled one is a different problem with
// its own non-uniqueness.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * NOT A MODEL OF THE INTER-SAMPLE RESPONSE. The equations above describe the
//   state AT THE TICKS and nothing between them. A trajectory that satisfies
//   them can still be doing something unacceptable between samples — the
//   classic case is a hidden oscillation at exactly the sample rate, which this
//   model shows as a constant. Reading `y[k]` as "the output" is the mistake
//   this note exists for.
// * NOT AN APPROXIMATION OF THE HOLD, but exact FOR the hold. A zero-order hold
//   is assumed: the input is constant across each interval and steps at the
//   ticks. Given that assumption the state transition is exact to the matrix
//   exponential's backward error. If the real actuator ramps, slews, or updates
//   between ticks, the assumption is wrong and no accuracy figure here says so.
// * NOT A DELAY MODEL. Computational delay, transport delay and the sampler's
//   own latency are separate and are not folded in. A loop whose delay matters
//   needs it stated and represented, not absorbed into a sample time.
// * NOT ANTI-ALIASED. Discretisation does not filter. A continuous mode faster
//   than the Nyquist rate aliases to a slower one, and the discrete model shows
//   the alias as if it were real dynamics. `fastest_mode_rad_s` and
//   `nyquist_rad_s` are both reported so the comparison can be made; neither is
//   turned into a pass or a refusal, because the acceptable ratio is an
//   engineering decision and not this routine's to make.
// * NOT A STATEMENT ABOUT THE NONLINEAR PLANT. Discretising a linearisation
//   gives a discrete model of the linearisation, valid where that is.
#ifndef GALATA_MODEL_DISCRETE_SYSTEM_HPP
#define GALATA_MODEL_DISCRETE_SYSTEM_HPP

#include "galata/model/linear_system.hpp"

#include <Eigen/Core>

#include <complex>
#include <string>
#include <vector>

namespace galata::model {

// How the input behaves between ticks. Only one is implemented; the enum exists
// so that a model records which assumption produced it rather than leaving a
// reader to infer it from the fact that only one was available at the time.
enum class InputHold {
  ZeroOrder,  // constant across the interval, stepping at the tick
};

[[nodiscard]] const char* to_string(InputHold hold);

struct DiscreteLinearSystem {
  Eigen::MatrixXd a;  // n x n, dimensionless
  Eigen::MatrixXd b;  // n x m, state per unit input
  // p x n. Empty means the outputs ARE the states, as in `LinearSystem`.
  Eigen::MatrixXd c;
  Eigen::MatrixXd d;  // p x m. Empty means no direct feedthrough.

  // The interval between ticks. Required: there is no sensible default, and a
  // discrete model whose sample time is unknown cannot be compared with
  // anything, simulated at a rate, or turned back into a frequency.
  double sample_time_s = 0.0;  // s
  InputHold hold = InputHold::ZeroOrder;

  std::vector<std::string> state_names;   // n entries
  std::vector<std::string> input_names;   // m entries
  std::vector<std::string> output_names;  // p entries; empty mirrors state_names

  std::string description;
  std::string citation;
  std::string units;

  // Throws std::invalid_argument describing the first inconsistency found,
  // including a sample time that is not positive and finite.
  void validate() const;

  [[nodiscard]] Eigen::Index state_count() const {
    return a.rows();
  }

  [[nodiscard]] Eigen::Index input_count() const {
    return b.cols();
  }

  [[nodiscard]] Eigen::Index output_count() const {
    return c.size() == 0 ? a.rows() : c.rows();
  }

  [[nodiscard]] Eigen::MatrixXd output_matrix() const;
  [[nodiscard]] Eigen::MatrixXd feedthrough_matrix() const;
  [[nodiscard]] std::vector<std::string> output_labels() const;

  // The sample rate, for reports that quote a rate rather than an interval.
  [[nodiscard]] double sample_rate_hz() const {
    return 1.0 / sample_time_s;
  }
};

struct DiscretisationEvidence {
  double sample_time_s = 0.0;  // s
  InputHold hold = InputHold::ZeroOrder;

  // From the block-matrix exponential this discretisation is built on, carried
  // through so the discrete model's accuracy is traceable to the routine that
  // set it rather than asserted here.
  double exponential_one_norm = 0.0;     // dimensionless
  int exponential_pade_order = 0;        // dimensionless
  int exponential_squarings = 0;         // dimensionless
  double exponential_error_bound = 0.0;  // dimensionless, backward, declared

  // max |lambda| over the DISCRETE A. Strictly inside 1 is a stable sampled
  // model; this is reported and never gated on, for the same reason the
  // continuous routines report rather than judge.
  double spectral_radius = 0.0;  // dimensionless
  // max |lambda| over the CONTINUOUS A that was handed in, and the Nyquist
  // angular frequency of the chosen sample time. Reported as two raw numbers
  // rather than a ratio with an implied threshold; see the aliasing note above.
  double fastest_mode_rad_s = 0.0;  // rad/s
  double nyquist_rad_s = 0.0;       // rad/s

  std::vector<std::complex<double>> discrete_eigenvalues;  // dimensionless
  // The one sentence a report quotes about what was assumed.
  std::string assumptions;
};

struct Discretisation {
  DiscreteLinearSystem system;
  DiscretisationEvidence evidence;
};

// Exact zero-order-hold discretisation, by the exponential of the block matrix
// [[A, B], [0, 0]] * T. State and input names, description, citation and units
// are carried across unchanged; C and D are carried across because the output
// at a tick is the continuous output at that instant.
//
// Refuses: a system that does not validate; a sample time that is not positive
// and finite; a system with no inputs, for which a hold means nothing.
[[nodiscard]] Discretisation discretize_zoh(const LinearSystem& system, double sample_time_s);

}  // namespace galata::model

#endif  // GALATA_MODEL_DISCRETE_SYSTEM_HPP
