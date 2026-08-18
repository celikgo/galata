// SPDX-License-Identifier: Apache-2.0
//
// Frequency response of a linear system: G(jw) = C (jwI - A)^-1 B + D.
//
// Reference:
//   A. J. Laub, "Efficient multivariable frequency response computations",
//   IEEE Transactions on Automatic Control, vol. 26, no. 2, pp. 407-408, 1981
//   — the Hessenberg method used here.
//   G. H. Golub and C. F. Van Loan, "Matrix Computations", 4th ed., Johns
//   Hopkins University Press, 2013, chapters 5 and 7 — Hessenberg reduction and
//   Givens rotations.
//
// THE INVERSE IS NEVER FORMED. (jwI - A)^-1 B is obtained by SOLVING
// (jwI - A) X = B. Forming the inverse costs more and is less accurate, and the
// difference is not academic: near a lightly damped pole the matrix is close to
// singular, which is exactly the frequency a control engineer is looking at.
//
// The matrix is reduced to upper Hessenberg form ONCE, before the sweep:
//
//   A = Q H Q^T   =>   G(jw) = (C Q) (jwI - H)^-1 (Q^T B) + D
//
// Q^T B and C Q are precomputed, and each frequency then costs one Hessenberg
// solve — O(n^2) by Givens rotations rather than the O(n^3) a general LU would
// cost. Over a 500-point grid that is the difference between the reduction
// being an optimisation and being the whole cost.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not valid ON a pole. At a frequency where jw is an eigenvalue of A the
//   matrix is exactly singular and the response is infinite. `conditioning`
//   reports how close each frequency came, because a response computed a
//   hair's breadth from an undamped pole is a large number whose digits are
//   noise, and it looks exactly like a large number that means something.
//
// * Not a spectral estimate. This is the response of a MODEL, computed
//   exactly. Nothing here touches measured data, and there is no coherence to
//   report because there is no experiment.
//
// * Not aware of time delays. A pure delay is not a rational transfer
//   function and does not appear in a state-space model. A loop with transport
//   delay must have it approximated (Pade) before it gets here, or the margins
//   computed from this will be optimistic.
//
// * Continuous-time only. There is no z-domain path.

#ifndef GALATA_ANALYZE_FREQUENCY_RESPONSE_HPP
#define GALATA_ANALYZE_FREQUENCY_RESPONSE_HPP

#include "galata/model/linear_system.hpp"

#include <Eigen/Core>

#include <complex>
#include <string>
#include <vector>

namespace galata::analyze {

// Log-spaced frequencies, inclusive of both endpoints.
[[nodiscard]] std::vector<double> logarithmic_grid(double start_rad_s,
                                                   double stop_rad_s,
                                                   int count);

// A log-spaced grid with extra points clustered around every lightly damped
// mode of `a`.
//
// Refinement by construction rather than by adaptive subdivision. The
// resonances of a linear system are known before the sweep starts — they are
// its eigenvalues — so placing points there is both cheaper and deterministic,
// where an adaptive scheme's point count would depend on floating-point
// comparisons and therefore on the platform (ADR-0004).
//
// Modes with damping above `damping_threshold` get no cluster: they have no
// peak worth resolving.
[[nodiscard]] std::vector<double> grid_refined_for_modes(const Eigen::MatrixXd& a,
                                                         double start_rad_s,
                                                         double stop_rad_s,
                                                         int count,
                                                         double damping_threshold = 0.7);

struct FrequencyResponse {
  std::vector<double> frequencies_rad_s;
  // One (outputs x inputs) matrix per frequency.
  std::vector<Eigen::MatrixXcd> response;
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;

  // min|U_ii| / max|U_ii| of the triangular factor at each frequency, in [0, 1].
  // Small means the frequency sits near a pole: the response there is large for
  // a reason, but its trailing digits are not to be believed. This is an
  // INDICATOR, not a condition number — a matrix can be ill-conditioned with a
  // healthy pivot ratio (Kahan). It catches the near-pole case, which is what
  // it is here for, and it is not evidence of anything else.
  std::vector<double> pivot_ratio;

  [[nodiscard]] bool is_single_loop() const;

  // SISO views. Throw unless the response is 1x1.
  [[nodiscard]] std::vector<double> magnitude() const;
  [[nodiscard]] std::vector<double> magnitude_db() const;
  // Unwrapped, in RADIANS, continuous across the sweep.
  //
  // Radians because this is the numerical core and ADR-0003 admits no other
  // angle unit here. Degrees are a presentation choice and are applied at the
  // boundary — in the pipeline's report writers — not in a value a solver may
  // go on to consume.
  //
  // Unwrapped because a wrapped phase makes a margin search find crossings
  // that are not there.
  [[nodiscard]] std::vector<double> phase_rad() const;
};

// G(jw) over the given frequencies.
[[nodiscard]] FrequencyResponse frequency_response(const model::LinearSystem& system,
                                                   const std::vector<double>& frequencies_rad_s);

// The scalar response from one input to one output, which is what a margin
// calculation needs. Names are carried through so a report can say which loop
// it measured.
[[nodiscard]] FrequencyResponse single_loop_response(const model::LinearSystem& system,
                                                     int input_index,
                                                     int output_index,
                                                     const std::vector<double>& frequencies_rad_s);

}  // namespace galata::analyze

#endif  // GALATA_ANALYZE_FREQUENCY_RESPONSE_HPP
