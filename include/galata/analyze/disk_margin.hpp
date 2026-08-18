// SPDX-License-Identifier: Apache-2.0
//
// Disk margins: robustness to SIMULTANEOUS gain and phase variation.
//
// Reference (and the source of every formula below, transcribed from it rather
// than recalled):
//   P. Seiler, A. Packard and P. Gahinet, "An Introduction to Disk Margins",
//   IEEE Control Systems Magazine, vol. 40, no. 5, pp. 78-95, October 2020,
//   doi:10.1109/MCS.2020.3005277. Read as the author preprint
//   arXiv:2003.04771v2, https://arxiv.org/abs/2003.04771 (the IEEE version is
//   paywalled). Equation labels below are the preprint's LaTeX labels.
//
// WHY THIS EXISTS ALONGSIDE THE CLASSICAL MARGINS
//
// Gain margin is the tolerable gain change with NO phase change; phase margin
// is the tolerable phase change with NO gain change. A real loop never varies
// one alone. A loop can show a comfortable gain margin and a comfortable phase
// margin and still be destabilised by a small change in both together, and
// nothing in the classical pair will have hinted at it.
//
// THE MODEL (eq:Fe)
//
// Gain and phase variation is a multiplicative factor f on the loop, drawn
// from a disk parameterised by a size alpha and a skew sigma:
//
//   D(alpha, sigma) = { (1 + ((1-sigma)/2) d) / (1 - ((1+sigma)/2) d)
//                       : d complex, |d| < alpha }
//
// THE MARGIN (Theorem, eq:alphadm)
//
// For a given skew, with the nominal closed loop well-posed and stable, the
// disk margin is the largest alpha keeping it stable for every f in the disk:
//
//   alpha_max = 1 / || S + (sigma - 1)/2 ||_inf,     S = 1 / (1 + L)
//
// where ||G||_inf is the peak of |G(jw)| over w. Three skews have names:
//   sigma =  0   the SYMMETRIC or balanced disk margin, ||(S-T)/2||_inf^-1,
//                where gain increase and decrease are treated alike;
//   sigma = -1   the T-based margin, ||T||_inf^-1;
//   sigma = +1   the S-based margin, ||S||_inf^-1.
//
// WHAT IT GUARANTEES (eq:galphamax, eq:cosphim)
//
// The disk's real-axis intercepts bound gain-only variation, and its
// intersection with the unit circle bounds phase-only variation:
//
//   gamma_min = (2 - alpha(1-sigma)) / (2 + alpha(1+sigma))
//   gamma_max = (2 + alpha(1-sigma)) / (2 - alpha(1+sigma))
//   cos(phi_m) = (1 + gamma_min gamma_max) / (gamma_min + gamma_max)
//
// These are LOWER estimates of the classical margins, not equal to them: the
// disk is inscribed in the stable region, so it concedes some gain-only and
// phase-only room in exchange for covering the combinations between.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * The peak is found by SEARCHING A GRID and then refining, NOT by the exact
//   Hamiltonian-eigenvalue method (Boyd & Balakrishnan; Bruinsma & Steinbuch).
//   A grid maximum is a LOWER bound on the true peak, so the disk margin
//   reported here is an UPPER bound on the true one — the error is in the
//   OPTIMISTIC direction. The grid is refined around the closed-loop system's
//   own lightly damped modes, which is where such peaks are, but a peak
//   narrower than the refined spacing would still be understated. Treat a
//   marginal result as marginal.
//
// * `destabilising_delta` is NOT comparable with MATLAB's delta. The paper
//   bounds |delta| < alpha; MATLAB's diskmargin bounds |delta| < 1 and moves
//   alpha inside the factor instead. The two describe the SAME disk, related by
//   delta_here = alpha * delta_matlab, but the numbers differ. alpha, the gain
//   range and the phase range agree between the two; delta does not.
//
// * Not a MIMO disk margin. This is the SISO condition. The multi-loop case
//   needs a structured singular value, which galata does not have.
//
// * The theorem ASSUMES the nominal closed loop is stable. Applied to an
//   already-unstable loop the formula still returns a number, and that number
//   means nothing. The LinearSystem overload checks and refuses; the evaluator
//   overload cannot check and does not.
//
// * The peak at w = infinity is not evaluated as a limit — the top of the
//   frequency grid stands in for it. Sweep well past the loop bandwidth.

#ifndef GALATA_ANALYZE_DISK_MARGIN_HPP
#define GALATA_ANALYZE_DISK_MARGIN_HPP

#include "galata/analyze/margins.hpp"
#include "galata/model/linear_system.hpp"

#include <complex>

namespace galata::analyze {

struct DiskMargin {
  double skew;  // sigma

  // The margin itself, and the peak it came from.
  double alpha;
  double peak_gain;  // || S + (sigma-1)/2 ||_inf, as found on the grid
  double critical_frequency_rad_s;

  // Guaranteed gain-only variation, as a multiplicative factor either side of
  // one. gain_variation_max is infinite, and gain_variation_is_bounded false,
  // when alpha >= 2/|1+sigma| and the disk stops being a bounded disk.
  double gain_variation_min;
  double gain_variation_max;
  bool gain_variation_is_bounded;
  double gain_variation_min_db;
  double gain_variation_max_db;

  // Guaranteed phase-only variation, +/- this many degrees. Infinite, and
  // phase_variation_is_bounded false, when the disk swallows the unit circle
  // and no phase variation destabilises.
  double phase_variation_deg;
  bool phase_variation_is_bounded;

  // The construction from the theorem's proof: a perturbation ON the boundary
  // that destabilises, placing a closed-loop pole exactly at j*critical
  // frequency. This is what makes the margin checkable rather than merely
  // computed — see the test that closes the loop with it.
  //   delta_0 = 1 / (S(jw_0) + (sigma-1)/2)
  //   f_0     = (2 + (1-sigma) delta_0) / (2 - (1+sigma) delta_0)
  std::complex<double> destabilising_delta;
  std::complex<double> destabilising_perturbation;

  double searched_from_rad_s;
  double searched_to_rad_s;
  int grid_points;
};

// Throws std::invalid_argument if the skew is not finite.
[[nodiscard]] DiskMargin disk_margin(const LoopEvaluator& loop, double skew = 0.0,
                                     const MarginOptions& options = {});

// Refines the search grid around the CLOSED-loop modes, which is where the
// peak of S sits, and refuses a loop whose nominal closed loop is unstable.
[[nodiscard]] DiskMargin disk_margin(const model::LinearSystem& loop, int input_index,
                                     int output_index, double skew = 0.0,
                                     const MarginOptions& options = {});

}  // namespace galata::analyze

#endif  // GALATA_ANALYZE_DISK_MARGIN_HPP
