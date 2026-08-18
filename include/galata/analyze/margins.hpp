// SPDX-License-Identifier: Apache-2.0
//
// Stability margins of a single open-loop transfer function L(s).
//
// References:
//   G. F. Franklin, J. D. Powell and A. Emami-Naeini, "Feedback Control of
//   Dynamic Systems", 8th ed., Pearson, 2019 — gain, phase and delay margins.
//   K. J. Astrom and R. M. Murray, "Feedback Systems: An Introduction for
//   Scientists and Engineers", 2nd ed., Princeton University Press, 2021,
//   chapter 10 — the same margins as distances from the critical point.
//
// The definitions used here, stated so that a reader can check the code
// against them rather than against a recollection:
//
//   PHASE CROSSOVER  a frequency where L(jw) is real and negative, i.e. where
//                    the phase is an odd multiple of 180 degrees.
//   GAIN MARGIN      1 / |L(jw_pc)| at a phase crossover: the factor the loop
//                    gain can be multiplied by before the Nyquist plot reaches
//                    -1. Greater than one is the healthy case; LESS than one is
//                    a loop that needs its gain REDUCED, which is why this is
//                    reported as a ratio and not only in decibels.
//   GAIN CROSSOVER   a frequency where |L(jw)| = 1.
//   PHASE MARGIN     180 degrees + arg L(jw_gc) at a gain crossover, reduced
//                    into (-180, 180]: the extra phase lag the loop tolerates.
//   DELAY MARGIN     phase_margin_in_radians / w_gc, in seconds: the smallest
//                    transport delay that consumes the phase margin. A delay
//                    contributes -w*tau of phase, so the same phase margin at a
//                    higher crossover frequency buys LESS time.
//
// ALL crossovers are reported, not just one. A loop whose magnitude crosses
// unity three times has three phase margins, and an implementation that
// returned the first one it found could report a comfortable margin for a loop
// that is fragile at a different frequency. The headline `gain_margin` and
// `phase_margin_deg` are the governing ones — the smallest perturbation of
// either kind that destabilises, which for the gain margin means the crossing
// nearest 0 dB rather than the numerically smallest ratio.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * This does NOT form the loop transfer function. L is what you pass in.
//   Breaking a feedback loop at the plant input and at the plant output give
//   different L, and therefore different margins, for the same closed-loop
//   system; which one you want is a modelling decision and this code cannot
//   make it for you.
//
// * These are SINGLE-LOOP margins. Applied one loop at a time to a multi-loop
//   system they are known to be optimistic: a MIMO loop can have healthy gain
//   and phase margins in every channel and still be destabilised by small
//   simultaneous perturbations in several. That is the failure the disk margin
//   exists to catch.
//
// * Gain and phase margin are separately-varied margins. The gain margin is
//   the tolerable gain change WITH NO phase change, and vice versa. Neither
//   says anything about tolerance to the two together, which is again the disk
//   margin's job.
//
// * Nothing here proves closed-loop stability. Margins are distances from the
//   critical point, not a Nyquist encirclement count; an unstable open loop can
//   show a comfortable-looking gain margin and still close unstable. Check the
//   closed-loop eigenvalues.
//
// * Crossovers are found by SEARCHING A FREQUENCY GRID. A crossover pair
//   narrower than the grid spacing is not found, and the reported range says
//   what was searched so that a missing crossing is not read as an absent one.

#ifndef GALATA_ANALYZE_MARGINS_HPP
#define GALATA_ANALYZE_MARGINS_HPP

#include "galata/model/linear_system.hpp"

#include <complex>
#include <functional>
#include <vector>

namespace galata::analyze {

// Where |L(jw)| = 1.
struct GainCrossing {
  double frequency_rad_s;
  double phase_margin_deg;
  // phase_margin_deg in radians divided by the frequency. Negative when the
  // phase margin is negative: no delay makes such a loop stable, and reporting
  // a positive time there would invent a margin that does not exist.
  double delay_margin_s;
};

// Where L(jw) is real and negative.
struct PhaseCrossing {
  double frequency_rad_s;
  double gain_margin;  // 1 / |L(jw)|, a ratio: < 1 means the gain must come DOWN
  double gain_margin_db;
};

struct StabilityMargins {
  std::vector<GainCrossing> gain_crossings;
  std::vector<PhaseCrossing> phase_crossings;

  // The governing gain margin: the crossing nearest 0 dB, i.e. the smallest
  // multiplicative change in either direction that reaches -1. Infinite, with
  // has_gain_margin false, when the phase never reaches an odd multiple of 180
  // degrees inside the searched range.
  bool has_gain_margin;
  double gain_margin;
  double gain_margin_db;
  double gain_margin_frequency_rad_s;

  // The governing phase margin: the crossing of smallest magnitude. Infinite,
  // with has_phase_margin false, when |L| never reaches unity in range.
  bool has_phase_margin;
  double phase_margin_deg;
  double phase_margin_frequency_rad_s;

  // The smallest transport delay that destabilises: the minimum over gain
  // crossings with a positive phase margin. Absent when there is no such
  // crossing — including when a phase margin exists but is negative, since the
  // loop is then already unstable and no delay can be blamed for it.
  bool has_delay_margin;
  double delay_margin_s;
  double delay_margin_frequency_rad_s;

  // What was actually searched, so that "no crossing" can be read as "none in
  // this band" rather than as "none anywhere".
  double searched_from_rad_s;
  double searched_to_rad_s;
  int grid_points;
};

struct MarginOptions {
  double start_rad_s = 1.0e-3;
  double stop_rad_s = 1.0e3;
  int grid_points = 2000;
  // FIXED, not a convergence criterion (ADR-0004). Eighty bisections shrink a
  // bracket by 2^80, which reaches the floating-point floor from any starting
  // width; running "until converged" would make the answer depend on rounding.
  int bisection_iterations = 80;
  // Golden-section steps used to refine a peak once the grid has bracketed it
  // (disk margins). Fixed for the same reason.
  int peak_refinement_iterations = 100;
  // Frequencies to search. Empty means "build one": the LinearSystem overload
  // refines around the loop's own lightly damped modes, the evaluator overload
  // uses a plain logarithmic grid.
  std::vector<double> frequencies;
};

// L evaluated at jw. Any loop that can be evaluated at a frequency can be
// measured, including one carrying a Pade-approximated delay or a
// non-rational term the state-space path cannot express.
using LoopEvaluator = std::function<std::complex<double>(double frequency_rad_s)>;

[[nodiscard]] StabilityMargins stability_margins(const LoopEvaluator& loop,
                                                 const MarginOptions& options = {});

// The single loop from `input_index` to `output_index` of an open-loop system.
[[nodiscard]] StabilityMargins stability_margins(const model::LinearSystem& loop,
                                                 int input_index, int output_index,
                                                 const MarginOptions& options = {});

}  // namespace galata::analyze

#endif  // GALATA_ANALYZE_MARGINS_HPP
