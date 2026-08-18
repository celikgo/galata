// SPDX-License-Identifier: Apache-2.0
//
// Sensitivity S and complementary sensitivity T, and their peaks.
//
// References:
//   S. Skogestad and I. Postlethwaite, "Multivariable Feedback Control:
//   Analysis and Design", 2nd ed., Wiley, 2005 — S, T, and the peaks M_S and
//   M_T as robustness measures.
//   J. C. Doyle, B. A. Francis and A. R. Tannenbaum, "Feedback Control
//   Theory", Macmillan, 1992.
//
// For a loop L closed with NEGATIVE unit feedback:
//
//   S = (I + L)^-1        T = (I + L)^-1 L        S + T = I
//
// S maps reference to error and disturbance to output; T maps reference to
// output and measurement noise to output. S + T = I is why control design is a
// trade-off rather than an optimisation: both cannot be small at the same
// frequency, because they sum to one.
//
// WHY THE PEAKS ARE THE HEADLINE
//
// M_S = max over w of sigma_max(S(jw)) is the reciprocal of the shortest
// distance from the Nyquist curve of L to the critical point -1. That makes it
// a SINGLE number covering gain and phase together, where the classical
// margins each hold the other fixed — the same objection the disk margin
// answers, arrived at from the other direction. The two agree exactly: the
// disk margin at skew +1 is 1/M_S, and at skew -1 is 1/M_T (Seiler, Packard &
// Gahinet 2020). galata computes those by two independent routes and a test
// requires them to match.
//
// HOW THIS IS COMPUTED, AND WHY NO MATRIX IS EVER INVERTED
//
//   sigma_max(S) = sigma_max((I+L)^-1) = 1 / sigma_min(I + L)
//
// so the sensitivity is read straight off the singular values of I + L, with
// no inverse and no solve at all. T is obtained by SOLVING (I + L) T = L,
// which is a factorisation and a back-substitution rather than an inversion.
// Near the sensitivity peak I + L is close to singular — that is precisely
// what a peak in S means — so this is the one place in the calculation where
// forming an inverse would do visible damage.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * This does NOT form the loop. L is what you pass in, and where you broke
//   the loop determines what S and T mean. Broken at the plant input they are
//   the input sensitivity and input complementary sensitivity, which for a
//   MIMO plant are DIFFERENT matrices from the output ones, with different
//   peaks. galata cannot tell which you meant.
//
// * The loop must be SQUARE. S = (I+L)^-1 is not defined otherwise, and a
//   non-square system is refused rather than silently reduced.
//
// * The peaks are GRID MAXIMA and therefore LOWER bounds on the true
//   H-infinity norms. A peak narrower than the grid spacing is understated.
//   For M_S that error is optimistic — it makes the loop look more robust than
//   it is — so the searched band and point count travel with the result.
//
// * Nothing here establishes closed-loop stability. M_S is a distance from the
//   critical point, not a Nyquist encirclement count. An unstable closed loop
//   has an S with unstable poles, and its peak over a finite grid is a number
//   that means nothing. Check the closed-loop eigenvalues.

#ifndef GALATA_ANALYZE_SENSITIVITY_HPP
#define GALATA_ANALYZE_SENSITIVITY_HPP

#include "galata/analyze/margins.hpp"
#include "galata/model/linear_system.hpp"

#include <vector>

namespace galata::analyze {

struct SensitivityPeaks {
  std::vector<double> frequencies_rad_s;
  // sigma_max at each frequency. For a single-loop system these are |S| and
  // |T|, since a 1x1 matrix has one singular value equal to its magnitude.
  std::vector<double> sensitivity;
  std::vector<double> complementary_sensitivity;

  // M_S and M_T, and the frequencies at which they occur.
  double sensitivity_peak;
  double sensitivity_peak_frequency_rad_s;
  double complementary_peak;
  double complementary_peak_frequency_rad_s;

  double searched_from_rad_s;
  double searched_to_rad_s;
  int grid_points;

  // True when the loop is 1x1. The guaranteed-margin bounds below apply only
  // in that case; see GuaranteedMargins.
  bool is_single_loop;
};

// The classical margins that a given M_S or M_T GUARANTEES.
//
// Skogestad & Postlethwaite, 2nd ed., equations (2.47) and (2.48), printed
// page 36, transcribed verbatim:
//
//   (2.47)   GM >= M_S / (M_S - 1)      PM >= 2 arcsin(1/(2 M_S)) >= 1/M_S  [rad]
//   (2.48)   GM >= 1 + 1/M_T            PM >= 2 arcsin(1/(2 M_T)) >= 1/M_T  [rad]
//
// The "[rad]" is printed on the equations themselves; the book converts to
// degrees only in prose. Note the two gain-margin bounds have DIFFERENT
// functional forms — M_S/(M_S-1) against 1 + 1/M_T — which is easy to blur
// from memory and is why they are transcribed here.
//
// These are LOWER BOUNDS. A loop's actual margins are at least this good and
// are usually better; the value of the bounds is that one number, M_S, implies
// both, so specifying M_S "can make specifications on the GM and PM
// unnecessary" (ibid., p. 37).
//
// THE DERIVATION, so a reader can check these without the book. All four fall
// out of two exact identities in three lines each.
//
//   At the phase crossover w_180, L is real and negative with |L| = 1/GM, so
//   L = -1/GM, and therefore
//
//       S = 1/(1+L) = GM/(GM - 1)        T = L/(1+L) = -1/(GM - 1)
//
//   Since |S| <= M_S everywhere, GM/(GM-1) <= M_S rearranges to
//   GM >= M_S/(M_S - 1). Since |T| <= M_T, 1/(GM-1) <= M_T rearranges to
//   GM >= 1 + 1/M_T. The two bounds differ in form because S and T differ.
//
//   At the gain crossover w_c, |L| = 1, so L and the critical point -1 are two
//   points on the unit circle separated by the angle PM. The chord between
//   them is |1 + L| = 2 sin(PM/2), giving
//
//       |S(j w_c)| = |T(j w_c)| = 1 / (2 sin(PM/2))
//
//   which is the book's (2.50). Bounding that by M_S gives
//   sin(PM/2) >= 1/(2 M_S), i.e. PM >= 2 arcsin(1/(2 M_S)); likewise for M_T.
//
//   A useful sanity check falls out: M_S = 1 gives PM = 60 degrees exactly, so
//   these bounds can never promise more than 60 degrees however good M_S is.
//
// ===========================================================================
// SISO ONLY. This is not a hedge, it is the source's own scope.
// ===========================================================================
// Equations (2.47) and (2.48) appear in Chapter 2, whose stated remit is SISO,
// and the book never restates them for MIMO. It goes further: its spinning
// satellite example shows a plant with excellent gain and phase margins
// "when considering one loop at a time" that is destabilised by small
// SIMULTANEOUS input gain errors. Applying these bounds per-channel to a
// multi-loop system would reproduce exactly that error, so `applies` is false
// for a loop that is not 1x1 and the bounds are not computed.
struct GuaranteedMargins {
  // False for a MIMO loop — see above. Nothing else in this struct is
  // meaningful when this is false.
  bool applies;
  // False when the peaks put the bounds outside their domain: M_S <= 1 makes
  // M_S/(M_S - 1) divergent or negative, and a peak below 0.5 puts
  // 1/(2 M) outside the domain of arcsin.
  bool valid;

  double gain_margin_from_sensitivity;         // (2.47)
  double gain_margin_from_complementary;       // (2.48)
  double phase_margin_from_sensitivity_rad;    // (2.47)
  double phase_margin_from_complementary_rad;  // (2.48)
};

[[nodiscard]] GuaranteedMargins guaranteed_margins(const SensitivityPeaks& peaks);

// S and T peaks for an open-loop system closed with negative unit feedback.
//
// Throws std::invalid_argument if the loop is not square, if I + D is singular
// (an ill-posed loop), or if the nominal closed loop is unstable — in which
// case a peak over a finite grid would be a number without meaning.
[[nodiscard]] SensitivityPeaks sensitivity_peaks(const model::LinearSystem& loop,
                                                 const MarginOptions& options = {});

}  // namespace galata::analyze

#endif  // GALATA_ANALYZE_SENSITIVITY_HPP
