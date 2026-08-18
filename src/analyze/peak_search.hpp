// SPDX-License-Identifier: Apache-2.0
//
// Locating the maximum of a scalar function of frequency on a grid.
//
// PRIVATE to src/analyze. Not installed, not part of the public API.
//
// Shared by every quantity in this library whose headline value is a peak over
// frequency — the disk margin, the sensitivity peak, the complementary
// sensitivity peak, the largest singular value. They must all locate a peak the
// same way, or two of galata's own numbers will disagree about the same maximum
// for no reason a user could discover.
//
// WHAT THIS IS NOT
// * Not an H-infinity norm. A grid maximum is a LOWER bound on the true
//   supremum: a peak narrower than the grid spacing is understated. The exact
//   computation is the Hamiltonian-eigenvalue method (Boyd & Balakrishnan 1990;
//   Bruinsma & Steinbuch 1990), which galata does not have. Every caller must
//   say so where its result is reported, because for a robustness margin the
//   error is in the OPTIMISTIC direction.
// * Not a global optimiser. It refines the single largest grid sample. A second,
//   slightly lower peak elsewhere is not reported.

#ifndef GALATA_SRC_ANALYZE_PEAK_SEARCH_HPP
#define GALATA_SRC_ANALYZE_PEAK_SEARCH_HPP

#include <functional>
#include <vector>

namespace galata::analyze::detail {

struct Peak {
  double frequency_rad_s;
  double value;
};

// The largest sample on `grid`, refined by golden section within the bracket
// its neighbours provide.
//
// `refinement_iterations` is FIXED, not a convergence criterion (ADR-0004):
// stopping on a tolerance would make the result depend on the order in which
// rounding happened, and therefore on the platform.
//
// The returned value is never smaller than the largest sample actually seen. A
// refinement that somehow did worse is discarded rather than reported, because
// a function called "find the peak" must not return less than a value it
// already evaluated.
[[nodiscard]] Peak find_peak(const std::function<double(double)>& f,
                             const std::vector<double>& grid,
                             int refinement_iterations);

}  // namespace galata::analyze::detail

#endif  // GALATA_SRC_ANALYZE_PEAK_SEARCH_HPP
