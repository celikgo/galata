// SPDX-License-Identifier: Apache-2.0
//
// Singular values of a transfer matrix over frequency — the principal gains.
//
// References:
//   S. Skogestad and I. Postlethwaite, "Multivariable Feedback Control:
//   Analysis and Design", 2nd ed., Wiley, 2005 — singular values as the gains
//   of a MIMO system, and the directions that achieve them.
//   G. H. Golub and C. F. Van Loan, "Matrix Computations", 4th ed., Johns
//   Hopkins University Press, 2013, chapter 2 — the singular value
//   decomposition.
//
// WHY A MIMO SYSTEM NEEDS THIS AND A BODE MAGNITUDE PLOT WILL NOT DO
//
// A SISO system has one gain at each frequency. A MIMO system has a RANGE of
// gains at each frequency, because the gain depends on the DIRECTION of the
// input:
//
//   sigma_min(G) <= ||G u|| / ||u|| <= sigma_max(G)
//
// The singular values are exactly that range. Plotting the individual
// element-by-element transfer functions of a MIMO system instead is not a
// weaker analysis, it is a misleading one: every element can look small while
// sigma_max is large, because the elements can reinforce along one input
// direction.
//
// The spread between sigma_max and sigma_min — the condition number — says how
// directional the system is. A large condition number means the plant responds
// strongly to some input directions and barely at all to others, which is what
// makes a plant hard to control regardless of the controller.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * The peak reported here is a GRID MAXIMUM, and therefore a LOWER bound on
//   the true H-infinity norm. It is not computed by the exact
//   Hamiltonian-eigenvalue method (Boyd & Balakrishnan 1990; Bruinsma &
//   Steinbuch 1990). A peak narrower than the grid spacing is understated, so
//   `peak_gain` must be read as "at least this much", and the searched band and
//   point count are reported so that can be judged.
//
// * Singular values are NOT eigenvalues, and for a non-normal system they can
//   be wildly different. A system whose eigenvalues are all small can still
//   have a large sigma_max. Do not read a singular value plot as a pole plot.
//
// * Singular values carry no phase. Two systems with identical singular values
//   at every frequency can have completely different closed-loop behaviour.
//   There is no MIMO equivalent of the Bode phase plot here.
//
// * The values depend on the SCALING of the inputs and outputs. Comparing
//   sigma_max between two models scaled differently compares the scalings.
//   ADR-0003's strict SI helps but does not make a metre and a radian
//   commensurate; scale deliberately before reading a condition number.

#ifndef GALATA_ANALYZE_SINGULAR_VALUES_HPP
#define GALATA_ANALYZE_SINGULAR_VALUES_HPP

#include "galata/analyze/frequency_response.hpp"
#include "galata/model/linear_system.hpp"

#include <string>
#include <vector>

namespace galata::analyze {

struct SingularValueResponse {
  std::vector<double> frequencies_rad_s;
  // One vector per frequency, in DESCENDING order. Its length is
  // min(outputs, inputs) at every frequency.
  std::vector<std::vector<double>> singular_values;

  std::vector<std::string> input_names;
  std::vector<std::string> output_names;

  // The largest sigma_max over the grid, and where. See "WHAT THIS IS NOT" on
  // why this is a lower bound on the H-infinity norm rather than equal to it.
  double peak_gain;
  double peak_frequency_rad_s;

  double searched_from_rad_s;
  double searched_to_rad_s;
  int grid_points;

  [[nodiscard]] std::size_t channel_count() const;

  [[nodiscard]] std::vector<double> largest() const;
  [[nodiscard]] std::vector<double> smallest() const;
  // sigma_max / sigma_min at each frequency. Infinite where the system is rank
  // deficient at that frequency, which is a real property of the system and not
  // an error: it means there is an input direction the system does not respond
  // to at all.
  [[nodiscard]] std::vector<double> condition_number() const;
};

// Singular values of G(jw) = C (jwI - A)^-1 B + D.
[[nodiscard]] SingularValueResponse singular_values(const model::LinearSystem& system,
                                                    const std::vector<double>& frequencies_rad_s);

// The same, from a frequency response already computed. Free of any further
// solves, and guarantees the two views describe the same numbers.
[[nodiscard]] SingularValueResponse singular_values(const FrequencyResponse& response);

}  // namespace galata::analyze

#endif  // GALATA_ANALYZE_SINGULAR_VALUES_HPP
