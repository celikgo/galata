// SPDX-License-Identifier: Apache-2.0
// Boyd, Balakrishnan and Kabamba (1989), MCSS 2:207-219; Boyd and
// Balakrishnan (1990), SCL 15:1-7; Bruinsma and Steinbuch (1990), SCL 14:287-293.
// Hamiltonian level-set bracketing of the continuous-time H-infinity norm.
// WHAT THIS IS NOT: no descriptor, discrete-time, unstable or uncertain plant;
// no interval arithmetic or mathematically certified floating-point enclosure.
// Bounds use the analytic characterization with numerical separation/residual
// checks. Poorly conditioned cases fail instead of returning a plausible norm.
#ifndef GALATA_ANALYZE_HINFINITY_HPP
#define GALATA_ANALYZE_HINFINITY_HPP

#include "galata/model/linear_system.hpp"

#include <string>

namespace galata::analyze {
struct HinfinityOptions {
  double relative_tolerance = 1e-6;   // dimensionless, final interval width / upper bound
  double absolute_tolerance = 1e-12;  // output / input, final absolute interval width
  int bracket_expansions = 32;        // fixed budget, dimensionless
  int bisection_iterations = 48;      // fixed budget, dimensionless
};

struct HinfinityNorm {
  double lower_bound = 0.0;                  // output / input
  double upper_bound = 0.0;                  // output / input
  double dc_gain = 0.0;                      // output / input, at zero rad/s
  double feedthrough_gain = 0.0;             // output / input, at infinite frequency
  double relative_gap = 0.0;                 // dimensionless
  double worst_eigenvector_condition = 0.0;  // dimensionless, accepted upper tests
  double smallest_axis_separation = 0.0;     // 1/s, accepted upper tests
  int hamiltonian_evaluations = 0;           // count
  bool numerically_reliable = false;         // numerical checks passed, not an interval proof
  bool tolerance_met = false;                // requested final bracket width achieved
  std::string diagnostic;
};

// Requires an internally Hurwitz A, even if an unstable pole would cancel in
// the transfer. Limits dense models to 128 states. Includes both DC and D.
// Throws for invalid inputs, unresolved upper bounds or numerical conditioning.
// The shared internal-stability check can conservatively refuse a defective
// or poorly scaled stable realization before beginning the norm search.
[[nodiscard]] HinfinityNorm hinfinity_norm(const model::LinearSystem& system,
                                           const HinfinityOptions& options = {});

struct SensitivityNormBounds {
  HinfinityNorm sensitivity;    // S=(I+L)^-1, dimensionless for compatible channel scaling
  HinfinityNorm complementary;  // T=I-S, dimensionless for compatible channel scaling
  bool internally_stable = false;
  std::string diagnostic;
};

// Negative identity feedback, with input/output order and scaling supplied by
// the caller. Square loops only; an ill-posed I+D or non-Hurwitz internal
// closed-loop realization is refused, including hidden unstable modes.
[[nodiscard]] SensitivityNormBounds sensitivity_norm_bounds(const model::LinearSystem& loop,
                                                            const HinfinityOptions& options = {});

struct DiskMarginBounds {
  double skew = 0.0;         // dimensionless, Seiler/Packard/Gahinet disk skew
  double alpha_lower = 0.0;  // dimensionless; infinite for an identically zero shifted sensitivity
  double alpha_upper = 0.0;  // dimensionless, may be infinite when norm lower bound is zero
  HinfinityNorm shifted_sensitivity;
  bool internally_stable = false;
  std::string diagnostic;
};

// SISO only: alpha=1/||S+(skew-1)/2||inf (Seiler, Packard & Gahinet, 2020).
// The lower alpha bound uses the upper norm bound, so missed grid peaks cannot
// inflate it. Finite-precision reliability limits of hinfinity_norm still apply.
[[nodiscard]] DiskMarginBounds disk_margin_bounds(const model::LinearSystem& loop,
                                                  double skew = 0.0,
                                                  const HinfinityOptions& options = {});
}  // namespace galata::analyze
#endif
