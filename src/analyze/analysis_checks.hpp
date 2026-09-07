// SPDX-License-Identifier: Apache-2.0
// Private analysis preconditions; not an installed interface.
#ifndef GALATA_SRC_ANALYZE_ANALYSIS_CHECKS_HPP
#define GALATA_SRC_ANALYZE_ANALYSIS_CHECKS_HPP

#include "galata/analyze/margins.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace galata::analyze::detail {
enum class HurwitzStatus { Stable, NonStable, Boundary, Unresolved };

struct HurwitzAssessment {
  HurwitzStatus status = HurwitzStatus::Unresolved;
  std::string diagnostic;
};

// A numerical sufficient check, not a proof with directed rounding. Defective
// or poorly scaled stable realizations may be refused rather than misclassified.
[[nodiscard]] HurwitzAssessment assess_hurwitz(const Eigen::MatrixXd& a);
void require_hurwitz(const Eigen::MatrixXd& a, const char* context);

[[nodiscard]] std::complex<double> checked_loop_value(const LoopEvaluator& loop,
                                                      double frequency,
                                                      const char* context);
void require_frequency_grid(const std::vector<double>& grid, const char* context);
}  // namespace galata::analyze::detail
#endif
