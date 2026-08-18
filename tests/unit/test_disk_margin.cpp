// SPDX-License-Identifier: Apache-2.0
//
// Disk margin: the behaviour of the state-space entry point, and the cases
// where a number must NOT be returned.
//
// The comparison against published values is in the validation tier
// (tests/validation/test_disk_margin_seiler.cpp). What is here is what that
// comparison cannot reach: the closed-loop stability precondition, the
// degenerate disks, and the equivalence of the state-space and evaluator paths.

#include "galata/analyze/disk_margin.hpp"

#include "galata/analyze/frequency_response.hpp"
#include "galata/analyze/margins.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <limits>

namespace {

using galata::analyze::disk_margin;
using galata::analyze::LoopEvaluator;
using galata::analyze::MarginOptions;
using galata::model::LinearSystem;

// 1 / (s (s+1) (s+2)) as a companion realisation, scaled by `gain`.
LinearSystem chain(double gain) {
  LinearSystem system;
  system.a = Eigen::MatrixXd::Zero(3, 3);
  system.a << 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -2.0, -3.0;
  system.b = Eigen::MatrixXd::Zero(3, 1);
  system.b(2, 0) = 1.0;
  system.c = Eigen::MatrixXd::Zero(1, 3);
  system.c(0, 0) = gain;
  system.state_names = {"x0", "x1", "x2"};
  system.input_names = {"u"};
  system.output_names = {"y"};
  return system;
}

TEST(DiskMargin, StateSpaceAndEvaluatorPathsAgree) {
  const LinearSystem loop = chain(1.0);
  MarginOptions options;
  options.frequencies = galata::analyze::logarithmic_grid(1.0e-2, 1.0e2, 3000);

  const auto from_system = disk_margin(loop, 0, 0, 0.0, options);
  const LoopEvaluator evaluator = [](double w) {
    const std::complex<double> s{0.0, w};
    return 1.0 / (s * (s + 1.0) * (s + 2.0));
  };
  const auto from_evaluator = disk_margin(evaluator, 0.0, options);

  // The MARGIN agrees to near machine precision. This is the quantity the two
  // paths are supposed to produce identically, and it does not depend on
  // locating the peak precisely — only on its height.
  EXPECT_NEAR(from_system.alpha, from_evaluator.alpha, 1.0e-12 * from_evaluator.alpha);
  EXPECT_NEAR(from_system.gain_variation_min, from_evaluator.gain_variation_min, 1.0e-12);
  EXPECT_NEAR(from_system.gain_variation_max, from_evaluator.gain_variation_max, 1.0e-12);

  // The FREQUENCY cannot agree that closely, and demanding that it should
  // would be asserting something false about maxima. Near a smooth peak
  // g(w) = g(w0) - k (w - w0)^2, so a relative disturbance eps in the
  // evaluated gain displaces the located peak by order sqrt(eps/k): the
  // position is square-root conditioned relative to the height. Two evaluation
  // paths differing at the rounding level therefore locate the peak to about
  // sqrt(eps) ~ 1.5e-8 relative, and that is the honest bound.
  const double relative_frequency_difference =
      std::abs(from_system.critical_frequency_rad_s - from_evaluator.critical_frequency_rad_s)
      / from_evaluator.critical_frequency_rad_s;
  EXPECT_LT(relative_frequency_difference, std::sqrt(std::numeric_limits<double>::epsilon()))
      << "relative peak-position disagreement " << relative_frequency_difference;
}

TEST(DiskMargin, ConstructedPerturbationDestabilisesWhateverTheSkew) {
  // The theorem's construction is what makes the result checkable, so it is
  // checked at every skew rather than only at the balanced one.
  const LinearSystem loop = chain(1.0);
  MarginOptions options;
  options.frequencies = galata::analyze::logarithmic_grid(1.0e-2, 1.0e2, 4000);

  for (const double skew : {-1.0, -0.4, 0.0, 0.4, 1.0}) {
    const auto margin = disk_margin(loop, 0, 0, skew, options);
    const double w = margin.critical_frequency_rad_s;
    const std::complex<double> s{0.0, w};
    const std::complex<double> l = 1.0 / (s * (s + 1.0) * (s + 2.0));

    EXPECT_NEAR(std::abs(1.0 + margin.destabilising_perturbation * l), 0.0, 1.0e-9)
        << "skew " << skew << ": the boundary perturbation must put a pole at j" << w;
    EXPECT_NEAR(std::abs(margin.destabilising_delta), margin.alpha, 1.0e-11)
        << "skew " << skew << ": delta_0 must lie ON the boundary |delta| = alpha";
  }
}

TEST(DiskMargin, RefusesALoopWhoseNominalClosedLoopIsUnstable) {
  // The theorem assumes nominal stability. 1/(s(s+1)(s+2)) closes stable at
  // unit gain and unstable at 12 (its gain margin is 6), so the same loop with
  // two gains exercises both sides of the precondition.
  EXPECT_NO_THROW((void)disk_margin(chain(1.0), 0, 0));
  EXPECT_THROW((void)disk_margin(chain(12.0), 0, 0), std::invalid_argument);

  // The message has to say what is wrong, or a user will read the refusal as a
  // bug in galata rather than as a fact about their loop.
  try {
    (void)disk_margin(chain(12.0), 0, 0);
    FAIL() << "expected a refusal";
  } catch (const std::invalid_argument& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("nominal closed loop"), std::string::npos) << message;
    EXPECT_NE(message.find("unstable"), std::string::npos) << message;
  }
}

TEST(DiskMargin, RefusesAnIllPosedLoop) {
  // Direct feedthrough of -1 means 1 + L is zero at infinite frequency: the
  // closed loop has no solution there and no margin can be reported.
  LinearSystem loop = chain(1.0);
  loop.d = Eigen::MatrixXd::Constant(1, 1, -1.0);
  EXPECT_THROW((void)disk_margin(loop, 0, 0), std::invalid_argument);
}

TEST(DiskMargin, RefusesWhatItCannotCompute) {
  const LoopEvaluator evaluator = [](double w) {
    const std::complex<double> s{0.0, w};
    return 1.0 / (s + 1.0);
  };
  EXPECT_THROW((void)disk_margin(LoopEvaluator{}), std::invalid_argument);
  EXPECT_THROW((void)disk_margin(evaluator, std::nan("")), std::invalid_argument);

  MarginOptions no_refinement;
  no_refinement.peak_refinement_iterations = 0;
  EXPECT_THROW((void)disk_margin(evaluator, 0.0, no_refinement), std::invalid_argument);

  MarginOptions single_point;
  single_point.frequencies = {1.0};
  EXPECT_THROW((void)disk_margin(evaluator, 0.0, single_point), std::invalid_argument);
}

TEST(DiskMargin, AMoreDampedLoopHasALargerMargin) {
  // A monotonicity property rather than a value: pulling the third pole of
  // 1/(s(s+1)(s+p)) further left makes the loop more robust, so alpha must
  // increase. Nothing about the formula forces this, so it is evidence the
  // formula describes robustness and not merely arithmetic.
  double previous = 0.0;
  for (const double pole : {2.0, 4.0, 8.0, 16.0}) {
    LinearSystem loop = chain(1.0);
    loop.a(2, 1) = -pole;
    loop.a(2, 2) = -(1.0 + pole);
    const auto margin = disk_margin(loop, 0, 0);
    EXPECT_GT(margin.alpha, previous) << "third pole at -" << pole;
    previous = margin.alpha;
  }
}

TEST(DiskMargin, ReportsTheSearchedBandSoAMissedPeakIsVisible) {
  const LinearSystem loop = chain(1.0);
  MarginOptions options;
  options.start_rad_s = 0.1;
  options.stop_rad_s = 10.0;
  options.grid_points = 500;
  const auto margin = disk_margin(loop, 0, 0, 0.0, options);

  EXPECT_GE(margin.searched_from_rad_s, 0.1);
  EXPECT_LE(margin.searched_to_rad_s, 10.0);
  EXPECT_GE(margin.grid_points, 500);
  EXPECT_GT(margin.critical_frequency_rad_s, margin.searched_from_rad_s);
  EXPECT_LT(margin.critical_frequency_rad_s, margin.searched_to_rad_s);
}

}  // namespace
