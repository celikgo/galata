// SPDX-License-Identifier: Apache-2.0
//
// VALIDATION: disk margin and classical margins against the PUBLISHED worked
// example of the tutorial that defines the disk margin.
//
// Reference:
//   P. Seiler, A. Packard and P. Gahinet, "An Introduction to Disk Margins
//   [Lecture Notes]", IEEE Control Systems Magazine, vol. 40, no. 5,
//   pp. 78-95, October 2020, doi:10.1109/MCS.2020.3005277. Consulted as the
//   authors' preprint arXiv:2003.04771v2.
//
// The loop, stated by the paper in one line:
//   L(s) = 25 / (s^3 + 10 s^2 + 10 s + 10)
//
// Reference values, their exact location in the document and the rights
// position are in reference/seiler2020_disk_margin.csv. See ADR-0007 for why
// values from a copyrighted paper may be committed here at all, and what the
// limits on that are.
//
// TWO KINDS OF CHECK, and the second matters more than the first:
//
//   * Against the PUBLISHED NUMBERS, bounded by each value's own printed
//     precision. This is the comparison against a stranger's document.
//
//   * Against CLOSED FORMS derived here, which do not depend on the paper at
//     all: this loop's phase crossover is exactly sqrt(10) and its gain margin
//     is exactly 3.6, because Im L(jw) = 0 requires 10w - w^3 = 0 and
//     |L(j sqrt10)| = 25/90. A published number and an exact one agreeing is
//     considerably better evidence than either alone.
//
//   * And against the PROPERTY the theorem asserts: the perturbation the
//     construction returns must actually destabilise, placing a closed-loop
//     pole exactly on the imaginary axis at the critical frequency. That check
//     cannot be passed by a formula transcribed wrongly but consistently.

#include "galata/analyze/disk_margin.hpp"
#include "galata/analyze/frequency_response.hpp"
#include "galata/analyze/margins.hpp"
#include "galata/units.hpp"

#include "reference_table.hpp"
#include "validation_config.hpp"
#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <map>
#include <string>

namespace {

// The core reports angles in radians (ADR-0003). Tests state their
// expectations in degrees, as the literature does, and convert here — the
// same boundary conversion the report writers make.
double degrees(double radians) {
  return galata::units::radians_to_degrees(radians);
}

using galata::analyze::disk_margin;
using galata::analyze::LoopEvaluator;
using galata::analyze::MarginOptions;
using galata::analyze::stability_margins;
using galata::testing::load_reference;
using galata::testing::printed_precision_tolerance;
using galata::testing::ReferenceTable;

// L(s) = 25 / (s^3 + 10 s^2 + 10 s + 10), written out rather than realised in
// state space so that the reference loop is the paper's, not a translation of
// it that could differ.
std::complex<double> published_loop(double frequency_rad_s) {
  const std::complex<double> s{0.0, frequency_rad_s};
  return 25.0 / (s * s * s + 10.0 * s * s + 10.0 * s + 10.0);
}

struct Published {
  double value;
  int significant_figures;
  double units_in_last_place;
  std::string location;

  [[nodiscard]] double tolerance() const {
    return printed_precision_tolerance(value, significant_figures, units_in_last_place);
  }
};

std::map<std::string, Published> published_values() {
  const ReferenceTable table =
      load_reference(GALATA_VALIDATION_REFERENCE_DIR, "seiler2020_disk_margin.csv");
  std::map<std::string, Published> values;
  for (std::size_t row = 0; row < table.rows.size(); ++row) {
    Published entry{};
    entry.value = table.at(row, "value");
    entry.significant_figures = static_cast<int>(table.at(row, "significant_figures"));
    entry.units_in_last_place = table.at(row, "units_in_last_place");
    entry.location = table.text(row, "location");
    values.emplace(table.text(row, "quantity"), entry);
  }
  return values;
}

MarginOptions wide_sweep() {
  MarginOptions options;
  options.start_rad_s = 1.0e-2;
  options.stop_rad_s = 1.0e3;
  options.grid_points = 4000;
  return options;
}

#define EXPECT_MATCHES_PUBLISHED(computed, entry, name)                                          \
  EXPECT_NEAR((computed), (entry).value, (entry).tolerance())                                    \
      << name << ": published " << (entry).value << " at " << (entry).location << ", allowance " \
      << (entry).tolerance() << " (" << (entry).units_in_last_place                              \
      << " units in the last printed figure)"

TEST(DiskMarginSeiler2020, ClassicalMarginsMatchThePublishedValuesAndTheClosedForm) {
  const auto published = published_values();
  const auto margins = stability_margins(LoopEvaluator{published_loop}, wide_sweep());

  ASSERT_TRUE(margins.has_gain_margin);
  ASSERT_TRUE(margins.has_phase_margin);

  EXPECT_MATCHES_PUBLISHED(
      margins.gain_margin, published.at("classical_gain_margin"), "gain margin");
  EXPECT_MATCHES_PUBLISHED(
      degrees(margins.phase_margin_rad), published.at("classical_phase_margin"), "phase margin");

  // Independent of the paper: Im L(jw) = 0 requires 10 w - w^3 = 0, so the
  // phase crossover is exactly sqrt(10); there L(jw) = 25 / -90, so the gain
  // margin is exactly 90/25 = 3.6. The published 3.6 is that number, and both
  // agreeing to twelve digits is stronger than agreeing to two.
  EXPECT_NEAR(margins.gain_margin_frequency_rad_s, std::sqrt(10.0), 1.0e-11);
  EXPECT_NEAR(margins.gain_margin, 90.0 / 25.0, 1.0e-11);
}

TEST(DiskMarginSeiler2020, SymmetricDiskMarginMatchesThePublishedValues) {
  const auto published = published_values();
  const auto margin = disk_margin(LoopEvaluator{published_loop}, 0.0, wide_sweep());

  EXPECT_MATCHES_PUBLISHED(
      margin.peak_gain, published.at("peak_shifted_sensitivity"), "peak of |S - 1/2|");
  EXPECT_MATCHES_PUBLISHED(margin.alpha, published.at("alpha_max"), "alpha_max");
  EXPECT_MATCHES_PUBLISHED(margin.gain_variation_min, published.at("gamma_min"), "gamma_min");
  EXPECT_MATCHES_PUBLISHED(margin.gain_variation_max, published.at("gamma_max"), "gamma_max");
  EXPECT_MATCHES_PUBLISHED(
      margin.destabilising_delta.real(), published.at("delta_0_real"), "Re delta_0");
  EXPECT_MATCHES_PUBLISHED(
      margin.destabilising_delta.imag(), published.at("delta_0_imag"), "Im delta_0");
  EXPECT_MATCHES_PUBLISHED(
      margin.destabilising_perturbation.real(), published.at("f_0_real"), "Re f_0");
  EXPECT_MATCHES_PUBLISHED(
      margin.destabilising_perturbation.imag(), published.at("f_0_imag"), "Im f_0");
  EXPECT_MATCHES_PUBLISHED(
      margin.critical_frequency_rad_s, published.at("critical_frequency"), "critical frequency");
}

TEST(DiskMarginSeiler2020, PublishedCriticalFrequencyDisagreesWithItsOwnPublishedPerturbation) {
  // The one genuine discrepancy with the source, isolated so that it is a
  // recorded finding rather than a loose tolerance hidden in the test above.
  //
  // The paper prints omega_0 = 1.94 rad/s. Its OWN printed delta_0 and f_0 are
  // evaluated at omega_0, and neither is reproduced there: at 1.94 the
  // construction gives delta_0 = 0.1955 - 0.4148j against a printed
  // 0.212 - 0.406j, a disagreement of 33 units in the last printed figure.
  // Both are reproduced near 1.955 rad/s, which is where |S - 1/2| actually
  // peaks. The printed critical frequency is therefore inconsistent with the
  // rest of the example, and galata's 1.955 is the self-consistent value.
  //
  // This test asserts the inconsistency, so that if a future reading of the
  // paper resolves it, this test fails and the finding gets revisited.
  const auto published = published_values();
  const double printed_frequency = published.at("critical_frequency").value;

  const auto delta_at = [](double frequency) {
    const std::complex<double> sensitivity = 1.0 / (1.0 + published_loop(frequency));
    return 1.0 / (sensitivity - 0.5);
  };

  const std::complex<double> at_printed = delta_at(printed_frequency);
  const double published_real = published.at("delta_0_real").value;
  const double half_unit_in_last_figure = 0.0005;

  // At the printed frequency the printed delta_0 is not reproduced.
  EXPECT_GT(std::abs(at_printed.real() - published_real), 20.0 * half_unit_in_last_figure)
      << "the paper's printed omega_0 now reproduces its printed delta_0; the recorded "
         "discrepancy no longer holds and docs/VERIFICATION.md must be revisited";

  // At the frequency galata finds, it is — to within the paper's own rounding.
  const auto margin = disk_margin(LoopEvaluator{published_loop}, 0.0, wide_sweep());
  const std::complex<double> at_computed = delta_at(margin.critical_frequency_rad_s);
  EXPECT_LT(std::abs(at_computed.real() - published_real), 3.0 * half_unit_in_last_figure);
  EXPECT_GT(margin.critical_frequency_rad_s, printed_frequency);
}

TEST(DiskMarginSeiler2020, TheConstructedPerturbationActuallyDestabilisesTheLoop) {
  // The theorem's proof constructs a perturbation f_0 on the boundary of the
  // disk that puts a closed-loop pole on the imaginary axis at omega_0. That is
  // a checkable claim about the loop, not a restatement of the formula: it
  // holds only if alpha, omega_0, delta_0 and f_0 are ALL right together.
  //
  // A pole of the perturbed closed loop at s = j omega_0 means 1 + f_0 L(j
  // omega_0) = 0 exactly.
  const auto margin = disk_margin(LoopEvaluator{published_loop}, 0.0, wide_sweep());
  const std::complex<double> perturbed =
      1.0 + margin.destabilising_perturbation * published_loop(margin.critical_frequency_rad_s);

  EXPECT_NEAR(std::abs(perturbed), 0.0, 1.0e-9)
      << "the constructed perturbation should place a closed-loop pole exactly at j*"
      << margin.critical_frequency_rad_s << ", but 1 + f_0 L = " << perturbed;

  // And it must genuinely be on the boundary of the disk: |delta_0| = alpha.
  EXPECT_NEAR(std::abs(margin.destabilising_delta), margin.alpha, 1.0e-12);
}

TEST(DiskMarginSeiler2020, DiskMarginIsMoreConservativeThanTheClassicalPair) {
  // The tutorial's central point, stated as a test: the disk margin's
  // guaranteed gain and phase variation are SMALLER than the classical
  // margins, because the disk covers the combinations between them.
  //
  // If this ever inverts, either the disk margin is being computed as
  // something other than the largest inscribed disk, or the classical margins
  // are wrong.
  const auto margin = disk_margin(LoopEvaluator{published_loop}, 0.0, wide_sweep());
  const auto classical = stability_margins(LoopEvaluator{published_loop}, wide_sweep());

  ASSERT_TRUE(margin.gain_variation_is_bounded);
  ASSERT_TRUE(margin.phase_variation_is_bounded);
  ASSERT_TRUE(classical.has_gain_margin);
  ASSERT_TRUE(classical.has_phase_margin);

  EXPECT_LT(margin.gain_variation_max, classical.gain_margin)
      << "guaranteed gain increase " << margin.gain_variation_max
      << " should be below the classical gain margin " << classical.gain_margin;
  // Compared in radians on both sides: an inequality needs no conversion, and
  // converting one side only is exactly the mistake ADR-0003 exists to stop.
  EXPECT_LT(margin.phase_variation_rad, classical.phase_margin_rad)
      << "guaranteed phase variation " << degrees(margin.phase_variation_rad)
      << " deg should be below the classical phase margin " << degrees(classical.phase_margin_rad)
      << " deg";
}

TEST(DiskMarginSeiler2020, NamedSkewsReduceToTheirPublishedSpecialCases) {
  // The theorem's named special cases, which are a transcription check on
  // eq:alphadm: sigma = +1 must give ||S||_inf^-1 and sigma = -1 must give
  // ||T||_inf^-1. Computing those peaks directly and comparing catches a sign
  // error in (sigma-1)/2 that the sigma = 0 case alone would not.
  const auto options = wide_sweep();
  const auto grid = galata::analyze::logarithmic_grid(
      options.start_rad_s, options.stop_rad_s, options.grid_points);

  double peak_sensitivity = 0.0;
  double peak_complementary = 0.0;
  for (const double frequency : grid) {
    const std::complex<double> l = published_loop(frequency);
    const std::complex<double> sensitivity = 1.0 / (1.0 + l);
    peak_sensitivity = std::max(peak_sensitivity, std::abs(sensitivity));
    peak_complementary = std::max(peak_complementary, std::abs(1.0 - sensitivity));
  }

  const auto s_based = disk_margin(LoopEvaluator{published_loop}, 1.0, options);
  const auto t_based = disk_margin(LoopEvaluator{published_loop}, -1.0, options);

  // The grid peaks above are un-refined, so they are slightly below the
  // refined ones the disk margin uses; compare to the grid's own resolution.
  EXPECT_NEAR(s_based.alpha, 1.0 / peak_sensitivity, 1.0e-3 * s_based.alpha);
  EXPECT_NEAR(t_based.alpha, 1.0 / peak_complementary, 1.0e-3 * t_based.alpha);

  // And the symmetric case sits between the two one-sided ones for this loop.
  const auto symmetric = disk_margin(LoopEvaluator{published_loop}, 0.0, options);
  EXPECT_GT(symmetric.alpha, 0.0);
  EXPECT_LT(symmetric.alpha, std::max(s_based.alpha, t_based.alpha));
}

TEST(DiskMarginSeiler2020, GainInterceptsFollowTheDiskParameterisationAtEverySkew) {
  // The gate the sigma = 0 comparison CANNOT provide.
  //
  // At sigma = 0 the factors (1 - sigma) and (1 + sigma) are both 1, so the
  // published example says nothing about which is which. Swapping them in
  // gamma_min leaves every sigma = 0 result identical — verified by mutating
  // the implementation and watching the whole published comparison still pass.
  //
  // The intercepts are, by definition, the disk of eq:Fe evaluated at
  // delta = -alpha and delta = +alpha:
  //
  //   f(delta) = (1 + ((1-sigma)/2) delta) / (1 - ((1+sigma)/2) delta)
  //
  // which is an independent statement of the same geometry and is sensitive to
  // the skew.
  const auto options = wide_sweep();
  for (const double skew : {-1.0, -0.5, 0.0, 0.2, 0.5, 1.0}) {
    const auto margin = disk_margin(LoopEvaluator{published_loop}, skew, options);
    ASSERT_TRUE(margin.gain_variation_is_bounded) << "skew " << skew;

    const auto disk_factor = [skew](double delta) {
      return (1.0 + (1.0 - skew) / 2.0 * delta) / (1.0 - (1.0 + skew) / 2.0 * delta);
    };
    EXPECT_NEAR(margin.gain_variation_min, disk_factor(-margin.alpha), 1.0e-12)
        << "gamma_min at skew " << skew;
    EXPECT_NEAR(margin.gain_variation_max, disk_factor(margin.alpha), 1.0e-12)
        << "gamma_max at skew " << skew;
  }
}

TEST(DiskMarginSeiler2020, NamedSkewsHaveThePublishedInterceptClosedForms) {
  // The paper states the intercepts for the two named one-sided margins:
  //   sigma = -1 (T-based): "gamma_min = 1-alpha and gamma_max = 1+alpha"
  //   sigma = +1 (S-based): "gamma_min = (1+alpha)^-1 and gamma_max = (1-alpha)^-1"
  // Both are asymmetric in the skew and so pin down the formula's orientation.
  const auto options = wide_sweep();

  const auto t_based = disk_margin(LoopEvaluator{published_loop}, -1.0, options);
  EXPECT_NEAR(t_based.gain_variation_min, 1.0 - t_based.alpha, 1.0e-12);
  EXPECT_NEAR(t_based.gain_variation_max, 1.0 + t_based.alpha, 1.0e-12);

  const auto s_based = disk_margin(LoopEvaluator{published_loop}, 1.0, options);
  EXPECT_NEAR(s_based.gain_variation_min, 1.0 / (1.0 + s_based.alpha), 1.0e-12);
  EXPECT_NEAR(s_based.gain_variation_max, 1.0 / (1.0 - s_based.alpha), 1.0e-12);

  // And the published property of the balanced case: "For sigma=0, the nominal
  // factor f=1 is the geometric mean of the range", i.e. gamma_max = 1/gamma_min.
  const auto symmetric = disk_margin(LoopEvaluator{published_loop}, 0.0, options);
  EXPECT_NEAR(symmetric.gain_variation_max, 1.0 / symmetric.gain_variation_min, 1.0e-12);
}

TEST(DiskMarginSeiler2020, AgreesWithASecondImplementationIncludingThePhaseMargin) {
  // Cross-check against MathWorks' published `diskmargin` output for the same
  // loop at the same skew — a second implementation, not a second reading of
  // the same document.
  //
  // This exists mainly for phi_m. The paper derives the formula for the
  // guaranteed phase variation but never prints a number for this example, so
  // without a second source galata's phi_m would be gated against nothing at
  // all. See the reference file header for why vendor documentation is used
  // here and how it is labelled.
  const auto published = published_values();
  const auto margin = disk_margin(LoopEvaluator{published_loop}, 0.0, wide_sweep());

  EXPECT_MATCHES_PUBLISHED(margin.alpha, published.at("mathworks_alpha"), "alpha_max");
  EXPECT_MATCHES_PUBLISHED(
      margin.gain_variation_min, published.at("mathworks_gamma_min"), "gamma_min");
  EXPECT_MATCHES_PUBLISHED(
      margin.gain_variation_max, published.at("mathworks_gamma_max"), "gamma_max");

  ASSERT_TRUE(margin.phase_variation_is_bounded);
  EXPECT_MATCHES_PUBLISHED(
      degrees(margin.phase_variation_rad), published.at("mathworks_phase_margin"), "phi_m");

  // The two sources must also agree with each other, or one of them is being
  // read wrongly and this test would be averaging the error.
  EXPECT_NEAR(published.at("mathworks_alpha").value,
              published.at("alpha_max").value,
              published.at("alpha_max").tolerance());
  EXPECT_NEAR(published.at("mathworks_gamma_min").value,
              published.at("gamma_min").value,
              published.at("gamma_min").tolerance());
}

}  // namespace
