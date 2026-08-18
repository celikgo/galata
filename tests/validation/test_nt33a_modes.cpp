// SPDX-License-Identifier: Apache-2.0
//
// VALIDATION: modal decomposition and classification against the PUBLISHED
// modal characteristics of a real aircraft.
//
// Reference:
//   Robert K. Heffley and Wayne F. Jewell, "Aircraft Handling Qualities Data",
//   NASA CR-2144, Systems Technology Inc., December 1972.
//   NTRS 19730003312, https://ntrs.nasa.gov/citations/19730003312
//
// Aircraft and condition: NT-33A, flight condition 1 — sea level, M = 0.204,
// power approach. Derivatives, geometry and published modal results are
// transcribed in reference/nt33a_fc1.csv, with the transcription method and
// the rights position in that file's header.
//
// WHAT THIS VALIDATES, precisely. galata's contribution here is
// analyze_modes(): the eigenvalue decomposition, the modal metrics, the
// participation factors and the classification. The state matrix is assembled
// from the report's OWN equations — Appendix C pages C-1 and C-3 — because that
// is the only way to compare against the report's own answers.
//
// The assembly itself lives in tools/validation/nt33a_hand_assembly.cpp rather
// than here, because docs/VERIFICATION.md quotes its results and a second copy
// would be a second answer.
//
// A STRONGER TEST EXISTS. test_nt33a_trim_linearize.cpp reaches the same
// published numbers from the NON-dimensional derivatives, by building a
// nonlinear aircraft, trimming it and linearising. Everything below is the
// weaker of the two comparisons and is kept because the difference between them
// is itself informative — see PhugoidDampingDiscrepancyDoesNotGrow.
//
// HOW THE COMPARISON IS BOUNDED. Both sides are rounded numbers: the report
// prints its derivatives to three significant figures and its modal results to
// three. Rather than pick a tolerance that happens to pass, this test MEASURES
// the sensitivity — each input is perturbed by half a unit in its own last
// printed digit, the resulting spread is accumulated, and the published value's
// own half-unit rounding is added. The gate is that the disagreement falls
// inside that band.

#include "galata/analyze/modes.hpp"

#include "nt33a_hand_assembly.hpp"
#include "reference_table.hpp"
#include "validation_config.hpp"
#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace {

using galata::analyze::analyze_modes;
using galata::analyze::ModeLabel;
using galata::analyze::StateRoles;
using galata::validation::HandAssemblyInputs;

// Every quantity in this reference set is printed to three significant figures.
constexpr int kPrintedFigures = 3;

// Half a unit in the last of `figures` significant digits.
double half_ulp(double value, int figures) {
  if (value == 0.0) {
    return 0.0;
  }
  return 0.5 * std::pow(10.0, std::floor(std::log10(std::fabs(value))) - (figures - 1));
}

// The steps are taken on the values AS PRINTED, which is why the shared
// assembly takes its inputs in the report's own units rather than in SI. An
// earlier version of this test converted first and took the ULP afterwards,
// which understated the uncertainty of every per-foot derivative by the
// conversion factor of 3.28.
HandAssemblyInputs half_ulp_steps(const HandAssemblyInputs& inputs) {
  HandAssemblyInputs steps;
  const auto knobs =
      std::vector<double HandAssemblyInputs::*>{&HandAssemblyInputs::x_u_star,
                                                &HandAssemblyInputs::z_u_star,
                                                &HandAssemblyInputs::m_u_star,
                                                &HandAssemblyInputs::x_w,
                                                &HandAssemblyInputs::z_w,
                                                &HandAssemblyInputs::m_w,
                                                &HandAssemblyInputs::m_w_dot,
                                                &HandAssemblyInputs::m_q,
                                                &HandAssemblyInputs::y_v,
                                                &HandAssemblyInputs::l_beta_prime,
                                                &HandAssemblyInputs::n_beta_prime,
                                                &HandAssemblyInputs::l_p_prime,
                                                &HandAssemblyInputs::n_p_prime,
                                                &HandAssemblyInputs::l_r_prime,
                                                &HandAssemblyInputs::n_r_prime,
                                                &HandAssemblyInputs::true_airspeed_ft_s,
                                                &HandAssemblyInputs::trim_alpha_deg};
  for (double HandAssemblyInputs::* knob : knobs) {
    steps.*knob = half_ulp(inputs.*knob, kPrintedFigures);
  }
  return steps;
}

const std::vector<double HandAssemblyInputs::*>& lateral_knobs() {
  static const std::vector<double HandAssemblyInputs::*> knobs = {
      &HandAssemblyInputs::y_v,
      &HandAssemblyInputs::l_beta_prime,
      &HandAssemblyInputs::n_beta_prime,
      &HandAssemblyInputs::l_p_prime,
      &HandAssemblyInputs::n_p_prime,
      &HandAssemblyInputs::l_r_prime,
      &HandAssemblyInputs::n_r_prime,
      &HandAssemblyInputs::true_airspeed_ft_s,
      &HandAssemblyInputs::trim_alpha_deg};
  return knobs;
}

const std::vector<double HandAssemblyInputs::*>& longitudinal_knobs() {
  static const std::vector<double HandAssemblyInputs::*> knobs = {
      &HandAssemblyInputs::x_u_star,
      &HandAssemblyInputs::z_u_star,
      &HandAssemblyInputs::m_u_star,
      &HandAssemblyInputs::x_w,
      &HandAssemblyInputs::z_w,
      &HandAssemblyInputs::m_w,
      &HandAssemblyInputs::m_w_dot,
      &HandAssemblyInputs::m_q,
      &HandAssemblyInputs::true_airspeed_ft_s,
      &HandAssemblyInputs::trim_alpha_deg};
  return knobs;
}

// The four lateral and four longitudinal quantities the report publishes.
struct Modes {
  double spiral_root = 0.0;
  double roll_root = 0.0;
  double dutch_omega = 0.0;
  double dutch_zeta = 0.0;
  double phugoid_omega = 0.0;
  double phugoid_zeta = 0.0;
  double short_period_omega = 0.0;
  double short_period_zeta = 0.0;
  bool lateral_complete = false;
  bool longitudinal_complete = false;
};

Modes lateral_modes(const HandAssemblyInputs& inputs) {
  const auto names = galata::validation::lateral_state_names();
  const auto result = analyze_modes(
      galata::validation::hand_assembled_lateral(inputs), names, StateRoles::from_names(names));
  Modes modes;
  const auto* spiral = result.find(ModeLabel::Spiral);
  const auto* roll = result.find(ModeLabel::RollSubsidence);
  const auto* dutch = result.find(ModeLabel::DutchRoll);
  if (spiral == nullptr || roll == nullptr || dutch == nullptr) {
    return modes;
  }
  // The report tabulates 1/T, so a stable root of -0.0318 prints as +0.0318.
  // Appendix A fixes this: "(s + 1/T_x)_i = 1/T_x_i".
  modes.spiral_root = -spiral->eigenvalue.real();
  modes.roll_root = -roll->eigenvalue.real();
  modes.dutch_omega = dutch->natural_frequency_rad_s;
  modes.dutch_zeta = dutch->damping_ratio;
  modes.lateral_complete = true;
  return modes;
}

Modes longitudinal_modes(const HandAssemblyInputs& inputs) {
  const auto names = galata::validation::longitudinal_state_names();
  const auto result = analyze_modes(galata::validation::hand_assembled_longitudinal(inputs),
                                    names,
                                    StateRoles::from_names(names));
  Modes modes;
  const auto* phugoid = result.find(ModeLabel::Phugoid);
  const auto* short_period = result.find(ModeLabel::ShortPeriod);
  if (phugoid == nullptr || short_period == nullptr) {
    return modes;
  }
  modes.phugoid_omega = phugoid->natural_frequency_rad_s;
  modes.phugoid_zeta = phugoid->damping_ratio;
  modes.short_period_omega = short_period->natural_frequency_rad_s;
  modes.short_period_zeta = short_period->damping_ratio;
  modes.longitudinal_complete = true;
  return modes;
}

class Nt33aHandAssembled : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    const auto table =
        galata::testing::load_reference(GALATA_VALIDATION_REFERENCE_DIR, "nt33a_fc1.csv");
    citation_ = new std::string(table.citation);
    published_ = new std::map<std::string, double>(table.as_lookup("quantity", "value"));
    inputs_ = new HandAssemblyInputs(HandAssemblyInputs::from_published(*published_));
    steps_ = new HandAssemblyInputs(half_ulp_steps(*inputs_));
  }

  static void TearDownTestSuite() {
    delete citation_;
    delete published_;
    delete inputs_;
    delete steps_;
    citation_ = nullptr;
    published_ = nullptr;
    inputs_ = nullptr;
    steps_ = nullptr;
  }

  // Worst-case spread in each modal quantity when every input is moved by half
  // a unit in its own last printed digit, one at a time, and the deviations are
  // summed. First-order and deliberately conservative.
  static Modes precision_band(const std::vector<double HandAssemblyInputs::*>& knobs,
                              Modes (*evaluate)(const HandAssemblyInputs&)) {
    const Modes nominal = evaluate(*inputs_);
    Modes band;
    const auto accumulate = [](double& into, double high, double low, double reference) {
      into += std::fmax(std::fabs(high - reference), std::fabs(low - reference));
    };
    for (double HandAssemblyInputs::* knob : knobs) {
      HandAssemblyInputs high = *inputs_;
      HandAssemblyInputs low = *inputs_;
      high.*knob += steps_->*knob;
      low.*knob -= steps_->*knob;
      const Modes a = evaluate(high);
      const Modes b = evaluate(low);
      accumulate(band.spiral_root, a.spiral_root, b.spiral_root, nominal.spiral_root);
      accumulate(band.roll_root, a.roll_root, b.roll_root, nominal.roll_root);
      accumulate(band.dutch_omega, a.dutch_omega, b.dutch_omega, nominal.dutch_omega);
      accumulate(band.dutch_zeta, a.dutch_zeta, b.dutch_zeta, nominal.dutch_zeta);
      accumulate(band.phugoid_omega, a.phugoid_omega, b.phugoid_omega, nominal.phugoid_omega);
      accumulate(band.phugoid_zeta, a.phugoid_zeta, b.phugoid_zeta, nominal.phugoid_zeta);
      accumulate(band.short_period_omega,
                 a.short_period_omega,
                 b.short_period_omega,
                 nominal.short_period_omega);
      accumulate(band.short_period_zeta,
                 a.short_period_zeta,
                 b.short_period_zeta,
                 nominal.short_period_zeta);
    }
    return band;
  }

  static std::string* citation_;
  static std::map<std::string, double>* published_;
  static HandAssemblyInputs* inputs_;
  static HandAssemblyInputs* steps_;
};

std::string* Nt33aHandAssembled::citation_ = nullptr;
std::map<std::string, double>* Nt33aHandAssembled::published_ = nullptr;
HandAssemblyInputs* Nt33aHandAssembled::inputs_ = nullptr;
HandAssemblyInputs* Nt33aHandAssembled::steps_ = nullptr;

TEST_F(Nt33aHandAssembled, ReferenceDataIsCitedAndComplete) {
  ASSERT_NE(citation_, nullptr);
  EXPECT_NE(citation_->find("NASA CR-2144"), std::string::npos);
  EXPECT_NE(citation_->find("19730003312"), std::string::npos);
  EXPECT_NE(citation_->find("Unclassified - Unlimited"), std::string::npos);
  ASSERT_NE(published_, nullptr);
  EXPECT_GE(published_->size(), 70U);
}

TEST_F(Nt33aHandAssembled, AllThreeLateralModesAreFoundAndCorrectlyLabelled) {
  const auto names = galata::validation::lateral_state_names();
  const auto result = analyze_modes(
      galata::validation::hand_assembled_lateral(*inputs_), names, StateRoles::from_names(names));

  ASSERT_TRUE(result.participation_is_meaningful)
      << "eigenvector condition number " << result.eigenvector_condition_number;
  EXPECT_LT(result.eigenvector_condition_number, 100.0)
      << "a well-behaved conventional aeroplane should have well-separated modes";
  ASSERT_EQ(result.modes.size(), 3U) << "the Dutch roll pair counts once";

  const auto* spiral = result.find(ModeLabel::Spiral);
  const auto* roll = result.find(ModeLabel::RollSubsidence);
  const auto* dutch = result.find(ModeLabel::DutchRoll);
  ASSERT_NE(spiral, nullptr);
  ASSERT_NE(roll, nullptr);
  ASSERT_NE(dutch, nullptr);

  // The labels must rest on real evidence, not on being the last one left.
  EXPECT_GT(spiral->label_score, 0.5) << spiral->label_reason;
  EXPECT_GT(roll->label_score, 0.5) << roll->label_reason;
  EXPECT_GT(dutch->label_score, 0.5) << dutch->label_reason;

  // Physical character, independent of the published numbers.
  EXPECT_TRUE(dutch->is_oscillatory);
  EXPECT_FALSE(roll->is_oscillatory);
  EXPECT_FALSE(spiral->is_oscillatory);
  EXPECT_LT(dutch->damping_ratio, 0.2);
  EXPECT_GT(roll->natural_frequency_rad_s, 10.0 * spiral->natural_frequency_rad_s);
}

TEST_F(Nt33aHandAssembled, BothLongitudinalModesAreFoundAndCorrectlyLabelled) {
  const auto names = galata::validation::longitudinal_state_names();
  const auto result = analyze_modes(galata::validation::hand_assembled_longitudinal(*inputs_),
                                    names,
                                    StateRoles::from_names(names));
  ASSERT_TRUE(result.participation_is_meaningful);
  ASSERT_EQ(result.modes.size(), 2U);

  const auto* phugoid = result.find(ModeLabel::Phugoid);
  const auto* short_period = result.find(ModeLabel::ShortPeriod);
  ASSERT_NE(phugoid, nullptr);
  ASSERT_NE(short_period, nullptr);
  EXPECT_GT(phugoid->label_score, 0.8) << phugoid->label_reason;
  EXPECT_GT(short_period->label_score, 0.8) << short_period->label_reason;

  EXPECT_LT(phugoid->natural_frequency_rad_s, short_period->natural_frequency_rad_s);
  EXPECT_LT(phugoid->damping_ratio, short_period->damping_ratio);
  EXPECT_GT(phugoid->period_s, 5.0 * short_period->period_s);
}

TEST_F(Nt33aHandAssembled, LateralModesMatchThePublishedValuesWithinTheSourcesOwnPrecision) {
  const Modes computed = lateral_modes(*inputs_);
  ASSERT_TRUE(computed.lateral_complete);
  const Modes band = precision_band(lateral_knobs(), &lateral_modes);

  struct Case {
    const char* name;
    double computed;
    double published;
    double input_band;
  };

  const Case cases[] = {
      {"spiral root", computed.spiral_root, published_->at("spiral_root"), band.spiral_root},
      {"roll subsidence root",
       computed.roll_root,
       published_->at("roll_subsidence_root"),
       band.roll_root},
      {"Dutch roll omega_n",
       computed.dutch_omega,
       published_->at("dutch_roll_natural_frequency"),
       band.dutch_omega},
      {"Dutch roll zeta",
       computed.dutch_zeta,
       published_->at("dutch_roll_damping_ratio"),
       band.dutch_zeta},
  };
  for (const Case& c : cases) {
    // Both sides are rounded: the inputs the computation used, and the answer
    // the report printed. The allowance is the sum.
    const double tolerance = c.input_band + half_ulp(c.published, kPrintedFigures);
    EXPECT_LE(std::fabs(c.computed - c.published), tolerance)
        << c.name << ": computed " << c.computed << ", published " << c.published
        << ", input-precision band " << c.input_band;
  }
}

TEST_F(Nt33aHandAssembled, LongitudinalModesMatchThePublishedValuesWithinTheSourcesOwnPrecision) {
  const Modes computed = longitudinal_modes(*inputs_);
  ASSERT_TRUE(computed.longitudinal_complete);
  const Modes band = precision_band(longitudinal_knobs(), &longitudinal_modes);

  // The phugoid DAMPING RATIO is deliberately absent. It does not reproduce
  // within the source's precision, and the next test records that rather than
  // hiding it by widening a tolerance here.
  struct Case {
    const char* name;
    double computed;
    double published;
    double input_band;
  };

  const Case cases[] = {
      {"phugoid omega_n",
       computed.phugoid_omega,
       published_->at("phugoid_natural_frequency"),
       band.phugoid_omega},
      {"short period omega_n",
       computed.short_period_omega,
       published_->at("short_period_natural_frequency"),
       band.short_period_omega},
      {"short period zeta",
       computed.short_period_zeta,
       published_->at("short_period_damping_ratio"),
       band.short_period_zeta},
  };
  for (const Case& c : cases) {
    const double tolerance = c.input_band + half_ulp(c.published, kPrintedFigures);
    EXPECT_LE(std::fabs(c.computed - c.published), tolerance)
        << c.name << ": computed " << c.computed << ", published " << c.published
        << ", input-precision band " << c.input_band;
  }
}

TEST_F(Nt33aHandAssembled, TheDutchRollPeriodAgreesWithThePublishedPeriod) {
  // An independent published cross-check: the report tabulates the Dutch roll
  // PERIOD separately, in Table II-10, derived from the same roots.
  const auto names = galata::validation::lateral_state_names();
  const auto result = analyze_modes(
      galata::validation::hand_assembled_lateral(*inputs_), names, StateRoles::from_names(names));
  const auto* dutch = result.find(ModeLabel::DutchRoll);
  ASSERT_NE(dutch, nullptr);
  const double published_period = published_->at("dutch_roll_period");
  EXPECT_NEAR(dutch->period_s, published_period, 0.01 * published_period);
}

TEST_F(Nt33aHandAssembled, TheSpiralIsConvergentAsThePublishedSignImplies) {
  // Appendix A: "(s + 1/T_x)_i = 1/T_x_i", so the printed +0.0318 is the factor
  // (s + 0.0318) and the root is NEGATIVE — a convergent spiral.
  const Modes computed = lateral_modes(*inputs_);
  ASSERT_TRUE(computed.lateral_complete);
  EXPECT_GT(computed.spiral_root, 0.0);
  EXPECT_GT(published_->at("spiral_root"), 0.0);
}

// REGRESSION LOCK, not a validation. Anchored to the validated case in
// test_nt33a_trim_linearize.cpp.
//
// The phugoid damping ratio does NOT reproduce within the source's own printed
// precision from this hand assembly, and this test exists to say so precisely
// and to stop the gap growing unnoticed. It is labelled a regression lock
// because its bound comes from galata's current output, not from a published
// value — which is exactly the thing charter rule 8 forbids doing silently.
//
// WHAT DISAGREES. The published phugoid is zeta = 0.0948, omega_n = 0.172, an
// eigenvalue of -0.016306 +/- 0.171226j. This assembly gives approximately
// -0.015974 +/- 0.171170j: the imaginary part agrees to 6e-5, the real part to
// 3.3e-4, and the RATIO disagrees by 2% because zeta = |Re|/|lambda| divides a
// small real part by a small natural frequency.
//
// SINCE RESOLVED, in the sense that matters. The full chain — nonlinear model,
// trim, linearise — reaches the published number to 0.1%. The two routes share
// no arithmetic beyond the source data, so the discrepancy is in THIS
// assembly. Which term it omits is still not established; the leading
// candidate is the derivatives the report leaves blank for this aircraft,
// which this assembly reads as zeros.
TEST_F(Nt33aHandAssembled, PhugoidDampingDiscrepancyDoesNotGrow) {
  const Modes computed = longitudinal_modes(*inputs_);
  ASSERT_TRUE(computed.longitudinal_complete);

  const double published = published_->at("phugoid_damping_ratio");
  const double disagreement = std::fabs(computed.phugoid_zeta - published);

  // Measured at 0.00193 when this lock was written. Bounded on BOTH sides: a
  // change that makes the agreement worse fails, and so does one that makes it
  // better — because that would mean the discrepancy has been explained and
  // this lock should be replaced by a validation.
  EXPECT_GT(disagreement, 0.0015)
      << "the phugoid damping now agrees better than when this lock was written. That is good "
         "news: work out why, and replace this lock with a validation.";
  EXPECT_LT(disagreement, 0.0025) << "the phugoid damping disagreement has grown to "
                                  << disagreement;

  // The eigenvalue itself does agree closely, which is the point. If this
  // stopped being true the problem would be somewhere else entirely.
  const double published_real = -published * published_->at("phugoid_natural_frequency");
  const double computed_real = -computed.phugoid_zeta * computed.phugoid_omega;
  EXPECT_LT(std::fabs(computed_real - published_real), 5e-4)
      << "the discrepancy is no longer confined to the derived damping ratio";
}

TEST_F(Nt33aHandAssembled, TheDownwashLagTermMatters) {
  // M_wdot is small and it is tempting to drop it. This shows what that costs:
  // removing it moves the short period by more than the published value's own
  // precision, so a model that dropped it would fail the comparison above.
  HandAssemblyInputs without = *inputs_;
  without.m_w_dot = 0.0;

  const Modes with_term = longitudinal_modes(*inputs_);
  const Modes without_term = longitudinal_modes(without);
  ASSERT_TRUE(with_term.longitudinal_complete);
  ASSERT_TRUE(without_term.longitudinal_complete);

  const double shift = std::fabs(with_term.short_period_zeta - without_term.short_period_zeta);
  EXPECT_GT(shift, half_ulp(published_->at("short_period_damping_ratio"), kPrintedFigures))
      << "dropping M_wdot moved the short-period damping by only " << shift
      << ", which would make this test vacuous";
}

}  // namespace
