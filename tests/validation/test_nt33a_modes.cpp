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
// below from the report's OWN equations — Appendix C page C-3 — because that is
// the only way to compare against the report's own answers. So this is a
// validation of the analysis, using a model built to the source's specification;
// it is not yet a validation of a trim-and-linearise chain, which does not
// exist.
//
// HOW THE COMPARISON IS BOUNDED, which is the part worth reading. Both sides of
// this comparison are rounded numbers. The report prints its derivatives to
// three significant figures and its modal results to three significant figures,
// so a computation from those inputs cannot agree with those outputs more
// closely than that rounding allows. Rather than pick a tolerance that happens
// to pass, this test MEASURES the sensitivity: each input is perturbed by half
// a unit in its own last printed digit, the resulting spread in each modal
// quantity is accumulated, and the published value's own half-unit rounding is
// added. The gate is that the disagreement falls inside that band.
//
// That makes the tolerance a property of the source document rather than of
// the author's patience, and it is why the roll-subsidence root — which
// disagrees by 0.0071 against an input-propagated band of 0.0052 — is
// nonetheless a pass: the published 2.20 is itself only known to +/-0.005.

#include "galata/analyze/modes.hpp"
#include "galata/core/constants.hpp"
#include "galata/units.hpp"

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

// Every quantity the lateral-directional matrix needs, in SI. The reference
// file holds the report's own units (feet, per second); conversion happens here
// at the file-format boundary, which is where ADR-0003 permits it.
struct LateralInputs {
  double y_v = 0.0;             // 1/s
  double l_beta_prime = 0.0;    // 1/s^2
  double n_beta_prime = 0.0;    // 1/s^2
  double l_p_prime = 0.0;       // 1/s
  double n_p_prime = 0.0;       // 1/s
  double l_r_prime = 0.0;       // 1/s
  double n_r_prime = 0.0;       // 1/s
  double airspeed_m_s = 0.0;    // m/s
  double trim_alpha_rad = 0.0;  // rad
  double trim_gamma_rad = 0.0;  // rad
};

// The lateral-directional state matrix, exactly as NASA CR-2144 Appendix C
// page C-3 prints it, rearranged from its Laplace matrix form into the
// equivalent state-space form.
//
// The printed matrix equation is
//
//   | s - Y_v   -(W_o s + g cos th_o)/V_To   (U_o s - g sin th_o)/(V_To s) | | beta |
//   | -L_beta'   s(s - L_p')                 -L_r'                         | | p/s  |
//   | -N_beta'  -N_p' s                       s - N_r'                     | | r    |
//
// with the auxiliary relation phi = p/s + (r/s) tan th_o printed beneath it.
// Note that phi is RECOVERED from the solved states there rather than being a
// fourth solved state; expressing it as a state is what turns this into a
// 4x4 A matrix.
//
// Resulting equations, with x = [beta, p, r, phi]:
//
//   beta_dot = Y_v beta + (W_o/V_To) p - (U_o/V_To) r + (g cos th_o / V_To) phi
//   p_dot    = L_beta' beta + L_p' p + L_r' r
//   r_dot    = N_beta' beta + N_p' p + N_r' r
//   phi_dot  = p + r tan th_o
//
// with U_o = V_To cos(alpha_o), W_o = V_To sin(alpha_o), and, for wings-level
// flight with no sideslip, th_o = alpha_o + gamma_o.
//
// Note that W_o/V_To is sin(alpha_o) and U_o/V_To is cos(alpha_o), so the
// matrix is dimensionally identical whether the speeds are in feet per second
// or metres per second — only the ratio g/V_To carries a unit. Converting to SI
// therefore changes no entry, which is a pleasant way to see that the SI
// boundary rule costs nothing here.
Eigen::MatrixXd assemble_lateral(const LateralInputs& in) {
  const double theta_o = in.trim_alpha_rad + in.trim_gamma_rad;
  const double sin_alpha = std::sin(in.trim_alpha_rad);
  const double cos_alpha = std::cos(in.trim_alpha_rad);
  const double gravity_term = galata::core::kStandardGravity * std::cos(theta_o) / in.airspeed_m_s;

  Eigen::MatrixXd a(4, 4);
  a << in.y_v, sin_alpha, -cos_alpha, gravity_term,      //
      in.l_beta_prime, in.l_p_prime, in.l_r_prime, 0.0,  //
      in.n_beta_prime, in.n_p_prime, in.n_r_prime, 0.0,  //
      0.0, 1.0, std::tan(theta_o), 0.0;
  return a;
}

// The four modal quantities the report publishes for the lateral set.
struct LateralModes {
  double spiral_root = 0.0;                   // 1/s, the report's 1/T(DET)1
  double roll_subsidence_root = 0.0;          // 1/s, the report's 1/T(DET)2
  double dutch_roll_natural_frequency = 0.0;  // rad/s
  double dutch_roll_damping_ratio = 0.0;      // dimensionless
  bool complete = false;
};

LateralModes extract(const Eigen::MatrixXd& a) {
  const std::vector<std::string> names = {"beta", "p", "r", "phi"};
  const auto result = analyze_modes(a, names, StateRoles::from_names(names));

  LateralModes modes;
  const auto* spiral = result.find(ModeLabel::Spiral);
  const auto* roll = result.find(ModeLabel::RollSubsidence);
  const auto* dutch = result.find(ModeLabel::DutchRoll);
  if (spiral == nullptr || roll == nullptr || dutch == nullptr) {
    return modes;
  }
  // The report tabulates 1/T, so a stable root of -0.0318 prints as +0.0318.
  // Appendix A fixes this: "(s + 1/T_x)_i = 1/T_x_i".
  modes.spiral_root = -spiral->eigenvalue.real();
  modes.roll_subsidence_root = -roll->eigenvalue.real();
  modes.dutch_roll_natural_frequency = dutch->natural_frequency_rad_s;
  modes.dutch_roll_damping_ratio = dutch->damping_ratio;
  modes.complete = true;
  return modes;
}

// Half a unit in the last of `figures` significant digits.
double half_ulp(double value, int figures) {
  if (value == 0.0) {
    return 0.0;
  }
  return 0.5 * std::pow(10.0, std::floor(std::log10(std::fabs(value))) - (figures - 1));
}

// Every quantity in this reference set is printed to three significant figures.
constexpr int kPrintedFigures = 3;

class Nt33aLateral : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    const auto table =
        galata::testing::load_reference(GALATA_VALIDATION_REFERENCE_DIR, "nt33a_fc1.csv");
    citation_ = new std::string(table.citation);
    const std::map<std::string, double> value = table.as_lookup("quantity", "value");

    inputs_ = new LateralInputs{};
    inputs_->y_v = value.at("Y_v");
    inputs_->l_beta_prime = value.at("L_beta_prime");
    inputs_->n_beta_prime = value.at("N_beta_prime");
    inputs_->l_p_prime = value.at("L_p_prime");
    inputs_->n_p_prime = value.at("N_p_prime");
    inputs_->l_r_prime = value.at("L_r_prime");
    inputs_->n_r_prime = value.at("N_r_prime");
    inputs_->airspeed_m_s = galata::units::feet_to_metres(value.at("true_airspeed"));
    inputs_->trim_alpha_rad = galata::units::degrees_to_radians(value.at("trim_alpha"));
    inputs_->trim_gamma_rad = galata::units::degrees_to_radians(value.at("trim_gamma"));

    published_ = new std::map<std::string, double>(value);
  }

  static void TearDownTestSuite() {
    delete inputs_;
    delete published_;
    delete citation_;
    inputs_ = nullptr;
    published_ = nullptr;
    citation_ = nullptr;
  }

  // Worst-case spread in each modal quantity when every input is moved by half
  // a unit in its own last printed digit, one at a time, and the deviations are
  // summed. First-order and deliberately conservative.
  static LateralModes input_precision_band() {
    const LateralModes nominal = extract(assemble_lateral(*inputs_));

    LateralModes band;
    const std::vector<double LateralInputs::*> knobs = {
        &LateralInputs::y_v,
        &LateralInputs::l_beta_prime,
        &LateralInputs::n_beta_prime,
        &LateralInputs::l_p_prime,
        &LateralInputs::n_p_prime,
        &LateralInputs::l_r_prime,
        &LateralInputs::n_r_prime,
        &LateralInputs::airspeed_m_s,
        &LateralInputs::trim_alpha_rad,
    };
    for (double LateralInputs::* knob : knobs) {
      LateralInputs high = *inputs_;
      LateralInputs low = *inputs_;
      const double step = half_ulp(inputs_->*knob, kPrintedFigures);
      high.*knob += step;
      low.*knob -= step;

      const LateralModes a = extract(assemble_lateral(high));
      const LateralModes b = extract(assemble_lateral(low));
      band.spiral_root += std::fmax(std::fabs(a.spiral_root - nominal.spiral_root),
                                    std::fabs(b.spiral_root - nominal.spiral_root));
      band.roll_subsidence_root +=
          std::fmax(std::fabs(a.roll_subsidence_root - nominal.roll_subsidence_root),
                    std::fabs(b.roll_subsidence_root - nominal.roll_subsidence_root));
      band.dutch_roll_natural_frequency += std::fmax(
          std::fabs(a.dutch_roll_natural_frequency - nominal.dutch_roll_natural_frequency),
          std::fabs(b.dutch_roll_natural_frequency - nominal.dutch_roll_natural_frequency));
      band.dutch_roll_damping_ratio +=
          std::fmax(std::fabs(a.dutch_roll_damping_ratio - nominal.dutch_roll_damping_ratio),
                    std::fabs(b.dutch_roll_damping_ratio - nominal.dutch_roll_damping_ratio));
    }
    return band;
  }

  static LateralInputs* inputs_;
  static std::map<std::string, double>* published_;
  static std::string* citation_;
};

LateralInputs* Nt33aLateral::inputs_ = nullptr;
std::map<std::string, double>* Nt33aLateral::published_ = nullptr;
std::string* Nt33aLateral::citation_ = nullptr;

TEST_F(Nt33aLateral, ReferenceDataIsCitedAndComplete) {
  ASSERT_NE(citation_, nullptr);
  EXPECT_NE(citation_->find("NASA CR-2144"), std::string::npos);
  EXPECT_NE(citation_->find("19730003312"), std::string::npos);
  EXPECT_NE(citation_->find("Unclassified - Unlimited"), std::string::npos);
  ASSERT_NE(published_, nullptr);
  EXPECT_GE(published_->size(), 70U);
}

TEST_F(Nt33aLateral, AllThreeLateralModesAreFoundAndCorrectlyLabelled) {
  const std::vector<std::string> names = {"beta", "p", "r", "phi"};
  const auto result =
      analyze_modes(assemble_lateral(*inputs_), names, StateRoles::from_names(names));

  ASSERT_TRUE(result.participation_is_meaningful)
      << "eigenvector condition number " << result.eigenvector_condition_number;
  EXPECT_LT(result.eigenvector_condition_number, 100.0)
      << "a well-behaved conventional aeroplane should have well-separated modes";

  // Three modes from a fourth-order system: the Dutch roll pair counts once.
  ASSERT_EQ(result.modes.size(), 3U);

  const auto* spiral = result.find(ModeLabel::Spiral);
  const auto* roll = result.find(ModeLabel::RollSubsidence);
  const auto* dutch = result.find(ModeLabel::DutchRoll);
  ASSERT_NE(spiral, nullptr) << "no mode was labelled spiral";
  ASSERT_NE(roll, nullptr) << "no mode was labelled roll subsidence";
  ASSERT_NE(dutch, nullptr) << "no mode was labelled Dutch roll";

  // The labels must rest on real evidence, not on being the last one left.
  EXPECT_GT(spiral->label_score, 0.5) << spiral->label_reason;
  EXPECT_GT(roll->label_score, 0.5) << roll->label_reason;
  EXPECT_GT(dutch->label_score, 0.5) << dutch->label_reason;

  // Physical character, independent of the published numbers: the Dutch roll
  // oscillates and is lightly damped, roll subsidence is a fast convergence,
  // the spiral is slow.
  EXPECT_TRUE(dutch->is_oscillatory);
  EXPECT_FALSE(roll->is_oscillatory);
  EXPECT_FALSE(spiral->is_oscillatory);
  EXPECT_LT(dutch->damping_ratio, 0.2);
  EXPECT_GT(roll->natural_frequency_rad_s, 10.0 * spiral->natural_frequency_rad_s);
}

TEST_F(Nt33aLateral, ModesMatchThePublishedValuesWithinTheSourcesOwnPrecision) {
  const LateralModes computed = extract(assemble_lateral(*inputs_));
  ASSERT_TRUE(computed.complete);

  const LateralModes band = input_precision_band();

  struct Case {
    const char* name;
    double computed;
    double published;
    double input_band;
  };

  const Case cases[] = {
      {"spiral root", computed.spiral_root, published_->at("spiral_root"), band.spiral_root},
      {"roll subsidence root",
       computed.roll_subsidence_root,
       published_->at("roll_subsidence_root"),
       band.roll_subsidence_root},
      {"Dutch roll natural frequency",
       computed.dutch_roll_natural_frequency,
       published_->at("dutch_roll_natural_frequency"),
       band.dutch_roll_natural_frequency},
      {"Dutch roll damping ratio",
       computed.dutch_roll_damping_ratio,
       published_->at("dutch_roll_damping_ratio"),
       band.dutch_roll_damping_ratio},
  };

  for (const Case& c : cases) {
    // Both sides are rounded: the inputs the computation used, and the answer
    // the report printed. The allowance is the sum.
    const double tolerance = c.input_band + half_ulp(c.published, kPrintedFigures);
    EXPECT_LE(std::fabs(c.computed - c.published), tolerance)
        << c.name << ": computed " << c.computed << ", published " << c.published
        << ", input-precision band " << c.input_band << ", published rounding "
        << half_ulp(c.published, kPrintedFigures);
  }
}

TEST_F(Nt33aLateral, TheDutchRollPeriodAgreesWithThePublishedPeriod) {
  // An independent published cross-check: the report tabulates the Dutch roll
  // PERIOD separately, in Table II-10, derived from the same roots. Agreeing
  // with both the (zeta, omega_n) pair and the period is a stronger statement
  // than agreeing with either alone.
  const std::vector<std::string> names = {"beta", "p", "r", "phi"};
  const auto result =
      analyze_modes(assemble_lateral(*inputs_), names, StateRoles::from_names(names));
  const auto* dutch = result.find(ModeLabel::DutchRoll);
  ASSERT_NE(dutch, nullptr);

  const double published_period = published_->at("dutch_roll_period");
  // The period is printed to three significant figures and is a nonlinear
  // function of two other three-figure quantities, so 1% is the honest bound.
  EXPECT_NEAR(dutch->period_s, published_period, 0.01 * published_period)
      << "computed period " << dutch->period_s << " s, published " << published_period << " s";
}

TEST_F(Nt33aLateral, TheSpiralIsConvergentAsThePublishedSignImplies) {
  // Appendix A: "(s + 1/T_x)_i = 1/T_x_i", so the printed +0.0318 is the factor
  // (s + 0.0318) and the root is NEGATIVE — a convergent spiral. Had the report
  // meant a divergence it would have printed a negative 1/T, as it does for the
  // numerator root -0.0494 in its own worked example.
  const LateralModes computed = extract(assemble_lateral(*inputs_));
  ASSERT_TRUE(computed.complete);
  EXPECT_GT(computed.spiral_root, 0.0)
      << "the spiral root should be positive in the report's 1/T convention, "
         "meaning a convergent mode";
  EXPECT_GT(published_->at("spiral_root"), 0.0);
}

}  // namespace
