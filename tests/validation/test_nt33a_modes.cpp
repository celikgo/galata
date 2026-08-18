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

// Half a unit in each input's last PRINTED digit, expressed in the same units
// as the corresponding field of LateralInputs.
using LateralSteps = LateralInputs;

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

// Half a unit in the last printed digit of a value that has since been
// converted into other units.
//
// This distinction is not pedantry and getting it wrong quietly breaks the
// gate. The report prints M_u* as 0.000318 per second-foot, so its uncertainty
// is +/-0.0000005 per second-FOOT. Converted to per second-metre the VALUE
// becomes 0.0010433 and the UNCERTAINTY becomes 0.00000164 — but taking
// half_ulp of the converted value gives 0.0000005, understating it by the
// conversion factor of 3.28. Angles are worse: 2.20 degrees is known to
// +/-0.005 degrees, which is 8.7e-5 rad, while half_ulp of 0.0384 rad is
// 5e-5 rad.
//
// So the step is computed on the PRINTED value and then carried through the
// same conversion the value went through.
double converted_half_ulp(double printed_value, int figures, double conversion_factor) {
  return half_ulp(printed_value, figures) * conversion_factor;
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

    steps_ = new LateralSteps{};
    steps_->y_v = half_ulp(value.at("Y_v"), kPrintedFigures);
    steps_->l_beta_prime = half_ulp(value.at("L_beta_prime"), kPrintedFigures);
    steps_->n_beta_prime = half_ulp(value.at("N_beta_prime"), kPrintedFigures);
    steps_->l_p_prime = half_ulp(value.at("L_p_prime"), kPrintedFigures);
    steps_->n_p_prime = half_ulp(value.at("N_p_prime"), kPrintedFigures);
    steps_->l_r_prime = half_ulp(value.at("L_r_prime"), kPrintedFigures);
    steps_->n_r_prime = half_ulp(value.at("N_r_prime"), kPrintedFigures);
    steps_->airspeed_m_s = converted_half_ulp(
        value.at("true_airspeed"), kPrintedFigures, galata::units::kMetresPerFoot);
    steps_->trim_alpha_rad = converted_half_ulp(
        value.at("trim_alpha"), kPrintedFigures, galata::units::kRadiansPerDegree);

    published_ = new std::map<std::string, double>(value);
  }

  static void TearDownTestSuite() {
    delete inputs_;
    delete steps_;
    delete published_;
    delete citation_;
    inputs_ = nullptr;
    steps_ = nullptr;
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
      const double step = steps_->*knob;
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
  static LateralSteps* steps_;
  static std::map<std::string, double>* published_;
  static std::string* citation_;
};

LateralInputs* Nt33aLateral::inputs_ = nullptr;
LateralSteps* Nt33aLateral::steps_ = nullptr;
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

// ===========================================================================
// Longitudinal
// ===========================================================================
//
// NASA CR-2144 Appendix C page C-1 prints the longitudinal system as a
// descriptor form in [u, w, theta], with q supplied by the auxiliary relation
// q = s theta:
//
//   | (1 - X_udot)s - X_u*   -X_wdot s - X_w      (-X_q + W_o)s + g cos th_o | | u  |   | X_de |
//   | -Z_udot s - Z_u*       (1 - Z_wdot)s - Z_w  (-Z_q - U_o)s + g sin th_o | | w  | = | Z_de | de
//   | -M_udot s - M_u*       -(M_wdot s + M_w)     s^2 - M_q s               | | th |   | M_de |
//
// Three things about this are easy to get wrong and are worth stating.
//
// The second state is w IN FEET PER SECOND, not alpha in radians. The report's
// tabulated derivatives are X_w, Z_w, M_w — w-derivatives — and there is no
// U_o folded into the state. A reader who assumes alpha gets every entry in
// the second column wrong by a factor of the trim speed.
//
// M_wdot couples w_dot into the pitching-moment equation, so the system is
// implicit and must be rearranged before it is a state matrix. Dropping that
// term is tempting because it is small; it is also the term that carries the
// downwash lag, and it moves the short period.
//
// The asterisk on X_u*, Z_u* and M_u* does NOT mean a normalisation. Appendix A
// page A-16 defines X_u* = X_u + T_u cos(xi_o) — it is the derivative WITH the
// thrust-speed contribution included. Since the tables print the starred forms,
// they are what goes into the matrix directly.
//
// Derivatives the report does not tabulate for this aircraft — X_udot, X_wdot,
// X_q, Z_udot, M_udot — are taken as zero. That is an assumption this test
// makes, not a statement the document makes, and it is recorded here.

struct LongitudinalInputs {
  double x_u_star = 0.0;  // 1/s
  double z_u_star = 0.0;  // 1/s
  double m_u_star = 0.0;  // 1/(s m)   — per LENGTH, so it needs converting
  double x_w = 0.0;       // 1/s
  double z_w = 0.0;       // 1/s
  double m_w = 0.0;       // 1/(s m)   — per length
  double z_w_dot = 0.0;   // dimensionless
  double m_w_dot = 0.0;   // 1/m       — per length
  double z_q = 0.0;       // m/s
  double m_q = 0.0;       // 1/s
  double airspeed_m_s = 0.0;
  double trim_alpha_rad = 0.0;
  double trim_gamma_rad = 0.0;
};

using LongitudinalSteps = LongitudinalInputs;

// Unlike the lateral matrix, this one is NOT invariant under a change of length
// unit: M_u*, M_w and M_wdot are per-foot quantities, so they must be converted
// or the pitching-moment row is wrong by a factor of 3.28. A good demonstration
// of why ADR-0003 puts the conversion at the boundary and bans it everywhere
// else — this is exactly the mistake it exists to prevent.
Eigen::MatrixXd assemble_longitudinal(const LongitudinalInputs& in) {
  const double theta_o = in.trim_alpha_rad + in.trim_gamma_rad;
  const double u_o = in.airspeed_m_s * std::cos(in.trim_alpha_rad);
  const double w_o = in.airspeed_m_s * std::sin(in.trim_alpha_rad);
  const double g = galata::core::kStandardGravity;

  // Rearranged out of the descriptor form:
  //   u_dot = X_u* u + X_w w - W_o q - g cos(th_o) th
  //   w_dot = [Z_u* u + Z_w w + (Z_q + U_o) q - g sin(th_o) th] / (1 - Z_wdot)
  //   q_dot = M_u* u + M_w w + M_q q + M_wdot w_dot
  const double scale = 1.0 / (1.0 - in.z_w_dot);
  const double w_row_u = in.z_u_star * scale;
  const double w_row_w = in.z_w * scale;
  const double w_row_q = (in.z_q + u_o) * scale;
  const double w_row_theta = -g * std::sin(theta_o) * scale;

  Eigen::MatrixXd a(4, 4);
  a << in.x_u_star, in.x_w, -w_o, -g * std::cos(theta_o),                 //
      w_row_u, w_row_w, w_row_q, w_row_theta,                             //
      in.m_u_star + in.m_w_dot * w_row_u, in.m_w + in.m_w_dot * w_row_w,  //
      in.m_q + in.m_w_dot * w_row_q, in.m_w_dot * w_row_theta,            //
      0.0, 0.0, 1.0, 0.0;
  return a;
}

struct LongitudinalModes {
  double phugoid_natural_frequency = 0.0;
  double phugoid_damping_ratio = 0.0;
  double short_period_natural_frequency = 0.0;
  double short_period_damping_ratio = 0.0;
  bool complete = false;
};

LongitudinalModes extract_longitudinal(const Eigen::MatrixXd& a) {
  const std::vector<std::string> names = {"u", "w", "q", "theta"};
  const auto result = analyze_modes(a, names, StateRoles::from_names(names));

  LongitudinalModes modes;
  const auto* phugoid = result.find(ModeLabel::Phugoid);
  const auto* short_period = result.find(ModeLabel::ShortPeriod);
  if (phugoid == nullptr || short_period == nullptr) {
    return modes;
  }
  modes.phugoid_natural_frequency = phugoid->natural_frequency_rad_s;
  modes.phugoid_damping_ratio = phugoid->damping_ratio;
  modes.short_period_natural_frequency = short_period->natural_frequency_rad_s;
  modes.short_period_damping_ratio = short_period->damping_ratio;
  modes.complete = true;
  return modes;
}

class Nt33aLongitudinal : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    const auto table =
        galata::testing::load_reference(GALATA_VALIDATION_REFERENCE_DIR, "nt33a_fc1.csv");
    const std::map<std::string, double> value = table.as_lookup("quantity", "value");
    published_ = new std::map<std::string, double>(value);

    // Per-length derivatives convert; per-second ones do not.
    const double per_foot_to_per_metre = 1.0 / galata::units::kMetresPerFoot;

    inputs_ = new LongitudinalInputs{};
    inputs_->x_u_star = value.at("X_u_star");
    inputs_->z_u_star = value.at("Z_u_star");
    inputs_->m_u_star = value.at("M_u_star") * per_foot_to_per_metre;
    inputs_->x_w = value.at("X_w");
    inputs_->z_w = value.at("Z_w");
    inputs_->m_w = value.at("M_w") * per_foot_to_per_metre;
    inputs_->z_w_dot = value.at("Z_w_dot");
    inputs_->m_w_dot = value.at("M_w_dot") * per_foot_to_per_metre;
    inputs_->z_q = galata::units::feet_to_metres(value.at("Z_q"));
    inputs_->m_q = value.at("M_q");
    inputs_->airspeed_m_s = galata::units::feet_to_metres(value.at("true_airspeed"));
    inputs_->trim_alpha_rad = galata::units::degrees_to_radians(value.at("trim_alpha"));
    inputs_->trim_gamma_rad = galata::units::degrees_to_radians(value.at("trim_gamma"));

    // Steps taken on the PRINTED values and carried through the same
    // conversions — see converted_half_ulp above for why this matters.
    steps_ = new LongitudinalSteps{};
    steps_->x_u_star = half_ulp(value.at("X_u_star"), kPrintedFigures);
    steps_->z_u_star = half_ulp(value.at("Z_u_star"), kPrintedFigures);
    steps_->m_u_star =
        converted_half_ulp(value.at("M_u_star"), kPrintedFigures, per_foot_to_per_metre);
    steps_->x_w = half_ulp(value.at("X_w"), kPrintedFigures);
    steps_->z_w = half_ulp(value.at("Z_w"), kPrintedFigures);
    steps_->m_w = converted_half_ulp(value.at("M_w"), kPrintedFigures, per_foot_to_per_metre);
    steps_->m_w_dot =
        converted_half_ulp(value.at("M_w_dot"), kPrintedFigures, per_foot_to_per_metre);
    steps_->m_q = half_ulp(value.at("M_q"), kPrintedFigures);
    steps_->airspeed_m_s = converted_half_ulp(
        value.at("true_airspeed"), kPrintedFigures, galata::units::kMetresPerFoot);
    steps_->trim_alpha_rad = converted_half_ulp(
        value.at("trim_alpha"), kPrintedFigures, galata::units::kRadiansPerDegree);
  }

  static void TearDownTestSuite() {
    delete inputs_;
    delete steps_;
    delete published_;
    inputs_ = nullptr;
    steps_ = nullptr;
    published_ = nullptr;
  }

  static LongitudinalModes input_precision_band() {
    const LongitudinalModes nominal = extract_longitudinal(assemble_longitudinal(*inputs_));
    LongitudinalModes band;
    const std::vector<double LongitudinalInputs::*> knobs = {
        &LongitudinalInputs::x_u_star,
        &LongitudinalInputs::z_u_star,
        &LongitudinalInputs::m_u_star,
        &LongitudinalInputs::x_w,
        &LongitudinalInputs::z_w,
        &LongitudinalInputs::m_w,
        &LongitudinalInputs::m_w_dot,
        &LongitudinalInputs::m_q,
        &LongitudinalInputs::airspeed_m_s,
        &LongitudinalInputs::trim_alpha_rad,
    };
    for (double LongitudinalInputs::* knob : knobs) {
      LongitudinalInputs high = *inputs_;
      LongitudinalInputs low = *inputs_;
      const double step = steps_->*knob;
      high.*knob += step;
      low.*knob -= step;
      const LongitudinalModes a = extract_longitudinal(assemble_longitudinal(high));
      const LongitudinalModes b = extract_longitudinal(assemble_longitudinal(low));
      band.phugoid_natural_frequency +=
          std::fmax(std::fabs(a.phugoid_natural_frequency - nominal.phugoid_natural_frequency),
                    std::fabs(b.phugoid_natural_frequency - nominal.phugoid_natural_frequency));
      band.phugoid_damping_ratio +=
          std::fmax(std::fabs(a.phugoid_damping_ratio - nominal.phugoid_damping_ratio),
                    std::fabs(b.phugoid_damping_ratio - nominal.phugoid_damping_ratio));
      band.short_period_natural_frequency += std::fmax(
          std::fabs(a.short_period_natural_frequency - nominal.short_period_natural_frequency),
          std::fabs(b.short_period_natural_frequency - nominal.short_period_natural_frequency));
      band.short_period_damping_ratio +=
          std::fmax(std::fabs(a.short_period_damping_ratio - nominal.short_period_damping_ratio),
                    std::fabs(b.short_period_damping_ratio - nominal.short_period_damping_ratio));
    }
    return band;
  }

  static LongitudinalInputs* inputs_;
  static LongitudinalSteps* steps_;
  static std::map<std::string, double>* published_;
};

LongitudinalInputs* Nt33aLongitudinal::inputs_ = nullptr;
LongitudinalSteps* Nt33aLongitudinal::steps_ = nullptr;
std::map<std::string, double>* Nt33aLongitudinal::published_ = nullptr;

TEST_F(Nt33aLongitudinal, BothLongitudinalModesAreFoundAndCorrectlyLabelled) {
  const std::vector<std::string> names = {"u", "w", "q", "theta"};
  const auto result =
      analyze_modes(assemble_longitudinal(*inputs_), names, StateRoles::from_names(names));

  ASSERT_TRUE(result.participation_is_meaningful);
  ASSERT_EQ(result.modes.size(), 2U) << "two conjugate pairs, reported once each";

  const auto* phugoid = result.find(ModeLabel::Phugoid);
  const auto* short_period = result.find(ModeLabel::ShortPeriod);
  ASSERT_NE(phugoid, nullptr);
  ASSERT_NE(short_period, nullptr);

  // Strong evidence, not a coin toss between two oscillations.
  EXPECT_GT(phugoid->label_score, 0.8) << phugoid->label_reason;
  EXPECT_GT(short_period->label_score, 0.8) << short_period->label_reason;

  // The physical character, independent of the published numbers: the phugoid
  // is slow and lightly damped, the short period fast and well damped.
  EXPECT_LT(phugoid->natural_frequency_rad_s, short_period->natural_frequency_rad_s);
  EXPECT_LT(phugoid->damping_ratio, short_period->damping_ratio);
  EXPECT_GT(phugoid->period_s, 5.0 * short_period->period_s);
}

TEST_F(Nt33aLongitudinal, ModesMatchThePublishedValuesWithinTheSourcesOwnPrecision) {
  const LongitudinalModes computed = extract_longitudinal(assemble_longitudinal(*inputs_));
  ASSERT_TRUE(computed.complete);
  const LongitudinalModes band = input_precision_band();

  struct Case {
    const char* name;
    double computed;
    double published;
    double input_band;
  };

  // The phugoid DAMPING RATIO is deliberately absent from this list. It does not
  // reproduce within the source's precision, and the next test records that
  // rather than hiding it by widening a tolerance here.
  const Case cases[] = {
      {"phugoid natural frequency",
       computed.phugoid_natural_frequency,
       published_->at("phugoid_natural_frequency"),
       band.phugoid_natural_frequency},
      {"short period natural frequency",
       computed.short_period_natural_frequency,
       published_->at("short_period_natural_frequency"),
       band.short_period_natural_frequency},
      {"short period damping ratio",
       computed.short_period_damping_ratio,
       published_->at("short_period_damping_ratio"),
       band.short_period_damping_ratio},
  };

  for (const Case& c : cases) {
    const double tolerance = c.input_band + half_ulp(c.published, kPrintedFigures);
    EXPECT_LE(std::fabs(c.computed - c.published), tolerance)
        << c.name << ": computed " << c.computed << ", published " << c.published
        << ", input-precision band " << c.input_band;
  }
}

// REGRESSION LOCK, not a validation. Anchored to the validated cases above.
//
// The phugoid damping ratio does NOT reproduce within the source's own printed
// precision, and this test exists to say so precisely and to stop the gap
// growing unnoticed. It is labelled a regression lock because its bound comes
// from galata's current output, not from a published value — which is exactly
// the thing charter rule 8 forbids doing silently.
//
// WHAT DISAGREES, measured. The published phugoid is zeta = 0.0948,
// omega_n = 0.172 rad/s, i.e. an eigenvalue of -0.016306 +/- 0.171226j.
// galata computes -0.015974 +/- 0.171170j. So:
//
//   imaginary part   agrees to 6e-5  (3 parts in 10,000)
//   real part        agrees to 3.3e-4
//   damping RATIO    disagrees by 0.0019, which is 2% relative
//
// The mode itself is in very nearly the right place. The disagreement is
// concentrated in a derived quantity: zeta = |Re| / |lambda|, and dividing a
// small real part by a small omega_n amplifies the residual. 3.3e-4 / 0.172 is
// 0.0019 — the entire discrepancy, accounted for.
//
// WHY IT MIGHT DISAGREE. Two candidates, neither confirmed:
//
//  1. Terms taken as zero here because the report does not tabulate them for
//     this aircraft — X_udot, X_wdot, X_q, Z_udot, M_udot. X_q in particular
//     enters the phugoid through the (-X_q + W_o)s term of Appendix C's first
//     row, and the report's blank is being read as "zero" when it may mean
//     "not supplied".
//  2. The report's own quartic in Appendix C is written in terms of M_alpha
//     and M_alpha_dot rather than M_w and M_wdot, and its D and E coefficients
//     were not transcribed. If its D coefficient carries a term this
//     assembly omits, the phugoid is where it would show, since the phugoid
//     roots are set by the low-order coefficients.
//
// Resolving this needs the rest of Appendix C, and until it is resolved
// docs/VERIFICATION.md lists the phugoid damping ratio as an open discrepancy
// rather than as validated.
TEST_F(Nt33aLongitudinal, PhugoidDampingDiscrepancyDoesNotGrow) {
  const LongitudinalModes computed = extract_longitudinal(assemble_longitudinal(*inputs_));
  ASSERT_TRUE(computed.complete);

  const double published = published_->at("phugoid_damping_ratio");
  const double disagreement = std::fabs(computed.phugoid_damping_ratio - published);

  // Measured at the time of writing: 0.00193. The lock is set just above it, so
  // any change that makes the agreement WORSE fails, and any change that makes
  // it better fails too — because that would mean the discrepancy has been
  // explained and this test should be replaced by a real validation.
  EXPECT_GT(disagreement, 0.0015)
      << "the phugoid damping now agrees better than when this lock was written. "
         "That is good news: work out why, and replace this regression lock with a "
         "validation in ModesMatchThePublishedValuesWithinTheSourcesOwnPrecision.";
  EXPECT_LT(disagreement, 0.0025) << "the phugoid damping disagreement has grown to "
                                  << disagreement
                                  << " from the 0.00193 recorded when this lock was written";

  // The eigenvalue itself does agree closely, which is the point. If this
  // stopped being true the problem would be somewhere else entirely.
  const double published_omega = published_->at("phugoid_natural_frequency");
  const double published_real = -published * published_omega;
  const double computed_real = -computed.phugoid_damping_ratio * computed.phugoid_natural_frequency;
  EXPECT_LT(std::fabs(computed_real - published_real), 5e-4)
      << "the phugoid eigenvalue's real part no longer agrees closely, so the "
         "discrepancy is no longer confined to the derived damping ratio";
}

TEST_F(Nt33aLongitudinal, TheDownwashLagTermMatters) {
  // M_wdot is small and it is tempting to drop it. This shows what that costs:
  // removing it moves the short period by more than the published value's own
  // precision, so a model that dropped it would fail the comparison above.
  //
  // Recorded as a test because "this term is negligible" is an assertion that
  // should be measured rather than assumed.
  LongitudinalInputs without = *inputs_;
  without.m_w_dot = 0.0;

  const LongitudinalModes with_term = extract_longitudinal(assemble_longitudinal(*inputs_));
  const LongitudinalModes without_term = extract_longitudinal(assemble_longitudinal(without));
  ASSERT_TRUE(with_term.complete);
  ASSERT_TRUE(without_term.complete);

  const double shift =
      std::fabs(with_term.short_period_damping_ratio - without_term.short_period_damping_ratio);
  EXPECT_GT(shift, half_ulp(published_->at("short_period_damping_ratio"), kPrintedFigures))
      << "dropping M_wdot moved the short-period damping by only " << shift
      << ", which would make this test vacuous";
}

}  // namespace
