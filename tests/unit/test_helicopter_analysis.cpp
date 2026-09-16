// SPDX-License-Identifier: Apache-2.0
//
// Linearising the helicopter about its hover trim, classifying what comes out,
// and measuring how well the linearisation predicts the nonlinear model.
//
// The last of those is the one that matters. A linearisation can be wrong in a
// way no eigenvalue shows — a sign error, a missing term, a point that is not
// quite an equilibrium — and no single tolerance on a single perturbation size
// distinguishes it from a correct one. What does distinguish them is the ORDER:
// a correct linearisation's error falls as the SQUARE of the perturbation.

#include "galata/analyze/modes.hpp"
#include "galata/linearize/vehicle.hpp"
#include "galata/model/helicopter.hpp"
#include "galata/trim/problem.hpp"

#include <Eigen/Eigenvalues>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

using galata::analyze::ModeLabel;
using galata::analyze::StateRoles;
using galata::analyze::analyze_modes;
using galata::core::State;
using galata::core::identity_attitude;
using galata::linearize::VehicleLinearisation;
using galata::linearize::linearize_vehicle;
using galata::linearize::nonlinear_agreement;
using galata::model::Environment;
using galata::model::HelicopterModel;
using galata::model::kCollectivePosition;
using galata::model::load_helicopter;
using galata::trim::TrimCondition;
using galata::trim::helicopter_trim_problem;
using galata::trim::solve_trim;

namespace {

HelicopterModel souxmar() {
  return load_helicopter(std::string(GALATA_SOURCE_DIR)
                         + "/models/souxmar-heli/souxmar-heli.yaml");
}

// Trim, then set the COMMANDS equal to the solved actuator POSITIONS. Without
// that the actuator rates are non-zero at the point and it is not an
// equilibrium of the whole model — which linearize_vehicle refuses, correctly.
struct TrimmedHelicopter {
  HelicopterModel model;
  Environment environment;
  Eigen::VectorXd extended_state;
  Eigen::VectorXd controls;
};

TrimmedHelicopter trimmed_hover(double forward_speed_m_s = 0.0) {
  TrimmedHelicopter out{souxmar(), Environment::sea_level_still_air(), {}, {}};
  TrimCondition condition;
  condition.environment = out.environment;
  condition.controls = Eigen::VectorXd::Zero(out.model.control_count());
  State state;
  state.attitude_body_to_ned = identity_attitude();
  condition.extended_state =
      out.model.join(state, out.model.initial_auxiliary(condition.controls, out.environment));
  condition.extended_state(galata::core::kVelocityU) = forward_speed_m_s;

  const auto result = solve_trim(out.model, helicopter_trim_problem(out.model), condition);
  EXPECT_TRUE(result.converged);
  out.extended_state = result.extended_state;
  out.controls = result.controls;
  const Eigen::VectorXd auxiliary = out.model.auxiliary_part(out.extended_state);
  for (int i = 0; i < out.model.control_count(); ++i) {
    out.controls(i) = auxiliary(kCollectivePosition + i);
  }
  return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// Linearisation.
// ---------------------------------------------------------------------------

TEST(HelicopterLinearisation, LinearisesAboutTheHoverTrim) {
  const auto trimmed = trimmed_hover();
  const auto linear = linearize_vehicle(trimmed.model, trimmed.extended_state, trimmed.controls,
                                        trimmed.environment);

  // Twelve Euler coordinates plus the model's seven auxiliary states.
  EXPECT_EQ(linear.a.rows(), 19);
  EXPECT_EQ(linear.a.cols(), 19);
  EXPECT_EQ(linear.b.rows(), 19);
  EXPECT_EQ(linear.b.cols(), 4);
  EXPECT_EQ(linear.state_names.size(), 19u);

  // The quaternion never appears: that is the whole reason for Euler
  // coordinates, and a spurious zero eigenvalue from an over-parameterised
  // attitude would otherwise have to be discarded by every modal result.
  for (const auto& name : linear.state_names) {
    EXPECT_EQ(name.find("quaternion"), std::string::npos) << name;
  }

  // Measured, not assumed.
  EXPECT_LT(linear.equilibrium_residual, 1.0e-6);
  EXPECT_TRUE(linear.a.allFinite());
  EXPECT_TRUE(linear.b.allFinite());
}

TEST(HelicopterLinearisation, RefusesAPointThatIsNotAnEquilibriumAndNamesTheState) {
  const auto trimmed = trimmed_hover();
  Eigen::VectorXd disturbed = trimmed.extended_state;
  disturbed(galata::core::kVelocityW) += 3.0;  // a 3 m/s heave is not a trim

  try {
    (void)linearize_vehicle(trimmed.model, disturbed, trimmed.controls, trimmed.environment);
    FAIL() << "a linearisation about a non-equilibrium must be refused";
  } catch (const std::runtime_error& error) {
    const std::string what = error.what();
    EXPECT_NE(what.find("not an equilibrium"), std::string::npos) << what;
    EXPECT_NE(what.find("constant term"), std::string::npos)
        << "the refusal must say WHY it matters: " << what;
  }
}

TEST(HelicopterLinearisation, TheActuatorLagsAppearAsTheirOwnEigenvalues) {
  const auto trimmed = trimmed_hover();
  const auto linear = linearize_vehicle(trimmed.model, trimmed.extended_state, trimmed.controls,
                                        trimmed.environment);
  const Eigen::VectorXcd eigenvalues = linear.a.eigenvalues();

  // A first-order actuator with time constant tau contributes a real
  // eigenvalue at exactly -1/tau. Four actuators, four time constants. This is
  // a closed-form check on the whole linearisation path: state layout,
  // perturbation, and the derivative all have to be right for these to land.
  for (const double tau : {0.08, 0.06, 0.06, 0.05}) {
    const double expected = -1.0 / tau;
    bool found = false;
    for (Eigen::Index i = 0; i < eigenvalues.size(); ++i) {
      if (std::fabs(eigenvalues(i).imag()) < 1e-9
          && std::fabs(eigenvalues(i).real() - expected) < 1e-6) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "no eigenvalue at -1/tau = " << expected << " for tau = " << tau;
  }
}

TEST(HelicopterLinearisation, ReducedDropsThePositionAndHeadingIntegratorsByName) {
  const auto trimmed = trimmed_hover();
  const auto full = linearize_vehicle(trimmed.model, trimmed.extended_state, trimmed.controls,
                                      trimmed.environment);
  const auto reduced = full.reduced();

  EXPECT_EQ(reduced.a.rows(), 15);
  for (const char* dropped :
       {"position_north_m", "position_east_m", "position_down_m", "yaw_rad"}) {
    EXPECT_EQ(std::find(reduced.state_names.begin(), reduced.state_names.end(), dropped),
              reduced.state_names.end())
        << dropped << " should have been dropped";
  }
  EXPECT_NE(std::find(reduced.state_names.begin(), reduced.state_names.end(), "velocity_u_m_s"),
            reduced.state_names.end());
}

// ---------------------------------------------------------------------------
// THE ORDER CHECK. This is the one that would catch a wrong sign.
// ---------------------------------------------------------------------------

TEST(HelicopterLinearisation, NonlinearAndLinearAgreeToSecondOrderInForwardFlight) {
  const auto trimmed = trimmed_hover(20.0);
  const auto linear = linearize_vehicle(trimmed.model, trimmed.extended_state, trimmed.controls,
                                        trimmed.environment);

  Eigen::VectorXd direction = Eigen::VectorXd::Zero(linear.a.rows());
  direction(3) = 1.0;  // velocity_u_m_s
  direction(5) = 0.5;  // velocity_w_m_s

  const auto agreement =
      nonlinear_agreement(trimmed.model, linear, trimmed.environment, direction,
                          {0.4, 0.2, 0.1, 0.05, 0.025}, 0.5, 0.001);

  ASSERT_EQ(agreement.discrepancies.size(), 5u);

  // The discrepancy must FALL as the perturbation shrinks. A linearisation with
  // a sign error would not.
  for (std::size_t i = 1; i < agreement.discrepancies.size(); ++i) {
    EXPECT_LT(agreement.discrepancies[i], agreement.discrepancies[i - 1])
        << "at epsilon = " << agreement.epsilons[i];
  }

  // And it must fall as epsilon SQUARED, which is the check that would catch a
  // wrong sign, a missing term, or a point that is not quite an equilibrium —
  // none of which a single tolerance on a single perturbation size separates
  // from a correct linearisation.
  //
  // BUDGET DECLARED BEFORE THE MEASUREMENT: the Taylor remainder is exactly
  // second order, so the observed order should sit at 2 with only the
  // fourth-order integration error and the Jacobian's own truncation to
  // account for. 1.9 to 2.1 is that band.
  for (std::size_t i = 0; i < agreement.observed_orders.size(); ++i) {
    EXPECT_GT(agreement.observed_orders[i], 1.9)
        << "order " << agreement.observed_orders[i] << " between epsilon "
        << agreement.epsilons[i] << " and " << agreement.epsilons[i + 1]
        << " — a first-order error would sit near 1";
    EXPECT_LT(agreement.observed_orders[i], 2.1) << agreement.observed_orders[i];
  }
}

TEST(HelicopterLinearisation, HasANonVanishingErrorFloorInExactHoverAndThatIsThePhysics) {
  // LOCALISED AND PUBLISHED RATHER THAN ABSORBED INTO A TOLERANCE.
  //
  // In forward flight the linearisation is second order, and the test above
  // measures 2.004. In EXACT hover it is not, and the discrepancy stops falling
  // at a floor of about 3.1e-3 instead of vanishing. Two terms cause it and
  // both are properties of the physics rather than of this implementation:
  //
  //   1. The advance ratio mu = |V_inplane| / (Omega R). |V| has no derivative
  //      at V = 0 — the one-sided slopes are +1 and -1 — so a central
  //      difference returns zero for d(mu)/dV, and the linearisation carries no
  //      flap-back response to a speed perturbation while the nonlinear model
  //      has one proportional to |V|.
  //   2. The empennage's incidence, atan2(w, u), evaluated at a tail sitting in
  //      4 m/s of rotor downwash with no free stream. That is atan2(4, 0) and
  //      the angle swings through a right angle for an arbitrarily small
  //      perturbation in u.
  //
  // Measured by removing the empennage: the floor drops away and the order
  // becomes a clean 1.00, which is term 1 alone. With the empennage the two
  // combine and no single order describes the result.
  //
  // THIS TEST IS TWO-SIDED, like the regression locks in docs/VERIFICATION.md.
  // It fails if the floor GROWS, which would mean a defect beyond the two known
  // kinks. It also fails if the floor VANISHES, because a hover linearisation
  // that converges cleanly would mean the advance ratio's kink had been
  // smoothed away — and a smoothed mu is a better Jacobian of a worse model.
  const auto trimmed = trimmed_hover(0.0);
  const auto linear = linearize_vehicle(trimmed.model, trimmed.extended_state, trimmed.controls,
                                        trimmed.environment);

  Eigen::VectorXd direction = Eigen::VectorXd::Zero(linear.a.rows());
  direction(3) = 1.0;
  direction(5) = 0.5;

  const auto agreement =
      nonlinear_agreement(trimmed.model, linear, trimmed.environment, direction,
                          {0.1, 0.0125, 0.00625}, 0.5, 0.001);

  const double floor_value = agreement.discrepancies.back();

  // The floor is real: halving the perturbation twice more does not halve it.
  EXPECT_GT(floor_value, 0.5 * agreement.discrepancies.front())
      << "the hover discrepancy converged away; the advance ratio's kink at V = 0 has been "
         "smoothed, which improves the Jacobian and worsens the model";

  // And it is small: about 3.1e-3 in mixed state units over half a second, so
  // the hover linearisation is still usable for stability analysis — which is
  // what it is for — while being unfit for trajectory prediction.
  EXPECT_LT(floor_value, 1.0e-2)
      << "the hover error floor has grown beyond the two known kinks: " << floor_value;
}

TEST(HelicopterLinearisation, RefusesAnAgreementStudyWithOneEpsilon) {
  const auto trimmed = trimmed_hover();
  const auto linear = linearize_vehicle(trimmed.model, trimmed.extended_state, trimmed.controls,
                                        trimmed.environment);
  Eigen::VectorXd direction = Eigen::VectorXd::Zero(linear.a.rows());
  direction(3) = 1.0;
  const auto run = [&] {
    (void)nonlinear_agreement(trimmed.model, linear, trimmed.environment, direction, {0.1}, 1.0,
                              0.002);
  };
  EXPECT_THROW(run(), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Rotorcraft mode classification.
// ---------------------------------------------------------------------------

TEST(HelicopterModes, ClassifiesTheHoverModesAndFindsAnUnstableOscillation) {
  const auto trimmed = trimmed_hover();
  const auto reduced = linearize_vehicle(trimmed.model, trimmed.extended_state, trimmed.controls,
                                         trimmed.environment)
                           .reduced();
  const auto roles = StateRoles::from_names(reduced.state_names);

  // The rotor-speed role is what makes the rotorcraft labels candidates at all.
  ASSERT_TRUE(roles.has_rotorcraft());
  EXPECT_GE(roles.heave_velocity, 0);

  const auto modes = analyze_modes(reduced.a, reduced.state_names, roles);

  // A HOVERING HELICOPTER IS UNSTABLE, and this is the check that it came out
  // that way. The hover oscillation has negative damping — it is the mode a
  // pilot is continuously correcting and the reason a stability augmentation
  // system exists at all. A model that produced a stable hover would be wrong
  // in a way no tolerance would catch.
  bool found_unstable_oscillation = false;
  for (const auto& mode : modes.modes) {
    if (mode.is_oscillatory && mode.eigenvalue.real() > 0.0) {
      found_unstable_oscillation = true;
      EXPECT_LT(mode.damping_ratio, 0.0);
      // Slow: a period of tens of seconds, not tenths.
      EXPECT_LT(mode.natural_frequency_rad_s, 1.0);
      EXPECT_GT(mode.natural_frequency_rad_s, 0.01);
    }
  }
  EXPECT_TRUE(found_unstable_oscillation)
      << "a hovering helicopter has an unstable low-frequency oscillation; this model produced "
         "none, which would mean the rotor's thrust-vector tilt is not reaching the attitude "
         "equations";

  // The heave root and the rotor-speed mode are both identified, and both are
  // stable convergences.
  const auto* heave = modes.find(ModeLabel::HeaveSubsidence);
  ASSERT_NE(heave, nullptr) << "the heave root should be identified in hover";
  EXPECT_LT(heave->eigenvalue.real(), 0.0);

  const auto* rotor = modes.find(ModeLabel::RotorSpeedMode);
  ASSERT_NE(rotor, nullptr) << "the governed rotor-speed mode should be identified";
  EXPECT_LT(rotor->eigenvalue.real(), 0.0);
}

TEST(HelicopterModes, AModelWithNoRotorSpeedRoleCannotBeGivenARotorcraftLabel) {
  // THE GATE. Every rotorcraft signature is refused for a model that declares
  // no rotor speed, which is what keeps the classical five returning exactly
  // what they returned before these labels existed. Asserted here directly
  // rather than only implied by the NT-33A validation case.
  const auto trimmed = trimmed_hover();
  const auto reduced = linearize_vehicle(trimmed.model, trimmed.extended_state, trimmed.controls,
                                         trimmed.environment)
                           .reduced();

  auto roles = StateRoles::from_names(reduced.state_names);
  roles.rotor_speed = -1;  // pretend this is a fixed-wing model
  ASSERT_FALSE(roles.has_rotorcraft());

  const auto modes = analyze_modes(reduced.a, reduced.state_names, roles);
  for (const auto& mode : modes.modes) {
    EXPECT_NE(mode.label, ModeLabel::HoveringCubic);
    EXPECT_NE(mode.label, ModeLabel::LateralHoveringOscillation);
    EXPECT_NE(mode.label, ModeLabel::HeaveSubsidence);
    EXPECT_NE(mode.label, ModeLabel::YawSubsidence);
    EXPECT_NE(mode.label, ModeLabel::RotorSpeedMode);
  }
}

TEST(HelicopterLinearisation, RepeatedLinearisationsAreBitIdentical) {
  const auto trimmed = trimmed_hover();
  const auto a = linearize_vehicle(trimmed.model, trimmed.extended_state, trimmed.controls,
                                   trimmed.environment);
  const auto b = linearize_vehicle(trimmed.model, trimmed.extended_state, trimmed.controls,
                                   trimmed.environment);
  for (Eigen::Index i = 0; i < a.a.rows(); ++i) {
    for (Eigen::Index j = 0; j < a.a.cols(); ++j) {
      EXPECT_EQ(a.a(i, j), b.a(i, j)) << "A(" << i << ", " << j << ")";
    }
  }
  EXPECT_EQ(a.equilibrium_residual, b.equilibrium_residual);
}
