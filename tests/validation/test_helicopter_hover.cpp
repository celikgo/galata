// SPDX-License-Identifier: Apache-2.0
//
// The Souxmar helicopter's hover, against two things that are not this code.
//
// THERE IS NO PUBLISHED ROTORCRAFT REFERENCE HERE, and that is stated rather
// than worked around. A published measured helicopter dataset with the printed
// precision this project's validation ritual needs was not obtained, so no case
// in this file claims agreement with one. What is checked instead is two kinds
// of evidence that ARE available and that are not the implementation:
//
//   1. CLOSED-FORM INVARIANTS. Momentum theory's hover induced velocity, the
//      thrust-equals-weight balance, and the anti-torque balance are exact
//      statements about the equations, independent of how they are coded. A
//      disagreement is a defect with nowhere to hide.
//
//   2. A CROSS-IMPLEMENTATION CHECK against the Souxmar design package's own
//      hover computation — a separate Python momentum-theory script, written
//      independently of this code, from the same geometry. Agreement is not
//      validation: both are momentum theory, so a shared modelling error would
//      pass. It is evidence that the two codings of the same physics agree, and
//      it is recorded as exactly that.
//
// The budget for the cross-check is derived from the source's own precision
// BEFORE the comparison is made, and the derivation is written out below.

#include "galata/model/helicopter.hpp"
#include "galata/model/rotor/rotor.hpp"
#include "galata/trim/problem.hpp"

#include "validation_config.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <string>

namespace {

using galata::core::State;
using galata::core::identity_attitude;
using galata::model::Environment;
using galata::model::HelicopterModel;
using galata::model::VehicleModel;
using galata::model::load_helicopter;
using galata::trim::helicopter_trim_problem;
using galata::trim::solve_trim;

constexpr double kStandardGravity = 9.80665;  // m/s^2, 3rd CGPM 1901

HelicopterModel souxmar() {
  return load_helicopter(std::string(GALATA_MODELS_DIR) + "/souxmar-heli/souxmar-heli.yaml");
}

// ---------------------------------------------------------------------------
// THE BUDGET, DERIVED BEFORE THE COMPARISON.
//
// The design package prints its hover results to a precision far beyond their
// meaning — `main_ct_sigma: 0.08545490299451122` — so the printed digits are not
// the limit. What limits the comparison is the INPUTS the two implementations
// share and the assumptions they do not.
//
// Shared exactly: rotor radius (6.0 m), tip speed (195 m/s), solidity (0.072),
// blade count (4), profile drag coefficient (0.011), induced power factor
// (1.15), hover download (1.04), and the mass the package sizes power at
// (3170 kg). Those contribute no disagreement.
//
// NOT shared, and each contributing:
//
//   * TIP LOSS. This model applies a Prandtl tip-loss factor B = 0.97 inside the
//     blade-element thrust integral. The design package's momentum-theory
//     script has no tip-loss term. Thrust scales roughly as B^2 in the inflow
//     term, so the disagreement from this alone is of order 1 - 0.97^2 ~ 6%
//     in thrust at fixed collective — but the comparison below is made at
//     FIXED THRUST, where tip loss moves the required collective rather than
//     the thrust, and its effect on C_T/sigma is second order. Budget: 2%.
//
//   * BLADE TWIST. This model carries -8 degrees of linear washout; the package
//     assumes none. At fixed thrust this shifts the collective, not C_T/sigma.
//     Budget: 1%.
//
//   * AIR DENSITY. Both use sea-level ISA, 1.225 kg/m^3. No contribution.
//
// Total budget: 3% relative on C_T/sigma at the package's own sizing condition.
// The gate is set at the budget and not at the observed value.
// ---------------------------------------------------------------------------
constexpr double kCrossCheckBudget = 0.03;

// The design package's own printed figures at sea-level ISA, 3170 kg, from
// turboshaft_rev_i/results.json, hover[0]. Transcribed programmatically, not
// retyped; see models/souxmar-heli/PROVENANCE.md §5.
constexpr double kPackageMainCtSigma = 0.08545490299451122;
constexpr double kPackageSizingMassKg = 3170.0;
constexpr double kPackageHoverDownload = 1.04;

}  // namespace

// ---------------------------------------------------------------------------
// 1. Closed-form invariants.
// ---------------------------------------------------------------------------

TEST(SouxmarHelicopterHover, InducedVelocityMatchesMomentumTheoryExactly) {
  const auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  const auto trimmed = solve_trim(heli, helicopter_trim_problem(heli),
                                  [&] {
                                    galata::trim::TrimCondition condition;
                                    condition.environment = environment;
                                    condition.controls =
                                        Eigen::VectorXd::Zero(heli.control_count());
                                    State state;
                                    state.attitude_body_to_ned = identity_attitude();
                                    condition.extended_state = heli.join(
                                        state, heli.initial_auxiliary(condition.controls,
                                                                      environment));
                                    return condition;
                                  }());
  ASSERT_TRUE(trimmed.converged);

  const auto parts = heli.breakdown(VehicleModel::rigid_body_part(trimmed.extended_state),
                                    heli.auxiliary_part(trimmed.extended_state),
                                    trimmed.controls, environment);

  // v_h = sqrt(T / (2 rho A)), evaluated independently of the solver's own
  // iteration. The two agreeing says the fixed-point converged to the momentum
  // relation and not merely to a fixed point of something.
  const double closed_form = galata::model::rotor::hover_induced_velocity_m_s(
      heli.main_rotor, parts.main.thrust_n, environment.density_kg_m3);

  // Budget: the inflow Newton runs a fixed sixteen iterations from the hover
  // momentum guess and converges quadratically, so the residual is at machine
  // precision. 1e-6 relative is four orders of margin and is not the observed
  // value.
  EXPECT_NEAR(parts.main.induced_velocity_m_s, closed_form, 1.0e-6 * closed_form);
}

TEST(SouxmarHelicopterHover, MainRotorThrustCarriesTheWeight) {
  const auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  galata::trim::TrimCondition condition;
  condition.environment = environment;
  condition.controls = Eigen::VectorXd::Zero(heli.control_count());
  State state;
  state.attitude_body_to_ned = identity_attitude();
  condition.extended_state =
      heli.join(state, heli.initial_auxiliary(condition.controls, environment));

  const auto trimmed = solve_trim(heli, helicopter_trim_problem(heli), condition);
  ASSERT_TRUE(trimmed.converged);
  const auto parts = heli.breakdown(VehicleModel::rigid_body_part(trimmed.extended_state),
                                    heli.auxiliary_part(trimmed.extended_state),
                                    trimmed.controls, environment);

  const double weight = heli.mass.mass_kg * kStandardGravity;
  // Not exactly the weight: the thrust vector is tilted by the shaft's 3 degrees
  // of forward tilt, by the flapping, and by the bank the tail-rotor side force
  // demands. cos(3 deg) alone is 0.9986. 3% carries all three.
  EXPECT_NEAR(parts.main.thrust_n, weight, 0.03 * weight);
}

TEST(SouxmarHelicopterHover, TheAntiTorqueBalancesAndTheYawResidualVanishes) {
  const auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  galata::trim::TrimCondition condition;
  condition.environment = environment;
  condition.controls = Eigen::VectorXd::Zero(heli.control_count());
  State state;
  state.attitude_body_to_ned = identity_attitude();
  condition.extended_state =
      heli.join(state, heli.initial_auxiliary(condition.controls, environment));

  const auto trimmed = solve_trim(heli, helicopter_trim_problem(heli), condition);
  ASSERT_TRUE(trimmed.converged);
  const auto parts = heli.breakdown(VehicleModel::rigid_body_part(trimmed.extended_state),
                                    heli.auxiliary_part(trimmed.extended_state),
                                    trimmed.controls, environment);

  // The residual vanishing IS the balance, and it is what the sixth trim
  // residual asked for.
  EXPECT_NEAR(parts.anti_torque_residual_n_m, 0.0, 1.0e-6);

  // And it balances the way the physics says: tail thrust at its arm against the
  // main rotor's shaft torque, to within the tail's own shaft torque and the
  // fin's contribution.
  const double arm = -heli.tail_rotor.position_cg_to_hub_body_m.x();
  const double tail_moment = arm * std::fabs(parts.tail.wrench.force_body_n.y());
  EXPECT_NEAR(tail_moment, parts.main.torque_n_m, 0.05 * parts.main.torque_n_m);
}

// ---------------------------------------------------------------------------
// 2. The cross-implementation check, against its predeclared budget.
// ---------------------------------------------------------------------------

TEST(SouxmarHelicopterHover, ThrustCoefficientAgreesWithTheDesignPackagesOwnComputation) {
  auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();

  // The package sizes hover power at 3170 kg with a 1.04 download factor, which
  // is a different mass from this model's 2829.7 kg gross. The comparison is
  // made at the PACKAGE's condition, because that is the condition it printed a
  // number for.
  const double target_thrust = kPackageSizingMassKg * kStandardGravity * kPackageHoverDownload;

  // Collective solved by bisection on a monotone function, so the comparison is
  // made at matched thrust rather than at matched collective — the two
  // implementations do not share a collective definition, and they do share a
  // thrust.
  double low = 0.0;
  double high = 0.5;
  for (int i = 0; i < 80; ++i) {
    const double mid = 0.5 * (low + high);
    galata::model::rotor::RotorState rotor_state;
    rotor_state.speed_rad_s = heli.drivetrain.reference_rotor_speed_rad_s;
    galata::model::rotor::RotorControls controls;
    controls.collective_rad = mid;
    const auto solution =
        galata::model::rotor::solve_rotor(heli.main_rotor, rotor_state, controls,
                                          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                          environment.density_kg_m3);
    (solution.thrust_n < target_thrust ? low : high) = mid;
  }

  galata::model::rotor::RotorState rotor_state;
  rotor_state.speed_rad_s = heli.drivetrain.reference_rotor_speed_rad_s;
  galata::model::rotor::RotorControls controls;
  controls.collective_rad = 0.5 * (low + high);
  const auto solution =
      galata::model::rotor::solve_rotor(heli.main_rotor, rotor_state, controls,
                                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                        environment.density_kg_m3);

  const double relative =
      std::fabs(solution.thrust_coefficient_solidity - kPackageMainCtSigma) / kPackageMainCtSigma;
  EXPECT_LT(relative, kCrossCheckBudget)
      << "C_T/sigma " << solution.thrust_coefficient_solidity << " against the design package's "
      << kPackageMainCtSigma << " — relative difference " << relative << " against a budget of "
      << kCrossCheckBudget << " derived before the comparison";
}

// ---------------------------------------------------------------------------
// 3. Determinism of the whole chain.
// ---------------------------------------------------------------------------

TEST(SouxmarHelicopterHover, TheWholeTrimChainIsBitIdenticalOnRepeat) {
  const auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  const auto build = [&] {
    galata::trim::TrimCondition condition;
    condition.environment = environment;
    condition.controls = Eigen::VectorXd::Zero(heli.control_count());
    State state;
    state.attitude_body_to_ned = identity_attitude();
    condition.extended_state =
        heli.join(state, heli.initial_auxiliary(condition.controls, environment));
    return condition;
  };
  const auto a = solve_trim(heli, helicopter_trim_problem(heli), build());
  const auto b = solve_trim(heli, helicopter_trim_problem(heli), build());
  ASSERT_TRUE(a.converged);
  for (Eigen::Index i = 0; i < a.extended_state.size(); ++i) {
    EXPECT_EQ(a.extended_state(i), b.extended_state(i)) << "state " << i;
  }
  EXPECT_EQ(a.residual_norm, b.residual_norm);
}
