// SPDX-License-Identifier: Apache-2.0
//
// The rotor, against closed-form answers.
//
// These are the checks that decide whether the audit's first blocking
// capability is closed. Before this module, rotor force was the body-axis
// vector (0, 0, -k_T omega^2) for every rotor: a tail rotor produced no side
// force and cyclic produced no in-plane force at all. Each of those is now a
// test that would have been structurally impossible to write.
//
// The fixture is the Souxmar main rotor: R = 6.0 m, four blades, 195 m/s tip
// speed, solidity 0.072. Its provenance is models/souxmar-heli/PROVENANCE.md.

#include "galata/model/rotor/rotor.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <stdexcept>

using galata::model::rotor::hover_induced_velocity_m_s;
using galata::model::rotor::main_rotor_hub;
using galata::model::rotor::RotorControls;
using galata::model::rotor::RotorGeometry;
using galata::model::rotor::RotorSolution;
using galata::model::rotor::RotorState;
using galata::model::rotor::solve_rotor;
using galata::model::rotor::tail_rotor_hub;

namespace {

constexpr double kSeaLevelDensity = 1.225;                  // kg/m^3
constexpr double kMainTipSpeed = 195.0;                     // m/s
constexpr double kMainRadius = 6.0;                         // m
constexpr double kMainOmega = kMainTipSpeed / kMainRadius;  // 32.5 rad/s

// Souxmar main rotor. Chord follows from the design's declared solidity of
// 0.072 with four blades: c = sigma pi R / N_b.
RotorGeometry souxmar_main() {
  RotorGeometry g;
  g.name = "main";
  g.position_cg_to_hub_body_m = Eigen::Vector3d(0.0, 0.0, -2.5);  // above the CG (FRD: -z is up)
  g.hub_to_body = Eigen::Matrix3d::Identity();
  g.spin_about_shaft = 1;
  g.radius_m = kMainRadius;
  g.chord_m = 0.072 * std::numbers::pi * kMainRadius / 4.0;
  g.blade_count = 4;
  g.lift_curve_slope = 5.73;
  g.profile_drag_coefficient = 0.011;
  g.blade_twist_rad = -0.14;
  g.tip_loss_factor = 0.97;
  g.polar_inertia_kg_m2 = 1400.0;
  g.induced_power_factor = 1.15;  // the Rev I design's declared kappa
  return g;
}

RotorGeometry souxmar_tail(bool starboard = true) {
  RotorGeometry g;
  g.name = "tail";
  g.position_cg_to_hub_body_m = Eigen::Vector3d(-6.7, 0.0, -1.0);
  g.hub_to_body = tail_rotor_hub(starboard);
  g.spin_about_shaft = 1;
  g.radius_m = 1.2;
  g.chord_m = 0.18 * std::numbers::pi * 1.2 / 4.0;
  g.blade_count = 4;
  g.lift_curve_slope = 5.73;
  g.profile_drag_coefficient = 0.012;
  g.tip_loss_factor = 0.95;
  g.polar_inertia_kg_m2 = 6.0;
  g.induced_power_factor = 1.2;  // the Rev I design's declared tail kappa
  return g;
}

// The Souxmar gross hover thrust: 2829.7 kg with the design's 1.04 download.
constexpr double kHoverThrustN = 2829.7 * 9.80665 * 1.04;

RotorSolution solve_at(const RotorGeometry& g,
                       double collective_rad,
                       const Eigen::Vector3d& velocity_body = Eigen::Vector3d::Zero(),
                       const Eigen::Vector3d& rate = Eigen::Vector3d::Zero(),
                       double omega = kMainOmega,
                       const RotorControls& cyclic = {}) {
  RotorState state;
  state.speed_rad_s = omega;
  RotorControls controls = cyclic;
  controls.collective_rad = collective_rad;
  return solve_rotor(g, state, controls, velocity_body, rate, kSeaLevelDensity);
}

// The collective that carries a given thrust, by bisection on a monotone
// function. SOLVED rather than written down, so the fixture cannot drift away
// from the mass it claims to lift when the aerodynamics change.
double collective_for_thrust(const RotorGeometry& g, double target_n, double omega) {
  double low = 0.0;
  double high = 0.5;
  for (int i = 0; i < 80; ++i) {
    const double mid = 0.5 * (low + high);
    RotorState state;
    state.speed_rad_s = omega;
    RotorControls controls;
    controls.collective_rad = mid;
    const auto s = solve_rotor(
        g, state, controls, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), kSeaLevelDensity);
    (s.thrust_n < target_n ? low : high) = mid;
  }
  return 0.5 * (low + high);
}

}  // namespace

// ---------------------------------------------------------------------------
// ACCEPTANCE 1: a main rotor generates vertical thrust.
// ---------------------------------------------------------------------------

TEST(Rotor, MainRotorGeneratesVerticalThrust) {
  const auto g = souxmar_main();
  const double collective = collective_for_thrust(g, kHoverThrustN, kMainOmega);
  const auto s = solve_at(g, collective);
  // A four-tonne-class rotor lifting its own gross mass sits near 16 degrees of
  // root collective once the -8 degree twist is accounted for.
  EXPECT_GT(collective, 0.15);
  EXPECT_LT(collective, 0.40);

  EXPECT_GT(s.thrust_n, 0.0);
  // Thrust is upward in NED body axes, which is NEGATIVE z.
  EXPECT_LT(s.wrench.force_body_n.z(), 0.0);
  EXPECT_NEAR(s.wrench.force_body_n.x(), 0.0, 1e-9);
  EXPECT_NEAR(s.wrench.force_body_n.y(), 0.0, 1e-9);
  // And the solve reached the Souxmar gross hover thrust, 28.9 kN.
  EXPECT_NEAR(s.thrust_n, kHoverThrustN, 1.0);
  // C_T/sigma at that point is the design's own screen quantity. Rev I computes
  // 0.08545 at its 3170 kg power-sizing mass; at the 2829.7 kg gross mass this
  // fixture uses it is proportionally lower. The check is that the model lands
  // in the band a helicopter main rotor actually operates in.
  EXPECT_GT(s.thrust_coefficient_solidity, 0.05);
  EXPECT_LT(s.thrust_coefficient_solidity, 0.12);
}

// ---------------------------------------------------------------------------
// ACCEPTANCE 2: a tail rotor generates body-Y side force. THIS IS THE ONE THAT
// WAS STRUCTURALLY IMPOSSIBLE before the module existed.
// ---------------------------------------------------------------------------

TEST(Rotor, TailRotorGeneratesBodyYSideForceAndAYawMoment) {
  const auto g = souxmar_tail(/*starboard=*/true);
  const auto s = solve_at(g, 0.18, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 162.5);

  ASSERT_GT(s.thrust_n, 0.0);
  // The whole point: a force along +y, not along -z.
  EXPECT_GT(s.wrench.force_body_n.y(), 0.0);
  EXPECT_NEAR(s.wrench.force_body_n.z(), 0.0, 1e-9)
      << "a tail rotor must not produce a vertical force; the old model gave it nothing else";
  EXPECT_NEAR(s.wrench.force_body_n.x(), 0.0, 1e-9);
  // PUSHING THE TAIL TO STARBOARD SWINGS THE NOSE TO PORT, so the yaw moment
  // is NEGATIVE. This is the sign that decides which way the pedal works, and
  // the arithmetic is written out rather than asserted from memory:
  // r x F with r = (-6.7, 0, -1.0) and F = (0, +T, 0) has z-component -6.7 T.
  EXPECT_LT(s.wrench.moment_cg_body_n_m.z(), 0.0);
  // And the moment is the force times the arm, to the sign of the shaft torque.
  const double expected_yaw = -6.7 * s.wrench.force_body_n.y();
  EXPECT_NEAR(
      s.wrench.moment_cg_body_n_m.z() - expected_yaw, 0.0, 0.05 * std::fabs(expected_yaw) + 50.0);
}

TEST(Rotor, TailRotorCanBeMountedToPushEitherWay) {
  const auto starboard = solve_at(souxmar_tail(true), 0.18, {}, {}, 162.5);
  const auto port =
      solve_at(souxmar_tail(false), 0.18, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 162.5);
  EXPECT_GT(starboard.wrench.force_body_n.y(), 0.0);
  EXPECT_LT(port.wrench.force_body_n.y(), 0.0);
  EXPECT_NEAR(starboard.wrench.force_body_n.y(), -port.wrench.force_body_n.y(), 1e-9);
}

// ---------------------------------------------------------------------------
// ACCEPTANCE 3: cyclic produces longitudinal and lateral force components.
// ---------------------------------------------------------------------------

TEST(Rotor, LongitudinalCyclicProducesALongitudinalForce) {
  auto g = souxmar_main();
  const auto neutral = solve_at(g, 0.20);
  RotorControls forward;
  forward.longitudinal_cyclic_rad = 0.05;  // about 2.9 degrees, disc forward
  const auto tilted = solve_at(g, 0.20, {}, {}, kMainOmega, forward);

  EXPECT_NEAR(neutral.wrench.force_body_n.x(), 0.0, 1e-9);
  // Disc tilted forward gives a FORWARD force: this is how a helicopter
  // translates, and it was identically zero before this module.
  EXPECT_GT(tilted.wrench.force_body_n.x(), 0.0);
  EXPECT_GT(std::fabs(tilted.wrench.force_body_n.x()), 500.0);
  // Small-angle check: F_x ~ T sin(theta_1s) for a rotor rigid in flap.
  EXPECT_NEAR(tilted.wrench.force_body_n.x(), tilted.thrust_n * std::sin(0.05), 1.0);
}

TEST(Rotor, LateralCyclicProducesALateralForce) {
  auto g = souxmar_main();
  RotorControls right;
  right.lateral_cyclic_rad = 0.04;
  const auto tilted = solve_at(g, 0.20, {}, {}, kMainOmega, right);
  EXPECT_LT(tilted.wrench.force_body_n.y(), 0.0);
  EXPECT_NEAR(tilted.wrench.force_body_n.y(), -tilted.thrust_n * std::sin(0.04), 1.0);
}

// ---------------------------------------------------------------------------
// ACCEPTANCE 4: thrust decreases with axial inflow.
// ---------------------------------------------------------------------------

TEST(Rotor, ThrustFallsAsAxialInflowRises) {
  const auto g = souxmar_main();
  const double hover = solve_at(g, 0.20).thrust_n;
  double previous = hover;
  // Climbing is velocity along body -z (upwards), so a NEGATIVE w.
  for (double climb : {2.0, 5.0, 10.0, 15.0}) {
    const double thrust = solve_at(g, 0.20, Eigen::Vector3d(0.0, 0.0, -climb)).thrust_n;
    EXPECT_LT(thrust, previous) << "thrust must fall monotonically with climb rate, at " << climb
                                << " m/s";
    previous = thrust;
  }
  EXPECT_LT(previous, hover * 0.95)
      << "a 15 m/s climb should cost a material fraction of the hover thrust";
}

// ---------------------------------------------------------------------------
// ACCEPTANCE 5: hover induced velocity agrees with momentum theory.
// ---------------------------------------------------------------------------

TEST(Rotor, HoverInducedVelocityAgreesWithMomentumTheory) {
  const auto g = souxmar_main();
  const auto s = solve_at(g, 0.20);
  ASSERT_GT(s.thrust_n, 0.0);

  // Momentum theory, independently: v_h = sqrt(T / (2 rho A)).
  const double closed_form = hover_induced_velocity_m_s(g, s.thrust_n, kSeaLevelDensity);

  // The solver's own induced velocity must reproduce it. The two are not the
  // same computation: one is the fixed point of the coupled blade-element /
  // momentum iteration, the other is the momentum relation evaluated at the
  // thrust that iteration produced. Agreement is a statement that the iteration
  // actually converged.
  //
  // BUDGET, DECLARED BEFORE THE MEASUREMENT: the iteration is under-relaxed at
  // 0.5 and run 16 times, contracting by about 0.5 per step from an initial
  // error of order lambda_h itself, so the residual should sit near
  // 0.5^16 ~ 1.5e-5 relative. The gate is 1e-4 relative, a factor of six of
  // headroom, and NOT the observed value.
  EXPECT_NEAR(s.induced_velocity_m_s, closed_form, 1.0e-4 * closed_form);
}

TEST(Rotor, TheInflowFixedPointSatisfiesTheMomentumRelation) {
  const auto g = souxmar_main();
  // Checked in hover and in forward flight, because the Glauert relation has a
  // different character in each.
  for (const auto& velocity : {Eigen::Vector3d(0.0, 0.0, 0.0),
                               Eigen::Vector3d(30.0, 0.0, 0.0),
                               Eigen::Vector3d(50.0, 0.0, -3.0)}) {
    const auto s = solve_at(g, 0.20, velocity);
    const double lambda = s.axial_inflow_ratio + s.induced_inflow_ratio;
    const double momentum =
        s.thrust_coefficient
        / (2.0 * std::sqrt(s.advance_ratio * s.advance_ratio + lambda * lambda));
    EXPECT_NEAR(s.induced_inflow_ratio, momentum, 1.0e-4 * std::fabs(momentum) + 1e-9)
        << "at V = " << velocity.transpose();
  }
}

// ---------------------------------------------------------------------------
// ACCEPTANCE 6: shaft tilt changes the thrust direction correctly.
// ---------------------------------------------------------------------------

TEST(Rotor, ShaftTiltRotatesTheThrustVectorByExactlyTheTiltAngle) {
  auto upright = souxmar_main();
  auto tilted = souxmar_main();
  const double tilt = 0.0873;  // 5 degrees forward
  tilted.hub_to_body = main_rotor_hub(tilt, 0.0);

  const auto a = solve_at(upright, 0.20);
  const auto b = solve_at(tilted, 0.20);

  // Same thrust magnitude — the tilt moves the vector, it does not change the
  // aerodynamics in still air.
  EXPECT_NEAR(a.thrust_n, b.thrust_n, 1e-9);
  // Forward shaft tilt gives a forward force component of exactly T sin(tilt).
  EXPECT_NEAR(b.wrench.force_body_n.x(), b.thrust_n * std::sin(tilt), 1e-6);
  EXPECT_NEAR(b.wrench.force_body_n.z(), -b.thrust_n * std::cos(tilt), 1e-6);
  EXPECT_GT(b.wrench.force_body_n.x(), 0.0);
}

TEST(Rotor, AnUntiltedRotorWithNoCyclicReproducesTheOldBodyZOnlyForce) {
  // The multirotor convention is the degenerate case of this model, exactly.
  const auto g = souxmar_main();
  const auto s = solve_at(g, 0.20);
  EXPECT_EQ(s.wrench.force_body_n.x(), 0.0);
  EXPECT_EQ(s.wrench.force_body_n.y(), 0.0);
  EXPECT_LT(s.wrench.force_body_n.z(), 0.0);
}

// ---------------------------------------------------------------------------
// Torque, power and the anti-torque balance.
// ---------------------------------------------------------------------------

TEST(Rotor, TorqueReactsOntoTheAirframeOppositeToTheRotorsRotation) {
  auto clockwise = souxmar_main();
  clockwise.spin_about_shaft = 1;
  auto anticlockwise = souxmar_main();
  anticlockwise.spin_about_shaft = -1;

  const auto a = solve_at(clockwise, 0.20);
  const auto b = solve_at(anticlockwise, 0.20);

  EXPECT_GT(a.torque_n_m, 0.0) << "torque DEMANDED is positive whichever way it turns";
  EXPECT_NEAR(a.torque_n_m, b.torque_n_m, 1e-9);
  // The yaw moment on the airframe flips with the spin. Getting this backwards
  // gives a helicopter that hovers, trims and linearises with yaw inverted.
  EXPECT_LT(a.wrench.moment_cg_body_n_m.z(), 0.0);
  EXPECT_GT(b.wrench.moment_cg_body_n_m.z(), 0.0);
}

TEST(Rotor, PowerIsTorqueTimesSpeedAndIsTheRightOrderForTheDesign) {
  const auto g = souxmar_main();
  // Collective set to carry roughly the 3170 kg the design sizes power at.
  double collective = 0.0;
  for (int i = 0; i < 60; ++i) {
    const auto s = solve_at(g, collective);
    const double target = 3170.0 * 9.80665 * 1.04;  // with the design's download factor
    collective += 1.0e-6 * (target - s.thrust_n);
  }
  const auto s = solve_at(g, collective);
  EXPECT_NEAR(s.power_w, s.torque_n_m * kMainOmega, 1e-6);
  // The Rev I design computes 504.9 kW of main-rotor power at this condition.
  // This model is a different formulation reaching the same physics, so the
  // gate is loose (25%) and exists to catch an order-of-magnitude error, not
  // to claim agreement. The validation tier makes the tight comparison.
  EXPECT_GT(s.power_w, 0.75 * 504.9e3);
  EXPECT_LT(s.power_w, 1.25 * 504.9e3);
}

// ---------------------------------------------------------------------------
// Gyroscopic coupling — the term the multirotor model documents as absent.
// ---------------------------------------------------------------------------

TEST(Rotor, ASpinningRotorProducesGyroscopicCrossCoupling) {
  const auto g = souxmar_main();
  const Eigen::Vector3d pitch_rate(0.0, 0.3, 0.0);
  const auto with_rate = solve_at(g, 0.20, Eigen::Vector3d::Zero(), pitch_rate);
  const auto without = solve_at(g, 0.20);

  const Eigen::Vector3d difference =
      with_rate.wrench.moment_cg_body_n_m - without.wrench.moment_cg_body_n_m;
  // A pitch rate on a rotor whose angular momentum is along +z produces a ROLL
  // moment. h = I_R Omega z = 1400 * 32.5 = 45500 kg m^2/s, and
  // -(omega x h) has x-component -(q h_z - r h_y) = -q h_z.
  EXPECT_NEAR(difference.x(), -0.3 * 1400.0 * kMainOmega, 1.0);
  EXPECT_LT(difference.x(), -10000.0) << "this coupling is large, not a correction";
}

// ---------------------------------------------------------------------------
// Validation of the geometry itself.
// ---------------------------------------------------------------------------

TEST(Rotor, GeometryValidationRefusesTheMistakesThatAreSilentWhenMade) {
  auto g = souxmar_main();
  EXPECT_NO_THROW(g.validate());

  auto not_a_rotation = souxmar_main();
  not_a_rotation.hub_to_body = Eigen::Matrix3d::Identity() * 1.01;
  EXPECT_THROW(not_a_rotation.validate(), std::invalid_argument);

  auto reflection = souxmar_main();
  reflection.hub_to_body = Eigen::Matrix3d::Identity();
  reflection.hub_to_body(2, 2) = -1.0;
  EXPECT_THROW(reflection.validate(), std::invalid_argument);

  auto bad_spin = souxmar_main();
  bad_spin.spin_about_shaft = 0;
  EXPECT_THROW(bad_spin.validate(), std::invalid_argument);

  auto one_blade = souxmar_main();
  one_blade.blade_count = 1;
  EXPECT_THROW(one_blade.validate(), std::invalid_argument);

  auto hinge_past_tip = souxmar_main();
  hinge_past_tip.hinge_offset_m = 7.0;
  EXPECT_THROW(hinge_past_tip.validate(), std::invalid_argument);
}

TEST(Rotor, SolidityFollowsFromChordAndBladeCountRatherThanBeingDeclared) {
  const auto g = souxmar_main();
  EXPECT_NEAR(g.solidity(), 0.072, 1e-12)
      << "the fixture's chord was derived from the design's declared solidity, so the round trip "
         "must be exact";
  EXPECT_NEAR(g.disc_area_m2(), std::numbers::pi * 36.0, 1e-9);
}

TEST(Rotor, AStoppedRotorProducesNothingRatherThanDividingByZero) {
  const auto g = souxmar_main();
  const auto s = solve_at(g, 0.20, {}, {}, 0.0);
  EXPECT_EQ(s.thrust_n, 0.0);
  EXPECT_EQ(s.torque_n_m, 0.0);
  EXPECT_TRUE(s.wrench.force_body_n.isZero());
  EXPECT_TRUE(s.wrench.moment_cg_body_n_m.isZero());
}

TEST(Rotor, DynamicInflowLagMakesTheInflowStateDriveTheThrust) {
  auto g = souxmar_main();
  g.inflow_time_constant_s = 0.1;

  RotorState low;
  low.speed_rad_s = kMainOmega;
  low.inflow_ratio = 0.01;  // wake not yet built
  RotorState settled;
  settled.speed_rad_s = kMainOmega;
  settled.inflow_ratio = 0.06;

  RotorControls controls;
  controls.collective_rad = 0.20;
  const auto a = solve_rotor(
      g, low, controls, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), kSeaLevelDensity);
  const auto b = solve_rotor(
      g, settled, controls, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), kSeaLevelDensity);

  // Less inflow means more thrust: the collective-step overshoot a lag exists
  // to represent. If the state did not drive thrust these would be equal, and
  // the lag would be a parameter with no effect.
  EXPECT_GT(a.thrust_n, b.thrust_n);
  // And the state relaxes towards the quasi-static value.
  EXPECT_GT(a.inflow_rate_per_s, 0.0);
  EXPECT_NEAR(a.inflow_rate_per_s, (a.quasi_static_inflow_ratio - 0.01) / 0.1, 1e-9);
}

TEST(Rotor, RepeatedSolvesAreBitIdentical) {
  const auto g = souxmar_main();
  for (int i = 0; i < 8; ++i) {
    const double collective = 0.05 + 0.02 * static_cast<double>(i);
    const Eigen::Vector3d velocity(3.0 * static_cast<double>(i), 1.0, -2.0);
    const auto a = solve_at(g, collective, velocity);
    const auto b = solve_at(g, collective, velocity);
    EXPECT_EQ(a.thrust_n, b.thrust_n);
    EXPECT_EQ(a.torque_n_m, b.torque_n_m);
    EXPECT_EQ(a.induced_inflow_ratio, b.induced_inflow_ratio);
    EXPECT_EQ(a.wrench.force_body_n.x(), b.wrench.force_body_n.x());
    EXPECT_EQ(a.wrench.moment_cg_body_n_m.z(), b.wrench.moment_cg_body_n_m.z());
  }
}
