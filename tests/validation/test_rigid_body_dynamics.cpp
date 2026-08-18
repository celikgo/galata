// SPDX-License-Identifier: Apache-2.0
//
// VALIDATION: the six-degree-of-freedom rigid body against closed-form
// solutions of Euler's equations.
//
// Reference:
//   H. Goldstein, C. P. Poole and J. L. Safko, "Classical Mechanics", 3rd ed.,
//   Addison Wesley, 2002 — Euler's equations and the torque-free motion of a
//   symmetric rigid body.
//   L. D. Landau and E. M. Lifshitz, "Mechanics", 3rd ed., Course of
//   Theoretical Physics vol. 1, Butterworth-Heinemann, 1976 — the same, and
//   the stability of rotation about the principal axes.
//
// These cases carry no transcribed numbers. The reference is an analytic
// solution, and the derivation is written out below so a reader can check it
// against the equations rather than against a table. That makes them the
// strongest validation in the suite: there is no transcription step to get
// wrong, and the expected values are exact rather than rounded.

#include "galata/core/quaternion.hpp"
#include "galata/core/state.hpp"
#include "galata/numerics/integrator.hpp"
#include "galata/sim/rigid_body.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using galata::core::State;
using galata::core::StateVector;
using galata::sim::MassProperties;
using galata::sim::Wrench;

// No gravity and no applied wrench: the pure rotational problem.
galata::numerics::DerivativeFunction torque_free(const MassProperties& mass) {
  return [mass](double, const Eigen::VectorXd& x) -> Eigen::VectorXd {
    const State state = State::from_vector(StateVector(x));
    return galata::sim::rigid_body_derivative(state, mass, Wrench{}, Eigen::Vector3d::Zero());
  };
}

galata::numerics::ProjectionFunction renormalise_quaternion() {
  return [](Eigen::VectorXd& x) {
    State state = State::from_vector(StateVector(x));
    state.renormalise_attitude();
    x = state.to_vector();
  };
}

MassProperties make_mass(double ixx, double iyy, double izz, double ixz = 0.0) {
  MassProperties mass;
  mass.mass_kg = 4000.0;
  // Aerospace sign convention: the off-diagonal entries are the NEGATIVE
  // products of inertia.
  mass.inertia_cg_body_kg_m2 << ixx, 0.0, -ixz,  //
      0.0, iyy, 0.0,                             //
      -ixz, 0.0, izz;
  return mass;
}

// ===========================================================================
// Case 1: torque-free precession of a symmetric top.
//
// Euler's equations for a body with principal moments I1, I2, I3 and no
// applied moment:
//
//   I1 w1' = (I2 - I3) w2 w3
//   I2 w2' = (I3 - I1) w3 w1
//   I3 w3' = (I1 - I2) w1 w2
//
// Take a body symmetric about its z-axis, I1 = I2 = It, I3 = Ia. The third
// equation gives w3' = 0, so w3 = n is constant. Substituting into the first
// two, with lambda = (Ia - It) n / It:
//
//   w1' = -lambda w2
//   w2' = +lambda w1
//
// which is a rotation of the transverse angular-velocity vector in the BODY
// frame at rate lambda, with constant magnitude:
//
//   w1(t) = A cos(lambda t + phi)
//   w2(t) = A sin(lambda t + phi)
//
// Note the sign: for a prolate body (Ia < It, a "pencil") lambda is negative
// and the precession runs the other way. Both are checked.
// ===========================================================================

struct PrecessionCase {
  const char* name;
  double transverse_inertia;  // It, kg m^2
  double axial_inertia;       // Ia, kg m^2
};

class SymmetricTop : public ::testing::TestWithParam<PrecessionCase> {};

// Worst deviation of the integrated transverse rates from the closed form,
// over a fixed 20 s window, at a given step.
double worst_precession_error(const MassProperties& mass,
                              double transverse,
                              double axial,
                              double spin,
                              double amplitude,
                              double step_s) {
  const double lambda = (axial - transverse) * spin / transverse;

  State initial;
  initial.velocity_body_m_s = Eigen::Vector3d(100.0, 0.0, 0.0);
  initial.attitude_body_to_ned = galata::core::identity_attitude();
  initial.angular_rate_body_rad_s = Eigen::Vector3d(amplitude, 0.0, spin);

  const int steps = static_cast<int>(20.0 / step_s);
  const auto trajectory = galata::numerics::integrate_fixed_step(torque_free(mass),
                                                                 initial.to_vector(),
                                                                 0.0,
                                                                 step_s,
                                                                 steps,
                                                                 steps / 200,
                                                                 renormalise_quaternion());

  double worst = 0.0;
  for (std::size_t i = 0; i < trajectory.states.size(); ++i) {
    const double t = trajectory.times_s[i];
    const State state = State::from_vector(StateVector(trajectory.states[i]));
    worst = std::fmax(
        worst, std::fabs(state.angular_rate_body_rad_s.x() - amplitude * std::cos(lambda * t)));
    worst = std::fmax(
        worst, std::fabs(state.angular_rate_body_rad_s.y() - amplitude * std::sin(lambda * t)));
    // The axial rate is exactly constant in the closed form.
    EXPECT_NEAR(state.angular_rate_body_rad_s.z(), spin, 1e-11) << "at t = " << t;
  }
  return worst;
}

TEST_P(SymmetricTop, ConvergesToTheClosedFormPrecessionAtFourthOrder) {
  const PrecessionCase& parameters = GetParam();
  const double transverse = parameters.transverse_inertia;
  const double axial = parameters.axial_inertia;

  const MassProperties mass = make_mass(transverse, transverse, axial);
  ASSERT_NO_THROW(mass.validate());

  const double spin = 2.0;        // n,  rad/s about the symmetry axis
  const double amplitude = 0.35;  // A,  rad/s transverse
  const double lambda = (axial - transverse) * spin / transverse;

  // Several precession cycles in the window, so this checks phase and not
  // merely amplitude.
  EXPECT_GT(std::fabs(lambda) * 20.0 / (2.0 * M_PI), 1.0)
      << "case does not complete a precession cycle";

  // Two steps rather than one very small one. Checking that the error falls
  // like h^4 as the step halves is a far stronger statement than checking that
  // it is small at one step: a solution that converged to the WRONG closed form
  // would sit at a small constant error and pass an absolute check, while
  // failing this one outright.
  const double coarse = worst_precession_error(mass, transverse, axial, spin, amplitude, 2e-3);
  const double fine = worst_precession_error(mass, transverse, axial, spin, amplitude, 1e-3);

  EXPECT_LT(coarse, 1e-5) << parameters.name << ": worst error at h = 2 ms";
  EXPECT_LT(fine, 1e-6) << parameters.name << ": worst error at h = 1 ms";

  const double ratio = coarse / fine;
  EXPECT_GT(ratio, 10.0) << parameters.name << ": error ratio " << ratio
                         << " on halving the step is too small for fourth order — the "
                            "integrated solution is not converging to the closed form";
  EXPECT_LT(ratio, 24.0) << parameters.name << ": error ratio " << ratio << " is implausible";
}

TEST_P(SymmetricTop, TransverseMagnitudeIsConserved) {
  const PrecessionCase& parameters = GetParam();
  const MassProperties mass = make_mass(
      parameters.transverse_inertia, parameters.transverse_inertia, parameters.axial_inertia);

  State initial;
  initial.angular_rate_body_rad_s = Eigen::Vector3d(0.35, 0.0, 2.0);
  const double expected = std::hypot(0.35, 0.0);

  const auto trajectory = galata::numerics::integrate_fixed_step(
      torque_free(mass), initial.to_vector(), 0.0, 1e-3, 20000, 100, renormalise_quaternion());

  for (const Eigen::VectorXd& raw : trajectory.states) {
    const State state = State::from_vector(StateVector(raw));
    EXPECT_NEAR(std::hypot(state.angular_rate_body_rad_s.x(), state.angular_rate_body_rad_s.y()),
                expected,
                1e-9);
  }
}

INSTANTIATE_TEST_SUITE_P(Shapes,
                         SymmetricTop,
                         ::testing::Values(
                             // Oblate: axial moment largest, like a discus. lambda > 0.
                             PrecessionCase{"oblate", 12000.0, 20000.0},
                             // Prolate: axial moment smallest, like a rocket. lambda < 0, so the
                             // precession runs the other way round the body.
                             PrecessionCase{"prolate", 20000.0, 8000.0}),
                         [](const ::testing::TestParamInfo<PrecessionCase>& info) {
                           return std::string(info.param.name);
                         });

// ===========================================================================
// Case 2: conservation laws under torque-free motion.
// ===========================================================================

TEST(TorqueFreeConservation, EnergyAndAngularMomentumDriftIsBounded) {
  // Asymmetric body, so this exercises the general tensor rather than the
  // special case the closed form covers. A non-zero product of inertia is
  // included deliberately: I_xz is not zero for real aircraft, and code that
  // silently assumed it was would still pass the symmetric-top cases.
  const MassProperties mass = make_mass(12000.0, 40000.0, 48000.0, 1500.0);
  ASSERT_NO_THROW(mass.validate());

  State initial;
  initial.attitude_body_to_ned = galata::core::quaternion_from_euler({0.3, -0.2, 1.1});
  initial.angular_rate_body_rad_s = Eigen::Vector3d(0.9, -0.4, 0.6);

  const double energy_0 = galata::sim::rotational_kinetic_energy(initial, mass);
  const Eigen::Vector3d momentum_0 = galata::sim::angular_momentum_ned(initial, mass);

  const double step = 1e-3;
  const int steps = 60000;  // 60 s
  const auto trajectory = galata::numerics::integrate_fixed_step(
      torque_free(mass), initial.to_vector(), 0.0, step, steps, 100, renormalise_quaternion());

  double worst_energy_drift = 0.0;
  double worst_momentum_drift = 0.0;
  for (const Eigen::VectorXd& raw : trajectory.states) {
    const State state = State::from_vector(StateVector(raw));
    worst_energy_drift = std::fmax(
        worst_energy_drift,
        std::fabs(galata::sim::rotational_kinetic_energy(state, mass) - energy_0) / energy_0);
    // The angular momentum VECTOR is constant in NED, not in body axes. This is
    // a much stronger statement than magnitude conservation: it checks the
    // attitude kinematics and the rotational dynamics against each other.
    worst_momentum_drift = std::fmax(
        worst_momentum_drift,
        (galata::sim::angular_momentum_ned(state, mass) - momentum_0).norm() / momentum_0.norm());
  }

  // Bounds, not equalities: RK4 is not symplectic, so energy drifts secularly
  // rather than oscillating. The bound is set an order of magnitude above the
  // drift actually measured, so it fails on a real regression rather than on
  // the last bit.
  EXPECT_LT(worst_energy_drift, 1e-11) << "relative energy drift";
  EXPECT_LT(worst_momentum_drift, 1e-11) << "relative angular-momentum drift";
}

TEST(TorqueFreeConservation, AngularMomentumRotatesInBodyAxesButNotInNed) {
  // The other half of the previous test's claim: if the body-frame momentum
  // were also constant, the attitude kinematics would not be coupled to the
  // rotational dynamics at all, and both tests would pass with a broken DCM.
  const MassProperties mass = make_mass(12000.0, 40000.0, 48000.0, 1500.0);

  State initial;
  initial.angular_rate_body_rad_s = Eigen::Vector3d(0.9, -0.4, 0.6);
  const Eigen::Vector3d body_momentum_0 = galata::sim::angular_momentum_body(initial, mass);

  const auto trajectory = galata::numerics::integrate_fixed_step(
      torque_free(mass), initial.to_vector(), 0.0, 1e-3, 5000, 5000, renormalise_quaternion());

  const State final_state = State::from_vector(StateVector(trajectory.states.back()));
  const Eigen::Vector3d body_momentum_end = galata::sim::angular_momentum_body(final_state, mass);

  EXPECT_GT((body_momentum_end - body_momentum_0).norm(), 0.1 * body_momentum_0.norm())
      << "angular momentum did not move in body axes, so the body is not actually tumbling";
  EXPECT_NEAR(body_momentum_end.norm(), body_momentum_0.norm(), 1e-9 * body_momentum_0.norm());
}

// ===========================================================================
// Case 3: the intermediate-axis theorem (the Dzhanibekov effect).
//
// Linearising Euler's equations about steady rotation at rate W about axis 2,
// with a small perturbation (e1, W, e3):
//
//   I1 e1' = (I2 - I3) W e3
//   I3 e3' = (I1 - I2) e1 W
//
// so  e1'' = W^2 (I2 - I3)(I1 - I2) / (I1 I3) * e1.
//
// For I1 < I2 < I3 both factors are negative, the coefficient is positive, and
// the perturbation grows as exp(sigma t) with
//
//   sigma = W sqrt( (I3 - I2)(I2 - I1) / (I1 I3) ).
//
// For rotation about the major or minor axis the coefficient is negative and
// the perturbation oscillates instead. All three are checked.
//
// The initial condition matters, and it is easy to get wrong. Starting from
// (a, W, 0) gives e3(0) = 0, hence e1'(0) = 0, hence
//
//   e1(t) = a cosh(sigma t)          NOT  a exp(sigma t)
//   e3(t) = a sigma I1 sinh(sigma t) / ((I2 - I3) W)
//
// cosh only approaches exp(sigma t)/2 asymptotically, so fitting a log-slope to
// a window that starts near t = 0 measures a rate well below sigma — about
// 0.70 sigma for a window of 0.375/sigma to 1.5/sigma. This test originally did
// exactly that and reported 0.436 against a predicted 0.624, which is the cosh
// factor and not a defect in the dynamics. Asserting the closed form pointwise
// avoids the whole question.
// ===========================================================================

TEST(IntermediateAxis, PerturbationFollowsTheClosedFormHyperbolicGrowth) {
  const double i1 = 12000.0;
  const double i2 = 40000.0;
  const double i3 = 48000.0;
  const MassProperties mass = make_mass(i1, i2, i3);

  const double spin = 1.0;           // W, rad/s about the intermediate axis
  const double perturbation = 1e-6;  // rad/s
  const double sigma = spin * std::sqrt((i3 - i2) * (i2 - i1) / (i1 * i3));

  State initial;
  initial.angular_rate_body_rad_s = Eigen::Vector3d(perturbation, spin, 0.0);

  // Run to sigma*t = 4, where cosh has grown by a factor of 27 — enough dynamic
  // range that a wrong rate is unmissable — while the perturbation is still
  // 3e-5 rad/s against a 1 rad/s spin, so the linearisation remains excellent.
  const double window = 4.0 / sigma;
  const double step = 1e-3;
  const int steps = static_cast<int>(window / step);

  const auto trajectory = galata::numerics::integrate_fixed_step(torque_free(mass),
                                                                 initial.to_vector(),
                                                                 0.0,
                                                                 step,
                                                                 steps,
                                                                 steps / 40,
                                                                 renormalise_quaternion());

  // The coefficient relating e3 to sinh. (I2 - I3) is negative here, so e3
  // grows NEGATIVE while e1 grows positive — a sign the closed form pins down
  // and a transposed Euler equation would get backwards.
  const double e3_coefficient = perturbation * sigma * i1 / ((i2 - i3) * spin);
  EXPECT_LT(e3_coefficient, 0.0);

  double worst_e1_relative = 0.0;
  double worst_e3_relative = 0.0;
  for (std::size_t i = 0; i < trajectory.states.size(); ++i) {
    const double t = trajectory.times_s[i];
    const State state = State::from_vector(StateVector(trajectory.states[i]));

    const double expected_e1 = perturbation * std::cosh(sigma * t);
    const double expected_e3 = e3_coefficient * std::sinh(sigma * t);

    worst_e1_relative = std::fmax(
        worst_e1_relative,
        std::fabs(state.angular_rate_body_rad_s.x() - expected_e1) / std::fabs(expected_e1));
    if (std::fabs(expected_e3) > 1e-12) {
      worst_e3_relative = std::fmax(
          worst_e3_relative,
          std::fabs(state.angular_rate_body_rad_s.z() - expected_e3) / std::fabs(expected_e3));
    }
    // The spin rate itself is very nearly constant at second order.
    EXPECT_NEAR(state.angular_rate_body_rad_s.y(), spin, 1e-8) << "at t = " << t;
  }

  EXPECT_LT(worst_e1_relative, 1e-4)
      << "transverse perturbation departs from a cosh: worst relative error " << worst_e1_relative;
  EXPECT_LT(worst_e3_relative, 1e-4)
      << "axial perturbation departs from a sinh: worst relative error " << worst_e3_relative;

  // And it really did grow: the whole point of the intermediate axis.
  const State final_state = State::from_vector(StateVector(trajectory.states.back()));
  EXPECT_GT(final_state.angular_rate_body_rad_s.x(), 20.0 * perturbation);
}

TEST(IntermediateAxis, RotationAboutTheMajorAndMinorAxesIsStable) {
  const double i1 = 12000.0;
  const double i2 = 40000.0;
  const double i3 = 48000.0;
  const MassProperties mass = make_mass(i1, i2, i3);

  const double spin = 1.0;
  const double perturbation = 1e-6;

  // Minor axis (smallest moment) and major axis (largest moment): both stable,
  // the perturbation oscillates and stays bounded.
  const Eigen::Vector3d minor_axis(spin, perturbation, 0.0);
  const Eigen::Vector3d major_axis(perturbation, 0.0, spin);

  for (const Eigen::Vector3d& omega : {minor_axis, major_axis}) {
    State initial;
    initial.angular_rate_body_rad_s = omega;

    const auto trajectory = galata::numerics::integrate_fixed_step(
        torque_free(mass), initial.to_vector(), 0.0, 1e-3, 30000, 100, renormalise_quaternion());

    double worst = 0.0;
    for (const Eigen::VectorXd& raw : trajectory.states) {
      const State state = State::from_vector(StateVector(raw));
      // Whichever components started small must stay small.
      const Eigen::Vector3d deviation = state.angular_rate_body_rad_s - omega.normalized() * spin;
      worst = std::fmax(worst, deviation.norm());
    }
    // Bounded by a few times the initial perturbation, not growing.
    EXPECT_LT(worst, 10.0 * perturbation)
        << "perturbation grew about a stable axis: worst deviation " << worst;
  }
}

// ===========================================================================
// Case 4: the mass-property guard rails.
// ===========================================================================

TEST(MassProperties, RejectsTensorsNoRigidBodyCouldHave) {
  MassProperties mass;
  mass.mass_kg = 1000.0;

  // Asymmetric: a product of inertia entered on one side only.
  mass.inertia_cg_body_kg_m2 << 100.0, 5.0, 0.0, -5.0, 200.0, 0.0, 0.0, 0.0, 250.0;
  EXPECT_THROW(mass.validate(), std::invalid_argument);

  // Indefinite.
  mass.inertia_cg_body_kg_m2 << -100.0, 0.0, 0.0, 0.0, 200.0, 0.0, 0.0, 0.0, 250.0;
  EXPECT_THROW(mass.validate(), std::invalid_argument);

  // Symmetric and positive definite, but violating the triangle inequality:
  // no distribution of mass produces these three principal moments. This is
  // the one that catches a moment quoted about the wrong axis.
  mass.inertia_cg_body_kg_m2 << 100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 500.0;
  EXPECT_THROW(mass.validate(), std::invalid_argument);

  // Non-positive mass.
  mass.inertia_cg_body_kg_m2 << 12000.0, 0.0, -1500.0, 0.0, 40000.0, 0.0, -1500.0, 0.0, 48000.0;
  mass.mass_kg = 0.0;
  EXPECT_THROW(mass.validate(), std::invalid_argument);

  // A realistic transport-aircraft tensor, with I_xz, passes.
  mass.mass_kg = 60000.0;
  EXPECT_NO_THROW(mass.validate());
}

TEST(WrenchTransfer, MovingTheReferencePointAddsTheExpectedMoment) {
  // A pure upward force applied a metre ahead of the CG must produce a
  // nose-up pitching moment. In FRD, "up" is -z and "ahead" is +x, and
  // r x F = (1,0,0) x (0,0,-F) = (0, F, 0)... which is POSITIVE about y.
  //
  // Positive body-y moment is nose-UP in this frame, so the expected sign is
  // positive. Writing that reasoning out is the point of this test: the sign
  // of a pitching moment from a CG offset is the single easiest thing to get
  // backwards, and it inverts static margin.
  Wrench at_point;
  at_point.force_body_n = Eigen::Vector3d(0.0, 0.0, -1000.0);  // 1000 N upward

  const Eigen::Vector3d cg_to_point(1.0, 0.0, 0.0);  // one metre forward of the CG
  const Wrench at_cg = galata::sim::moved_to_cg(at_point, cg_to_point);

  EXPECT_TRUE(at_cg.force_body_n.isApprox(at_point.force_body_n));
  EXPECT_DOUBLE_EQ(at_cg.moment_cg_body_n_m.x(), 0.0);
  EXPECT_DOUBLE_EQ(at_cg.moment_cg_body_n_m.y(), 1000.0);
  EXPECT_DOUBLE_EQ(at_cg.moment_cg_body_n_m.z(), 0.0);

  // Applied at the CG, there is no transfer moment at all.
  const Wrench at_origin = galata::sim::moved_to_cg(at_point, Eigen::Vector3d::Zero());
  EXPECT_TRUE(at_origin.moment_cg_body_n_m.isZero());
}

TEST(Gravity, PointsDownInNedAndRotatesCorrectlyIntoBodyAxes) {
  const Eigen::Vector3d gravity_ned(0.0, 0.0, 9.80665);

  // Wings level, nose level: gravity is entirely along body z (downward).
  const Eigen::Vector3d level =
      galata::sim::gravity_body(galata::core::identity_attitude(), gravity_ned);
  EXPECT_NEAR(level.x(), 0.0, 1e-12);
  EXPECT_NEAR(level.y(), 0.0, 1e-12);
  EXPECT_NEAR(level.z(), 9.80665, 1e-12);

  // Nose up 30 degrees: gravity acquires a component along NEGATIVE body x,
  // i.e. it decelerates the aircraft. A sign error here shows up as an
  // aircraft that accelerates when it climbs.
  const Eigen::Vector3d climbing = galata::sim::gravity_body(
      galata::core::quaternion_from_euler({0.0, 30.0 * M_PI / 180.0, 0.0}), gravity_ned);
  EXPECT_LT(climbing.x(), 0.0);
  EXPECT_NEAR(climbing.x(), -9.80665 * std::sin(30.0 * M_PI / 180.0), 1e-9);
  EXPECT_NEAR(climbing.z(), 9.80665 * std::cos(30.0 * M_PI / 180.0), 1e-9);

  // Right wing down 90 degrees: gravity is along the POSITIVE body y axis.
  const Eigen::Vector3d rolled = galata::sim::gravity_body(
      galata::core::quaternion_from_euler({90.0 * M_PI / 180.0, 0.0, 0.0}), gravity_ned);
  EXPECT_NEAR(rolled.y(), 9.80665, 1e-9);
  EXPECT_NEAR(rolled.z(), 0.0, 1e-9);
}

TEST(FreeFall, AccelerationIsExactlyGravityWithNoAppliedForce) {
  // The simplest possible check that the translational equation has the right
  // sign and magnitude, and that the transport term vanishes when it should.
  const MassProperties mass = make_mass(12000.0, 40000.0, 48000.0);
  State state;
  state.velocity_body_m_s = Eigen::Vector3d(120.0, 0.0, 0.0);
  state.attitude_body_to_ned = galata::core::identity_attitude();

  const StateVector derivative =
      galata::sim::rigid_body_derivative(state, mass, Wrench{}, Eigen::Vector3d(0.0, 0.0, 9.80665));

  EXPECT_NEAR(derivative(galata::core::kVelocityU), 0.0, 1e-12);
  EXPECT_NEAR(derivative(galata::core::kVelocityW), 9.80665, 1e-12);
  // Level flight heading north: the position derivative is the airspeed north.
  EXPECT_NEAR(derivative(galata::core::kPositionNorth), 120.0, 1e-12);
  EXPECT_NEAR(derivative(galata::core::kPositionDown), 0.0, 1e-12);
}

}  // namespace
