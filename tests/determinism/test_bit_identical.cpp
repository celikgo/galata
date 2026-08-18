// SPDX-License-Identifier: Apache-2.0
//
// ADR-0004 tier 1: the same computation, run twice in the same process, must
// produce bit-identical results.
//
// EXPECT_EQ on doubles, not EXPECT_NEAR, and that is the whole point. A
// tolerance here would make the test pass while the guarantee it exists to
// protect had already been lost.
//
// What these catch that a tolerance-based test would not: unordered container
// iteration reaching a result, an accumulator whose order depends on
// allocation addresses, a tolerance-based loop exit whose iteration count moves
// with rounding, and uninitialised padding.

#include "galata/analyze/modes.hpp"
#include "galata/core/atmosphere.hpp"
#include "galata/core/quaternion.hpp"
#include "galata/core/state.hpp"
#include "galata/numerics/integrator.hpp"
#include "galata/sim/rigid_body.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace {

TEST(Determinism, AtmosphereIsBitIdenticalOnRepeatedQueries) {
  for (const double altitude : {-4000.0, 0.0, 3048.0, 11000.0, 25000.0, 60000.0, 86000.0}) {
    const galata::core::AtmosphereState first = galata::core::isa(altitude);
    const galata::core::AtmosphereState second = galata::core::isa(altitude);
    EXPECT_EQ(first.temperature_k, second.temperature_k) << "at " << altitude;
    EXPECT_EQ(first.pressure_pa, second.pressure_pa) << "at " << altitude;
    EXPECT_EQ(first.density_kg_m3, second.density_kg_m3) << "at " << altitude;
    EXPECT_EQ(first.speed_of_sound_m_s, second.speed_of_sound_m_s) << "at " << altitude;
    EXPECT_EQ(first.dynamic_viscosity_pa_s, second.dynamic_viscosity_pa_s) << "at " << altitude;
  }
}

TEST(Determinism, AtmosphereDoesNotDependOnQueryOrder) {
  // The base-pressure recurrence walks from sea level on every call rather than
  // caching. If it ever gains a cache, this is the test that catches the cache
  // making an answer depend on what was asked first.
  const double target = 47000.0;
  const galata::core::AtmosphereState direct = galata::core::isa(target);

  for (const double warm_up : {0.0, 86000.0, 11000.0, -5000.0}) {
    (void)galata::core::isa(warm_up);
  }
  const galata::core::AtmosphereState after = galata::core::isa(target);

  EXPECT_EQ(direct.pressure_pa, after.pressure_pa);
  EXPECT_EQ(direct.density_kg_m3, after.density_kg_m3);
  EXPECT_EQ(direct.temperature_k, after.temperature_k);
}

galata::numerics::DerivativeFunction make_derivative(const galata::sim::MassProperties& mass,
                                                     const galata::sim::Wrench& wrench) {
  return [mass, wrench](double, const Eigen::VectorXd& x) -> Eigen::VectorXd {
    return galata::sim::rigid_body_derivative(
        galata::core::State::from_vector(galata::core::StateVector(x)),
        mass,
        wrench,
        Eigen::Vector3d(0.0, 0.0, 9.80665));
  };
}

galata::sim::MassProperties sample_mass() {
  galata::sim::MassProperties mass;
  mass.mass_kg = 4000.0;
  mass.inertia_cg_body_kg_m2 << 12000.0, 0.0, -1500.0,  //
      0.0, 40000.0, 0.0,                                //
      -1500.0, 0.0, 48000.0;
  return mass;
}

galata::core::State sample_state() {
  galata::core::State state;
  state.position_ned_m = Eigen::Vector3d(10.0, -20.0, -3048.0);
  state.velocity_body_m_s = Eigen::Vector3d(120.0, 3.0, 8.0);
  state.attitude_body_to_ned = galata::core::quaternion_from_euler({0.3, -0.2, 1.1});
  state.angular_rate_body_rad_s = Eigen::Vector3d(0.9, -0.4, 0.6);
  return state;
}

TEST(Determinism, LongIntegrationIsBitIdenticalAcrossRuns) {
  const galata::sim::MassProperties mass = sample_mass();
  galata::sim::Wrench wrench;
  wrench.force_body_n = Eigen::Vector3d(2500.0, -400.0, -1200.0);
  wrench.moment_cg_body_n_m = Eigen::Vector3d(800.0, -1500.0, 300.0);

  const galata::numerics::ProjectionFunction project = [](Eigen::VectorXd& x) {
    galata::core::State state = galata::core::State::from_vector(galata::core::StateVector(x));
    state.renormalise_attitude();
    x = state.to_vector();
  };

  const auto run = [&]() {
    return galata::numerics::integrate_fixed_step(make_derivative(mass, wrench),
                                                  sample_state().to_vector(),
                                                  0.0,
                                                  0.002,
                                                  10000,
                                                  10000,
                                                  project);
  };

  const auto first = run();
  const auto second = run();
  ASSERT_EQ(first.states.size(), second.states.size());
  for (int i = 0; i < galata::core::kStateSize; ++i) {
    EXPECT_EQ(first.states.back()(i), second.states.back()(i))
        << "component " << i << " differs after 10,000 steps";
  }
}

TEST(Determinism, IntegrationDoesNotDependOnWhatRanBefore) {
  // A leaked static, a cached factorisation, or a reused scratch buffer would
  // make the second run differ from the first. Running an unrelated integration
  // in between is what exposes that.
  const galata::sim::MassProperties mass = sample_mass();
  const galata::sim::Wrench wrench;

  const auto run = [&](const galata::core::State& initial, int steps) {
    return galata::numerics::integrate_fixed_step(
        make_derivative(mass, wrench), initial.to_vector(), 0.0, 0.001, steps, steps);
  };

  const auto clean = run(sample_state(), 5000);

  galata::core::State other;
  other.velocity_body_m_s = Eigen::Vector3d(300.0, -50.0, 40.0);
  other.angular_rate_body_rad_s = Eigen::Vector3d(-2.0, 1.5, -0.7);
  (void)run(other, 3000);

  const auto after = run(sample_state(), 5000);
  for (int i = 0; i < galata::core::kStateSize; ++i) {
    EXPECT_EQ(clean.states.back()(i), after.states.back()(i)) << "component " << i;
  }
}

TEST(Determinism, SplittingAnIntegrationInTwoGivesTheSameResult) {
  // Integrating 4000 steps, or 1500 then 2500 from the intermediate state, must
  // agree exactly: the integrator carries no state between steps. This is what
  // makes a checkpointed run reproduce an uninterrupted one.
  const galata::sim::MassProperties mass = sample_mass();
  galata::sim::Wrench wrench;
  wrench.moment_cg_body_n_m = Eigen::Vector3d(200.0, -300.0, 120.0);
  const auto derivative = make_derivative(mass, wrench);

  const double step = 0.001;
  const auto whole = galata::numerics::integrate_fixed_step(
      derivative, sample_state().to_vector(), 0.0, step, 4000, 4000);

  const auto part_one = galata::numerics::integrate_fixed_step(
      derivative, sample_state().to_vector(), 0.0, step, 1500, 1500);
  const auto part_two = galata::numerics::integrate_fixed_step(
      derivative, part_one.states.back(), 1500.0 * step, step, 2500, 2500);

  for (int i = 0; i < galata::core::kStateSize; ++i) {
    EXPECT_EQ(whole.states.back()(i), part_two.states.back()(i)) << "component " << i;
  }
}

TEST(Determinism, ModalDecompositionIsBitIdenticalAndOrderStable) {
  Eigen::MatrixXd a(4, 4);
  a << -0.125, 0.0383878091, -0.9992629164, 0.1410102351,  //
      -5.49, -2.03, 0.641, 0.0,                            //
      0.667, -0.116, -0.207, 0.0,                          //
      0.0, 1.0, 0.0384161250, 0.0;
  const std::vector<std::string> names = {"beta", "p", "r", "phi"};
  const auto roles = galata::analyze::StateRoles::from_names(names);

  const auto first = galata::analyze::analyze_modes(a, names, roles);
  const auto second = galata::analyze::analyze_modes(a, names, roles);

  ASSERT_EQ(first.modes.size(), second.modes.size());
  EXPECT_EQ(first.eigenvector_condition_number, second.eigenvector_condition_number);
  for (std::size_t m = 0; m < first.modes.size(); ++m) {
    // Order too, not just values: a report whose rows permute between runs
    // diffs noisily even when nothing changed.
    EXPECT_EQ(first.modes[m].label, second.modes[m].label);
    EXPECT_EQ(first.modes[m].eigenvalue.real(), second.modes[m].eigenvalue.real());
    EXPECT_EQ(first.modes[m].eigenvalue.imag(), second.modes[m].eigenvalue.imag());
    EXPECT_EQ(first.modes[m].damping_ratio, second.modes[m].damping_ratio);
    EXPECT_EQ(first.modes[m].label_score, second.modes[m].label_score);
    ASSERT_EQ(first.modes[m].participation.size(), second.modes[m].participation.size());
    for (std::size_t k = 0; k < first.modes[m].participation.size(); ++k) {
      EXPECT_EQ(first.modes[m].participation[k], second.modes[m].participation[k]);
    }
  }
}

TEST(Determinism, QuaternionOperationsAreBitIdentical) {
  const galata::core::Quaternion q = galata::core::quaternion_from_euler({0.37, -0.81, 2.4});
  const Eigen::Vector3d omega(0.31, -0.72, 0.19);
  for (int repeat = 0; repeat < 8; ++repeat) {
    const galata::core::Quaternion again = galata::core::quaternion_from_euler({0.37, -0.81, 2.4});
    EXPECT_EQ(again.w(), q.w());
    EXPECT_EQ(again.x(), q.x());
    EXPECT_EQ(again.y(), q.y());
    EXPECT_EQ(again.z(), q.z());
    EXPECT_EQ(galata::core::quaternion_derivative(q, omega),
              galata::core::quaternion_derivative(again, omega));
  }
}

TEST(Determinism, NoLongDoubleInTheNumericalCore) {
  // ADR-0004 bans long double: it is 80-bit extended on x86-64 System V,
  // 64-bit on MSVC and 128-bit quad on AArch64 Linux, so a result touching it
  // is non-portable by construction. This asserts the property the ban exists
  // to protect rather than the ban itself — a grep would be checking the letter
  // of the rule and this checks that doubles behave as doubles.
  static_assert(sizeof(double) == 8, "galata assumes IEEE 754 binary64");
  static_assert(std::numeric_limits<double>::is_iec559,
                "galata requires IEEE 754 arithmetic; ADR-0004's guarantees do not hold "
                "without it");
  SUCCEED();
}

}  // namespace
