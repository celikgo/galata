// SPDX-License-Identifier: Apache-2.0
//
// State-vector invariants over sampled states.

#include "galata/core/constants.hpp"
#include "galata/core/quaternion.hpp"
#include "galata/core/state.hpp"

#include "property_generators.hpp"
#include <gtest/gtest.h>

#include <cmath>

namespace {

using galata::core::State;
using galata::testing::Generator;
using galata::testing::kPropertySamples;

constexpr std::uint64_t kSeed = 0x165667B19E3779F9ULL;

TEST(StateProperties, PackingRoundTripsBitwiseExactly) {
  // Bitwise, not approximately. Packing is a copy; anything less than exact
  // means a component is being recomputed rather than moved, and a state that
  // changes by an ulp when it is written out and read back destroys the
  // bit-identity guarantee in ADR-0004.
  Generator gen(kSeed);
  for (int i = 0; i < kPropertySamples; ++i) {
    const State original = gen.state();
    const galata::core::StateVector packed = original.to_vector();
    const galata::core::StateVector repacked = State::from_vector(packed).to_vector();
    for (int k = 0; k < galata::core::kStateSize; ++k) {
      EXPECT_EQ(repacked(k), packed(k))
          << "seed " << gen.seed() << " sample " << i << " component " << k;
    }
  }
}

TEST(StateProperties, AttitudeRotationPreservesSpeed) {
  // ||v_ned|| == ||v_body|| for every attitude: a rotation cannot change a
  // magnitude. Catches a DCM that has stopped being orthonormal.
  Generator gen(kSeed + 1);
  for (int i = 0; i < kPropertySamples; ++i) {
    const State s = gen.state();
    const double body_speed = galata::core::airspeed(s.velocity_body_m_s);
    EXPECT_NEAR(galata::core::velocity_ned(s).norm(), body_speed, 1e-10 * (body_speed + 1.0))
        << "seed " << gen.seed() << " sample " << i;
  }
}

TEST(StateProperties, WindAxisConversionRoundTripsForAnyVelocity) {
  Generator gen(kSeed + 2);
  for (int i = 0; i < kPropertySamples; ++i) {
    const Eigen::Vector3d velocity = gen.vector3(300.0);
    if (velocity.norm() < 1e-6) {
      continue;  // alpha and beta are conventions, not values, at zero speed
    }
    const Eigen::Vector3d recovered =
        galata::core::from_wind_axes(galata::core::to_wind_axes(velocity));
    EXPECT_TRUE(recovered.isApprox(velocity, 1e-10))
        << "seed " << gen.seed() << " sample " << i << " v = " << velocity.transpose();
  }
}

TEST(StateProperties, AerodynamicAnglesStayInTheirDocumentedRanges) {
  Generator gen(kSeed + 3);
  for (int i = 0; i < kPropertySamples; ++i) {
    const Eigen::Vector3d velocity = gen.vector3(300.0);
    EXPECT_LE(std::fabs(galata::core::angle_of_attack(velocity)), galata::core::kPi + 1e-12);
    EXPECT_LE(std::fabs(galata::core::sideslip_angle(velocity)), galata::core::kHalfPi + 1e-12);
    EXPECT_GE(galata::core::airspeed(velocity), 0.0);
  }
}

TEST(StateProperties, FlightPathAngleStaysWithinAQuarterTurn) {
  Generator gen(kSeed + 4);
  for (int i = 0; i < kPropertySamples; ++i) {
    EXPECT_LE(std::fabs(galata::core::flight_path_angle(gen.state())),
              galata::core::kHalfPi + 1e-12);
  }
}

TEST(StateProperties, RenormalisationDoesNotDrift) {
  // Renormalisation runs once per integrator step, so what matters is not that
  // it is a bitwise fixed point — it is not, and cannot be — but that repeated
  // application does not accumulate.
  //
  // It is not a fixed point because normalize() divides by a norm that is
  // itself only within an ulp of 1, so the division perturbs the mantissa. One
  // application moves an already-unit quaternion by around 1e-16 radians. The
  // question the integrator cares about is whether that perturbation is a
  // random walk, which over a million steps would be visible, or a bounded
  // oscillation about the representable unit sphere, which is invisible.
  //
  // It is bounded. Asserted here over many applications, which is the form a
  // long trajectory actually stresses.
  constexpr int kApplications = 200;
  constexpr double kOneApplicationBound = 1e-15;  // rad, a few ulp

  Generator gen(kSeed + 5);
  for (int i = 0; i < kPropertySamples / 10; ++i) {
    State s = gen.state();
    s.renormalise_attitude();
    const galata::core::Quaternion after_first = s.attitude_body_to_ned;

    for (int k = 0; k < kApplications; ++k) {
      s.renormalise_attitude();
      EXPECT_NEAR(s.attitude_body_to_ned.norm(), 1.0, 1e-15)
          << "seed " << gen.seed() << " sample " << i << " application " << k;
    }

    // The total excursion after 200 applications is bounded by what a couple of
    // applications cost. If this were a random walk it would grow like
    // sqrt(200) ~ 14x and this bound would not hold.
    EXPECT_LT(galata::core::angular_distance(after_first, s.attitude_body_to_ned),
              kOneApplicationBound)
        << "seed " << gen.seed() << " sample " << i << ": renormalisation drifts";
  }
}

TEST(StateProperties, SpeedIsUnchangedByAttitude) {
  // The body-axis velocity components define airspeed; the attitude cannot
  // affect it. Stated as a property because it is the invariant that would
  // break if velocity were ever silently stored in NED.
  Generator gen(kSeed + 6);
  for (int i = 0; i < kPropertySamples; ++i) {
    State s = gen.state();
    const double before = galata::core::airspeed(s.velocity_body_m_s);
    s.attitude_body_to_ned = gen.rotation();
    EXPECT_EQ(galata::core::airspeed(s.velocity_body_m_s), before);
  }
}

}  // namespace
