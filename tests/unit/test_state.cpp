// SPDX-License-Identifier: Apache-2.0
//
// State vector packing and derived quantities (ADR-0002).
//
// The packing tests are not busywork: the packed order IS the row and column
// order of every exported A and B matrix, so a permutation here silently
// relabels every state-space model the tool produces.

#include "galata/core/quaternion.hpp"
#include "galata/core/state.hpp"
#include "galata/units.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using galata::core::EulerAngles;
using galata::core::State;
using galata::core::StateVector;
using galata::units::degrees_to_radians;

constexpr double kTight = 1e-14;
constexpr double kLoose = 1e-12;

State sample_state() {
  State s;
  s.position_ned_m = Eigen::Vector3d(1000.0, -250.0, -3048.0);
  s.velocity_body_m_s = Eigen::Vector3d(180.0, 3.0, 12.0);
  s.attitude_body_to_ned = galata::core::quaternion_from_euler(
      EulerAngles{degrees_to_radians(5.0), degrees_to_radians(3.5), degrees_to_radians(275.0)});
  s.angular_rate_body_rad_s = Eigen::Vector3d(0.01, -0.02, 0.03);
  return s;
}

TEST(State, PackedOrderMatchesTheDocumentedContract) {
  const State s = sample_state();
  const StateVector x = s.to_vector();

  ASSERT_EQ(x.size(), 13);
  EXPECT_DOUBLE_EQ(x(galata::core::kPositionNorth), 1000.0);
  EXPECT_DOUBLE_EQ(x(galata::core::kPositionEast), -250.0);
  EXPECT_DOUBLE_EQ(x(galata::core::kPositionDown), -3048.0);
  EXPECT_DOUBLE_EQ(x(galata::core::kVelocityU), 180.0);
  EXPECT_DOUBLE_EQ(x(galata::core::kVelocityV), 3.0);
  EXPECT_DOUBLE_EQ(x(galata::core::kVelocityW), 12.0);
  EXPECT_DOUBLE_EQ(x(galata::core::kQuaternionW), s.attitude_body_to_ned.w());
  EXPECT_DOUBLE_EQ(x(galata::core::kQuaternionX), s.attitude_body_to_ned.x());
  EXPECT_DOUBLE_EQ(x(galata::core::kQuaternionY), s.attitude_body_to_ned.y());
  EXPECT_DOUBLE_EQ(x(galata::core::kQuaternionZ), s.attitude_body_to_ned.z());
  EXPECT_DOUBLE_EQ(x(galata::core::kRateP), 0.01);
  EXPECT_DOUBLE_EQ(x(galata::core::kRateQ), -0.02);
  EXPECT_DOUBLE_EQ(x(galata::core::kRateR), 0.03);
}

TEST(State, PackingRoundTripsExactly) {
  const State original = sample_state();
  const State recovered = State::from_vector(original.to_vector());

  EXPECT_TRUE(recovered.position_ned_m.isApprox(original.position_ned_m, kTight));
  EXPECT_TRUE(recovered.velocity_body_m_s.isApprox(original.velocity_body_m_s, kTight));
  EXPECT_TRUE(recovered.angular_rate_body_rad_s.isApprox(original.angular_rate_body_rad_s, kTight));
  EXPECT_NEAR(
      galata::core::angular_distance(recovered.attitude_body_to_ned, original.attitude_body_to_ned),
      0.0,
      kTight);
}

TEST(State, AirspeedIsTheNormOfBodyVelocity) {
  EXPECT_DOUBLE_EQ(galata::core::airspeed(Eigen::Vector3d(3.0, 4.0, 0.0)), 5.0);
  EXPECT_DOUBLE_EQ(galata::core::airspeed(Eigen::Vector3d(0.0, 0.0, 0.0)), 0.0);
}

TEST(State, PositiveAngleOfAttackMeansDownwardBodyVelocityComponent) {
  // z is DOWN, so relative wind coming from below the nose — positive alpha —
  // shows up as positive w. Reversing this sign reverses the sign of every
  // longitudinal stability derivative.
  EXPECT_GT(galata::core::angle_of_attack(Eigen::Vector3d(100.0, 0.0, 10.0)), 0.0);
  EXPECT_LT(galata::core::angle_of_attack(Eigen::Vector3d(100.0, 0.0, -10.0)), 0.0);
  EXPECT_NEAR(galata::core::angle_of_attack(Eigen::Vector3d(100.0, 0.0, 100.0)),
              degrees_to_radians(45.0),
              kLoose);
}

TEST(State, PositiveSideslipMeansVelocityOutTheRightWing) {
  EXPECT_GT(galata::core::sideslip_angle(Eigen::Vector3d(100.0, 10.0, 0.0)), 0.0);
  EXPECT_NEAR(galata::core::sideslip_angle(Eigen::Vector3d(0.0, 100.0, 0.0)),
              degrees_to_radians(90.0),
              kLoose);
}

TEST(State, AngleOfAttackUsesAtan2AndSurvivesRearwardFlight) {
  // atan(w/u) would fold this into the forward half-plane and report -45
  // degrees. atan2 reports the actual 135.
  EXPECT_NEAR(galata::core::angle_of_attack(Eigen::Vector3d(-100.0, 0.0, 100.0)),
              degrees_to_radians(135.0),
              kLoose);
}

TEST(State, AerodynamicAnglesAreZeroByConventionAtZeroVelocity) {
  const Eigen::Vector3d still = Eigen::Vector3d::Zero();
  EXPECT_DOUBLE_EQ(galata::core::angle_of_attack(still), 0.0);
  EXPECT_DOUBLE_EQ(galata::core::sideslip_angle(still), 0.0);
  EXPECT_DOUBLE_EQ(galata::core::airspeed(still), 0.0);
}

TEST(State, WindAxisRoundTripsInBothDirections) {
  for (double alpha_deg = -30.0; alpha_deg <= 40.0; alpha_deg += 7.0) {
    for (double beta_deg = -25.0; beta_deg <= 25.0; beta_deg += 5.0) {
      const galata::core::WindAxisVelocity original{
          243.7, degrees_to_radians(alpha_deg), degrees_to_radians(beta_deg)};
      const galata::core::WindAxisVelocity recovered =
          galata::core::to_wind_axes(galata::core::from_wind_axes(original));

      EXPECT_NEAR(recovered.airspeed_m_s, original.airspeed_m_s, 1e-10);
      EXPECT_NEAR(recovered.alpha_rad, original.alpha_rad, 1e-12);
      EXPECT_NEAR(recovered.beta_rad, original.beta_rad, 1e-12);
    }
  }
}

TEST(State, WindAxisComponentsMatchTheDocumentedFormula) {
  const double v = 200.0;
  const double alpha = degrees_to_radians(12.0);
  const double beta = degrees_to_radians(-5.0);
  const Eigen::Vector3d body = galata::core::from_wind_axes({v, alpha, beta});

  EXPECT_NEAR(body.x(), v * std::cos(alpha) * std::cos(beta), kLoose);
  EXPECT_NEAR(body.y(), v * std::sin(beta), kLoose);
  EXPECT_NEAR(body.z(), v * std::sin(alpha) * std::cos(beta), kLoose);
}

TEST(State, LevelFlightHasZeroFlightPathAngle) {
  State s;
  s.velocity_body_m_s = Eigen::Vector3d(150.0, 0.0, 0.0);
  s.attitude_body_to_ned = galata::core::identity_attitude();
  EXPECT_NEAR(galata::core::flight_path_angle(s), 0.0, kTight);
}

TEST(State, ClimbingFlightHasPositiveFlightPathAngle) {
  // Nose up 10 degrees at zero angle of attack: the velocity vector is 10
  // degrees above the horizon, so gamma is +10 even though p_d is decreasing.
  State s;
  s.velocity_body_m_s = Eigen::Vector3d(150.0, 0.0, 0.0);
  s.attitude_body_to_ned =
      galata::core::quaternion_from_euler(EulerAngles{0.0, degrees_to_radians(10.0), 0.0});
  EXPECT_NEAR(galata::core::flight_path_angle(s), degrees_to_radians(10.0), 1e-10);
  EXPECT_LT(galata::core::velocity_ned(s).z(), 0.0);  // climbing means descending p_d
}

TEST(State, GammaEqualsThetaMinusAlphaOnlyInSymmetricWingsLevelFlight) {
  // The approximation every textbook offers, and the case where it holds
  // exactly. Recorded as a test so that anyone tempted to use it elsewhere can
  // see the stated precondition.
  const double theta = degrees_to_radians(6.0);
  const double alpha = degrees_to_radians(4.0);

  State s;
  s.velocity_body_m_s = galata::core::from_wind_axes({160.0, alpha, 0.0});
  s.attitude_body_to_ned = galata::core::quaternion_from_euler(EulerAngles{0.0, theta, 0.0});
  EXPECT_NEAR(galata::core::flight_path_angle(s), theta - alpha, 1e-10);
}

TEST(State, RenormaliseRestoresUnitNorm) {
  State s = sample_state();
  s.attitude_body_to_ned = galata::core::Quaternion(1.001, 0.002, -0.003, 0.004);
  s.renormalise_attitude();
  EXPECT_NEAR(s.attitude_body_to_ned.norm(), 1.0, kTight);
}

}  // namespace
