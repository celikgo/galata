// SPDX-License-Identifier: Apache-2.0
//
// Frame rotations (ADR-0002).
//
// dcm_wind_from_body is tested against its DEFINING property — that it maps the
// body-axis velocity components onto (V, 0, 0) — rather than against a
// transcribed matrix. A transcribed matrix tests that two copies of the same
// possibly-wrong expression agree; the defining property tests the thing the
// matrix is for.

#include "galata/core/frames.hpp"
#include "galata/core/state.hpp"
#include "galata/units.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using galata::units::degrees_to_radians;

constexpr double kTight = 1e-14;
constexpr double kLoose = 1e-12;

TEST(Frames, ElementaryRotationsAreOrthonormalWithUnitDeterminant) {
  for (double angle_deg = -180.0; angle_deg <= 180.0; angle_deg += 17.0) {
    const double a = degrees_to_radians(angle_deg);
    for (const Eigen::Matrix3d& r :
         {galata::core::rotation_x(a), galata::core::rotation_y(a), galata::core::rotation_z(a)}) {
      EXPECT_TRUE((r * r.transpose()).isApprox(Eigen::Matrix3d::Identity(), kTight))
          << "angle " << angle_deg;
      EXPECT_NEAR(r.determinant(), 1.0, kLoose) << "angle " << angle_deg;
    }
  }
}

TEST(Frames, ElementaryRotationsLeaveTheirOwnAxisAlone) {
  const double a = degrees_to_radians(53.0);
  EXPECT_TRUE((galata::core::rotation_x(a) * Eigen::Vector3d::UnitX())
                  .isApprox(Eigen::Vector3d::UnitX(), kTight));
  EXPECT_TRUE((galata::core::rotation_y(a) * Eigen::Vector3d::UnitY())
                  .isApprox(Eigen::Vector3d::UnitY(), kTight));
  EXPECT_TRUE((galata::core::rotation_z(a) * Eigen::Vector3d::UnitZ())
                  .isApprox(Eigen::Vector3d::UnitZ(), kTight));
}

TEST(Frames, RotationDirectionIsTransformComponentsNotRotateVector) {
  // The distinction that decides every sign in this file. Rotating the FRAME by
  // +90 degrees about z sends a vector that pointed along old-x to pointing
  // along new-NEGATIVE-y. If these matrices rotated the vector instead of the
  // frame, the sign would be the other way.
  const Eigen::Vector3d transformed =
      galata::core::rotation_z(degrees_to_radians(90.0)) * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(transformed.x(), 0.0, kLoose);
  EXPECT_NEAR(transformed.y(), -1.0, kLoose);
  EXPECT_NEAR(transformed.z(), 0.0, kLoose);
}

TEST(Frames, RotationsComposeAndInvert) {
  const double a = degrees_to_radians(31.0);
  const double b = degrees_to_radians(-64.0);
  EXPECT_TRUE((galata::core::rotation_y(a) * galata::core::rotation_y(b))
                  .isApprox(galata::core::rotation_y(a + b), kLoose));
  EXPECT_TRUE(
      galata::core::rotation_x(-a).isApprox(galata::core::rotation_x(a).transpose(), kTight));
}

TEST(Frames, WindFrameXAxisLiesAlongTheVelocityVector) {
  // The defining property of the wind frame, swept over the aerodynamic-angle
  // envelope a conventional aircraft actually reaches.
  const double kSpeed = 137.4;  // m/s
  for (double alpha_deg = -20.0; alpha_deg <= 30.0; alpha_deg += 5.0) {
    for (double beta_deg = -15.0; beta_deg <= 15.0; beta_deg += 3.0) {
      const double alpha = degrees_to_radians(alpha_deg);
      const double beta = degrees_to_radians(beta_deg);

      const Eigen::Vector3d velocity_body = galata::core::from_wind_axes({kSpeed, alpha, beta});
      const Eigen::Vector3d velocity_wind =
          galata::core::dcm_wind_from_body(alpha, beta) * velocity_body;

      EXPECT_NEAR(velocity_wind.x(), kSpeed, 1e-10)
          << "alpha " << alpha_deg << " beta " << beta_deg;
      EXPECT_NEAR(velocity_wind.y(), 0.0, 1e-10) << "alpha " << alpha_deg << " beta " << beta_deg;
      EXPECT_NEAR(velocity_wind.z(), 0.0, 1e-10) << "alpha " << alpha_deg << " beta " << beta_deg;
    }
  }
}

TEST(Frames, StabilityFrameXAxisLiesInTheBodyPlaneOfSymmetry) {
  // With no sideslip, the stability frame's x-axis is the velocity vector, and
  // its y-axis is unchanged from the body y-axis.
  const double alpha = degrees_to_radians(8.0);
  const Eigen::Vector3d velocity_body = galata::core::from_wind_axes({100.0, alpha, 0.0});
  const Eigen::Vector3d velocity_stability =
      galata::core::dcm_stability_from_body(alpha) * velocity_body;

  EXPECT_NEAR(velocity_stability.x(), 100.0, 1e-10);
  EXPECT_NEAR(velocity_stability.y(), 0.0, 1e-10);
  EXPECT_NEAR(velocity_stability.z(), 0.0, 1e-10);

  EXPECT_TRUE((galata::core::dcm_stability_from_body(alpha) * Eigen::Vector3d::UnitY())
                  .isApprox(Eigen::Vector3d::UnitY(), kTight));
}

TEST(Frames, PositiveAlphaTiltsTheStabilityAxisNoseDownRelativeToBody) {
  // At positive angle of attack the velocity vector is BELOW the body x-axis in
  // a z-down frame, so the body x-axis expressed in stability axes has a
  // negative z-component. The sign that decides whether the short period damps
  // or diverges after linearisation.
  const double alpha = degrees_to_radians(10.0);
  const Eigen::Vector3d body_x_in_stability =
      galata::core::dcm_stability_from_body(alpha) * Eigen::Vector3d::UnitX();
  EXPECT_LT(body_x_in_stability.z(), 0.0);
  EXPECT_NEAR(body_x_in_stability.z(), -std::sin(alpha), kLoose);
}

TEST(Frames, WindAndBodyRotationsAreMutualInverses) {
  const double alpha = degrees_to_radians(6.5);
  const double beta = degrees_to_radians(-4.25);
  EXPECT_TRUE((galata::core::dcm_body_from_wind(alpha, beta)
               * galata::core::dcm_wind_from_body(alpha, beta))
                  .isApprox(Eigen::Matrix3d::Identity(), kTight));
}

}  // namespace
