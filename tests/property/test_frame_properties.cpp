// SPDX-License-Identifier: Apache-2.0
//
// Frame-rotation invariants over sampled aerodynamic angles.

#include "galata/core/constants.hpp"
#include "galata/core/frames.hpp"
#include "galata/core/state.hpp"

#include "property_generators.hpp"
#include <gtest/gtest.h>

#include <cmath>

namespace {

using galata::testing::Generator;
using galata::testing::kPropertySamples;

constexpr std::uint64_t kSeed = 0xC2B2AE3D27D4EB4FULL;
constexpr double kTight = 1e-12;

TEST(FrameProperties, ElementaryRotationsAreAlwaysProperRotations) {
  Generator gen(kSeed);
  for (int i = 0; i < kPropertySamples; ++i) {
    const double angle = gen.range(-galata::core::kTwoPi, galata::core::kTwoPi);
    for (const Eigen::Matrix3d& r : {galata::core::rotation_x(angle),
                                     galata::core::rotation_y(angle),
                                     galata::core::rotation_z(angle)}) {
      EXPECT_TRUE((r * r.transpose()).isApprox(Eigen::Matrix3d::Identity(), kTight));
      EXPECT_NEAR(r.determinant(), 1.0, kTight);
    }
  }
}

TEST(FrameProperties, ElementaryRotationsAddAngles) {
  Generator gen(kSeed + 1);
  for (int i = 0; i < kPropertySamples; ++i) {
    const double a = gen.range(-galata::core::kPi, galata::core::kPi);
    const double b = gen.range(-galata::core::kPi, galata::core::kPi);
    EXPECT_TRUE((galata::core::rotation_x(a) * galata::core::rotation_x(b))
                    .isApprox(galata::core::rotation_x(a + b), 1e-10));
    EXPECT_TRUE((galata::core::rotation_y(a) * galata::core::rotation_y(b))
                    .isApprox(galata::core::rotation_y(a + b), 1e-10));
    EXPECT_TRUE((galata::core::rotation_z(a) * galata::core::rotation_z(b))
                    .isApprox(galata::core::rotation_z(a + b), 1e-10));
  }
}

TEST(FrameProperties, WindFrameAlwaysAlignsWithTheVelocityVector) {
  // The defining property of the wind frame, over the full aerodynamic-angle
  // range rather than the conventional-flight subset covered by the unit test.
  Generator gen(kSeed + 2);
  for (int i = 0; i < kPropertySamples; ++i) {
    const double speed = gen.range(1.0, 400.0);
    const double alpha = gen.range(-galata::core::kHalfPi, galata::core::kHalfPi);
    const double beta = gen.range(-galata::core::kHalfPi * 0.9, galata::core::kHalfPi * 0.9);

    const Eigen::Vector3d velocity_wind = galata::core::dcm_wind_from_body(alpha, beta)
                                          * galata::core::from_wind_axes({speed, alpha, beta});

    const double scale = 1e-12 * speed + 1e-12;
    EXPECT_NEAR(velocity_wind.x(), speed, scale)
        << "seed " << gen.seed() << " sample " << i << " alpha " << alpha << " beta " << beta;
    EXPECT_NEAR(velocity_wind.y(), 0.0, scale);
    EXPECT_NEAR(velocity_wind.z(), 0.0, scale);
  }
}

TEST(FrameProperties, WindAndBodyRotationsAreAlwaysMutualInverses) {
  Generator gen(kSeed + 3);
  for (int i = 0; i < kPropertySamples; ++i) {
    const double alpha = gen.range(-galata::core::kPi, galata::core::kPi);
    const double beta = gen.range(-galata::core::kHalfPi, galata::core::kHalfPi);
    EXPECT_TRUE((galata::core::dcm_body_from_wind(alpha, beta)
                 * galata::core::dcm_wind_from_body(alpha, beta))
                    .isApprox(Eigen::Matrix3d::Identity(), kTight));
  }
}

TEST(FrameProperties, StabilityFrameNeverMovesTheBodyYAxis) {
  // The stability rotation is about y by construction, so the body y-axis is
  // its fixed axis. A composition-order error would break this.
  Generator gen(kSeed + 4);
  for (int i = 0; i < kPropertySamples; ++i) {
    const double alpha = gen.range(-galata::core::kPi, galata::core::kPi);
    EXPECT_TRUE((galata::core::dcm_stability_from_body(alpha) * Eigen::Vector3d::UnitY())
                    .isApprox(Eigen::Vector3d::UnitY(), kTight));
  }
}

TEST(FrameProperties, ZeroSideslipMakesWindAndStabilityFramesIdentical) {
  Generator gen(kSeed + 5);
  for (int i = 0; i < kPropertySamples; ++i) {
    const double alpha = gen.range(-galata::core::kPi, galata::core::kPi);
    EXPECT_TRUE(galata::core::dcm_wind_from_body(alpha, 0.0)
                    .isApprox(galata::core::dcm_stability_from_body(alpha), kTight));
  }
}

}  // namespace
