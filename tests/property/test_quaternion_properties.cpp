// SPDX-License-Identifier: Apache-2.0
//
// Invariants that must hold for every attitude, asserted over uniformly
// sampled rotations.
//
// These are the tests that catch a transposed rotation matrix. A hand-picked
// example can agree with a transposed matrix by accident — at zero roll, or at
// a symmetric attitude — and several of the obvious examples do. A uniform
// sample over the rotation group does not.

#include "galata/core/constants.hpp"
#include "galata/core/quaternion.hpp"

#include "property_generators.hpp"
#include <gtest/gtest.h>

#include <cmath>

namespace {

using galata::core::Quaternion;
using galata::testing::Generator;
using galata::testing::kPropertySamples;

constexpr std::uint64_t kSeed = 0x9E3779B97F4A7C15ULL;
constexpr double kTight = 1e-12;

TEST(QuaternionProperties, RotationMatrixIsAlwaysOrthonormalWithUnitDeterminant) {
  Generator gen(kSeed);
  for (int i = 0; i < kPropertySamples; ++i) {
    const Quaternion q = gen.rotation();
    const Eigen::Matrix3d r = galata::core::dcm_ned_from_body(q);
    EXPECT_TRUE((r * r.transpose()).isApprox(Eigen::Matrix3d::Identity(), kTight))
        << "seed " << gen.seed() << " sample " << i;
    EXPECT_NEAR(r.determinant(), 1.0, kTight) << "seed " << gen.seed() << " sample " << i;
  }
}

TEST(QuaternionProperties, HandWrittenDcmAlwaysAgreesWithEigen) {
  // The convention check from test_quaternion.cpp, over the whole rotation
  // group rather than a lattice of Euler angles.
  Generator gen(kSeed + 1);
  for (int i = 0; i < kPropertySamples; ++i) {
    const Quaternion q = gen.rotation();
    EXPECT_TRUE(galata::core::dcm_ned_from_body(q).isApprox(q.toRotationMatrix(), kTight))
        << "seed " << gen.seed() << " sample " << i;
  }
}

TEST(QuaternionProperties, BodyFromNedIsAlwaysTheInverseOfNedFromBody) {
  Generator gen(kSeed + 2);
  for (int i = 0; i < kPropertySamples; ++i) {
    const Quaternion q = gen.rotation();
    EXPECT_TRUE((galata::core::dcm_body_from_ned(q) * galata::core::dcm_ned_from_body(q))
                    .isApprox(Eigen::Matrix3d::Identity(), kTight))
        << "seed " << gen.seed() << " sample " << i;
  }
}

TEST(QuaternionProperties, RotationPreservesLength) {
  Generator gen(kSeed + 3);
  for (int i = 0; i < kPropertySamples; ++i) {
    const Quaternion q = gen.rotation();
    const Eigen::Vector3d v = gen.vector3(500.0);
    EXPECT_NEAR((galata::core::dcm_ned_from_body(q) * v).norm(), v.norm(), 1e-10 * (v.norm() + 1.0))
        << "seed " << gen.seed() << " sample " << i;
  }
}

TEST(QuaternionProperties, CompositionOfQuaternionsIsCompositionOfMatrices) {
  // R(a (x) b) == R(a) R(b). If the quaternion product and the matrix
  // convention disagreed in handedness this would fail immediately.
  Generator gen(kSeed + 4);
  for (int i = 0; i < kPropertySamples; ++i) {
    const Quaternion a = gen.rotation();
    const Quaternion b = gen.rotation();
    EXPECT_TRUE(galata::core::dcm_ned_from_body(a * b).isApprox(
        galata::core::dcm_ned_from_body(a) * galata::core::dcm_ned_from_body(b), kTight))
        << "seed " << gen.seed() << " sample " << i;
  }
}

TEST(QuaternionProperties, EulerRoundTripsWhereverTheSplitIsWellConditioned) {
  Generator gen(kSeed + 5);
  int exercised = 0;
  for (int i = 0; i < kPropertySamples; ++i) {
    const Quaternion q = gen.rotation();
    // Skip only genuinely ill-conditioned attitudes; the threshold is generous
    // enough that the bulk of near-vertical cases are still exercised.
    if (galata::core::gimbal_lock_proximity(q) < 1e-4) {
      continue;
    }
    ++exercised;
    const Quaternion reconstructed =
        galata::core::quaternion_from_euler(galata::core::euler_from_quaternion(q));
    EXPECT_NEAR(galata::core::angular_distance(q, reconstructed), 0.0, 1e-13)
        << "seed " << gen.seed() << " sample " << i;
  }
  // Guard against the skip condition quietly emptying the test.
  EXPECT_GT(exercised, kPropertySamples / 2);
}

TEST(QuaternionProperties, EulerAnglesStayInTheirDocumentedRanges) {
  Generator gen(kSeed + 6);
  for (int i = 0; i < kPropertySamples; ++i) {
    const galata::core::EulerAngles e = galata::core::euler_from_quaternion(gen.rotation());
    EXPECT_LE(std::fabs(e.roll_rad), galata::core::kPi + kTight);
    EXPECT_LE(std::fabs(e.pitch_rad), galata::core::kHalfPi + kTight);
    EXPECT_LE(std::fabs(e.yaw_rad), galata::core::kPi + kTight);
  }
}

TEST(QuaternionProperties, DerivativeIsAlwaysOrthogonalToTheAttitude) {
  // The norm-preservation invariant of the kinematic equation. If qdot had a
  // component along q the quaternion would grow or shrink under exact
  // integration, and renormalisation would be hiding a wrong equation rather
  // than correcting integration error.
  Generator gen(kSeed + 7);
  for (int i = 0; i < kPropertySamples; ++i) {
    const Quaternion q = gen.rotation();
    const Eigen::Vector3d omega = gen.vector3(5.0);
    const Eigen::Vector4d dq = galata::core::quaternion_derivative(q, omega);
    EXPECT_NEAR(galata::core::to_wxyz(q).dot(dq), 0.0, 1e-12 * (omega.norm() + 1.0))
        << "seed " << gen.seed() << " sample " << i;
  }
}

TEST(QuaternionProperties, DerivativeMagnitudeIsHalfTheAngularRate) {
  // ||qdot|| = ||omega|| / 2 for a unit quaternion, exactly.
  Generator gen(kSeed + 8);
  for (int i = 0; i < kPropertySamples; ++i) {
    const Quaternion q = gen.rotation();
    const Eigen::Vector3d omega = gen.vector3(5.0);
    EXPECT_NEAR(galata::core::quaternion_derivative(q, omega).norm(),
                0.5 * omega.norm(),
                1e-12 * (omega.norm() + 1.0))
        << "seed " << gen.seed() << " sample " << i;
  }
}

TEST(QuaternionProperties, AngularDistanceIsAMetricAndIgnoresTheDoubleCover) {
  Generator gen(kSeed + 9);
  for (int i = 0; i < kPropertySamples; ++i) {
    const Quaternion a = gen.rotation();
    const Quaternion b = gen.rotation();
    const Quaternion negated_b(-b.w(), -b.x(), -b.y(), -b.z());

    const double d = galata::core::angular_distance(a, b);
    EXPECT_GE(d, 0.0);
    EXPECT_LE(d, galata::core::kPi + kTight);
    EXPECT_NEAR(galata::core::angular_distance(b, a), d, 1e-13);
    EXPECT_NEAR(galata::core::angular_distance(a, negated_b), d, 1e-13);
    // Exactly zero, not nearly zero: the atan2 formulation gives 2*atan2(0, 1).
    EXPECT_EQ(galata::core::angular_distance(a, a), 0.0);
  }
}

TEST(QuaternionProperties, CanonicalPreservesTheRotationAndFixesTheSign) {
  Generator gen(kSeed + 10);
  for (int i = 0; i < kPropertySamples; ++i) {
    const Quaternion q = gen.rotation();
    const Quaternion c = galata::core::canonical(q);
    EXPECT_GE(c.w(), 0.0);
    EXPECT_NEAR(galata::core::angular_distance(q, c), 0.0, 1e-15);
  }
}

TEST(QuaternionProperties, PackingRoundTripsBitwise) {
  Generator gen(kSeed + 11);
  for (int i = 0; i < kPropertySamples; ++i) {
    const Quaternion q = gen.rotation();
    const Quaternion back = galata::core::from_wxyz(galata::core::to_wxyz(q));
    EXPECT_EQ(back.w(), q.w());
    EXPECT_EQ(back.x(), q.x());
    EXPECT_EQ(back.y(), q.y());
    EXPECT_EQ(back.z(), q.z());
  }
}

}  // namespace
