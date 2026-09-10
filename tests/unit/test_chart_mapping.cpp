// SPDX-License-Identifier: Apache-2.0
//
// The attitude-error chart, both ways (ADR-0017).
//
// The chart is how a gain designed against `linearize.extended` reaches a plant
// integrated in ADR-0002's coordinates. Every coordinate is additive except
// attitude, which is multiplicative — q = q0 * exp(e/2) — and it is the
// attitude one that fails quietly when it fails.
//
// Expected values here are written down from that definition, not read off the
// implementation.

#include "galata/core/quaternion.hpp"
#include "galata/core/state.hpp"
#include "galata/linearize/extended.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

namespace {

using galata::core::Quaternion;
using galata::linearize::chart_from_extended;
using galata::linearize::extended_from_chart;

constexpr double kPi = 3.14159265358979323846264338327950288;
constexpr int kAppended = 4;  // four rotors, as the shipped model has

Eigen::VectorXd reference_state() {
  Eigen::VectorXd x = Eigen::VectorXd::Zero(galata::core::kStateSize + kAppended);
  x(galata::core::kPositionNorth) = 10.0;
  x(galata::core::kPositionDown) = -120.0;
  x(galata::core::kVelocityU) = 3.0;
  // A reference attitude that is NOT the identity, because an identity
  // reference hides a composition written in the wrong order.
  const Quaternion nominal =
      galata::core::quaternion_from_rotation_vector(Eigen::Vector3d(0.2, -0.35, 0.7));
  x(galata::core::kQuaternionW) = nominal.w();
  x(galata::core::kQuaternionX) = nominal.x();
  x(galata::core::kQuaternionY) = nominal.y();
  x(galata::core::kQuaternionZ) = nominal.z();
  x(galata::core::kRateQ) = 0.05;
  for (int i = 0; i < kAppended; ++i) {
    x(galata::core::kStateSize + i) = 626.0 + static_cast<double>(i);
  }
  return x;
}

}  // namespace

TEST(ChartMapping, TheReferenceSitsAtTheChartOrigin) {
  const Eigen::VectorXd reference = reference_state();
  const Eigen::VectorXd chart = chart_from_extended(reference, reference);
  ASSERT_EQ(chart.size(), galata::linearize::kRigidChartSize + kAppended);
  // Exactly zero, not nearly: a displacement from a point to itself.
  EXPECT_LT(chart.norm(), 1e-15) << "the reference is not at its own chart origin: "
                                 << chart.transpose();
}

TEST(ChartMapping, RoundTripsThroughBothDirections) {
  const Eigen::VectorXd reference = reference_state();
  // Rotations spanning the series boundary at 1e-7 on the vector norm — below
  // it, across it, and well above — plus one near a half turn, where the
  // parameterisation stops being unique.
  for (const double angle : {0.0, 1e-12, 1e-9, 1e-6, 1e-3, 0.5, 2.0, kPi - 1e-6}) {
    Eigen::VectorXd chart = Eigen::VectorXd::Zero(galata::linearize::kRigidChartSize + kAppended);
    chart(galata::linearize::kChartPositionEast) = 2.5;
    chart(galata::linearize::kChartVelocityW) = -1.25;
    chart(galata::linearize::kChartRateP) = 0.03;
    chart(galata::linearize::kRigidChartSize + 2) = -7.5;
    const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -2.0, 0.5).normalized();
    chart.segment<3>(galata::linearize::kChartAttitudeErrorX) = angle * axis;

    const Eigen::VectorXd full = extended_from_chart(chart, reference);
    const Eigen::VectorXd back = chart_from_extended(full, reference);
    EXPECT_LT((back - chart).norm(), 1e-12)
        << "chart -> extended -> chart failed at |e| = " << angle << "\n  sent "
        << chart.transpose() << "\n  got  " << back.transpose();
  }
}

TEST(ChartMapping, TheDoubleCoverIsResolvedToTheShortWayRound) {
  const Eigen::VectorXd reference = reference_state();
  Eigen::VectorXd chart = Eigen::VectorXd::Zero(galata::linearize::kRigidChartSize + kAppended);
  chart.segment<3>(galata::linearize::kChartAttitudeErrorX) = Eigen::Vector3d(0.3, 0.1, -0.2);
  const Eigen::VectorXd full = extended_from_chart(chart, reference);

  // q and -q are the SAME attitude. A chart that did not canonicalise would
  // return a rotation vector larger by a full turn, and a controller told its
  // error had jumped by 2 pi would command the long way round at full authority.
  Eigen::VectorXd negated = full;
  negated.segment<4>(galata::core::kQuaternionW) *= -1.0;

  const Eigen::VectorXd from_positive = chart_from_extended(full, reference);
  const Eigen::VectorXd from_negated = chart_from_extended(negated, reference);
  EXPECT_LT((from_positive - from_negated).norm(), 1e-12)
      << "q and -q are one attitude and must give one chart coordinate";
  EXPECT_LT(from_positive.segment<3>(galata::linearize::kChartAttitudeErrorX).norm(), kPi)
      << "the result must be the short way round";
}

TEST(ChartMapping, MismatchedWidthsAreRefused) {
  const Eigen::VectorXd reference = reference_state();
  const Eigen::VectorXd wrong_state = Eigen::VectorXd::Zero(reference.size() + 1);
  EXPECT_THROW((void)chart_from_extended(wrong_state, reference), std::invalid_argument);
  const Eigen::VectorXd wrong_chart =
      Eigen::VectorXd::Zero(galata::linearize::kRigidChartSize + kAppended + 1);
  EXPECT_THROW((void)extended_from_chart(wrong_chart, reference), std::invalid_argument);
  const Eigen::VectorXd too_short = Eigen::VectorXd::Zero(5);
  EXPECT_THROW((void)chart_from_extended(too_short, too_short), std::invalid_argument);
}

// The exponential and the logarithm are each other's inverse, tested on the
// quaternion primitives directly so a failure here localises to `core` rather
// than to the chart that uses them.
TEST(RotationVector, ExponentialAndLogarithmInvertEachOther) {
  for (const double angle : {0.0, 1e-12, 1e-8, 1e-7, 1e-6, 0.01, 1.0, 3.0}) {
    const Eigen::Vector3d axis = Eigen::Vector3d(-0.3, 0.9, 0.2).normalized();
    const Eigen::Vector3d rotation = angle * axis;
    const Quaternion q = galata::core::quaternion_from_rotation_vector(rotation);
    EXPECT_NEAR(q.norm(), 1.0, 1e-15) << "the exponential must return a unit quaternion";
    const Eigen::Vector3d back = galata::core::rotation_vector_from_quaternion(q);
    EXPECT_LT((back - rotation).norm(), 1e-12)
        << "log(exp(e)) != e at |e| = " << angle << ": got " << back.transpose();
  }
}
