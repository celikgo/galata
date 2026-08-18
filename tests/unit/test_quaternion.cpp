// SPDX-License-Identifier: Apache-2.0
//
// Quaternion conventions (ADR-0002).
//
// The most valuable test in this file is DcmMatchesEigensOwnRotationMatrix. The
// hand-written matrix in quaternion.cpp and Eigen's toRotationMatrix() are two
// independent expressions of "Hamilton, scalar-first, body-to-NED". If they
// agree, the convention documented in ADR-0002 is the convention the code
// implements. If galata had simply called Eigen, that check would not exist and
// a convention mismatch would be invisible.

#include "galata/core/quaternion.hpp"
#include "galata/units.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using galata::core::EulerAngles;
using galata::core::Quaternion;
using galata::units::degrees_to_radians;

constexpr double kTight = 1e-14;
constexpr double kLoose = 1e-12;

TEST(Quaternion, IdentityAttitudeIsTheIdentityRotation) {
  const Quaternion q = galata::core::identity_attitude();
  EXPECT_DOUBLE_EQ(q.w(), 1.0);
  EXPECT_TRUE(galata::core::dcm_ned_from_body(q).isApprox(Eigen::Matrix3d::Identity(), kTight));
}

TEST(Quaternion, DcmMatchesEigensOwnRotationMatrix) {
  // Eigen's Quaterniond is Hamilton and its constructor is scalar-first, so if
  // ADR-0002's formula is transcribed correctly these must agree bit-for-bit up
  // to rounding. Swept over attitudes rather than checked at one point, because
  // a sign error in a single off-diagonal term survives most single samples.
  for (double roll_deg = -170.0; roll_deg <= 170.0; roll_deg += 37.0) {
    for (double pitch_deg = -85.0; pitch_deg <= 85.0; pitch_deg += 23.0) {
      for (double yaw_deg = -170.0; yaw_deg <= 170.0; yaw_deg += 41.0) {
        const Quaternion q =
            galata::core::quaternion_from_euler(EulerAngles{degrees_to_radians(roll_deg),
                                                            degrees_to_radians(pitch_deg),
                                                            degrees_to_radians(yaw_deg)});
        EXPECT_TRUE(galata::core::dcm_ned_from_body(q).isApprox(q.toRotationMatrix(), kTight))
            << "roll " << roll_deg << " pitch " << pitch_deg << " yaw " << yaw_deg;
      }
    }
  }
}

TEST(Quaternion, NinetyDegreeYawPutsTheNoseEast) {
  // The single most useful sanity check on a body-to-NED rotation: yaw right by
  // 90 degrees and the aircraft's forward axis, expressed in NED, must be the
  // east unit vector. If the quaternion were NED-to-body this would come out
  // pointing west.
  const Quaternion q =
      galata::core::quaternion_from_euler(EulerAngles{0.0, 0.0, degrees_to_radians(90.0)});
  const Eigen::Vector3d nose_in_ned = galata::core::dcm_ned_from_body(q) * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(nose_in_ned.x(), 0.0, kLoose);
  EXPECT_NEAR(nose_in_ned.y(), 1.0, kLoose);
  EXPECT_NEAR(nose_in_ned.z(), 0.0, kLoose);
}

TEST(Quaternion, NoseUpPitchPutsTheNoseAboveTheHorizon) {
  // z is DOWN, so a nose above the horizon has a NEGATIVE down-component.
  // This is the test that catches a z-up frame having crept in.
  const Quaternion q =
      galata::core::quaternion_from_euler(EulerAngles{0.0, degrees_to_radians(30.0), 0.0});
  const Eigen::Vector3d nose_in_ned = galata::core::dcm_ned_from_body(q) * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(nose_in_ned.x(), std::cos(degrees_to_radians(30.0)), kLoose);
  EXPECT_NEAR(nose_in_ned.z(), -std::sin(degrees_to_radians(30.0)), kLoose);
}

TEST(Quaternion, RightRollPutsTheRightWingDown) {
  const Quaternion q =
      galata::core::quaternion_from_euler(EulerAngles{degrees_to_radians(45.0), 0.0, 0.0});
  const Eigen::Vector3d wing_in_ned = galata::core::dcm_ned_from_body(q) * Eigen::Vector3d::UnitY();
  EXPECT_NEAR(wing_in_ned.y(), std::cos(degrees_to_radians(45.0)), kLoose);
  EXPECT_NEAR(wing_in_ned.z(), std::sin(degrees_to_radians(45.0)), kLoose);
}

TEST(Quaternion, EulerRoundTripsAwayFromGimbalLock) {
  for (double roll_deg = -170.0; roll_deg <= 170.0; roll_deg += 29.0) {
    for (double pitch_deg = -80.0; pitch_deg <= 80.0; pitch_deg += 19.0) {
      for (double yaw_deg = -170.0; yaw_deg <= 170.0; yaw_deg += 31.0) {
        const EulerAngles original{degrees_to_radians(roll_deg),
                                   degrees_to_radians(pitch_deg),
                                   degrees_to_radians(yaw_deg)};
        const EulerAngles recovered =
            galata::core::euler_from_quaternion(galata::core::quaternion_from_euler(original));
        EXPECT_NEAR(recovered.roll_rad, original.roll_rad, kLoose);
        EXPECT_NEAR(recovered.pitch_rad, original.pitch_rad, kLoose);
        EXPECT_NEAR(recovered.yaw_rad, original.yaw_rad, kLoose);
      }
    }
  }
}

TEST(Quaternion, GimbalLockProximityIsCosineOfPitch) {
  for (double pitch_deg = -90.0; pitch_deg <= 90.0; pitch_deg += 15.0) {
    const Quaternion q = galata::core::quaternion_from_euler(EulerAngles{
        degrees_to_radians(20.0), degrees_to_radians(pitch_deg), degrees_to_radians(-40.0)});
    EXPECT_NEAR(galata::core::gimbal_lock_proximity(q),
                std::fabs(std::cos(degrees_to_radians(pitch_deg))),
                1e-9);
  }
}

TEST(Quaternion, AtGimbalLockTheRotationIsStillRepresentedExactly) {
  // The Euler split is undefined at 90 degrees of pitch, but the quaternion and
  // its rotation matrix are not. The attitude must survive the round trip even
  // though the roll/yaw split does not.
  const EulerAngles original{
      degrees_to_radians(35.0), degrees_to_radians(90.0), degrees_to_radians(70.0)};
  const Quaternion q = galata::core::quaternion_from_euler(original);
  EXPECT_NEAR(galata::core::gimbal_lock_proximity(q), 0.0, 1e-8);

  const EulerAngles recovered = galata::core::euler_from_quaternion(q);
  EXPECT_NEAR(recovered.pitch_rad, degrees_to_radians(90.0), 1e-7);
  EXPECT_DOUBLE_EQ(recovered.roll_rad, 0.0);  // the documented convention

  // The recovered angles describe the same rotation even though they are not
  // the same numbers.
  EXPECT_NEAR(
      galata::core::angular_distance(q, galata::core::quaternion_from_euler(recovered)), 0.0, 1e-7);
}

TEST(Quaternion, DerivativeForPureYawRate) {
  // At identity attitude, spinning about body z at r must move only q_z.
  const double yaw_rate = 0.37;  // rad/s
  const Eigen::Vector4d dq = galata::core::quaternion_derivative(
      galata::core::identity_attitude(), Eigen::Vector3d(0.0, 0.0, yaw_rate));
  EXPECT_NEAR(dq(0), 0.0, kTight);
  EXPECT_NEAR(dq(1), 0.0, kTight);
  EXPECT_NEAR(dq(2), 0.0, kTight);
  EXPECT_NEAR(dq(3), 0.5 * yaw_rate, kTight);
}

TEST(Quaternion, DerivativeIsOrthogonalToTheQuaternion) {
  // ||q|| = 1 is preserved by the true dynamics, so qdot must have no component
  // along q. This is the invariant that makes renormalisation a correction of
  // integration error rather than a correction of the equation.
  const Quaternion q = galata::core::quaternion_from_euler(
      EulerAngles{degrees_to_radians(20.0), degrees_to_radians(-35.0), degrees_to_radians(110.0)});
  const Eigen::Vector4d dq =
      galata::core::quaternion_derivative(q, Eigen::Vector3d(0.11, -0.42, 0.27));
  const double along = galata::core::to_wxyz(q).dot(dq);
  EXPECT_NEAR(along, 0.0, kTight);
}

TEST(Quaternion, PackingUsesStateOrderNotEigenCoeffsOrder) {
  // The trap this guards: Eigen stores [x, y, z, w]. If to_wxyz ever delegates
  // to coeffs() this fails immediately.
  const Quaternion q(0.1, 0.2, 0.3, 0.4);
  const Eigen::Vector4d packed = galata::core::to_wxyz(q);
  EXPECT_DOUBLE_EQ(packed(0), 0.1);
  EXPECT_DOUBLE_EQ(packed(1), 0.2);
  EXPECT_DOUBLE_EQ(packed(2), 0.3);
  EXPECT_DOUBLE_EQ(packed(3), 0.4);

  const Quaternion back = galata::core::from_wxyz(packed);
  EXPECT_DOUBLE_EQ(back.w(), 0.1);
  EXPECT_DOUBLE_EQ(back.x(), 0.2);
  EXPECT_DOUBLE_EQ(back.y(), 0.3);
  EXPECT_DOUBLE_EQ(back.z(), 0.4);
}

TEST(Quaternion, NormalisedProducesAUnitQuaternion) {
  const Quaternion q = galata::core::normalised(Quaternion(2.0, -3.0, 6.0, 4.0));
  EXPECT_NEAR(q.norm(), 1.0, kTight);
}

TEST(Quaternion, CanonicalCollapsesTheDoubleCoverWithoutChangingTheRotation) {
  const Quaternion q = galata::core::quaternion_from_euler(
      EulerAngles{degrees_to_radians(150.0), degrees_to_radians(60.0), degrees_to_radians(170.0)});
  const Quaternion negated(-q.w(), -q.x(), -q.y(), -q.z());

  EXPECT_TRUE(galata::core::dcm_ned_from_body(q).isApprox(galata::core::dcm_ned_from_body(negated),
                                                          kTight));
  EXPECT_GE(galata::core::canonical(negated).w(), 0.0);
  EXPECT_GE(galata::core::canonical(q).w(), 0.0);
  EXPECT_NEAR(galata::core::angular_distance(q, negated), 0.0, kTight);
}

TEST(Quaternion, AngularDistanceIsTheRotationAngle) {
  for (double angle_deg = 0.0; angle_deg <= 180.0; angle_deg += 15.0) {
    const Quaternion a = galata::core::identity_attitude();
    const Quaternion b =
        galata::core::quaternion_from_euler(EulerAngles{0.0, 0.0, degrees_to_radians(angle_deg)});
    EXPECT_NEAR(galata::core::angular_distance(a, b), degrees_to_radians(angle_deg), 1e-7);
  }
}

}  // namespace
