// SPDX-License-Identifier: Apache-2.0
//
// Attitude quaternion operations.
//
// Reference for the conventions and every formula below:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation: Dynamics, Controls Design, and Autonomous Systems", 3rd ed.,
//   Wiley, 2016, chapter 1 (reference frames) and chapter 2 (rigid-body
//   equations of motion).
//   J. Diebel, "Representing Attitude: Euler Angles, Unit Quaternions, and
//   Rotation Vectors", Stanford University, 2006 — for the Hamilton-convention
//   rotation matrix and the Euler extraction, and for the enumeration of the
//   twelve Euler sequences that makes convention mismatches findable.
//
// The normative statement of which of the many conventions in those references
// galata uses is docs/adr/0002-state-and-frame-conventions.md. This file
// implements that one and no other.
//
// What this is NOT: none of this models attitude *uncertainty*. These are exact
// kinematic relations on an exactly-known attitude. Nothing here belongs in an
// estimator's covariance propagation.

#include "galata/core/quaternion.hpp"

#include "galata/core/constants.hpp"

#include <cmath>

namespace galata::core {

Quaternion identity_attitude() noexcept {
  return Quaternion(1.0, 0.0, 0.0, 0.0);
}

Eigen::Matrix3d dcm_ned_from_body(const Quaternion& q_body_to_ned) noexcept {
  // Written out from ADR-0002 rather than delegated to Eigen, so that the
  // matrix this project documents and the matrix it computes are the same
  // object. test_quaternion.cpp asserts this equals Eigen's toRotationMatrix(),
  // which is what turns the duplication into a convention check rather than a
  // second place to be wrong.
  const double w = q_body_to_ned.w();
  const double x = q_body_to_ned.x();
  const double y = q_body_to_ned.y();
  const double z = q_body_to_ned.z();

  Eigen::Matrix3d r;
  r(0, 0) = 1.0 - 2.0 * (y * y + z * z);
  r(0, 1) = 2.0 * (x * y - w * z);
  r(0, 2) = 2.0 * (x * z + w * y);

  r(1, 0) = 2.0 * (x * y + w * z);
  r(1, 1) = 1.0 - 2.0 * (x * x + z * z);
  r(1, 2) = 2.0 * (y * z - w * x);

  r(2, 0) = 2.0 * (x * z - w * y);
  r(2, 1) = 2.0 * (y * z + w * x);
  r(2, 2) = 1.0 - 2.0 * (x * x + y * y);
  return r;
}

Eigen::Matrix3d dcm_body_from_ned(const Quaternion& q_body_to_ned) noexcept {
  return dcm_ned_from_body(q_body_to_ned).transpose();
}

double gimbal_lock_proximity(const Quaternion& q_body_to_ned) noexcept {
  // sin(pitch) = 2 (w y - x z); the reported quantity is |cos(pitch)|.
  const double sin_pitch =
      2.0 * (q_body_to_ned.w() * q_body_to_ned.y() - q_body_to_ned.x() * q_body_to_ned.z());
  const double clamped = std::fmin(1.0, std::fmax(-1.0, sin_pitch));
  return std::sqrt(std::fmax(0.0, 1.0 - clamped * clamped));
}

EulerAngles euler_from_quaternion(const Quaternion& q_body_to_ned) noexcept {
  const double w = q_body_to_ned.w();
  const double x = q_body_to_ned.x();
  const double y = q_body_to_ned.y();
  const double z = q_body_to_ned.z();

  // sin(theta) = -R_body<-ned(0,2) = 2 (w y - x z).
  //
  // Clamped before asin because a quaternion that is normalised only to within
  // rounding can push this a few ulp outside [-1, 1], and asin of 1.0000000001
  // is NaN. A NaN attitude propagates silently through an entire trajectory,
  // so this clamp is load-bearing rather than defensive.
  const double sin_pitch = 2.0 * (w * y - x * z);
  const double sin_pitch_clamped = std::fmin(1.0, std::fmax(-1.0, sin_pitch));

  EulerAngles euler;
  euler.pitch_rad = std::asin(sin_pitch_clamped);

  // At |sin(pitch)| = 1 the roll and yaw axes are collinear and only their
  // combination is determined. Rather than let atan2(0, 0) decide, fold the
  // whole rotation into yaw and report zero roll. This is a documented choice,
  // not a recovery of information that is not there — see the header.
  const double kGimbalLockThreshold = 1.0 - 1e-12;
  if (std::fabs(sin_pitch_clamped) >= kGimbalLockThreshold) {
    euler.roll_rad = 0.0;
    euler.yaw_rad = 2.0 * std::atan2(z, w);
    return euler;
  }

  // phi = atan2(R_body<-ned(1,2), R_body<-ned(2,2)), psi = atan2(R(0,1), R(0,0)),
  // with R_body<-ned = R_ned<-body^T. Expanded in quaternion components so this
  // does not depend on having formed the matrix.
  euler.roll_rad = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  euler.yaw_rad = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return euler;
}

Quaternion quaternion_from_euler(const EulerAngles& euler) noexcept {
  const double half_roll = 0.5 * euler.roll_rad;
  const double half_pitch = 0.5 * euler.pitch_rad;
  const double half_yaw = 0.5 * euler.yaw_rad;

  const double cr = std::cos(half_roll);
  const double sr = std::sin(half_roll);
  const double cp = std::cos(half_pitch);
  const double sp = std::sin(half_pitch);
  const double cy = std::cos(half_yaw);
  const double sy = std::sin(half_yaw);

  // 3-2-1 composition, giving the body-to-NED quaternion.
  const double w = cr * cp * cy + sr * sp * sy;
  const double x = sr * cp * cy - cr * sp * sy;
  const double y = cr * sp * cy + sr * cp * sy;
  const double z = cr * cp * sy - sr * sp * cy;
  return normalised(Quaternion(w, x, y, z));
}

Eigen::Vector4d quaternion_derivative(const Quaternion& q_body_to_ned,
                                      const Eigen::Vector3d& omega_body_rad_s) noexcept {
  const double w = q_body_to_ned.w();
  const double x = q_body_to_ned.x();
  const double y = q_body_to_ned.y();
  const double z = q_body_to_ned.z();

  const double p = omega_body_rad_s.x();
  const double q = omega_body_rad_s.y();
  const double r = omega_body_rad_s.z();

  // qdot = 0.5 q (x) [0, p, q, r], Hamilton product, expanded.
  Eigen::Vector4d dq;
  dq(0) = 0.5 * (-x * p - y * q - z * r);
  dq(1) = 0.5 * (w * p + y * r - z * q);
  dq(2) = 0.5 * (w * q + z * p - x * r);
  dq(3) = 0.5 * (w * r + x * q - y * p);
  return dq;
}

Quaternion normalised(const Quaternion& q) noexcept {
  Quaternion out = q;
  out.normalize();
  return out;
}

Quaternion canonical(const Quaternion& q) noexcept {
  if (q.w() < 0.0) {
    return Quaternion(-q.w(), -q.x(), -q.y(), -q.z());
  }
  return q;
}

Eigen::Vector4d to_wxyz(const Quaternion& q) noexcept {
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

Quaternion from_wxyz(const Eigen::Vector4d& wxyz) noexcept {
  return Quaternion(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
}

double angular_distance(const Quaternion& a, const Quaternion& b) noexcept {
  // Formulated as an atan2 of the relative rotation rather than as
  // acos(|<a, b>|), which is the obvious spelling and is badly conditioned
  // exactly where it matters most.
  //
  // Near zero angle, <a, b> approaches 1 and acos has infinite derivative
  // there: d(acos)/dx = -1/sqrt(1 - x^2). An error of one ulp in the dot
  // product, which is unavoidable, becomes an error of about 2*sqrt(2*eps)
  // ~ 3e-8 radians in the answer. That is 25 million times worse than the
  // representation deserves, and it showed up immediately as three failing
  // property tests asserting that renormalisation and canonicalisation do not
  // move an attitude.
  //
  // atan2(||v||, |w|) has no such problem: it is accurate over the whole
  // range, and returns exactly zero for identical inputs.
  //
  // Precondition: a and b are unit quaternions, so that the conjugate is the
  // inverse. The callers in galata always satisfy this; a non-unit input
  // scales the relative quaternion, and atan2 is homogeneous, so the answer
  // degrades gracefully rather than becoming NaN.
  const Quaternion relative = a.conjugate() * b;
  const double vector_norm = std::sqrt(relative.x() * relative.x() + relative.y() * relative.y()
                                       + relative.z() * relative.z());
  // |w| rather than w folds the double cover: a and -a are the same attitude,
  // so the answer must be the shorter of the two arcs, in [0, pi].
  return 2.0 * std::atan2(vector_norm, std::fabs(relative.w()));
}

}  // namespace galata::core
