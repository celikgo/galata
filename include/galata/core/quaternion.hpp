// SPDX-License-Identifier: Apache-2.0
//
// Attitude quaternions.
//
// Convention, normative, from ADR-0002
// (docs/adr/0002-state-and-frame-conventions.md):
//
//   Hamilton convention, scalar-first q = [w, x, y, z], representing the
//   body-to-NED rotation. That is, q rotates a vector's components FROM the
//   body frame TO the navigation frame:  v_ned = q (x) v_body (x) q*.
//
// Half the defects in flight software are quaternion-convention mismatches, so
// this is stated in the ADR, restated here, and checked by a test that
// reconstructs the rotation matrix from the ADR's formula and compares it to
// Eigen's own.
//
// TRAP: Eigen::Quaterniond stores its coefficients scalar-LAST. Its
// constructor is Quaterniond(w, x, y, z) — scalar-first, matching this
// project — but `q.coeffs()` returns [x, y, z, w]. Never index coeffs()
// directly. Use w()/x()/y()/z(), or the packing helpers below, which are the
// only place in galata that touches the storage order.
//
// What this is NOT: these functions do not enforce that a quaternion is a unit
// quaternion. Passing a non-normalised quaternion to dcm_ned_from_body()
// produces a matrix that is not a rotation — it will scale as well as rotate.
// The integrator renormalises every step; anything constructing a quaternion by
// other means is responsible for calling normalised().

#ifndef GALATA_CORE_QUATERNION_HPP
#define GALATA_CORE_QUATERNION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace galata::core {

// Scalar-first at construction, per ADR-0002. Aliased rather than wrapped: a
// wrapper would have to re-export most of Eigen's geometry module to be useful,
// and the trap this file exists to prevent is coeffs(), which a wrapper would
// not remove.
using Quaternion = Eigen::Quaterniond;

// Euler angles, 3-2-1 (ZYX) sequence: yaw about z, then pitch about y, then
// roll about x, taking NED to body. Derived output only — never integrated
// state (ADR-0002).
struct EulerAngles {
  double roll_rad = 0.0;   // phi, rad, in (-pi, pi]
  double pitch_rad = 0.0;  // theta, rad, in [-pi/2, pi/2]
  double yaw_rad = 0.0;    // psi, rad, in (-pi, pi]
};

// Identity attitude: body axes aligned with NED.
[[nodiscard]] Quaternion identity_attitude() noexcept;

// R_ned<-body. Transforms vector COMPONENTS from body axes to NED axes.
[[nodiscard]] Eigen::Matrix3d dcm_ned_from_body(const Quaternion& q_body_to_ned) noexcept;

// R_body<-ned, the transpose of the above.
[[nodiscard]] Eigen::Matrix3d dcm_body_from_ned(const Quaternion& q_body_to_ned) noexcept;

// Euler angles from the attitude quaternion.
//
// At pitch = +/- 90 degrees the roll/yaw split is not defined: only their sum
// or difference is. This function returns roll = 0 and folds the whole rotation
// into yaw at the singularity, which is a choice, not a recovery. Use
// gimbal_lock_proximity() to find out whether you are near it before believing
// the roll and yaw it returns.
[[nodiscard]] EulerAngles euler_from_quaternion(const Quaternion& q_body_to_ned) noexcept;

// Attitude quaternion from Euler angles, inverse of the above away from the
// singularity. The returned quaternion is normalised.
[[nodiscard]] Quaternion quaternion_from_euler(const EulerAngles& euler) noexcept;

// |cos(pitch)|, in [0, 1]. Zero at gimbal lock, one wings-level. The Euler
// extraction's conditioning degrades as this approaches zero; below about 1e-6
// the roll and yaw it reports are numerical noise about their true sum.
[[nodiscard]] double gimbal_lock_proximity(const Quaternion& q_body_to_ned) noexcept;

// Quaternion kinematics: qdot = 0.5 * q (x) [0, p, q, r], with omega in body
// axes. Returned in the state vector's storage order [w, x, y, z] — NOT
// Eigen's coeffs() order — so it can be written straight into a state
// derivative.
[[nodiscard]] Eigen::Vector4d quaternion_derivative(
    const Quaternion& q_body_to_ned,
    const Eigen::Vector3d& omega_body_rad_s) noexcept;

// q / ||q||. Unconditional, branch-free, applied every integrator step; see
// ADR-0004 for why this is not conditional on a drift threshold.
[[nodiscard]] Quaternion normalised(const Quaternion& q) noexcept;

// q and -q are the same rotation. This picks the representative with w >= 0.
//
// Use it for comparison and serialisation ONLY. Applying it inside a
// trajectory makes the state discontinuous wherever w crosses zero, which
// destroys any finite-difference Jacobian taken across that point (ADR-0002).
[[nodiscard]] Quaternion canonical(const Quaternion& q) noexcept;

// Pack to [w, x, y, z] — the state vector's order, not Eigen's.
[[nodiscard]] Eigen::Vector4d to_wxyz(const Quaternion& q) noexcept;

// Unpack from [w, x, y, z]. Does not normalise.
[[nodiscard]] Quaternion from_wxyz(const Eigen::Vector4d& wxyz) noexcept;

// Angle of the rotation taking a to b, in radians, in [0, pi]. Sign-agnostic:
// a and -a give the same answer, which is what you want when comparing
// attitudes.
//
// Accurate at small angles, which acos(|<a, b>|) is not — see the
// implementation for why that matters. Returns exactly 0.0 for identical
// inputs, so it is usable as a convergence test.
//
// Requires a and b to be unit quaternions.
[[nodiscard]] double angular_distance(const Quaternion& a, const Quaternion& b) noexcept;

// The exponential map: the attitude perturbation exp(e/2) written as a
// quaternion, where `e` is a rotation vector — an axis scaled by an angle in
// radians. The half-angle is the quaternion's, because a quaternion rotates by
// twice its own angle.
//
// This is the `delta` that `linearize::extended_from_chart` composes onto a
// reference attitude, and it is here rather than beside its one caller because
// its inverse below has to agree with it exactly, and two functions that must
// agree belong in one place.
[[nodiscard]] Quaternion quaternion_from_rotation_vector(const Eigen::Vector3d& rotation_rad);

// The logarithm: the rotation vector `e` for which
// `quaternion_from_rotation_vector(e)` reproduces `q`.
//
// TWO THINGS THIS HAS TO GET RIGHT, both of which are silent when wrong.
//
// THE DOUBLE COVER. q and -q are the same attitude and their naive logarithms
// differ by a full turn. The input is canonicalised first, so the result is
// always the SHORT way round — |e| <= pi. Without that a controller that
// crossed a sign boundary would command a rotation the long way at full
// authority, having been told the error had suddenly grown by 2 pi.
//
// THE SMALL-ANGLE LIMIT. The closed form divides by the vector part's norm,
// which vanishes at zero rotation — exactly where a controller spends its time.
// Below a declared threshold a series is used instead. The threshold is a
// stated constant, not whichever value happened to be numerically quiet.
//
// Round-trips with the exponential to a bound the tests state, at the identity,
// across the series boundary, and up to a half turn where the parameterisation
// itself stops being unique.
[[nodiscard]] Eigen::Vector3d rotation_vector_from_quaternion(const Quaternion& q);

}  // namespace galata::core

#endif  // GALATA_CORE_QUATERNION_HPP
