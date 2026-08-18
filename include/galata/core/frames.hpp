// SPDX-License-Identifier: Apache-2.0
//
// Reference frames and the rotations between them.
//
// Frames, normative, from ADR-0002
// (docs/adr/0002-state-and-frame-conventions.md):
//
//   NED       navigation: x north, y east, z DOWN. Flat, non-rotating.
//   Body FRD  x forward out the nose, y out the RIGHT wing, z down.
//   Stability the body frame rotated by -alpha about the body y-axis.
//   Wind      the stability frame rotated by +beta about the stability z-axis.
//
// Every matrix here follows the "transform components" convention: R_b_from_a
// takes a vector's components expressed in frame a and returns its components
// in frame b. The rows of R_b_from_a are frame b's basis vectors expressed in
// frame a. Both statements say the same thing; the second is the one to check
// a sign against.
//
// What this is NOT: these are pure kinematic rotations. Nothing here knows
// about wind, so "wind frame" means "aligned with the aircraft's velocity
// relative to the air mass" — the air-relative velocity is an input, and any
// steady wind or gust field has already been accounted for by whoever computed
// it. A rotation into the wind frame using an inertial velocity in a moving
// air mass is a mistake this file cannot detect.

#ifndef GALATA_CORE_FRAMES_HPP
#define GALATA_CORE_FRAMES_HPP

#include <Eigen/Core>

namespace galata::core {

// Elementary rotations. rotation_x(angle) is the matrix transforming components
// into a frame rotated by +angle about the x-axis of the original frame.
[[nodiscard]] Eigen::Matrix3d rotation_x(double angle_rad) noexcept;
[[nodiscard]] Eigen::Matrix3d rotation_y(double angle_rad) noexcept;
[[nodiscard]] Eigen::Matrix3d rotation_z(double angle_rad) noexcept;

// R_stability<-body = Ry(-alpha).
[[nodiscard]] Eigen::Matrix3d dcm_stability_from_body(double alpha_rad) noexcept;

// R_body<-stability, the transpose of the above.
//
// This is the rotation an aerodynamic buildup needs, and it is NOT the same as
// dcm_body_from_wind. Lift and drag are conventionally referred to the
// STABILITY axes — rotated from body by alpha alone — with the side force given
// directly in body axes. Rotating the lift/drag/side triple through the full
// wind rotation instead adds a spurious -D sin(beta) to the body y force, which
// shows up as a side-force derivative roughly 19% too large.
[[nodiscard]] Eigen::Matrix3d dcm_body_from_stability(double alpha_rad) noexcept;

// R_wind<-body = Rz(beta) Ry(-alpha).
//
// Fixed by the requirement that it map the body-axis velocity components
// (u, v, w) onto (V, 0, 0). That requirement, not the composition order, is
// what the test checks.
[[nodiscard]] Eigen::Matrix3d dcm_wind_from_body(double alpha_rad, double beta_rad) noexcept;

// R_body<-wind, the transpose of the above.
[[nodiscard]] Eigen::Matrix3d dcm_body_from_wind(double alpha_rad, double beta_rad) noexcept;

}  // namespace galata::core

#endif  // GALATA_CORE_FRAMES_HPP
