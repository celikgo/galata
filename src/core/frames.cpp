// SPDX-License-Identifier: Apache-2.0
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 1 — reference frames, the
//   wind/stability/body relationships and the aerodynamic angle definitions.
//   B. Etkin and L. D. Reid, "Dynamics of Flight: Stability and Control",
//   3rd ed., Wiley, 1996, chapter 1 — the same relationships in the notation
//   most of the flight-dynamics literature uses.
//
// The normative convention statement is docs/adr/0002-state-and-frame-
// conventions.md.

#include "galata/core/frames.hpp"

#include <cmath>

namespace galata::core {

Eigen::Matrix3d rotation_x(double angle_rad) noexcept {
  const double c = std::cos(angle_rad);
  const double s = std::sin(angle_rad);
  Eigen::Matrix3d r;
  r << 1.0, 0.0, 0.0,  //
      0.0, c, s,       //
      0.0, -s, c;
  return r;
}

Eigen::Matrix3d rotation_y(double angle_rad) noexcept {
  const double c = std::cos(angle_rad);
  const double s = std::sin(angle_rad);
  Eigen::Matrix3d r;
  r << c, 0.0, -s,    //
      0.0, 1.0, 0.0,  //
      s, 0.0, c;
  return r;
}

Eigen::Matrix3d rotation_z(double angle_rad) noexcept {
  const double c = std::cos(angle_rad);
  const double s = std::sin(angle_rad);
  Eigen::Matrix3d r;
  r << c, s, 0.0,  //
      -s, c, 0.0,  //
      0.0, 0.0, 1.0;
  return r;
}

Eigen::Matrix3d dcm_stability_from_body(double alpha_rad) noexcept {
  // Minus alpha, because the stability x-axis points along the projection of
  // the velocity vector, which lies at +alpha BELOW the body x-axis in a
  // z-down frame. Getting this sign backwards produces a stability-axis
  // linearisation whose Z-force derivative has the wrong sign, which then
  // produces a short period that damps when it should diverge.
  return rotation_y(-alpha_rad);
}

Eigen::Matrix3d dcm_wind_from_body(double alpha_rad, double beta_rad) noexcept {
  return rotation_z(beta_rad) * dcm_stability_from_body(alpha_rad);
}

Eigen::Matrix3d dcm_body_from_wind(double alpha_rad, double beta_rad) noexcept {
  return dcm_wind_from_body(alpha_rad, beta_rad).transpose();
}

}  // namespace galata::core
