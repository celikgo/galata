// SPDX-License-Identifier: Apache-2.0
//
// Uniform rotation sampling follows:
//   K. Shoemake, "Uniform Random Rotations", in D. Kirk (ed.), Graphics Gems
//   III, Academic Press, 1992, pp. 124-132.
// The normal-deviate construction (four independent standard normals,
// normalised) is the equivalent formulation via the rotational invariance of
// the 4-D Gaussian, and is used here because it needs no branch.

#include "property_generators.hpp"

#include "galata/core/constants.hpp"
#include "galata/core/quaternion.hpp"

#include <cmath>

namespace galata::testing {

double Generator::unit() {
  // Top 53 bits scaled by 2^-53: exactly representable, uniform on [0, 1).
  const std::uint64_t bits = engine_() >> 11;
  return static_cast<double>(bits) * 0x1.0p-53;
}

double Generator::range(double low, double high) {
  return low + (high - low) * unit();
}

double Generator::normal() {
  // Box-Muller. The first uniform is nudged off zero because log(0) is -inf.
  const double u1 = std::fmax(unit(), 0x1.0p-53);
  const double u2 = unit();
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(galata::core::kTwoPi * u2);
}

Eigen::Vector3d Generator::vector3(double magnitude) {
  return Eigen::Vector3d(
      range(-magnitude, magnitude), range(-magnitude, magnitude), range(-magnitude, magnitude));
}

galata::core::Quaternion Generator::rotation() {
  const double w = normal();
  const double x = normal();
  const double y = normal();
  const double z = normal();
  const galata::core::Quaternion q(w, x, y, z);
  // A four-dimensional Gaussian is spherically symmetric, so normalising gives
  // a point uniform on S^3, which is a uniform rotation. Degenerate draws are
  // vanishingly unlikely but the fallback keeps the generator total.
  if (q.norm() < 1e-12) {
    return galata::core::identity_attitude();
  }
  return galata::core::normalised(q);
}

galata::core::State Generator::state() {
  galata::core::State s;
  s.position_ned_m = vector3(10000.0);
  s.velocity_body_m_s = vector3(300.0);
  s.attitude_body_to_ned = rotation();
  s.angular_rate_body_rad_s = vector3(2.0);
  return s;
}

}  // namespace galata::testing
