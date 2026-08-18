// SPDX-License-Identifier: Apache-2.0
//
// Mathematical and physical constants used across the numerical core.
//
// Each carries the body that defines it. A constant without a defining source
// is a magic number with better manners.

#ifndef GALATA_CORE_CONSTANTS_HPP
#define GALATA_CORE_CONSTANTS_HPP

#include <numbers>

namespace galata::core {

inline constexpr double kPi = std::numbers::pi_v<double>;
inline constexpr double kTwoPi = 2.0 * kPi;
inline constexpr double kHalfPi = kPi / 2.0;

// Standard acceleration of free fall. Exact by definition, fixed by the 3rd
// General Conference on Weights and Measures (CGPM), 1901, Resolution 2. The
// U.S. Standard Atmosphere, 1976 adopts the same value as its defining
// constant g_0.
//
// What this is NOT: this is not the local gravitational acceleration anywhere
// on Earth. Real g varies by roughly +/- 0.3% between the equator and the
// poles, and decreases with altitude by about 3.1e-6 m/s^2 per metre. A flat-
// Earth simulation using this constant is in error by that much, in the
// direction of overstating weight at the equator and understating it at the
// poles.
inline constexpr double kStandardGravity = 9.80665;  // m/s^2

}  // namespace galata::core

#endif  // GALATA_CORE_CONSTANTS_HPP
