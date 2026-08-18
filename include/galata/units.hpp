// SPDX-License-Identifier: Apache-2.0
//
// Unit conversion — the boundary, and the only one.
//
// ADR-0003 (docs/adr/0003-strict-si-and-boundary-conversion.md): the numerical
// core is strictly SI. Degrees, feet, knots and degrees Celsius exist in the
// user interface layer and in file-format adapters, nowhere else, and they are
// converted here.
//
// This file is the single exemption in scripts/check-si-boundary.sh, because it
// is the definition site. Every conversion factor below is exact by definition.
// A unit whose conversion factor cannot be written exactly is not a unit this
// project converts.
//
// What this is NOT: this is not a dimensional-analysis type system. It cannot
// stop you from passing feet to a function expecting metres — only the CI gate
// and review do that. ADR-0003 records why a `Quantity<Length>` type was
// rejected for the 1.x series, and it is a cost argument, not a claim that this
// is better.

#ifndef GALATA_UNITS_HPP
#define GALATA_UNITS_HPP

#include <numbers>

namespace galata::units {

// --- Exact defining factors ------------------------------------------------
// Each is exact by international agreement or by definition, with the basis
// named. These are the only magic numbers in the project that are permitted to
// look like unit conversions, because they are.

// International foot, exact since the 1959 international yard and pound
// agreement between the national standards laboratories of the US, UK,
// Canada, Australia, New Zealand and South Africa.
inline constexpr double kMetresPerFoot = 0.3048;

// International nautical mile, exact by definition (adopted 1929, First
// International Extraordinary Hydrographic Conference; US adoption 1954).
inline constexpr double kMetresPerNauticalMile = 1852.0;

// Knot: one nautical mile per hour. Exact, following from the above.
inline constexpr double kMetresPerSecondPerKnot = kMetresPerNauticalMile / 3600.0;

// Foot per minute — the unit every vertical-speed indicator is calibrated in.
inline constexpr double kMetresPerSecondPerFootPerMinute = kMetresPerFoot / 60.0;

// Exact by definition of the degree.
inline constexpr double kRadiansPerDegree = std::numbers::pi_v<double> / 180.0;

// Exact by the definition of the kelvin relative to the Celsius scale.
inline constexpr double kKelvinAtZeroCelsius = 273.15;

// Pound-force: the force exerted by one avoirdupois pound (0.45359237 kg
// exactly, 1959 agreement) under standard gravity (9.80665 m/s^2 exactly,
// 3rd CGPM 1901). The product is therefore exact.
inline constexpr double kNewtonsPerPoundForce = 0.45359237 * 9.80665;

// Slug: the mass accelerated at one foot per second squared by one
// pound-force. Exact, following from the two constants above.
inline constexpr double kKilogramsPerSlug = kNewtonsPerPoundForce / kMetresPerFoot;

// --- Length ----------------------------------------------------------------

[[nodiscard]] constexpr double feet_to_metres(double feet) noexcept {
  return feet * kMetresPerFoot;
}

[[nodiscard]] constexpr double metres_to_feet(double metres) noexcept {
  return metres / kMetresPerFoot;
}

[[nodiscard]] constexpr double nautical_miles_to_metres(double nautical_miles) noexcept {
  return nautical_miles * kMetresPerNauticalMile;
}

[[nodiscard]] constexpr double metres_to_nautical_miles(double metres) noexcept {
  return metres / kMetresPerNauticalMile;
}

// --- Speed -----------------------------------------------------------------

[[nodiscard]] constexpr double knots_to_metres_per_second(double knots) noexcept {
  return knots * kMetresPerSecondPerKnot;
}

[[nodiscard]] constexpr double metres_per_second_to_knots(double metres_per_second) noexcept {
  return metres_per_second / kMetresPerSecondPerKnot;
}

[[nodiscard]] constexpr double feet_per_minute_to_metres_per_second(
    double feet_per_minute) noexcept {
  return feet_per_minute * kMetresPerSecondPerFootPerMinute;
}

[[nodiscard]] constexpr double metres_per_second_to_feet_per_minute(
    double metres_per_second) noexcept {
  return metres_per_second / kMetresPerSecondPerFootPerMinute;
}

// --- Angle -----------------------------------------------------------------

[[nodiscard]] constexpr double degrees_to_radians(double degrees) noexcept {
  return degrees * kRadiansPerDegree;
}

[[nodiscard]] constexpr double radians_to_degrees(double radians) noexcept {
  return radians / kRadiansPerDegree;
}

// --- Temperature -----------------------------------------------------------

[[nodiscard]] constexpr double celsius_to_kelvin(double celsius) noexcept {
  return celsius + kKelvinAtZeroCelsius;
}

[[nodiscard]] constexpr double kelvin_to_celsius(double kelvin) noexcept {
  return kelvin - kKelvinAtZeroCelsius;
}

// --- Force and mass --------------------------------------------------------

[[nodiscard]] constexpr double pounds_force_to_newtons(double pounds_force) noexcept {
  return pounds_force * kNewtonsPerPoundForce;
}

[[nodiscard]] constexpr double newtons_to_pounds_force(double newtons) noexcept {
  return newtons / kNewtonsPerPoundForce;
}

[[nodiscard]] constexpr double slugs_to_kilograms(double slugs) noexcept {
  return slugs * kKilogramsPerSlug;
}

[[nodiscard]] constexpr double kilograms_to_slugs(double kilograms) noexcept {
  return kilograms / kKilogramsPerSlug;
}

}  // namespace galata::units

#endif  // GALATA_UNITS_HPP
