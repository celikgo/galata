// SPDX-License-Identifier: Apache-2.0
//
// Unit conversions (ADR-0003). Every factor these tests assert is exact by
// international agreement, so the expected values are written as the defining
// products rather than as decimal approximations, and compared exactly.
//
// The point of asserting exactness rather than a tolerance: a conversion factor
// that has been rounded is a conversion factor someone typed from memory.

#include "galata/units.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace {

using namespace galata::units;

TEST(Units, FootIsExactlyPointThreeZeroFourEightMetres) {
  // International yard and pound agreement, 1959.
  EXPECT_DOUBLE_EQ(feet_to_metres(1.0), 0.3048);
  // The altitude every cruise study is quoted at.
  EXPECT_DOUBLE_EQ(feet_to_metres(10000.0), 3048.0);
  EXPECT_DOUBLE_EQ(feet_to_metres(0.0), 0.0);
  EXPECT_DOUBLE_EQ(feet_to_metres(-1000.0), -304.8);
}

TEST(Units, NauticalMileIsExactlyEighteenFiftyTwoMetres) {
  EXPECT_DOUBLE_EQ(nautical_miles_to_metres(1.0), 1852.0);
  EXPECT_DOUBLE_EQ(metres_to_nautical_miles(1852.0), 1.0);
}

TEST(Units, KnotFollowsFromTheNauticalMile) {
  EXPECT_DOUBLE_EQ(knots_to_metres_per_second(1.0), 1852.0 / 3600.0);
  EXPECT_DOUBLE_EQ(knots_to_metres_per_second(3600.0), 1852.0);
  // A knot is a little over half a metre per second; this catches a factor
  // that has been inverted, which a round-trip test cannot.
  EXPECT_GT(knots_to_metres_per_second(1.0), 0.51);
  EXPECT_LT(knots_to_metres_per_second(1.0), 0.52);
}

TEST(Units, FootPerMinuteFollowsFromTheFoot) {
  EXPECT_DOUBLE_EQ(feet_per_minute_to_metres_per_second(60.0), 0.3048);
  // 1000 fpm is the vertical speed every climb checklist is written around.
  EXPECT_DOUBLE_EQ(feet_per_minute_to_metres_per_second(1000.0), 304.8 / 60.0);
}

TEST(Units, DegreeIsPiOverOneEighty) {
  EXPECT_DOUBLE_EQ(degrees_to_radians(180.0), std::numbers::pi_v<double>);
  EXPECT_DOUBLE_EQ(degrees_to_radians(90.0), std::numbers::pi_v<double> / 2.0);
  EXPECT_DOUBLE_EQ(radians_to_degrees(std::numbers::pi_v<double>), 180.0);
  EXPECT_DOUBLE_EQ(degrees_to_radians(0.0), 0.0);
}

TEST(Units, CelsiusOffsetIsExactlyTwoSevenThreePointOneFive) {
  EXPECT_DOUBLE_EQ(celsius_to_kelvin(0.0), 273.15);
  // ISA sea-level temperature, in both scales.
  EXPECT_DOUBLE_EQ(celsius_to_kelvin(15.0), 288.15);
  EXPECT_DOUBLE_EQ(kelvin_to_celsius(288.15), 15.0);
  EXPECT_DOUBLE_EQ(celsius_to_kelvin(-273.15), 0.0);
}

TEST(Units, PoundForceIsThePoundUnderStandardGravity) {
  // 0.45359237 kg exactly (1959) times 9.80665 m/s^2 exactly (3rd CGPM, 1901).
  EXPECT_DOUBLE_EQ(pounds_force_to_newtons(1.0), 0.45359237 * 9.80665);
  EXPECT_DOUBLE_EQ(newtons_to_pounds_force(0.45359237 * 9.80665), 1.0);
  // Sanity on magnitude: a pound-force is about four and a half newtons.
  EXPECT_GT(pounds_force_to_newtons(1.0), 4.44);
  EXPECT_LT(pounds_force_to_newtons(1.0), 4.45);
}

TEST(Units, SlugFollowsFromPoundForceAndFoot) {
  EXPECT_DOUBLE_EQ(slugs_to_kilograms(1.0), (0.45359237 * 9.80665) / 0.3048);
  // A slug is about 14.6 kg — the check that catches an inverted factor.
  EXPECT_GT(slugs_to_kilograms(1.0), 14.59);
  EXPECT_LT(slugs_to_kilograms(1.0), 14.60);
}

TEST(Units, EveryConversionRoundTrips) {
  const double kTolerance = 1e-12;
  for (const double value : {-1234.5, -1.0, 0.0, 1.0, 42.0, 98765.4}) {
    EXPECT_NEAR(
        metres_to_feet(feet_to_metres(value)), value, kTolerance * std::fabs(value) + kTolerance);
    EXPECT_NEAR(metres_to_nautical_miles(nautical_miles_to_metres(value)),
                value,
                kTolerance * std::fabs(value) + kTolerance);
    EXPECT_NEAR(metres_per_second_to_knots(knots_to_metres_per_second(value)),
                value,
                kTolerance * std::fabs(value) + kTolerance);
    EXPECT_NEAR(metres_per_second_to_feet_per_minute(feet_per_minute_to_metres_per_second(value)),
                value,
                kTolerance * std::fabs(value) + kTolerance);
    EXPECT_NEAR(radians_to_degrees(degrees_to_radians(value)),
                value,
                kTolerance * std::fabs(value) + kTolerance);
    EXPECT_NEAR(kelvin_to_celsius(celsius_to_kelvin(value)), value, 1e-9);
    EXPECT_NEAR(newtons_to_pounds_force(pounds_force_to_newtons(value)),
                value,
                kTolerance * std::fabs(value) + kTolerance);
    EXPECT_NEAR(kilograms_to_slugs(slugs_to_kilograms(value)),
                value,
                kTolerance * std::fabs(value) + kTolerance);
  }
}

TEST(Units, ConversionsAreUsableInConstantExpressions) {
  // The boundary is compile-time where it can be, so a UI constant costs
  // nothing at run time.
  static_assert(feet_to_metres(1.0) == 0.3048);
  static_assert(celsius_to_kelvin(15.0) == 288.15);
  static_assert(nautical_miles_to_metres(1.0) == 1852.0);
  SUCCEED();
}

}  // namespace
