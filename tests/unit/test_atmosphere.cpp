// SPDX-License-Identifier: Apache-2.0
//
// Domain checks for the ideal-gas and sound-speed equations in COESA,
// U.S. Standard Atmosphere, 1976, equations (42), (50), and (51).

#include "galata/core/atmosphere.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace {

TEST(AtmosphereDomain, RejectsNonFiniteTemperatureOffsets) {
  const double infinity = std::numeric_limits<double>::infinity();
  for (const double offset : {infinity, -infinity, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW((void)galata::core::isa(0.0, offset), std::invalid_argument);
  }
}

TEST(AtmosphereDomain, RejectsZeroAndNegativeAbsoluteTemperatureAtTheQueryAltitude) {
  for (const double altitude : {0.0, 11000.0, 50000.0, 86000.0}) {
    const double standard_temperature = galata::core::isa(altitude).temperature_k;
    EXPECT_THROW((void)galata::core::isa(altitude, -standard_temperature), std::invalid_argument);
    EXPECT_THROW((void)galata::core::isa(altitude, -standard_temperature - 1.0),
                 std::invalid_argument);
  }
}

TEST(AtmosphereDomain, RejectsFiniteOffsetsThatOverflowDerivedProperties) {
  EXPECT_THROW((void)galata::core::isa(0.0, std::numeric_limits<double>::max()),
               std::invalid_argument);
}

}  // namespace
