// SPDX-License-Identifier: Apache-2.0
//
// VALIDATION: the ISA implementation against the U.S. Standard Atmosphere,
// 1976, as published.
//
// Reference values live in reference/ussa1976.csv with their citation and their
// transcription method. This file computes and compares; it contains no
// expected values of its own, which is the point — a validation test whose
// expectations are inline in the test file is a regression lock wearing a
// costume.
//
// The comparison is at the precision the source actually prints: six
// significant figures for temperature, five for pressure, density and speed of
// sound. See the reference file's header for why not six across the board.

#include "galata/core/atmosphere.hpp"

#include "reference_table.hpp"
#include "validation_config.hpp"
#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace {

using galata::testing::load_reference;
using galata::testing::printed_precision_tolerance;

// 1 mb = 1 hPa = 100 Pa, exact. Applied here, at the file-format boundary,
// which is where ADR-0003 permits a conversion to live.
constexpr double kPascalsPerMillibar = 100.0;

// How far a computed value may sit from the printed one, in units of the last
// printed significant figure.
//
// 0.5 would mean "rounds to exactly what the table prints". Temperature and
// speed of sound meet that everywhere. Pressure and density do not, at three of
// the thirty-two tabulated cells, by margins between 1.3 and 1.9 times the
// rounding unit. docs/VERIFICATION.md lists those three cells with their
// measured deviations.
//
// The allowance is 1.0 — the last printed digit may differ by one — and it is
// worth being clear about why that is a gate and not a shrug. The measured
// disagreements are around one part in 10^5. A wrong lapse-rate sign, a
// geopotential/geometric mix-up, or the wrong Earth radius each move these
// values by one part in 10^2 or worse, so this bound still fails every error
// that is actually plausible. What it does not do is claim the 1976 tables and
// a modern double-precision recurrence agree to the last printed digit
// everywhere, because they do not.
constexpr double kLastPlaceAllowance = 1.0;

class Ussa1976 : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    table_ = new galata::testing::ReferenceTable(
        load_reference(GALATA_VALIDATION_REFERENCE_DIR, "ussa1976.csv"));
  }

  static void TearDownTestSuite() {
    delete table_;
    table_ = nullptr;
  }

  static galata::testing::ReferenceTable* table_;
};

galata::testing::ReferenceTable* Ussa1976::table_ = nullptr;

TEST_F(Ussa1976, ReferenceDataIsCitedAndLoaded) {
  ASSERT_NE(table_, nullptr);
  EXPECT_GE(table_->rows.size(), 8U);
  EXPECT_NE(table_->citation.find("NOAA-S/T 76-1562"), std::string::npos);
  EXPECT_NE(table_->citation.find("19770009539"), std::string::npos);
}

TEST_F(Ussa1976, TemperatureMatchesThePublishedTable) {
  for (std::size_t row = 0; row < table_->rows.size(); ++row) {
    const double altitude = table_->at(row, "geometric_altitude_m");
    const double published = table_->at(row, "temperature_k");
    const int figures = static_cast<int>(table_->at(row, "temperature_sig_figs"));

    const galata::core::AtmosphereState computed = galata::core::isa(altitude);
    // Temperature meets the strict claim: it rounds to exactly what is printed.
    EXPECT_NEAR(computed.temperature_k, published, printed_precision_tolerance(published, figures))
        << "at Z = " << altitude << " m";
  }
}

TEST_F(Ussa1976, PressureMatchesThePublishedTable) {
  for (std::size_t row = 0; row < table_->rows.size(); ++row) {
    const double altitude = table_->at(row, "geometric_altitude_m");
    const double published_mb = table_->at(row, "pressure_mb");
    const int figures = static_cast<int>(table_->at(row, "pressure_sig_figs"));

    const galata::core::AtmosphereState computed = galata::core::isa(altitude);
    EXPECT_NEAR(computed.pressure_pa / kPascalsPerMillibar,
                published_mb,
                printed_precision_tolerance(published_mb, figures, kLastPlaceAllowance))
        << "at Z = " << altitude << " m";
  }
}

TEST_F(Ussa1976, DensityMatchesThePublishedTable) {
  for (std::size_t row = 0; row < table_->rows.size(); ++row) {
    const double altitude = table_->at(row, "geometric_altitude_m");
    const double published = table_->at(row, "density_kg_m3");
    const int figures = static_cast<int>(table_->at(row, "density_sig_figs"));

    const galata::core::AtmosphereState computed = galata::core::isa(altitude);
    EXPECT_NEAR(computed.density_kg_m3,
                published,
                printed_precision_tolerance(published, figures, kLastPlaceAllowance))
        << "at Z = " << altitude << " m";
  }
}

TEST_F(Ussa1976, SpeedOfSoundMatchesThePublishedTable) {
  for (std::size_t row = 0; row < table_->rows.size(); ++row) {
    const double altitude = table_->at(row, "geometric_altitude_m");
    const double published = table_->at(row, "speed_of_sound_m_s");
    const int figures = static_cast<int>(table_->at(row, "speed_of_sound_sig_figs"));

    const galata::core::AtmosphereState computed = galata::core::isa(altitude);
    // Speed of sound also meets the strict claim.
    EXPECT_NEAR(
        computed.speed_of_sound_m_s, published, printed_precision_tolerance(published, figures))
        << "at Z = " << altitude << " m";
  }
}

// --- The layer recurrence, checked end to end -------------------------------

TEST_F(Ussa1976, DerivedBaseTemperaturesMatchTheStandardsOwnTabulation) {
  // The standard's Table 4 has NO base-temperature column: it gives seven
  // gradients and says they, "plus T_0 ... completely specify the profile". So
  // galata derives the base temperatures rather than transcribing them.
  //
  // That derivation is worth validating on its own, because it is the single
  // point where a wrong lapse-rate sign or a breakpoint entered in kilometres
  // instead of metres would go unnoticed by a check at eight sampled altitudes.
  // The expected values here are the standard's own Table I entries at each
  // breakpoint, read from the geopotential-altitude pages.
  struct Breakpoint {
    double geopotential_m;  // m'
    double tabulated_t_k;   // K, from Table I at that H
  };

  const Breakpoint kBreakpoints[] = {
      {0.0, 288.150},
      {11000.0, 216.650},
      {20000.0, 216.650},
      {32000.0, 228.650},
      {47000.0, 270.650},
      {51000.0, 270.650},
      {71000.0, 214.650},
  };

  for (const Breakpoint& point : kBreakpoints) {
    const double z = galata::core::geometric_from_geopotential(point.geopotential_m);
    const galata::core::AtmosphereState state = galata::core::isa(z);
    EXPECT_NEAR(state.temperature_k, point.tabulated_t_k, 5e-4)
        << "base temperature at H = " << point.geopotential_m << " m'";
  }
}

TEST_F(Ussa1976, PressureAtTheTopOfTheModelMatchesAfterSevenLayers) {
  // The pressure recurrence multiplies seven times from P_0. If any single
  // layer's exponent, gradient or base temperature were wrong, the error would
  // be largest here, at the end of the chain — so agreement at the top is a
  // much stronger statement than agreement in the troposphere.
  //
  // Reference: Table I at H = 84852 m' (Z = 86000 m), P = 3.7338e-3 mb.
  const galata::core::AtmosphereState top = galata::core::isa(86000.0);
  EXPECT_NEAR(
      top.pressure_pa / kPascalsPerMillibar, 3.7338e-3, printed_precision_tolerance(3.7338e-3, 5));

  // Molecular-scale temperature at the top of the layer system. The standard
  // tabulates T_M = 186.95 K and kinetic T = 186.87 K at Z = 86 km; galata
  // computes T_M, so it is the former this must match. The 0.08 K gap between
  // them is the composition effect described in the header, not an error.
  EXPECT_NEAR(top.temperature_k, 186.95, 5e-3);
  EXPECT_GT(top.temperature_k, 186.87);
}

// --- Properties of the model that the table alone does not pin down ---------

TEST_F(Ussa1976, SeaLevelReproducesTheDefiningConstantsExactly) {
  // P_0 and T_0 are DEFINING constants, not tabulated results, so these are
  // exact rather than rounded.
  const galata::core::AtmosphereState sea_level = galata::core::isa(0.0);
  EXPECT_DOUBLE_EQ(sea_level.pressure_pa, galata::core::ussa1976::kSeaLevelPressure);
  EXPECT_DOUBLE_EQ(sea_level.temperature_k, galata::core::ussa1976::kSeaLevelTemperature);
  EXPECT_DOUBLE_EQ(sea_level.geopotential_altitude_m, 0.0);
}

TEST_F(Ussa1976, GeopotentialAndGeometricAltitudeRoundTrip) {
  for (double z = -5000.0; z <= 86000.0; z += 1000.0) {
    const double h = galata::core::geopotential_from_geometric(z);
    EXPECT_NEAR(galata::core::geometric_from_geopotential(h), z, 1e-6 * (std::fabs(z) + 1.0))
        << "Z = " << z;
  }
}

TEST_F(Ussa1976, GeopotentialIsBelowGeometricAndTheGapGrowsWithAltitude) {
  // H < Z above sea level, because gravity weakens with altitude so a given
  // geometric rise buys less potential energy the higher you are. The tables
  // themselves show it: Z = 20000 m is H = 19937 m.
  EXPECT_NEAR(galata::core::geopotential_from_geometric(20000.0), 19937.0, 1.0);
  EXPECT_NEAR(galata::core::geopotential_from_geometric(71000.0), 70216.0, 1.0);

  double previous_gap = 0.0;
  for (double z = 1000.0; z <= 80000.0; z += 1000.0) {
    const double gap = z - galata::core::geopotential_from_geometric(z);
    EXPECT_GT(gap, previous_gap) << "Z = " << z;
    previous_gap = gap;
  }
}

TEST_F(Ussa1976, TemperatureIsContinuousAcrossEveryLayerBoundary) {
  // The layer system is piecewise-linear in T against H. A base temperature
  // derived with the wrong sign or a breakpoint entered in the wrong units
  // shows up as a step here, and nowhere else — the tabulated comparison above
  // samples only eight altitudes and could step between two of them.
  const double kStep = 0.5;  // m
  for (double z = 0.0; z <= 85000.0; z += 100.0) {
    if (!galata::core::is_within_envelope(z + kStep)) {
      break;
    }
    const double below = galata::core::isa(z).temperature_k;
    const double above = galata::core::isa(z + kStep).temperature_k;
    // The steepest lapse rate is 6.5 K/km, so half a metre can move T by at
    // most about 3.3 mK. Anything larger is a discontinuity.
    EXPECT_LT(std::fabs(above - below), 0.01) << "temperature steps at Z = " << z;
  }
}

TEST_F(Ussa1976, PressureIsContinuousAndStrictlyDecreasing) {
  double previous = galata::core::isa(-5000.0).pressure_pa;
  for (double z = -4900.0; z <= 86000.0; z += 100.0) {
    if (!galata::core::is_within_envelope(z)) {
      break;
    }
    const double pressure = galata::core::isa(z).pressure_pa;
    EXPECT_LT(pressure, previous) << "pressure not decreasing at Z = " << z;
    // Continuity: a 100 m step can never halve the pressure. A base-pressure
    // recurrence that skipped a layer would show up as a jump.
    EXPECT_GT(pressure, 0.9 * previous) << "pressure jumps at Z = " << z;
    previous = pressure;
  }
  EXPECT_GT(previous, 0.0);
}

TEST_F(Ussa1976, OutsideTheEnvelopeIsRefusedRatherThanExtrapolated) {
  // Charter rule: never silently extrapolate. A clamped atmosphere produces a
  // confidently wrong density.
  EXPECT_FALSE(galata::core::is_within_envelope(86001.0));
  // 86000 m is the standard's own stated ceiling and must be accepted. Its
  // geopotential is 84852.046 m', fractionally above the 84852 m' the standard
  // prints, so bounding the envelope in geopotential would reject it.
  EXPECT_TRUE(galata::core::is_within_envelope(86000.0));
  EXPECT_FALSE(galata::core::is_within_envelope(-5001.0));
  EXPECT_FALSE(galata::core::is_within_envelope(std::nan("")));
  EXPECT_THROW((void)galata::core::isa(100000.0), std::out_of_range);
  EXPECT_THROW((void)galata::core::isa(-6000.0), std::out_of_range);
  EXPECT_NO_THROW((void)galata::core::isa(86000.0));
}

TEST_F(Ussa1976, TemperatureOffsetMovesTemperatureAndDensityButNotPressure) {
  // The aviation convention: at a given pressure altitude, an ISA+15 day is
  // hotter and thinner, but the pressure defining that altitude is unchanged.
  const double kAltitude = 3048.0;  // m — 10,000 ft
  const galata::core::AtmosphereState standard = galata::core::isa(kAltitude, 0.0);
  const galata::core::AtmosphereState hot = galata::core::isa(kAltitude, 15.0);

  EXPECT_DOUBLE_EQ(hot.pressure_pa, standard.pressure_pa);
  EXPECT_DOUBLE_EQ(hot.temperature_k, standard.temperature_k + 15.0);
  EXPECT_LT(hot.density_kg_m3, standard.density_kg_m3);
  EXPECT_GT(hot.speed_of_sound_m_s, standard.speed_of_sound_m_s);
  EXPECT_DOUBLE_EQ(hot.delta_isa_k, 15.0);

  // Ideal gas: density scales as T_std/T_actual at fixed pressure.
  EXPECT_NEAR(hot.density_kg_m3,
              standard.density_kg_m3 * standard.temperature_k / hot.temperature_k,
              1e-12 * standard.density_kg_m3);
}

TEST_F(Ussa1976, ResultCarriesTheAltitudeAndOffsetItWasComputedFor) {
  // Charter rule 9: no number reaches the user without provenance.
  const galata::core::AtmosphereState state = galata::core::isa(5000.0, -10.0);
  EXPECT_DOUBLE_EQ(state.geometric_altitude_m, 5000.0);
  EXPECT_DOUBLE_EQ(state.delta_isa_k, -10.0);
  EXPECT_NEAR(
      state.geopotential_altitude_m, galata::core::geopotential_from_geometric(5000.0), 0.0);
}

}  // namespace
