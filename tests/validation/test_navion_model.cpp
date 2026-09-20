// SPDX-License-Identifier: Apache-2.0
//
// VALIDATION: the second published fixed-wing derivative set exercised through
// the complete nonlinear trim -> linearisation -> modal-classification path.
//
// Reference:
//   Gary L. Teper, "Aircraft Stability and Control Data", NASA CR-96008,
//   Systems Technology, Inc. for NASA Ames Research Center, April 1969,
//   Section X, Tables X-A through X-E, N69-31783.
//   https://ntrs.nasa.gov/citations/19690022405
//
// The source prints rounded derivative and modal values. The 3% comparison
// budget is fixed before the run: it is wider than the printed rounding and
// also covers the explicit model convention documented in
// models/navion/PROVENANCE.md for the unprinted pitching-moment intercept and
// the elevator coefficient derived from Table X-D.

#include "galata/analyze/modes.hpp"
#include "galata/linearize/finite_difference.hpp"
#include "galata/model/aircraft.hpp"
#include "galata/trim/level.hpp"
#include "galata/units.hpp"

#include "validation_config.hpp"
#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace {

using galata::analyze::ModeLabel;
using galata::analyze::StateRoles;
using galata::model::Aircraft;
using galata::trim::LevelTrimRequest;
using galata::trim::TrimPoint;

constexpr double kPublishedAirspeedMps = 53.6448;  // 176 ft/s
constexpr double kPublishedDynamicPressurePsf = 36.8;
constexpr double kThreePercentBudget = 0.03;

Aircraft navion() {
  return galata::model::load_aircraft(std::string(GALATA_MODELS_DIR)
                                      + "/navion/navion-nominal.yaml");
}

TrimPoint trim_navion(const Aircraft& aircraft) {
  LevelTrimRequest request;
  request.altitude_m = 0.0;
  request.airspeed_m_s = kPublishedAirspeedMps;
  request.flight_path_angle_rad = 0.0;
  request.residual_tolerance = 1.0e-10;
  return galata::trim::trim_level(aircraft, request);
}

double relative_error(double value, double reference) {
  return std::fabs(value - reference) / std::fabs(reference);
}

}  // namespace

TEST(NavionModel, PublishedNominalConditionTrimsAtTheDeclaredMachAndDynamicPressure) {
  const Aircraft aircraft = navion();
  const TrimPoint trim = trim_navion(aircraft);
  ASSERT_LT(trim.residual_norm, 1.0e-10);

  const double dynamic_pressure_psf =
      trim.dynamic_pressure_pa
      / (galata::units::kNewtonsPerPoundForce
         / (galata::units::kMetresPerFoot * galata::units::kMetresPerFoot));
  EXPECT_NEAR(trim.mach, 0.158, 0.0005);
  EXPECT_NEAR(dynamic_pressure_psf, kPublishedDynamicPressurePsf, 0.2);
  EXPECT_FALSE(trim.envelope.outside_advisory_envelope);
}

TEST(NavionModel, PublishedModesRemainWithinThePredeclaredRoundedSourceBudget) {
  const Aircraft aircraft = navion();
  const TrimPoint trim = trim_navion(aircraft);

  galata::linearize::LinearisationOptions longitudinal_options;
  longitudinal_options.state_subset = galata::linearize::longitudinal_states();
  const auto longitudinal =
      galata::linearize::linearize_finite_difference(aircraft, trim, longitudinal_options);

  galata::linearize::LinearisationOptions lateral_options;
  lateral_options.state_subset = galata::linearize::lateral_states();
  const auto lateral =
      galata::linearize::linearize_finite_difference(aircraft, trim, lateral_options);

  EXPECT_LT(longitudinal.worst_relative_truncation, 1.0e-8);
  EXPECT_LT(lateral.worst_relative_truncation, 1.0e-8);

  const auto longitudinal_modes = galata::analyze::analyze_modes(
      longitudinal.a, longitudinal.state_names, StateRoles::from_names(longitudinal.state_names));
  const auto lateral_modes = galata::analyze::analyze_modes(
      lateral.a, lateral.state_names, StateRoles::from_names(lateral.state_names));

  const auto* phugoid = longitudinal_modes.find(ModeLabel::Phugoid);
  const auto* short_period = longitudinal_modes.find(ModeLabel::ShortPeriod);
  const auto* spiral = lateral_modes.find(ModeLabel::Spiral);
  const auto* roll = lateral_modes.find(ModeLabel::RollSubsidence);
  const auto* dutch = lateral_modes.find(ModeLabel::DutchRoll);
  ASSERT_NE(phugoid, nullptr);
  ASSERT_NE(short_period, nullptr);
  ASSERT_NE(spiral, nullptr);
  ASSERT_NE(roll, nullptr);
  ASSERT_NE(dutch, nullptr);

  // Tables X-F through X-H. The source prints only three or four significant
  // figures, so compare against a fixed relative budget rather than the
  // output's observed agreement.
  const auto within_budget = [](const char* name, double value, double reference) {
    EXPECT_LT(relative_error(value, reference), kThreePercentBudget)
        << name << ": computed " << value << ", source " << reference;
  };
  within_budget("phugoid omega", phugoid->natural_frequency_rad_s, 0.2137);
  within_budget("phugoid damping", phugoid->damping_ratio, 0.0801);
  within_budget("short-period omega", short_period->natural_frequency_rad_s, 3.6083);
  within_budget("short-period damping", short_period->damping_ratio, 0.6977);
  within_budget("spiral root", -spiral->eigenvalue.real(), 0.00876);
  within_budget("roll subsidence root", -roll->eigenvalue.real(), 8.435);
  within_budget("Dutch-roll omega", dutch->natural_frequency_rad_s, 2.385);
  within_budget("Dutch-roll damping", dutch->damping_ratio, 0.204);
}
