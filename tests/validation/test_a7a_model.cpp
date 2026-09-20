// SPDX-License-Identifier: Apache-2.0
//
// VALIDATION: the A-7A NASA reference set through the complete nonlinear
// trim -> linearisation -> modal-classification path.

#include "galata/analyze/modes.hpp"
#include "galata/linearize/finite_difference.hpp"
#include "galata/model/aircraft.hpp"
#include "galata/trim/level.hpp"
#include "galata/units.hpp"

#include "reference_table.hpp"
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

Aircraft a7a() {
  return galata::model::load_aircraft(std::string(GALATA_MODELS_DIR) + "/a7a/a7a-fc1.yaml");
}

TrimPoint trim_a7a(const Aircraft& aircraft) {
  LevelTrimRequest request;
  request.altitude_m = 0.0;
  request.airspeed_m_s = 279.0 * 0.3048;
  request.flight_path_angle_rad = 0.0;
  request.residual_tolerance = 1.0e-10;
  return galata::trim::trim_level(aircraft, request);
}

double relative_error(double value, double reference) {
  return std::fabs(value - reference) / std::fabs(reference);
}

}  // namespace

TEST(A7AModel, PublishedConditionLoadsAndTrims) {
  const Aircraft aircraft = a7a();
  const TrimPoint trim = trim_a7a(aircraft);
  ASSERT_LT(trim.residual_norm, 1.0e-10);
  EXPECT_NEAR(trim.mach, 0.25, 0.0005);
  EXPECT_NEAR(trim.alpha_rad,
              galata::units::degrees_to_radians(11.2),
              galata::units::degrees_to_radians(0.4));
  EXPECT_FALSE(trim.envelope.outside_advisory_envelope);
}

TEST(A7AModel, PublishedDimensionalDerivativesAreReproducedAtTheReferencePoint) {
  const Aircraft aircraft = a7a();
  const auto reference =
      galata::testing::load_reference(GALATA_VALIDATION_REFERENCE_DIR, "a7a_fc1.csv");
  const auto published = reference.as_lookup("quantity", "value");
  const double alpha = aircraft.aero.reference_alpha_rad;
  const double speed = 279.0 * 0.3048;
  const double density = galata::core::isa(0.0).density_kg_m3;
  const double q = 0.5 * density * speed * speed;
  const double force_scale = q * aircraft.geometry.wing_area_m2 / aircraft.mass.mass_kg;
  const double moment_scale = q * aircraft.geometry.wing_area_m2
                              * aircraft.geometry.mean_aerodynamic_chord_m
                              / aircraft.mass.inertia_cg_body_kg_m2(1, 1);

  // These are the independent dimensional values obtained by linearising the
  // coefficient equations at the source point. They make the unit conversion
  // visible in the test rather than hiding it in the YAML transcription.
  const double c = std::cos(alpha);
  const double s = std::sin(alpha);
  const double cx = -aircraft.aero.drag_ref * c + aircraft.aero.lift_ref * s;
  const double cz = -aircraft.aero.drag_ref * s - aircraft.aero.lift_ref * c;
  const double cxa = -aircraft.aero.drag_alpha * c + aircraft.aero.drag_ref * s
                     + aircraft.aero.lift_alpha * s + aircraft.aero.lift_ref * c;
  const double cza = -aircraft.aero.drag_alpha * s - aircraft.aero.drag_ref * c
                     - aircraft.aero.lift_alpha * c + aircraft.aero.lift_ref * s;
  const double xu = force_scale * (2.0 * c / speed * cx - s / speed * cxa);
  const double xw = force_scale * (2.0 * s / speed * cx + c / speed * cxa);
  const double zu = force_scale * (2.0 * c / speed * cz - s / speed * cza);
  const double zw = force_scale * (2.0 * s / speed * cz + c / speed * cza);
  const double mu = moment_scale
                    * (2.0 * c / speed * aircraft.aero.pitching_moment_ref
                       - s / speed * aircraft.aero.pitching_moment_alpha);
  const double mw = moment_scale
                    * (2.0 * s / speed * aircraft.aero.pitching_moment_ref
                       + c / speed * aircraft.aero.pitching_moment_alpha);
  const double mq = moment_scale * aircraft.aero.pitching_moment_pitch_rate
                    * aircraft.geometry.mean_aerodynamic_chord_m / (2.0 * speed);
  const double mwd = moment_scale * aircraft.aero.pitching_moment_alpha_dot
                     * aircraft.geometry.mean_aerodynamic_chord_m / (2.0 * speed * speed);

  EXPECT_NEAR(xu, published.at("X_u"), 0.0005);
  EXPECT_NEAR(xw, published.at("X_w"), 0.0005);
  EXPECT_NEAR(zu, published.at("Z_u"), 0.003);
  EXPECT_NEAR(zw, published.at("Z_w"), 0.012);
  EXPECT_NEAR(mu, published.at("M_u") / galata::units::kMetresPerFoot, 0.0001);
  EXPECT_NEAR(mw, published.at("M_w") / galata::units::kMetresPerFoot, 0.0005);
  EXPECT_NEAR(mq, published.at("M_q"), 0.02);
  EXPECT_NEAR(mwd, published.at("M_w_dot") / galata::units::kMetresPerFoot, 0.00003);
}

TEST(A7AModel, PublishedModesRemainWithinThePredeclaredRoundedSourceBudget) {
  const Aircraft aircraft = a7a();
  const TrimPoint trim = trim_a7a(aircraft);

  galata::linearize::LinearisationOptions lon_options;
  lon_options.state_subset = galata::linearize::longitudinal_states();
  const auto lon = galata::linearize::linearize_finite_difference(aircraft, trim, lon_options);

  galata::linearize::LinearisationOptions lat_options;
  lat_options.state_subset = galata::linearize::lateral_states();
  const auto lat = galata::linearize::linearize_finite_difference(aircraft, trim, lat_options);

  EXPECT_LT(lon.worst_relative_truncation, 1.0e-8);
  EXPECT_LT(lat.worst_relative_truncation, 1.0e-8);

  const auto lon_modes = galata::analyze::analyze_modes(
      lon.a, lon.state_names, StateRoles::from_names(lon.state_names));
  const auto lat_modes = galata::analyze::analyze_modes(
      lat.a, lat.state_names, StateRoles::from_names(lat.state_names));
  const auto* short_period = lon_modes.find(ModeLabel::ShortPeriod);
  const auto* spiral = lat_modes.find(ModeLabel::Spiral);
  const auto* roll = lat_modes.find(ModeLabel::RollSubsidence);
  const auto* dutch = lat_modes.find(ModeLabel::DutchRoll);
  ASSERT_NE(short_period, nullptr);
  ASSERT_NE(spiral, nullptr);
  ASSERT_NE(roll, nullptr);
  ASSERT_NE(dutch, nullptr);

  const auto reference =
      galata::testing::load_reference(GALATA_VALIDATION_REFERENCE_DIR, "a7a_fc1.csv");
  const auto published = reference.as_lookup("quantity", "value");
  const auto within_budget = [](const char* name, double value, double expected) {
    EXPECT_LT(relative_error(value, expected), 0.05)
        << name << ": computed " << value << ", source " << expected;
  };
  within_budget("short-period natural frequency",
                short_period->natural_frequency_rad_s,
                published.at("short_period_natural_frequency"));
  within_budget("short-period damping",
                short_period->damping_ratio,
                published.at("short_period_damping_ratio"));
  within_budget("spiral root", -spiral->eigenvalue.real(), published.at("spiral_root"));
  within_budget(
      "roll subsidence root", -roll->eigenvalue.real(), published.at("roll_subsidence_root"));
  within_budget("Dutch-roll natural frequency",
                dutch->natural_frequency_rad_s,
                published.at("dutch_roll_natural_frequency"));
  within_budget(
      "Dutch-roll damping", dutch->damping_ratio, published.at("dutch_roll_damping_ratio"));
}
