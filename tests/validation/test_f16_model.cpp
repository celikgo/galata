// SPDX-License-Identifier: Apache-2.0
//
// VALIDATION: the NASA simupy-flight F-16 derivative slice loads and trims
// through the shared fixed-wing path. The source boundary and limitations are
// recorded in models/f16/PROVENANCE.md.

#include "galata/model/aircraft.hpp"
#include "galata/trim/level.hpp"
#include "galata/units.hpp"

#include "reference_table.hpp"
#include "validation_config.hpp"
#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace {

using galata::model::Aircraft;
using galata::trim::LevelTrimRequest;

Aircraft f16() {
  return galata::model::load_aircraft(std::string(GALATA_MODELS_DIR)
                                      + "/f16/f16-nominal.yaml");
}

}  // namespace

TEST(F16Model, NASAReferenceSliceLoadsWithItsDeclaredGeometryAndMass) {
  const Aircraft aircraft = f16();
  EXPECT_NEAR(aircraft.geometry.wing_area_m2, 27.870912, 1.0e-12);
  EXPECT_NEAR(aircraft.geometry.wing_span_m, 9.144, 1.0e-12);
  EXPECT_NEAR(aircraft.mass.mass_kg, 9300.108714, 1.0e-9);
  EXPECT_NEAR(aircraft.aero.reference_alpha_rad,
              galata::units::degrees_to_radians(5.0),
              1.0e-12);
  EXPECT_GT(aircraft.aero.lift_alpha, 0.0);
  EXPECT_NE(aircraft.aero.pitching_moment_elevator, 0.0);
}

TEST(F16Model, DerivedCoefficientsMatchTheCommittedSourceBoundary) {
  const Aircraft aircraft = f16();
  const auto reference =
      galata::testing::load_reference(GALATA_VALIDATION_REFERENCE_DIR, "f16_nominal.csv");
  const auto expected = reference.as_lookup("quantity", "value");
  const auto check = [&](const char* name, double actual) {
    EXPECT_NEAR(actual, expected.at(name), 2.0e-10) << name;
  };

  check("reference_alpha", galata::units::radians_to_degrees(aircraft.aero.reference_alpha_rad));
  check("reference_mach", aircraft.aero.reference_mach);
  check("lift_ref", aircraft.aero.lift_ref);
  check("drag_ref", aircraft.aero.drag_ref);
  check("pitching_moment_ref", aircraft.aero.pitching_moment_ref);
  check("lift_alpha", aircraft.aero.lift_alpha);
  check("drag_alpha", aircraft.aero.drag_alpha);
  check("pitching_moment_alpha", aircraft.aero.pitching_moment_alpha);
  check("lift_pitch_rate", aircraft.aero.lift_pitch_rate);
  check("pitching_moment_pitch_rate", aircraft.aero.pitching_moment_pitch_rate);
  check("lift_elevator", aircraft.aero.lift_elevator);
  check("drag_elevator", aircraft.aero.drag_elevator);
  check("pitching_moment_elevator", aircraft.aero.pitching_moment_elevator);
  check("side_force_beta", aircraft.aero.side_force_beta);
  check("rolling_moment_beta", aircraft.aero.rolling_moment_beta);
  check("yawing_moment_beta", aircraft.aero.yawing_moment_beta);
  check("rolling_moment_roll_rate", aircraft.aero.rolling_moment_roll_rate);
  check("yawing_moment_roll_rate", aircraft.aero.yawing_moment_roll_rate);
  check("rolling_moment_yaw_rate", aircraft.aero.rolling_moment_yaw_rate);
  check("yawing_moment_yaw_rate", aircraft.aero.yawing_moment_yaw_rate);
  check("side_force_aileron", aircraft.aero.side_force_aileron);
  check("rolling_moment_aileron", aircraft.aero.rolling_moment_aileron);
  check("yawing_moment_aileron", aircraft.aero.yawing_moment_aileron);
  check("side_force_rudder", aircraft.aero.side_force_rudder);
  check("rolling_moment_rudder", aircraft.aero.rolling_moment_rudder);
  check("yawing_moment_rudder", aircraft.aero.yawing_moment_rudder);
}

TEST(F16Model, NominalSliceTrimsAndReportsFiniteDynamics) {
  const Aircraft aircraft = f16();
  LevelTrimRequest request;
  request.altitude_m = 3048.0;  // Source study condition: 10,000 ft.
  request.airspeed_m_s = 300.0 * 0.3048;  // NASA source: 300 ft/s.
  request.flight_path_angle_rad = 0.0;
  request.residual_tolerance = 1.0e-10;

  const auto trim = galata::trim::trim_level(aircraft, request);
  EXPECT_LT(trim.residual_norm, request.residual_tolerance);
  EXPECT_TRUE(std::isfinite(trim.alpha_rad));
  EXPECT_TRUE(std::isfinite(trim.controls.elevator_rad));
  EXPECT_TRUE(std::isfinite(trim.controls.thrust_n));
}
