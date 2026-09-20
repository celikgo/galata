// SPDX-License-Identifier: Apache-2.0
//
// VALIDATION: the A-4D NASA CR-96008 condition-1 derivative set through the
// nonlinear trim -> finite-difference linearisation path.
//
// The source prints dimensional derivatives. The YAML stores a SI geometry
// and a non-dimensional derivative set, so this test keeps the source values
// in the committed CSV and reconstructs them from the loaded model at the
// source condition. That makes both the transcription and the shared engine
// observable.

#include "galata/linearize/finite_difference.hpp"
#include "galata/model/aircraft.hpp"
#include "galata/trim/level.hpp"
#include "galata/units.hpp"

#include "reference_table.hpp"
#include "validation_config.hpp"
#include <gtest/gtest.h>

#include <cmath>
#include <algorithm>
#include <map>
#include <string>
#include <utility>

namespace {

using galata::model::Aircraft;

constexpr double kQbarPsf = 237.0;
constexpr double kWingAreaFt2 = 260.0;
constexpr double kSpanFt = 27.5;
constexpr double kChordFt = 10.8;
constexpr double kMassSlug = 546.0;
constexpr double kIxxSlugFt2 = 8780.0;
constexpr double kIyySlugFt2 = 25900.0;
constexpr double kIzzSlugFt2 = 28500.0;
constexpr double kIxzSlugFt2 = -4070.0;
constexpr double kSpeedFtS = 447.0;
constexpr double kAlphaRad = 0.0820304748437;

Aircraft a4d() {
  return galata::model::load_aircraft(std::string(GALATA_MODELS_DIR) + "/a4d/a4d-fc1.yaml");
}

galata::trim::TrimPoint trim_a4d(const Aircraft& aircraft) {
  galata::trim::LevelTrimRequest request;
  request.altitude_m = 0.0;
  request.airspeed_m_s = galata::units::feet_to_metres(kSpeedFtS);
  request.flight_path_angle_rad = 0.0;
  request.residual_tolerance = 1.0e-10;
  return galata::trim::trim_level(aircraft, request);
}

double source(const std::map<std::string, double>& values, const char* name) {
  return values.at(name);
}

double relative_error(double value, double reference) {
  return std::fabs(value - reference) / std::max(1.0, std::fabs(reference));
}

std::pair<double, double> raw_moment_from_body_coefficients(double cl,
                                                            double cn,
                                                            double qbar,
                                                            double s,
                                                            double b) {
  return {cl * qbar * s * b, cn * qbar * s * b};
}

std::pair<double, double> primed_from_raw_moment(double raw_l, double raw_n) {
  const double determinant = kIxxSlugFt2 * kIzzSlugFt2 - kIxzSlugFt2 * kIxzSlugFt2;
  const double l_prime = (kIzzSlugFt2 * raw_l + kIxzSlugFt2 * raw_n) / determinant;
  const double n_prime = (kIxzSlugFt2 * raw_l + kIxxSlugFt2 * raw_n) / determinant;
  return {l_prime, n_prime};
}

}  // namespace

TEST(A4DModel, PublishedConditionLoadsAndTrims) {
  const Aircraft aircraft = a4d();
  const auto trim = trim_a4d(aircraft);
  ASSERT_LT(trim.residual_norm, 1.0e-10);
  EXPECT_NEAR(trim.mach, 0.4, 0.0005);
  EXPECT_FALSE(trim.envelope.outside_advisory_envelope);
}

TEST(A4DModel, LoadedCoefficientsReproduceEveryPublishedDimensionalDerivative) {
  const Aircraft aircraft = a4d();
  const auto table =
      galata::testing::load_reference(GALATA_VALIDATION_REFERENCE_DIR, "a4d_fc1.csv");
  const auto published = table.as_lookup("quantity", "value");

  const auto& aero = aircraft.aero;
  const double alpha = kAlphaRad;
  const double c = std::cos(alpha);
  const double s = std::sin(alpha);
  const double force_scale = kQbarPsf * kWingAreaFt2 / kMassSlug;
  const double moment_scale = kQbarPsf * kWingAreaFt2 * kChordFt / kIyySlugFt2;
  const double cx = -aero.drag_ref * c + aero.lift_ref * s;
  const double cz = -aero.drag_ref * s - aero.lift_ref * c;
  const double cxa = -aero.drag_alpha * c + aero.drag_ref * s + aero.lift_alpha * s
                     + aero.lift_ref * c;
  const double cza = -aero.drag_alpha * s - aero.drag_ref * c - aero.lift_alpha * c
                     + aero.lift_ref * s;
  const double xu = force_scale * (2.0 * c / kSpeedFtS * cx - s / kSpeedFtS * cxa);
  const double xw = force_scale * (2.0 * s / kSpeedFtS * cx + c / kSpeedFtS * cxa);
  const double zu = force_scale * (2.0 * c / kSpeedFtS * cz - s / kSpeedFtS * cza);
  const double zw = force_scale * (2.0 * s / kSpeedFtS * cz + c / kSpeedFtS * cza);
  const double mu = moment_scale
                    * (2.0 * c / kSpeedFtS * aero.pitching_moment_ref
                       - s / kSpeedFtS * aero.pitching_moment_alpha);
  const double mw = moment_scale
                    * (2.0 * s / kSpeedFtS * aero.pitching_moment_ref
                       + c / kSpeedFtS * aero.pitching_moment_alpha);
  const double mwd = moment_scale * aero.pitching_moment_alpha_dot * kChordFt
                     / (2.0 * kSpeedFtS * kSpeedFtS);
  const double mq = moment_scale * aero.pitching_moment_pitch_rate * kChordFt
                    / (2.0 * kSpeedFtS);
  const double xde = force_scale
                     * (-aero.drag_elevator * c + aero.lift_elevator * s);
  const double zde = force_scale
                     * (-aero.drag_elevator * s - aero.lift_elevator * c);
  const double mde = moment_scale * aero.pitching_moment_elevator;

  const std::pair<const char*, double> longitudinal[] = {
      {"X_u", xu},       {"X_w", xw},       {"X_delta_e", xde},
      {"Z_u", zu},       {"Z_w", zw},       {"Z_delta_e", zde},
      {"M_u", mu},       {"M_w", mw},       {"M_w_dot", mwd},
      {"M_q", mq},       {"M_delta_e", mde},
  };
  for (const auto& [name, value] : longitudinal) {
    EXPECT_LT(relative_error(value, source(published, name)), 0.001)
        << name << ": computed " << value << ", source " << source(published, name);
  }

  const double yv = aero.side_force_beta * kQbarPsf * kWingAreaFt2
                    / (kMassSlug * kSpeedFtS);
  const double yda = aero.side_force_aileron * kQbarPsf * kWingAreaFt2 / kMassSlug;
  const double ydr = aero.side_force_rudder * kQbarPsf * kWingAreaFt2 / kMassSlug;
  const auto beta = primed_from_raw_moment(
      raw_moment_from_body_coefficients(aero.rolling_moment_beta,
                                         aero.yawing_moment_beta,
                                         kQbarPsf,
                                         kWingAreaFt2,
                                         kSpanFt)
          .first,
      raw_moment_from_body_coefficients(aero.rolling_moment_beta,
                                         aero.yawing_moment_beta,
                                         kQbarPsf,
                                         kWingAreaFt2,
                                         kSpanFt)
          .second);
  const auto p = primed_from_raw_moment(
      aero.rolling_moment_roll_rate * kQbarPsf * kWingAreaFt2 * kSpanFt * kSpanFt
          / (2.0 * kSpeedFtS),
      aero.yawing_moment_roll_rate * kQbarPsf * kWingAreaFt2 * kSpanFt * kSpanFt
          / (2.0 * kSpeedFtS));
  const auto r = primed_from_raw_moment(
      aero.rolling_moment_yaw_rate * kQbarPsf * kWingAreaFt2 * kSpanFt * kSpanFt
          / (2.0 * kSpeedFtS),
      aero.yawing_moment_yaw_rate * kQbarPsf * kWingAreaFt2 * kSpanFt * kSpanFt
          / (2.0 * kSpeedFtS));
  const auto da = primed_from_raw_moment(
      aero.rolling_moment_aileron * kQbarPsf * kWingAreaFt2 * kSpanFt,
      aero.yawing_moment_aileron * kQbarPsf * kWingAreaFt2 * kSpanFt);
  const auto dr = primed_from_raw_moment(
      aero.rolling_moment_rudder * kQbarPsf * kWingAreaFt2 * kSpanFt,
      aero.yawing_moment_rudder * kQbarPsf * kWingAreaFt2 * kSpanFt);

  const std::pair<const char*, double> lateral[] = {
      {"Y_v", yv},
      {"Y_delta_a", yda},
      {"Y_delta_r", ydr},
      {"L_beta_prime", beta.first},
      {"N_beta_prime", beta.second},
      {"L_p_prime", p.first},
      {"N_p_prime", p.second},
      {"L_r_prime", r.first},
      {"N_r_prime", r.second},
      {"L_delta_a_prime", da.first},
      {"N_delta_a_prime", da.second},
      {"L_delta_r_prime", dr.first},
      {"N_delta_r_prime", dr.second},
  };
  for (const auto& [name, value] : lateral) {
    EXPECT_LT(relative_error(value, source(published, name)), 0.001)
        << name << ": computed " << value << ", source " << source(published, name);
  }
}

TEST(A4DModel, NonlinearTrimLinearisationIsFiniteWithMeasuredCoupling) {
  const Aircraft aircraft = a4d();
  const auto trim = trim_a4d(aircraft);

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
  EXPECT_LT(longitudinal.neglected_coupling, 1.0e-3);
  EXPECT_LT(lateral.neglected_coupling, 1.0e-9);

  EXPECT_TRUE(longitudinal.a.allFinite());
  EXPECT_TRUE(longitudinal.b.allFinite());
  EXPECT_TRUE(lateral.a.allFinite());
  EXPECT_TRUE(lateral.b.allFinite());

  const auto table =
      galata::testing::load_reference(GALATA_VALIDATION_REFERENCE_DIR, "a4d_fc1.csv");
  const auto published = table.as_lookup("quantity", "value");
  EXPECT_NEAR(trim.alpha_rad,
              galata::units::degrees_to_radians(source(published, "reference_alpha")),
              galata::units::degrees_to_radians(0.4));
  // Side-force beta is independent of the rounded longitudinal trim offset;
  // this is a useful cross-check that the shared body-axis path is active.
  EXPECT_NEAR(lateral.a(0, 0), source(published, "Y_v"), 0.002);
}
