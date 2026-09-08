// SPDX-License-Identifier: Apache-2.0
//
// The contract of `linearize::linearize_extended`: what it refuses, what its
// observation model produces, what freezing a state means, and the one claim
// its chart exists to make — that it is regular where the Euler chart is not.
// The numerical acceptance gates are in the validation tier.

#include "galata/core/constants.hpp"
#include "galata/core/frames.hpp"
#include "galata/core/quaternion.hpp"
#include "galata/core/state.hpp"
#include "galata/linearize/extended.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/trim/hover.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using galata::core::kStandardGravity;
using galata::linearize::ExtendedDynamics;
using galata::linearize::ExtendedLinearisation;
using galata::linearize::ExtendedLinearisationOptions;
using galata::linearize::OutputKind;
using galata::model::Battery;
using galata::model::Quadrotor;
using galata::model::Rotor;

Quadrotor test_model(bool with_battery = false) {
  Quadrotor model;
  model.mass.mass_kg = 1.6;
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  inertia.diagonal() << 0.023, 0.023, 0.042;
  model.mass.inertia_cg_body_kg_m2 = inertia;

  const double arm_m = 0.1626;
  const int spins[4] = {1, -1, 1, -1};
  const double x[4] = {arm_m, -arm_m, -arm_m, arm_m};
  const double y[4] = {-arm_m, -arm_m, arm_m, arm_m};
  for (int i = 0; i < 4; ++i) {
    Rotor rotor;
    rotor.position_cg_to_hub_body_m = Eigen::Vector3d(x[i], y[i], 0.0);
    rotor.spin_about_body_z = spins[i];
    rotor.thrust_coefficient_n_s2 = 1.0e-5;
    rotor.torque_coefficient_n_m_s2 = 1.7e-7;
    rotor.speed_time_constant_s = 0.035;
    rotor.minimum_speed_rad_s = 0.0;
    rotor.maximum_speed_rad_s = 1100.0;
    model.rotors.push_back(rotor);
  }
  model.drag_linear_n_s_m = Eigen::Vector3d(0.12, 0.12, 0.18);
  model.drag_quadratic_n_s2_m2 = Eigen::Vector3d(0.025, 0.025, 0.035);
  model.angular_drag_n_m_s = Eigen::Vector3d(0.002, 0.002, 0.003);
  if (with_battery) {
    Battery battery;
    battery.energy_j = 3600.0 * 100.0;
    battery.full_voltage_v = 25.2;
    battery.empty_voltage_v = 19.8;
    battery.internal_resistance_ohm = 0.03;
    battery.speed_at_full_voltage_rad_s = 1100.0;
    model.battery = battery;
  }
  model.validate();
  return model;
}

ExtendedDynamics dynamics_for(const Quadrotor& model) {
  const int rotors = model.rotor_count();
  return [&model, rotors](const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
    return model.derivative(x, u.head(rotors), Eigen::Vector3d(u.segment<3>(rotors)));
  };
}

ExtendedLinearisationOptions options_for(const Quadrotor& model) {
  ExtendedLinearisationOptions options;
  options.appended_state_names = model.extended_state_names();
  options.appended_state_names.erase(
      options.appended_state_names.begin(),
      options.appended_state_names.begin() + galata::core::kStateSize);
  options.input_names = model.input_names();
  options.wind_input_offset = static_cast<int>(options.input_names.size());
  options.input_names.push_back("wind_north_m_s");
  options.input_names.push_back("wind_east_m_s");
  options.input_names.push_back("wind_down_m_s");
  options.outputs = {OutputKind::BodySpecificForce,
                     OutputKind::BodyRates,
                     OutputKind::PositionNed,
                     OutputKind::Altitude,
                     OutputKind::GroundVelocityNed};
  if (model.has_battery()) {
    options.frozen_appended_states = {model.rotor_count()};
  }
  return options;
}

struct Point {
  Eigen::VectorXd state;
  Eigen::VectorXd input;
};

Point hover_point(const Quadrotor& model) {
  const galata::trim::HoverTrim trim = galata::trim::trim_hover(model, {});
  Eigen::VectorXd input = Eigen::VectorXd::Zero(model.rotor_count() + 3);
  input.head(model.rotor_count()) = trim.command_rad_s;
  return {trim.extended_state, input};
}

}  // namespace

// --- malformed requests ----------------------------------------------------

TEST(ExtendedLinearize, MalformedRequestsAreRefusedWithTheirOwnDiagnosis) {
  const Quadrotor model = test_model();
  const Point point = hover_point(model);
  const ExtendedDynamics dynamics = dynamics_for(model);

  EXPECT_THROW((void)galata::linearize::linearize_extended(
                   ExtendedDynamics{}, point.state, point.input, options_for(model)),
               std::invalid_argument)
      << "no dynamics function";

  {
    ExtendedLinearisationOptions options = options_for(model);
    options.appended_state_names.pop_back();
    EXPECT_THROW(
        (void)galata::linearize::linearize_extended(dynamics, point.state, point.input, options),
        std::invalid_argument)
        << "a state longer than the declared names must be an error, not a silent truncation";
  }
  {
    ExtendedLinearisationOptions options = options_for(model);
    options.input_names.pop_back();
    EXPECT_THROW(
        (void)galata::linearize::linearize_extended(dynamics, point.state, point.input, options),
        std::invalid_argument)
        << "one name per input column";
  }
  {
    ExtendedLinearisationOptions options = options_for(model);
    options.outputs.clear();
    EXPECT_THROW(
        (void)galata::linearize::linearize_extended(dynamics, point.state, point.input, options),
        std::invalid_argument)
        << "an empty observation model is not the identity";
  }
  {
    ExtendedLinearisationOptions options = options_for(model);
    options.wind_input_offset = static_cast<int>(options.input_names.size()) - 1;
    EXPECT_THROW(
        (void)galata::linearize::linearize_extended(dynamics, point.state, point.input, options),
        std::invalid_argument)
        << "a wind offset that does not admit three columns";
  }
  {
    ExtendedLinearisationOptions options = options_for(model);
    options.frozen_appended_states = {model.rotor_count()};  // no battery on this model
    EXPECT_THROW(
        (void)galata::linearize::linearize_extended(dynamics, point.state, point.input, options),
        std::invalid_argument)
        << "a frozen index past the appended block";
  }
  {
    ExtendedLinearisationOptions options = options_for(model);
    options.frozen_appended_states = {1, 1};
    EXPECT_THROW(
        (void)galata::linearize::linearize_extended(dynamics, point.state, point.input, options),
        std::invalid_argument)
        << "the same state frozen twice means the caller has miscounted";
  }
  {
    Eigen::VectorXd bad = point.state;
    bad(galata::core::kVelocityU) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(
        (void)galata::linearize::linearize_extended(dynamics, bad, point.input, options_for(model)),
        std::invalid_argument);
  }
}

TEST(ExtendedLinearize, APointThatIsNotAnEquilibriumIsRefused) {
  const Quadrotor model = test_model();
  Point point = hover_point(model);
  // Two percent more rotor speed than the weight needs. The vehicle climbs; the
  // linearisation about it would carry a constant term A cannot represent.
  point.state.segment(model.rotor_state_offset(), model.rotor_count()) *= 1.02;

  try {
    (void)galata::linearize::linearize_extended(
        dynamics_for(model), point.state, point.input, options_for(model));
    FAIL() << "a climbing point was accepted as an equilibrium";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("not an equilibrium"), std::string::npos)
        << error.what();
  }
}

TEST(ExtendedLinearize, ARelativeEquilibriumIsAcceptedBecausePositionRateIsExcluded) {
  const Quadrotor model = test_model();
  galata::trim::HoverTrimRequest request;
  request.ground_velocity_ned_m_s = Eigen::Vector3d(4.0, 0.0, 0.0);
  const galata::trim::HoverTrim trim = galata::trim::trim_hover(model, request);

  Eigen::VectorXd input = Eigen::VectorXd::Zero(model.rotor_count() + 3);
  input.head(model.rotor_count()) = trim.command_rad_s;

  // The vehicle is translating at 4 m/s, so its position rate is emphatically
  // not zero. Requiring it to vanish would reject every cruise condition.
  const ExtendedLinearisation linearisation = galata::linearize::linearize_extended(
      dynamics_for(model), trim.extended_state, input, options_for(model));
  EXPECT_LE(linearisation.equilibrium_residual_norm, linearisation.equilibrium_tolerance);
}

// --- the observation model -------------------------------------------------

TEST(ExtendedLinearize, EveryDeclaredOutputAppearsWithItsOwnNamesAndWidth) {
  const Quadrotor model = test_model();
  const Point point = hover_point(model);
  const ExtendedLinearisation linearisation = galata::linearize::linearize_extended(
      dynamics_for(model), point.state, point.input, options_for(model));

  const std::vector<std::string> expected = {"specific_force_x_m_s2",
                                             "specific_force_y_m_s2",
                                             "specific_force_z_m_s2",
                                             "body_rate_p_rad_s",
                                             "body_rate_q_rad_s",
                                             "body_rate_r_rad_s",
                                             "position_north_m",
                                             "position_east_m",
                                             "position_down_m",
                                             "altitude_m",
                                             "ground_velocity_north_m_s",
                                             "ground_velocity_east_m_s",
                                             "ground_velocity_down_m_s"};
  EXPECT_EQ(linearisation.output_names, expected);
  EXPECT_EQ(linearisation.c.rows(), static_cast<Eigen::Index>(expected.size()));
  EXPECT_EQ(linearisation.d.rows(), static_cast<Eigen::Index>(expected.size()));
  EXPECT_EQ(linearisation.c.cols(), linearisation.a.cols());
  EXPECT_EQ(linearisation.d.cols(), linearisation.b.cols());

  const auto row_of = [&](const std::string& name) {
    const auto found =
        std::find(linearisation.output_names.begin(), linearisation.output_names.end(), name);
    EXPECT_NE(found, linearisation.output_names.end()) << name;
    return std::distance(linearisation.output_names.begin(), found);
  };

  // Body rates and NED position are read straight off the state, so their rows
  // are selectors: one at their own coordinate and zero elsewhere.
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Index rate_row = row_of("body_rate_p_rad_s") + axis;
    for (Eigen::Index column = 0; column < linearisation.c.cols(); ++column) {
      const double expected_entry = column == galata::linearize::kChartRateP + axis ? 1.0 : 0.0;
      EXPECT_NEAR(linearisation.c(rate_row, column), expected_entry, 1e-9)
          << "body rate row " << axis << ", column " << column;
    }
  }
  // Altitude is the negative of the NED down state, and nothing else.
  const Eigen::Index altitude_row = row_of("altitude_m");
  EXPECT_NEAR(linearisation.c(altitude_row, galata::linearize::kChartPositionDown), -1.0, 1e-9);
  EXPECT_NEAR(linearisation.c(altitude_row, galata::linearize::kChartPositionNorth), 0.0, 1e-9);

  // Specific force at a level hover: an ideal accelerometer at the CG reads the
  // thrust, which is one gee upward, so its z row responds to rotor speed and
  // its x and y rows respond to the drag on body velocity.
  const Eigen::Index force_x = row_of("specific_force_x_m_s2");
  EXPECT_NEAR(linearisation.c(force_x, galata::linearize::kChartVelocityU),
              -model.drag_linear_n_s_m.x() / model.mass.mass_kg,
              1e-5);

  // Ground velocity at a level hover is the body velocity rotated out, which at
  // identity attitude is the body velocity itself.
  const Eigen::Index ground_north = row_of("ground_velocity_north_m_s");
  EXPECT_NEAR(linearisation.c(ground_north, galata::linearize::kChartVelocityU), 1.0, 1e-9);

  // All four truncation estimates are present and finite.
  for (const Eigen::MatrixXd* estimate : {&linearisation.a_truncation,
                                          &linearisation.b_truncation,
                                          &linearisation.c_truncation,
                                          &linearisation.d_truncation}) {
    ASSERT_GT(estimate->size(), 0) << "a matrix with no truncation estimate cannot be qualified";
    EXPECT_TRUE(estimate->allFinite());
  }
  EXPECT_GE(linearisation.worst_relative_truncation,
            std::max(linearisation.worst_relative_truncation_c,
                     linearisation.worst_relative_truncation_d))
      << "the summary must be the worst of all four, not of A and B alone";
}

TEST(ExtendedLinearize, DroppingTheWindOffsetGivesAZeroFeedthroughAndSaysSo) {
  const Quadrotor model = test_model();
  const Point point = hover_point(model);

  ExtendedLinearisationOptions options = options_for(model);
  options.wind_input_offset = -1;  // wind as a plain input, not as a disturbance
  const ExtendedLinearisation linearisation =
      galata::linearize::linearize_extended(dynamics_for(model), point.state, point.input, options);

  // Without the fixed-ground-velocity reading, perturbing the wind cannot move
  // the air-relative velocity, so it cannot move the drag, so D is zero. That
  // is the convention the header says a caller gets when it declines the
  // disturbance columns.
  EXPECT_LT(linearisation.d.rightCols(3).cwiseAbs().maxCoeff(), 1e-9);
}

// --- frozen states ---------------------------------------------------------

TEST(ExtendedLinearize, AFrozenBatteryIsAdmittedAndAnUnfrozenOneIsRefused) {
  const Quadrotor model = test_model(true);
  const Point point = hover_point(model);
  ASSERT_TRUE(model.has_battery());

  // Unfrozen, the point fails the equilibrium test — correctly. The pack is
  // discharging at hover, so its state has a nonzero derivative and there is no
  // equilibrium to be at.
  {
    ExtendedLinearisationOptions options = options_for(model);
    options.frozen_appended_states.clear();
    EXPECT_THROW((void)galata::linearize::linearize_extended(
                     dynamics_for(model), point.state, point.input, options),
                 std::runtime_error);
  }

  const ExtendedLinearisation linearisation = galata::linearize::linearize_extended(
      dynamics_for(model), point.state, point.input, options_for(model));

  // Seventeen states: twelve chart coordinates, four rotors, one battery.
  EXPECT_EQ(linearisation.a.rows(), 17);
  ASSERT_EQ(linearisation.frozen_state_names.size(), 1u);
  EXPECT_EQ(linearisation.frozen_state_names.front(), linearisation.state_names.back());

  // The declaration is auditable: the rate that was set to zero is reported,
  // and it is not zero.
  ASSERT_EQ(linearisation.frozen_state_rates.size(), 1u);
  EXPECT_LT(linearisation.frozen_state_rates.front(), 0.0)
      << "a hovering pack discharges, so the rate declared away must be negative";

  // The frozen row is zero: the model says the state never moves, because that
  // is what was declared.
  const Eigen::Index battery_row = linearisation.a.rows() - 1;
  EXPECT_LT(linearisation.a.row(battery_row).cwiseAbs().maxCoeff(), 1e-12)
      << "a frozen coordinate's own derivative is declared zero";
  EXPECT_LT(linearisation.b.row(battery_row).cwiseAbs().maxCoeff(), 1e-12)
      << "freezing must reach B too; a zeroed A row beside a live B row would be a state "
         "that moves only when pushed";
}

// --- the chart's own claim -------------------------------------------------

TEST(ExtendedLinearize, TheChartIsRegularAtNinetyDegreesOfPitchWhereTheEulerChartIsNot) {
  const Quadrotor model = test_model();
  const Point hover = hover_point(model);

  // Straight up. `linearize/finite_difference.hpp` says a vertical-climb trim
  // "needs a different chart, not a smaller step"; this is that chart. The
  // point is not an equilibrium of this plant — a nose-up quadrotor falls — so
  // the equilibrium test is relaxed to let the CHART be exercised in isolation,
  // which is the only thing this case is about.
  Eigen::VectorXd vertical = hover.state;
  const galata::core::Quaternion attitude =
      galata::core::quaternion_from_euler({0.0, M_PI / 2.0, 0.0});
  vertical(galata::core::kQuaternionW) = attitude.w();
  vertical(galata::core::kQuaternionX) = attitude.x();
  vertical(galata::core::kQuaternionY) = attitude.y();
  vertical(galata::core::kQuaternionZ) = attitude.z();

  ExtendedLinearisationOptions options = options_for(model);
  options.equilibrium_tolerance = std::numeric_limits<double>::infinity();

  const ExtendedLinearisation linearisation =
      galata::linearize::linearize_extended(dynamics_for(model), vertical, hover.input, options);

  EXPECT_TRUE(linearisation.a.allFinite())
      << "a chart singular at ninety degrees of pitch produces infinities here";
  EXPECT_TRUE(linearisation.c.allFinite());

  // The attitude kinematics are the identity onto the body rates at every
  // attitude, which is the property an Euler chart loses at this one: there,
  // the roll and yaw rate rows are divided by a cosine that has just reached
  // zero.
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(linearisation.a(galata::linearize::kChartAttitudeErrorX + axis,
                                galata::linearize::kChartRateP + axis),
                1.0,
                1e-6);
  }
}
