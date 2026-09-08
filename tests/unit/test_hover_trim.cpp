// SPDX-License-Identifier: Apache-2.0
//
// The contract of `trim::trim_hover`: what it refuses, and why each refusal is
// a refusal rather than a best effort. The numerical cases — that the answers
// are right — are in the validation tier; these hold the boundary.

#include "galata/core/constants.hpp"
#include "galata/core/state.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/trim/hover.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

using galata::core::kStandardGravity;
using galata::model::Battery;
using galata::model::Quadrotor;
using galata::model::Rotor;
using galata::trim::HoverTrim;
using galata::trim::HoverTrimRequest;

// A minimal X-configuration quadrotor written here rather than loaded, so a
// change to the shipped model file cannot silently change what these cases
// mean. The numbers are of the right order for a small multirotor and are not
// anybody's aircraft.
Quadrotor test_model() {
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
  model.validate();
  return model;
}

}  // namespace

// --- what is solved, and what is declared ----------------------------------

TEST(HoverTrim, YawIsReturnedAsDeclaredRatherThanSolvedFor) {
  const Quadrotor model = test_model();
  // A multirotor in still air is in equilibrium at every heading, so the
  // heading cannot be an output: the system would be singular in it. Two
  // headings must give the same rotor speeds and the heading each was given.
  for (const double heading_rad : {0.0, 1.3, -2.7}) {
    HoverTrimRequest request;
    request.heading_rad = heading_rad;
    const HoverTrim trim = galata::trim::trim_hover(model, request);
    EXPECT_DOUBLE_EQ(trim.yaw_rad, heading_rad);
    EXPECT_NEAR(trim.command_rad_s(0), model.hover_speed_rad_s(kStandardGravity), 1e-9);
  }
}

TEST(HoverTrim, TheReportedPositionCarriesTheDeclaredAltitudeAndNothingElse) {
  const Quadrotor model = test_model();
  HoverTrimRequest request;
  request.altitude_m = 250.0;
  const HoverTrim trim = galata::trim::trim_hover(model, request);

  EXPECT_DOUBLE_EQ(trim.extended_state(galata::core::kPositionDown), -250.0)
      << "NED down is the negative of altitude";
  EXPECT_DOUBLE_EQ(trim.extended_state(galata::core::kPositionNorth), 0.0)
      << "horizontal position is arbitrary at this equilibrium and must not be invented";
  EXPECT_DOUBLE_EQ(trim.extended_state(galata::core::kPositionEast), 0.0);
  EXPECT_DOUBLE_EQ(trim.altitude_m, 250.0);
}

TEST(HoverTrim, TheEvidenceTravelsWithTheAnswer) {
  const Quadrotor model = test_model();
  HoverTrimRequest request;
  const HoverTrim trim = galata::trim::trim_hover(model, request);

  EXPECT_EQ(trim.newton_iterations, request.iterations)
      << "the iteration count is fixed by ADR-0004, so reporting it must report the budget "
         "spent and not the budget needed";
  EXPECT_EQ(static_cast<int>(trim.residual_history.size()), request.iterations + 1);
  EXPECT_DOUBLE_EQ(trim.residual_tolerance, request.residual_tolerance);
  EXPECT_GT(trim.jacobian_condition_number, 0.0);
  EXPECT_EQ(static_cast<int>(trim.rotor_margin_fraction.size()), model.rotor_count());
  EXPECT_DOUBLE_EQ(trim.wind_ned_m_s.x(), request.wind_ned_m_s.x())
      << "the wind must travel with the trim so a linearisation cannot be taken at a "
         "different one";
}

// --- malformed requests ----------------------------------------------------

TEST(HoverTrim, AnOverActuatedVehicleIsRefusedRatherThanAllocatedArbitrarily) {
  Quadrotor hexarotor = test_model();
  hexarotor.rotors.push_back(hexarotor.rotors[0]);
  hexarotor.rotors.push_back(hexarotor.rotors[1]);
  hexarotor.rotors[4].position_cg_to_hub_body_m = Eigen::Vector3d(0.0, -0.23, 0.0);
  hexarotor.rotors[5].position_cg_to_hub_body_m = Eigen::Vector3d(0.0, 0.23, 0.0);

  // Six unknowns is not the problem; six rotors plus two attitude angles is
  // eight unknowns against six equations, and the null space is what makes a
  // Newton answer meaningless rather than merely non-unique.
  EXPECT_THROW((void)galata::trim::trim_hover(hexarotor, HoverTrimRequest{}),
               std::invalid_argument);

  Quadrotor trirotor = test_model();
  trirotor.rotors.pop_back();
  EXPECT_THROW((void)galata::trim::trim_hover(trirotor, HoverTrimRequest{}), std::invalid_argument);
}

TEST(HoverTrim, MalformedRequestsAreRefusedWithTheirOwnDiagnosis) {
  const Quadrotor model = test_model();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();

  const auto refused = [&](HoverTrimRequest request) {
    EXPECT_THROW((void)galata::trim::trim_hover(model, request), std::invalid_argument);
  };

  HoverTrimRequest charge_high;
  charge_high.battery_state_of_charge = 1.5;
  refused(charge_high);

  HoverTrimRequest charge_low;
  charge_low.battery_state_of_charge = -0.1;
  refused(charge_low);

  HoverTrimRequest charge_nan;
  charge_nan.battery_state_of_charge = nan;
  refused(charge_nan);

  HoverTrimRequest wind;
  wind.wind_ned_m_s = Eigen::Vector3d(nan, 0.0, 0.0);
  refused(wind);

  HoverTrimRequest velocity;
  velocity.ground_velocity_ned_m_s = Eigen::Vector3d(0.0, infinity, 0.0);
  refused(velocity);

  HoverTrimRequest heading;
  heading.heading_rad = nan;
  refused(heading);

  HoverTrimRequest altitude;
  altitude.altitude_m = infinity;
  refused(altitude);

  HoverTrimRequest iterations;
  iterations.iterations = 0;
  refused(iterations);

  HoverTrimRequest negative_iterations;
  negative_iterations.iterations = -3;
  refused(negative_iterations);

  // A budget of zero is a gate nothing can pass, not an exact one.
  HoverTrimRequest tolerance;
  tolerance.residual_tolerance = 0.0;
  refused(tolerance);

  HoverTrimRequest negative_tolerance;
  negative_tolerance.residual_tolerance = -1e-9;
  refused(negative_tolerance);
}

TEST(HoverTrim, AnInvalidModelIsRefusedBeforeAnythingIsSolved) {
  Quadrotor broken = test_model();
  broken.rotors[2].speed_time_constant_s = 0.0;
  EXPECT_THROW((void)galata::trim::trim_hover(broken, HoverTrimRequest{}), std::invalid_argument);
}

// --- infeasible trims ------------------------------------------------------

TEST(HoverTrim, ATrimNeedingMoreRotorThanTheVehicleHasIsRefusedNotReturned) {
  Quadrotor model = test_model();
  // A ceiling just under the hover speed. The equilibrium still EXISTS — it is
  // a root of the equations — and returning it with a note would let it be
  // linearised as though the vehicle could hold it.
  const double hover_speed = model.hover_speed_rad_s(kStandardGravity);
  for (Rotor& rotor : model.rotors) {
    rotor.maximum_speed_rad_s = 0.95 * hover_speed;
  }
  model.validate();

  try {
    const HoverTrim trim = galata::trim::trim_hover(model, HoverTrimRequest{});
    FAIL() << "an infeasible trim was returned with rotor speed " << trim.command_rad_s(0);
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("infeasible"), std::string::npos) << message;
    EXPECT_NE(message.find("Rotor"), std::string::npos)
        << "the message must name which rotor exceeded its ceiling: " << message;
  }
}

TEST(HoverTrim, ATrimBelowARotorsFloorIsAlsoRefused) {
  Quadrotor model = test_model();
  const double hover_speed = model.hover_speed_rad_s(kStandardGravity);
  for (Rotor& rotor : model.rotors) {
    rotor.minimum_speed_rad_s = 1.05 * hover_speed;
    rotor.maximum_speed_rad_s = 2.0 * hover_speed;
  }
  model.validate();
  EXPECT_THROW((void)galata::trim::trim_hover(model, HoverTrimRequest{}), std::runtime_error);
}

TEST(HoverTrim, AFeasibleTrimReportsEveryRotorsMarginAndTheSmallestOfThem) {
  Quadrotor model = test_model();
  const double hover_speed = model.hover_speed_rad_s(kStandardGravity);
  const double ceiling = 2.0 * hover_speed;
  for (Rotor& rotor : model.rotors) {
    rotor.maximum_speed_rad_s = ceiling;
  }
  model.validate();

  const HoverTrim trim = galata::trim::trim_hover(model, HoverTrimRequest{});
  ASSERT_EQ(static_cast<int>(trim.rotor_margin_fraction.size()), 4);
  for (const double margin : trim.rotor_margin_fraction) {
    EXPECT_GE(margin, 0.0) << "a negative margin is refused before it can be reported";
    // Four equal rotors at half their ceiling.
    EXPECT_NEAR(margin, 0.5, 1e-9);
  }
  EXPECT_NEAR(trim.smallest_rotor_margin_fraction, 0.5, 1e-9);
}

// --- the battery -----------------------------------------------------------

TEST(HoverTrim, StateOfChargeIsFrozenIntoTheStateAndNeverSolvedFor) {
  Quadrotor model = test_model();
  Battery battery;
  battery.energy_j = 3600.0 * 100.0;
  battery.full_voltage_v = 25.2;
  battery.empty_voltage_v = 19.8;
  battery.internal_resistance_ohm = 0.03;
  battery.speed_at_full_voltage_rad_s = 1100.0;
  model.battery = battery;
  model.validate();

  HoverTrimRequest request;
  request.battery_state_of_charge = 0.55;
  const HoverTrim trim = galata::trim::trim_hover(model, request);

  ASSERT_EQ(trim.extended_state.size(), model.extended_state_size());
  EXPECT_DOUBLE_EQ(trim.extended_state(model.battery_state_index()), 0.55)
      << "the declared charge must appear in the state unchanged; a powered battery has no "
         "zero-derivative equilibrium, so solving for it would make every trim infeasible";
  EXPECT_DOUBLE_EQ(trim.battery_state_of_charge, 0.55);

  // The charge reaches the answer through the speed ceiling and nowhere else,
  // so a lower charge is a smaller margin at the same rotor speed.
  HoverTrimRequest full = request;
  full.battery_state_of_charge = 1.0;
  const HoverTrim charged = galata::trim::trim_hover(model, full);
  EXPECT_NEAR(charged.command_rad_s(0), trim.command_rad_s(0), 1e-9)
      << "the hover speed itself does not depend on the charge";
  EXPECT_GT(charged.smallest_rotor_margin_fraction, trim.smallest_rotor_margin_fraction)
      << "a fuller pack must have more margin to its ceiling";
}
