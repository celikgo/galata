// SPDX-License-Identifier: Apache-2.0
//
// The quadrotor model file contract: what the loader accepts, what it refuses,
// and the battery block the shipped model omits.
//
// WHY THESE ARE WORTH WRITING. `load_aircraft` ignores unknown keys, and the
// adding-an-aircraft-model skill has to warn in bold that a typo there gives a
// model which "loads, trims, linearises and is quietly wrong". The quadrotor
// loader was written to refuse instead, and a refusal nobody tests is a refusal
// that will be removed by the next person who finds it inconvenient.

#include "galata/core/constants.hpp"
#include "galata/model/quadrotor.hpp"

#include <gtest/gtest.h>

#include <string>

namespace {

// A minimal well-formed document. Each rejection test perturbs exactly one
// thing about this, so a failure names the rule that fired.
//
// Izz is deliberately BELOW Ixx + Iyy rather than equal to it. A tensor on the
// triangle-inequality boundary is valid but has no margin, so adding any
// product of inertia to it is rejected — which would make the product-of-
// inertia case below untestable for a reason that has nothing to do with what
// it is testing.
std::string minimal_document() {
  return R"(description: "two-rotor test article"
mass:
  mass_kg: 1.0
  inertia_xx_kg_m2: 0.01
  inertia_yy_kg_m2: 0.01
  inertia_zz_kg_m2: 0.015
rotors:
  - position_cg_to_hub_body_m: [0.1, 0.0, 0.0]
    spin_about_body_z: 1
    thrust_coefficient_n_s2: 1.0e-5
    torque_coefficient_n_m_s2: 1.0e-7
    speed_time_constant_s: 0.03
    minimum_speed_rad_s: 0.0
    maximum_speed_rad_s: 900.0
  - position_cg_to_hub_body_m: [-0.1, 0.0, 0.0]
    spin_about_body_z: -1
    thrust_coefficient_n_s2: 1.0e-5
    torque_coefficient_n_m_s2: 1.0e-7
    speed_time_constant_s: 0.03
    minimum_speed_rad_s: 0.0
    maximum_speed_rad_s: 900.0
drag:
  linear_n_s_m: [0.1, 0.1, 0.15]
  quadratic_n_s2_m2: [0.02, 0.02, 0.03]
  angular_n_m_s: [0.001, 0.001, 0.002]
)";
}

std::string with(const std::string& find, const std::string& replace) {
  std::string document = minimal_document();
  const std::size_t at = document.find(find);
  EXPECT_NE(at, std::string::npos) << "test scaffold no longer matches the document";
  document.replace(at, find.size(), replace);
  return document;
}

}  // namespace

TEST(QuadrotorFile, MinimalDocumentLoads) {
  const galata::model::Quadrotor model = galata::model::parse_quadrotor(minimal_document());
  EXPECT_EQ(model.rotor_count(), 2);
  EXPECT_FALSE(model.has_battery());
  // Thirteen rigid-body components plus one state per rotor, and no more.
  EXPECT_EQ(model.extended_state_size(), 15);
  EXPECT_EQ(model.extended_state_names().size(), 15u);
  EXPECT_EQ(model.input_names().size(), 2u);
}

// THE RULE `load_aircraft` DOES NOT HAVE. A misspelled optional key there is
// silently ignored; here it is an error, at every level of the document.
TEST(QuadrotorFile, UnknownKeysAreRefusedAtEveryLevel) {
  EXPECT_THROW((void)galata::model::parse_quadrotor(minimal_document() + "wing_area_m2: 3.0\n"),
               std::invalid_argument);
  EXPECT_THROW((void)galata::model::parse_quadrotor(
                   with("  mass_kg: 1.0", "  mass_kg: 1.0\n  mass_lb: 2.2")),
               std::invalid_argument);
  EXPECT_THROW((void)galata::model::parse_quadrotor(with(
                   "    spin_about_body_z: 1", "    spin_about_body_z: 1\n    blade_count: 2")),
               std::invalid_argument);
  EXPECT_THROW((void)galata::model::parse_quadrotor(
                   with("  linear_n_s_m:", "  cubic_n_s3_m3: [1.0, 1.0, 1.0]\n  linear_n_s_m:")),
               std::invalid_argument);
}

TEST(QuadrotorFile, DuplicateKeysAreRefused) {
  EXPECT_THROW((void)galata::model::parse_quadrotor(
                   with("  mass_kg: 1.0", "  mass_kg: 1.0\n  mass_kg: 2.0")),
               std::invalid_argument);
}

TEST(QuadrotorFile, MissingRequiredBlocksAreRefused) {
  EXPECT_THROW((void)galata::model::parse_quadrotor("description: \"nothing else\"\n"),
               std::invalid_argument);
  EXPECT_THROW((void)galata::model::parse_quadrotor(with("rotors:", "wings:")),
               std::invalid_argument);
}

// The rejection list the RFC names, one assertion per rule.
TEST(QuadrotorFile, PhysicallyImpossibleModelsAreRefused) {
  // Non-finite values.
  EXPECT_THROW((void)galata::model::parse_quadrotor(with("  mass_kg: 1.0", "  mass_kg: .nan")),
               std::invalid_argument);
  // A non-positive-definite inertia tensor: the triangle inequality fails.
  EXPECT_THROW((void)galata::model::parse_quadrotor(
                   with("  inertia_zz_kg_m2: 0.015", "  inertia_zz_kg_m2: 0.5")),
               std::invalid_argument);
  // Zero rotors.
  EXPECT_THROW((void)galata::model::parse_quadrotor(with("rotors:", "rotors: []\nunused:")),
               std::invalid_argument);
  // A negative coefficient.
  EXPECT_THROW((void)galata::model::parse_quadrotor(
                   with("  linear_n_s_m: [0.1, 0.1, 0.15]", "  linear_n_s_m: [-0.1, 0.1, 0.15]")),
               std::invalid_argument);
  // A spin that is neither +1 nor -1 — the yaw axis has no other meaning.
  EXPECT_THROW((void)galata::model::parse_quadrotor(
                   with("    spin_about_body_z: 1\n", "    spin_about_body_z: 0\n")),
               std::invalid_argument);
  // A zero lag is an algebraic constraint, not a state.
  EXPECT_THROW((void)galata::model::parse_quadrotor(
                   with("    speed_time_constant_s: 0.03\n", "    speed_time_constant_s: 0.0\n")),
               std::invalid_argument);
  // An empty speed range.
  EXPECT_THROW((void)galata::model::parse_quadrotor(
                   with("    maximum_speed_rad_s: 900.0\n", "    maximum_speed_rad_s: 0.0\n")),
               std::invalid_argument);
}

TEST(QuadrotorFile, ProductsOfInertiaAreNegatedIntoTheTensorAsMassPropertiesDocuments) {
  const galata::model::Quadrotor model = galata::model::parse_quadrotor(
      with("  inertia_zz_kg_m2: 0.015",
           "  inertia_zz_kg_m2: 0.015\n  product_of_inertia_xz_kg_m2: 0.004"));
  // A source quoting a POSITIVE product of inertia contributes a NEGATIVE
  // off-diagonal entry. Pre-negating it in the file would flip the roll-yaw
  // coupling handedness, and the tensor would still be symmetric and positive
  // definite, so nothing downstream would catch it.
  EXPECT_DOUBLE_EQ(model.mass.inertia_cg_body_kg_m2(0, 2), -0.004);
  EXPECT_DOUBLE_EQ(model.mass.inertia_cg_body_kg_m2(2, 0), -0.004);
}

namespace {

std::string battery_block() {
  return R"(battery:
  energy_j: 432000.0
  full_voltage_v: 25.2
  empty_voltage_v: 19.8
  internal_resistance_ohm: 0.08
  speed_at_full_voltage_rad_s: 800.0
)";
}

}  // namespace

// The shipped model omits the battery, so this is where the block is exercised.
// These parameters are the test's own and describe no vehicle.
TEST(QuadrotorBattery, AddsOneStateAndScalesTheSpeedCeilingWithOpenCircuitVoltage) {
  const galata::model::Quadrotor model =
      galata::model::parse_quadrotor(minimal_document() + battery_block());
  ASSERT_TRUE(model.has_battery());
  EXPECT_EQ(model.extended_state_size(), 16);
  EXPECT_EQ(model.battery_state_index(), 15);
  EXPECT_EQ(model.extended_state_names().back(), "battery_soc");

  EXPECT_DOUBLE_EQ(model.open_circuit_voltage_v(1.0), 25.2);
  EXPECT_DOUBLE_EQ(model.open_circuit_voltage_v(0.0), 19.8);
  EXPECT_DOUBLE_EQ(model.open_circuit_voltage_v(0.5), 22.5);

  // The mechanical maximum binds at full charge, because 800 < 900.
  EXPECT_DOUBLE_EQ(model.speed_ceiling_rad_s(0, 1.0), 800.0);
  // Falling voltage lowers the ceiling proportionally.
  EXPECT_LT(model.speed_ceiling_rad_s(0, 0.0), model.speed_ceiling_rad_s(0, 1.0));
  EXPECT_DOUBLE_EQ(model.speed_ceiling_rad_s(0, 0.0), 800.0 * 19.8 / 25.2);

  // Internal resistance enters only through the terminal voltage.
  EXPECT_DOUBLE_EQ(model.terminal_voltage_v(1.0, 10.0), 25.2 - 0.8);
}

TEST(QuadrotorBattery, DischargesOnlyWhileTheRotorsTurnAndTheStateOfChargeStaysBounded) {
  const galata::model::Quadrotor model =
      galata::model::parse_quadrotor(minimal_document() + battery_block());
  Eigen::VectorXd extended = Eigen::VectorXd::Zero(model.extended_state_size());
  extended(galata::core::kQuaternionW) = 1.0;
  extended(model.battery_state_index()) = 1.0;
  const Eigen::VectorXd command = Eigen::VectorXd::Zero(model.rotor_count());

  // Rotors stopped: no shaft power, so no discharge.
  EXPECT_DOUBLE_EQ(model.derivative(extended, command)(model.battery_state_index()), 0.0);

  // Rotors turning: the state of charge falls.
  extended.segment(model.rotor_state_offset(), model.rotor_count()).setConstant(500.0);
  EXPECT_LT(model.derivative(extended, command)(model.battery_state_index()), 0.0);

  // The projection keeps the state of charge in [0, 1] and the rotors inside
  // the ceiling the battery allows.
  extended(model.battery_state_index()) = 1.5;
  extended(model.rotor_state_offset()) = 5000.0;
  model.project(extended);
  EXPECT_DOUBLE_EQ(extended(model.battery_state_index()), 1.0);
  EXPECT_DOUBLE_EQ(extended(model.rotor_state_offset()), 800.0);
}

TEST(QuadrotorModel, WrongLengthStateOrCommandIsRefusedRatherThanReinterpreted) {
  const galata::model::Quadrotor model = galata::model::parse_quadrotor(minimal_document());
  const Eigen::VectorXd good_state = Eigen::VectorXd::Zero(model.extended_state_size());
  const Eigen::VectorXd good_command = Eigen::VectorXd::Zero(model.rotor_count());
  EXPECT_THROW(
      (void)model.derivative(Eigen::VectorXd::Zero(model.extended_state_size() - 1), good_command),
      std::invalid_argument);
  EXPECT_THROW((void)model.derivative(good_state, Eigen::VectorXd::Zero(model.rotor_count() + 1)),
               std::invalid_argument);
}
