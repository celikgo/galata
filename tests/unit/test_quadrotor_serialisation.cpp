// SPDX-License-Identifier: Apache-2.0
//
// The quadrotor model file round trip: a model galata computed and a model
// galata was given must be the same kind of object.
//
// WHY BIT-FOR-BIT AND NOT "CLOSE". `identify.greybox` produces a plant and
// `model.quadrotor.export` writes it; if the file is a lossy picture of the
// model, then the fitted plant a study analyses and the fitted plant the next
// study loads are two different aircraft, and nothing would say so. ADR-0004
// wants the same bits, and `max_digits10` in the classic locale is what makes a
// binary64 survive the trip exactly. So these compare with `EXPECT_EQ` on
// doubles deliberately: a tolerance here would be hiding the one defect the
// round trip exists to catch.

#include "galata/model/quadrotor.hpp"

#include <gtest/gtest.h>

#include <string>

namespace {

using galata::model::parse_quadrotor;
using galata::model::Quadrotor;
using galata::model::serialize_quadrotor;

// Deliberately awkward: values that are not representable as short decimals, a
// full inertia tensor with all three products of inertia, and a battery.
std::string awkward_document() {
  return R"(description: "round-trip article"
citation: "no source; a fixture"
mass:
  mass_kg: 2.4750000000000001
  inertia_xx_kg_m2: 0.030100000000000002
  inertia_yy_kg_m2: 0.0332
  inertia_zz_kg_m2: 0.0581
  product_of_inertia_xy_kg_m2: 0.00013
  product_of_inertia_xz_kg_m2: -0.00027000000000000001
  product_of_inertia_yz_kg_m2: 7.0000000000000007e-05
rotors:
  - position_cg_to_hub_body_m: [0.16263455967290593, -0.16263455967290593, 0.012]
    spin_about_body_z: 1
    thrust_coefficient_n_s2: 1.0000000000000001e-05
    torque_coefficient_n_m_s2: 1.7000000000000001e-07
    speed_time_constant_s: 0.035000000000000003
    minimum_speed_rad_s: 12.3
    maximum_speed_rad_s: 1102.4200493562662
  - position_cg_to_hub_body_m: [-0.16263455967290593, 0.16263455967290593, 0.012]
    spin_about_body_z: -1
    thrust_coefficient_n_s2: 9.1999999999999998e-06
    torque_coefficient_n_m_s2: 1.63e-07
    speed_time_constant_s: 0.036999999999999998
    minimum_speed_rad_s: 12.3
    maximum_speed_rad_s: 1102.4200493562662
drag:
  linear_n_s_m: [0.12, 0.12, 0.17999999999999999]
  quadratic_n_s2_m2: [0.025000000000000001, 0.025000000000000001, 0.035000000000000003]
  angular_n_m_s: [0.002, 0.002, 0.0030000000000000001]
battery:
  energy_j: 266400
  full_voltage_v: 25.199999999999999
  empty_voltage_v: 19.800000000000001
  internal_resistance_ohm: 0.042000000000000003
  speed_at_full_voltage_rad_s: 1102.4200493562662
  sag: resistive
  motor_and_esc_efficiency: 0.82999999999999996
  auxiliary_load_w: 7.5
)";
}

void expect_same_model(const Quadrotor& left, const Quadrotor& right) {
  EXPECT_EQ(left.mass.mass_kg, right.mass.mass_kg);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      EXPECT_EQ(left.mass.inertia_cg_body_kg_m2(row, column),
                right.mass.inertia_cg_body_kg_m2(row, column))
          << "inertia entry (" << row << ", " << column << ")";
    }
  }
  ASSERT_EQ(left.rotor_count(), right.rotor_count());
  for (std::size_t index = 0; index < left.rotors.size(); ++index) {
    const auto& a = left.rotors[index];
    const auto& b = right.rotors[index];
    EXPECT_EQ(a.position_cg_to_hub_body_m, b.position_cg_to_hub_body_m) << "rotor " << index;
    EXPECT_EQ(a.spin_about_body_z, b.spin_about_body_z) << "rotor " << index;
    EXPECT_EQ(a.thrust_coefficient_n_s2, b.thrust_coefficient_n_s2) << "rotor " << index;
    EXPECT_EQ(a.torque_coefficient_n_m_s2, b.torque_coefficient_n_m_s2) << "rotor " << index;
    EXPECT_EQ(a.speed_time_constant_s, b.speed_time_constant_s) << "rotor " << index;
    EXPECT_EQ(a.minimum_speed_rad_s, b.minimum_speed_rad_s) << "rotor " << index;
    EXPECT_EQ(a.maximum_speed_rad_s, b.maximum_speed_rad_s) << "rotor " << index;
  }
  EXPECT_EQ(left.drag_linear_n_s_m, right.drag_linear_n_s_m);
  EXPECT_EQ(left.drag_quadratic_n_s2_m2, right.drag_quadratic_n_s2_m2);
  EXPECT_EQ(left.angular_drag_n_m_s, right.angular_drag_n_m_s);
  ASSERT_EQ(left.has_battery(), right.has_battery());
  if (left.has_battery()) {
    EXPECT_EQ(left.battery->energy_j, right.battery->energy_j);
    EXPECT_EQ(left.battery->full_voltage_v, right.battery->full_voltage_v);
    EXPECT_EQ(left.battery->empty_voltage_v, right.battery->empty_voltage_v);
    EXPECT_EQ(left.battery->internal_resistance_ohm, right.battery->internal_resistance_ohm);
    EXPECT_EQ(left.battery->speed_at_full_voltage_rad_s,
              right.battery->speed_at_full_voltage_rad_s);
    EXPECT_EQ(left.battery->sag, right.battery->sag);
    EXPECT_EQ(left.battery->motor_and_esc_efficiency, right.battery->motor_and_esc_efficiency);
    EXPECT_EQ(left.battery->auxiliary_load_w, right.battery->auxiliary_load_w);
  }
}

}  // namespace

TEST(QuadrotorSerialisation, EveryParameterSurvivesTheRoundTripExactly) {
  const Quadrotor original = parse_quadrotor(awkward_document(), "fixture");
  const std::string written = serialize_quadrotor(original);
  const Quadrotor reloaded = parse_quadrotor(written, "written");
  expect_same_model(original, reloaded);
}

// The writer's output must be a FIXED POINT of the pair. One round trip could
// pass while quietly normalising something; a second trip that produces
// different bytes would prove it did.
TEST(QuadrotorSerialisation, WritingWhatWasWrittenProducesTheSameBytes) {
  const Quadrotor original = parse_quadrotor(awkward_document(), "fixture");
  const std::string once = serialize_quadrotor(original);
  const std::string twice = serialize_quadrotor(parse_quadrotor(once, "written"));
  EXPECT_EQ(once, twice);
}

// The products of inertia are the NEGATIVE off-diagonal entries of the tensor.
// The loader negates once; the writer must undo it once. A double negation or a
// missing one leaves a model that round-trips through the reader and describes a
// different aircraft.
TEST(QuadrotorSerialisation, ProductsOfInertiaComeBackWithTheSignTheFileDeclared) {
  const Quadrotor original = parse_quadrotor(awkward_document(), "fixture");
  const std::string written = serialize_quadrotor(original);
  // Written at `max_digits10`, so the shortest decimal the file used comes back
  // as the full 17 significant digits of the same binary64 — the same value,
  // spelled so it cannot lose a bit on the next trip.
  EXPECT_NE(written.find("product_of_inertia_xy_kg_m2: 0.00012999999999999999"), std::string::npos)
      << written;
  EXPECT_NE(written.find("product_of_inertia_xz_kg_m2: -0.00027"), std::string::npos) << written;
  EXPECT_EQ(original.mass.inertia_cg_body_kg_m2(0, 1), -0.00013);
  EXPECT_EQ(original.mass.inertia_cg_body_kg_m2(0, 2), 0.00027);
}

// A diagonal-inertia model must come back as the file it came from, not as one
// carrying three explicit zeros. The shipped model is the case.
TEST(QuadrotorSerialisation, ADiagonalInertiaModelDoesNotAcquireThreeZeroProducts) {
  std::string document = awkward_document();
  const auto start = document.find("  product_of_inertia_xy_kg_m2");
  const auto end = document.find("rotors:");
  document.erase(start, end - start);
  const Quadrotor original = parse_quadrotor(document, "fixture");
  const std::string written = serialize_quadrotor(original);
  EXPECT_EQ(written.find("product_of_inertia"), std::string::npos) << written;
  expect_same_model(original, parse_quadrotor(written, "written"));
}

// The parser reads `sag` as optional so a file written before the resistive
// option existed keeps its behaviour. The WRITER has no history to preserve, so
// it always states which model is in force: leaving it implicit in a file a fit
// produced is exactly the silence the round trip removes.
TEST(QuadrotorSerialisation, TheBatterySagModelIsAlwaysWrittenOutEvenWhenItIsTheDefault) {
  std::string document = awkward_document();
  document.replace(document.find("  sag: resistive"), std::string("  sag: resistive").size(), "");
  const Quadrotor original = parse_quadrotor(document, "fixture");
  ASSERT_TRUE(original.has_battery());
  EXPECT_EQ(original.battery->sag, galata::model::Battery::SagModel::OpenCircuit);
  EXPECT_NE(serialize_quadrotor(original).find("sag: open_circuit"), std::string::npos);
}

// A model the writer would refuse must fail here rather than becoming a file
// that only fails when somebody loads it.
TEST(QuadrotorSerialisation, AMalformedModelIsRefusedByTheWriterRatherThanWritten) {
  Quadrotor broken = parse_quadrotor(awkward_document(), "fixture");
  broken.mass.mass_kg = -1.0;
  EXPECT_THROW((void)serialize_quadrotor(broken), std::invalid_argument);

  Quadrotor with_control_character = parse_quadrotor(awkward_document(), "fixture");
  with_control_character.description = std::string("a\ndescription");
  EXPECT_THROW((void)serialize_quadrotor(with_control_character), std::invalid_argument);
}
