// SPDX-License-Identifier: Apache-2.0
//
// The pack under load: terminal voltage, the ceiling it sets, and what happens
// when the rotors ask for more than the pack has.
//
// WHY THESE VALUES ARE CHECKABLE. The Thevenin solve has a closed form. With
// V = V_oc - I R and I = P / V,
//
//     V^2 - V_oc V + P R = 0,   V = (V_oc + sqrt(V_oc^2 - 4 P R)) / 2
//
// so every expectation below is arithmetic on the model's own declared numbers,
// evaluated here independently of the implementation.
//
// AND WHY THE SAME FORM APPEARS TWICE. Souxmar's plant reaches it from the same
// physics — fcs/plant/quadrotor.py, `discriminant = voc*voc - 4*r*power` and
// the same larger root. That is agreement between two implementations, not
// validation against a measured aircraft: no pack was discharged to produce any
// of it.
//
// WHAT THIS IS NOT. Not an endurance prediction and not a statement about any
// hardware. The efficiency and auxiliary load below are round numbers this test
// chose; the repository holds no measurement of either for any aircraft, and
// the model file that carries them says so.

#include "galata/model/quadrotor.hpp"
#include "galata/trim/hover.hpp"

#include "validation_config.hpp"
#include <gtest/gtest.h>

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

std::string measured(double value) {
  std::ostringstream out;
  out << std::scientific << std::setprecision(3) << value;
  return out.str();
}

using galata::model::Battery;
using galata::model::Quadrotor;

Quadrotor packed(Battery::SagModel sag, double resistance = 0.03) {
  Quadrotor model = galata::model::load_quadrotor(std::string(GALATA_MODELS_DIR)
                                                  + "/souxmar-quad/souxmar-quad.yaml");
  Battery cell;
  cell.energy_j = 5.6e5;
  cell.full_voltage_v = 25.2;
  cell.empty_voltage_v = 19.8;
  cell.internal_resistance_ohm = resistance;
  cell.speed_at_full_voltage_rad_s = 1102.4200493562662;
  cell.sag = sag;
  cell.motor_and_esc_efficiency = 0.8;
  cell.auxiliary_load_w = 30.0;
  model.battery = cell;
  model.validate();
  return model;
}

double closed_form_terminal(double open_circuit, double resistance, double electrical_w) {
  const double discriminant = open_circuit * open_circuit - 4.0 * resistance * electrical_w;
  return 0.5 * (open_circuit + std::sqrt(discriminant));
}

}  // namespace

TEST(BatterySag, TheTerminalVoltageSolvesItsOwnQuadratic) {
  const Quadrotor model = packed(Battery::SagModel::Resistive);
  const double open_circuit = model.open_circuit_voltage_v(1.0);
  EXPECT_DOUBLE_EQ(open_circuit, 25.2);

  // Unloaded: no current, so no sag at all. This is the boundary the loaded
  // case must approach, and it is exact rather than close.
  EXPECT_NEAR(model.terminal_voltage_under_load_v(1.0, 0.0),
              closed_form_terminal(25.2, 0.03, 30.0 / 1.0),
              1e-12);

  // Loaded, at a shaft power a hover draws. Electrical = shaft / 0.8 + 30.
  const double shaft_w = 200.0;
  const double electrical_w = shaft_w / 0.8 + 30.0;
  const double expected = closed_form_terminal(25.2, 0.03, electrical_w);
  EXPECT_NEAR(model.terminal_voltage_under_load_v(1.0, shaft_w), expected, 1e-12);
  EXPECT_LT(model.terminal_voltage_under_load_v(1.0, shaft_w), open_circuit)
      << "a pack under load must sag";
}

TEST(BatterySag, TheCeilingFallsWithTheLoadedVoltageAndOpenCircuitIgnoresIt) {
  const Quadrotor sagging = packed(Battery::SagModel::Resistive);
  const Quadrotor ideal = packed(Battery::SagModel::OpenCircuit);
  const double shaft_w = 200.0;

  const double loaded = sagging.speed_ceiling_rad_s(0, 1.0, shaft_w);
  const double unloaded = sagging.speed_ceiling_rad_s(0, 1.0);
  EXPECT_LT(loaded, unloaded) << "the loaded ceiling must be below the no-load one";

  // The open-circuit model is unchanged by load: that is its defining
  // limitation, held here so a future change cannot make it quietly disagree
  // with what its own header claims.
  EXPECT_DOUBLE_EQ(ideal.speed_ceiling_rad_s(0, 1.0, shaft_w), ideal.speed_ceiling_rad_s(0, 1.0));
}

TEST(BatterySag, PastTheMatchedLoadThePackSaturatesAndSaysSo) {
  const Quadrotor model = packed(Battery::SagModel::Resistive);
  // V_oc^2 / (4 R) = 25.2^2 / 0.12, evaluated here rather than read off.
  const double maximum_w = 25.2 * 25.2 / (4.0 * 0.03);
  EXPECT_NEAR(model.maximum_deliverable_power_w(1.0), maximum_w, 1e-9);

  const double within = 0.5 * maximum_w;
  EXPECT_FALSE(model.power_is_limited_at(1.0, within * 0.8 - 30.0));

  // Ask for twice what the pack can give.
  const double beyond_shaft_w = 2.0 * maximum_w * 0.8;
  EXPECT_TRUE(model.power_is_limited_at(1.0, beyond_shaft_w));
  // At and past the matched load the terminal voltage is V_oc / 2 — the pack
  // saturates rather than the solve failing, which is what the hardware does.
  EXPECT_NEAR(model.terminal_voltage_under_load_v(1.0, beyond_shaft_w), 0.5 * 25.2, 1e-12);
}

TEST(BatterySag, ADepletedPackHasTheEmptyVoltageAndLessAuthority) {
  const Quadrotor model = packed(Battery::SagModel::Resistive);
  EXPECT_DOUBLE_EQ(model.open_circuit_voltage_v(0.0), 19.8);
  const double shaft_w = 150.0;
  EXPECT_LT(model.speed_ceiling_rad_s(0, 0.0, shaft_w), model.speed_ceiling_rad_s(0, 1.0, shaft_w))
      << "an empty pack must offer less speed than a full one at the same load";
}

TEST(BatterySag, InvalidBatteryParametersAreRefused) {
  Quadrotor model = packed(Battery::SagModel::Resistive);
  Battery bad = *model.battery;

  bad.motor_and_esc_efficiency = 1.5;
  model.battery = bad;
  EXPECT_THROW(model.validate(), std::invalid_argument)
      << "a motor cannot produce more shaft power than the pack delivers";

  bad = *packed(Battery::SagModel::Resistive).battery;
  bad.motor_and_esc_efficiency = 0.0;
  model.battery = bad;
  EXPECT_THROW(model.validate(), std::invalid_argument);

  bad = *packed(Battery::SagModel::Resistive).battery;
  bad.auxiliary_load_w = -5.0;
  model.battery = bad;
  EXPECT_THROW(model.validate(), std::invalid_argument)
      << "a negative auxiliary load is a generator";

  // The resistive model needs a resistance to be about.
  bad = *packed(Battery::SagModel::Resistive).battery;
  bad.internal_resistance_ohm = 0.0;
  model.battery = bad;
  EXPECT_THROW(model.validate(), std::invalid_argument);
}

// The trim's reported margin must be the one that applies while the vehicle is
// holding the equilibrium, not the one an idle pack would offer.
TEST(BatterySag, TheTrimMarginIsTakenAgainstTheLoadedCeiling) {
  const Quadrotor sagging = packed(Battery::SagModel::Resistive);
  const Quadrotor ideal = packed(Battery::SagModel::OpenCircuit);

  galata::trim::HoverTrimRequest request;
  request.altitude_m = 120.0;
  const auto loaded = galata::trim::trim_hover(sagging, request);
  const auto unloaded = galata::trim::trim_hover(ideal, request);

  EXPECT_LE(loaded.residual_norm, loaded.residual_tolerance);
  // The rotor speeds are identical — the equilibrium does not depend on the
  // pack — but the MARGIN does.
  EXPECT_NEAR((loaded.command_rad_s - unloaded.command_rad_s).norm(), 0.0, 1e-12)
      << "the sag model must not move the equilibrium, only the authority left over it";
  EXPECT_LT(loaded.smallest_rotor_margin_fraction, unloaded.smallest_rotor_margin_fraction)
      << "a margin computed against an unloaded pack overstates what the vehicle has";

  RecordProperty("loaded_margin_fraction", measured(loaded.smallest_rotor_margin_fraction));
  RecordProperty("open_circuit_margin_fraction", measured(unloaded.smallest_rotor_margin_fraction));
}
