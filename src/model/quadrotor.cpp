// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the multirotor plant declared in
// include/galata/model/quadrotor.hpp. The references and the validity envelope
// are there; this file carries only what the equations do.

#include "galata/model/quadrotor.hpp"

#include "galata/core/constants.hpp"
#include "galata/core/frames.hpp"

#include "../io/strict_yaml.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace galata::model {
namespace {

void require_finite(double value, const std::string& what) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument("quadrotor: " + what + " must be finite");
  }
}

void require_finite(const Eigen::Vector3d& value, const std::string& what) {
  for (int axis = 0; axis < 3; ++axis) {
    require_finite(value(axis), what);
  }
}

// Per-axis drag magnitude. The quadratic term uses v |v| rather than v^2 so it
// opposes motion in either direction; v^2 would accelerate rearward flight.
[[nodiscard]] Eigen::Vector3d drag_force_body_n(const Eigen::Vector3d& velocity_body_m_s,
                                                const Eigen::Vector3d& linear,
                                                const Eigen::Vector3d& quadratic) noexcept {
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    const double speed = velocity_body_m_s(axis);
    force(axis) = -(linear(axis) * speed + quadratic(axis) * speed * std::abs(speed));
  }
  return force;
}

[[nodiscard]] double scalar(const YAML::Node& node,
                            const std::string& path,
                            const std::string& key) {
  if (!node[key]) {
    throw std::invalid_argument(path + ": missing required key '" + key + "'");
  }
  try {
    return node[key].as<double>();
  } catch (const YAML::Exception&) {
    throw std::invalid_argument(path + "." + key + ": expected a number");
  }
}

[[nodiscard]] double optional_scalar(const YAML::Node& node,
                                     const std::string& path,
                                     const std::string& key,
                                     double fallback) {
  if (!node[key]) {
    return fallback;
  }
  try {
    return node[key].as<double>();
  } catch (const YAML::Exception&) {
    throw std::invalid_argument(path + "." + key + ": expected a number");
  }
}

[[nodiscard]] Eigen::Vector3d vector3(const YAML::Node& node,
                                      const std::string& path,
                                      const std::string& key) {
  const YAML::Node entry = node[key];
  if (!entry) {
    throw std::invalid_argument(path + ": missing required key '" + key + "'");
  }
  if (!entry.IsSequence() || entry.size() != 3) {
    throw std::invalid_argument(path + "." + key + ": expected three numbers");
  }
  Eigen::Vector3d value = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    try {
      value(axis) = entry[static_cast<std::size_t>(axis)].as<double>();
    } catch (const YAML::Exception&) {
      throw std::invalid_argument(path + "." + key + ": expected three numbers");
    }
  }
  return value;
}

[[nodiscard]] std::string optional_text(const YAML::Node& node, const std::string& key) {
  return node[key] ? node[key].as<std::string>() : std::string();
}

}  // namespace

int Quadrotor::battery_state_index() const {
  if (!has_battery()) {
    throw std::invalid_argument("quadrotor: the model carries no battery state");
  }
  return core::kStateSize + rotor_count();
}

void Quadrotor::validate() const {
  mass.validate();

  if (rotors.empty()) {
    throw std::invalid_argument("quadrotor: a model needs at least one rotor");
  }

  require_finite(drag_linear_n_s_m, "linear drag");
  require_finite(drag_quadratic_n_s2_m2, "quadratic drag");
  require_finite(angular_drag_n_m_s, "angular drag");
  for (int axis = 0; axis < 3; ++axis) {
    if (drag_linear_n_s_m(axis) < 0.0 || drag_quadratic_n_s2_m2(axis) < 0.0
        || angular_drag_n_m_s(axis) < 0.0) {
      throw std::invalid_argument(
          "quadrotor: drag coefficients must be non-negative; a negative one is a sign error, "
          "not an exotic configuration, and it makes the plant unstable in a way that looks "
          "like a physical result");
    }
  }

  for (std::size_t index = 0; index < rotors.size(); ++index) {
    const Rotor& rotor = rotors[index];
    const std::string what = "rotor " + std::to_string(index);
    require_finite(rotor.position_cg_to_hub_body_m, what + " position");
    require_finite(rotor.thrust_coefficient_n_s2, what + " thrust coefficient");
    require_finite(rotor.torque_coefficient_n_m_s2, what + " torque coefficient");
    require_finite(rotor.speed_time_constant_s, what + " time constant");
    require_finite(rotor.minimum_speed_rad_s, what + " minimum speed");
    require_finite(rotor.maximum_speed_rad_s, what + " maximum speed");

    if (rotor.spin_about_body_z != 1 && rotor.spin_about_body_z != -1) {
      throw std::invalid_argument(what + ": spin_about_body_z must be +1 or -1");
    }
    if (rotor.thrust_coefficient_n_s2 < 0.0 || rotor.torque_coefficient_n_m_s2 < 0.0) {
      throw std::invalid_argument(what + ": thrust and torque coefficients must be non-negative");
    }
    if (!(rotor.speed_time_constant_s > 0.0)) {
      throw std::invalid_argument(
          what + ": speed_time_constant_s must be positive; a zero lag is an algebraic "
                 "constraint, not a state, and this model carries the rotor speed as a state");
    }
    if (rotor.minimum_speed_rad_s < 0.0) {
      throw std::invalid_argument(what + ": minimum_speed_rad_s must be non-negative");
    }
    if (!(rotor.maximum_speed_rad_s > rotor.minimum_speed_rad_s)) {
      throw std::invalid_argument(what + ": maximum_speed_rad_s must exceed minimum_speed_rad_s");
    }
  }

  if (battery.has_value()) {
    const Battery& cell = *battery;
    require_finite(cell.energy_j, "battery energy");
    require_finite(cell.full_voltage_v, "battery full voltage");
    require_finite(cell.empty_voltage_v, "battery empty voltage");
    require_finite(cell.internal_resistance_ohm, "battery internal resistance");
    require_finite(cell.speed_at_full_voltage_rad_s, "battery speed at full voltage");
    if (!(cell.energy_j > 0.0)) {
      throw std::invalid_argument("quadrotor: battery energy_j must be positive");
    }
    if (!(cell.full_voltage_v > cell.empty_voltage_v)) {
      throw std::invalid_argument("quadrotor: battery full_voltage_v must exceed empty_voltage_v");
    }
    if (!(cell.empty_voltage_v > 0.0)) {
      throw std::invalid_argument("quadrotor: battery empty_voltage_v must be positive");
    }
    if (cell.internal_resistance_ohm < 0.0) {
      throw std::invalid_argument(
          "quadrotor: battery internal_resistance_ohm must be non-negative");
    }
    if (!(cell.speed_at_full_voltage_rad_s > 0.0)) {
      throw std::invalid_argument(
          "quadrotor: battery speed_at_full_voltage_rad_s must be positive");
    }
    require_finite(cell.motor_and_esc_efficiency, "battery motor_and_esc_efficiency");
    require_finite(cell.auxiliary_load_w, "battery auxiliary_load_w");
    if (!(cell.motor_and_esc_efficiency > 0.0) || cell.motor_and_esc_efficiency > 1.0) {
      throw std::invalid_argument(
          "quadrotor: battery motor_and_esc_efficiency must be in (0, 1]. Above one is a motor "
          "that produces more shaft power than the pack delivers");
    }
    if (cell.auxiliary_load_w < 0.0) {
      throw std::invalid_argument(
          "quadrotor: battery auxiliary_load_w must be non-negative; a negative load is a "
          "generator, which this model does not carry");
    }
    if (cell.sag == Battery::SagModel::Resistive && !(cell.internal_resistance_ohm > 0.0)) {
      throw std::invalid_argument(
          "quadrotor: the resistive sag model needs a positive internal_resistance_ohm; with "
          "zero resistance there is no sag to model and `open_circuit` says so directly");
    }
  }
}

std::vector<std::string> Quadrotor::extended_state_names() const {
  std::vector<std::string> names = {
      "p_n", "p_e", "p_d", "u", "v", "w", "q_w", "q_x", "q_y", "q_z", "p", "q", "r"};
  for (int index = 0; index < rotor_count(); ++index) {
    names.push_back("omega_" + std::to_string(index));
  }
  if (has_battery()) {
    names.emplace_back("battery_soc");
  }
  return names;
}

std::vector<std::string> Quadrotor::input_names() const {
  std::vector<std::string> names;
  names.reserve(static_cast<std::size_t>(rotor_count()));
  for (int index = 0; index < rotor_count(); ++index) {
    names.push_back("omega_command_" + std::to_string(index));
  }
  return names;
}

double Quadrotor::open_circuit_voltage_v(double state_of_charge) const {
  if (!has_battery()) {
    throw std::invalid_argument("quadrotor: the model carries no battery");
  }
  const Battery& cell = *battery;
  const double fraction = std::clamp(state_of_charge, 0.0, 1.0);
  return cell.empty_voltage_v + fraction * (cell.full_voltage_v - cell.empty_voltage_v);
}

double Quadrotor::terminal_voltage_v(double state_of_charge, double current_a) const {
  if (!has_battery()) {
    throw std::invalid_argument("quadrotor: the model carries no battery");
  }
  return open_circuit_voltage_v(state_of_charge) - battery->internal_resistance_ohm * current_a;
}

bool Quadrotor::power_is_limited_at(double state_of_charge, double shaft_power) const {
  if (!has_battery() || !(battery->internal_resistance_ohm > 0.0)) {
    return false;
  }
  const double electrical_w =
      shaft_power / battery->motor_and_esc_efficiency + battery->auxiliary_load_w;
  return electrical_w > maximum_deliverable_power_w(state_of_charge);
}

double Quadrotor::maximum_deliverable_power_w(double state_of_charge) const {
  if (!has_battery()) {
    throw std::invalid_argument("quadrotor: the model carries no battery");
  }
  if (!(battery->internal_resistance_ohm > 0.0)) {
    return std::numeric_limits<double>::infinity();
  }
  const double open_circuit = open_circuit_voltage_v(state_of_charge);
  return open_circuit * open_circuit / (4.0 * battery->internal_resistance_ohm);
}

double Quadrotor::terminal_voltage_under_load_v(double state_of_charge,
                                                double shaft_power_w) const {
  if (!has_battery()) {
    throw std::invalid_argument("quadrotor: the model carries no battery");
  }
  const double open_circuit = open_circuit_voltage_v(state_of_charge);
  if (!(battery->internal_resistance_ohm > 0.0)) {
    return open_circuit;
  }
  const double electrical_w =
      shaft_power_w / battery->motor_and_esc_efficiency + battery->auxiliary_load_w;
  const double discriminant =
      open_circuit * open_circuit - 4.0 * battery->internal_resistance_ohm * electrical_w;
  // BEYOND THE MATCHED LOAD, the pack saturates rather than failing. The
  // discriminant vanishes at P = V_oc^2 / (4 R), where the terminal voltage is
  // V_oc / 2 and the delivered power is the most this pack can ever give. A
  // demand past that point does not produce "no answer": the motors simply do
  // not receive what they asked for and the rotors fall short, which is a real
  // flight condition and not an error.
  //
  // An earlier draft refused here, on the argument that a demand a battery
  // cannot meet is not a smaller demand. That argument is right about reporting
  // a voltage that does not exist and wrong about what the hardware does.
  // Souxmar's own plant — fcs/plant/quadrotor.py — reaches the same closed form
  // from the same physics and clamps at this point with a `limited` flag, and
  // diverging from it would make the cross-check this model exists to support
  // impossible.
  //
  // So it clamps, and `power_is_limited_at` exists so the clamp is VISIBLE. A
  // silent saturation is the thing worth refusing; a reported one is a
  // measurement.
  if (discriminant < 0.0) {
    return 0.5 * open_circuit;
  }
  // The larger root: the branch that tends to the open-circuit voltage as the
  // load tends to zero. The smaller one is the high-current solution a pack
  // does not sit at.
  return 0.5 * (open_circuit + std::sqrt(discriminant));
}

double Quadrotor::speed_ceiling_rad_s(int rotor_index, double state_of_charge) const {
  if (rotor_index < 0 || rotor_index >= rotor_count()) {
    throw std::invalid_argument("quadrotor: rotor index out of range");
  }
  const double mechanical_limit = rotors[static_cast<std::size_t>(rotor_index)].maximum_speed_rad_s;
  if (!has_battery()) {
    return mechanical_limit;
  }
  // Speed scales with terminal voltage, which without a current model is the
  // open-circuit voltage. The ceiling is whichever limit binds first.
  const double voltage_ratio = open_circuit_voltage_v(state_of_charge) / battery->full_voltage_v;
  return std::min(mechanical_limit, voltage_ratio * battery->speed_at_full_voltage_rad_s);
}

double Quadrotor::shaft_power_w(const Eigen::VectorXd& rotor_speed_rad_s) const {
  if (rotor_speed_rad_s.size() != rotor_count()) {
    throw std::invalid_argument("quadrotor: rotor speed vector has the wrong length");
  }
  double total = 0.0;
  for (int index = 0; index < rotor_count(); ++index) {
    const Rotor& rotor = rotors[static_cast<std::size_t>(index)];
    const double speed = rotor_speed_rad_s(index);
    total += rotor.torque_coefficient_n_m_s2 * speed * speed * std::abs(speed);
  }
  return total;
}

double Quadrotor::speed_ceiling_rad_s(int rotor_index,
                                      double state_of_charge,
                                      double shaft_power) const {
  if (rotor_index < 0 || rotor_index >= rotor_count()) {
    throw std::invalid_argument("quadrotor: rotor index out of range");
  }
  const double mechanical_limit = rotors[static_cast<std::size_t>(rotor_index)].maximum_speed_rad_s;
  if (!has_battery() || battery->sag == Battery::SagModel::OpenCircuit) {
    return speed_ceiling_rad_s(rotor_index, state_of_charge);
  }
  const double terminal = terminal_voltage_under_load_v(state_of_charge, shaft_power);
  const double voltage_ratio = terminal / battery->full_voltage_v;
  return std::min(mechanical_limit, voltage_ratio * battery->speed_at_full_voltage_rad_s);
}

sim::Wrench Quadrotor::rotor_wrench(const Eigen::VectorXd& rotor_speed_rad_s) const {
  if (rotor_speed_rad_s.size() != rotor_count()) {
    throw std::invalid_argument("quadrotor: rotor speed vector has the wrong length");
  }
  sim::Wrench total;
  for (int index = 0; index < rotor_count(); ++index) {
    const Rotor& rotor = rotors[static_cast<std::size_t>(index)];
    const double speed = rotor_speed_rad_s(index);
    const double thrust_n = rotor.thrust_coefficient_n_s2 * speed * speed;

    // Thrust acts along body -z, which is up in FRD.
    const Eigen::Vector3d force(0.0, 0.0, -thrust_n);
    total.force_body_n += force;
    total.moment_cg_body_n_m += rotor.position_cg_to_hub_body_m.cross(force);

    // Reaction to the motor torque, about body z. See the header for the sign.
    const double reaction_n_m = -static_cast<double>(rotor.spin_about_body_z)
                                * rotor.torque_coefficient_n_m_s2 * speed * speed;
    total.moment_cg_body_n_m.z() += reaction_n_m;
  }
  return total;
}

sim::Wrench Quadrotor::wrench(const core::State& state,
                              const Eigen::VectorXd& rotor_speed_rad_s) const {
  sim::Wrench total = rotor_wrench(rotor_speed_rad_s);
  total.force_body_n +=
      drag_force_body_n(state.velocity_body_m_s, drag_linear_n_s_m, drag_quadratic_n_s2_m2);
  total.moment_cg_body_n_m -= angular_drag_n_m_s.cwiseProduct(state.angular_rate_body_rad_s);
  return total;
}

Eigen::VectorXd Quadrotor::derivative(const Eigen::VectorXd& extended_state,
                                      const Eigen::VectorXd& command_rad_s,
                                      const Eigen::Vector3d& wind_ned_m_s) const {
  if (extended_state.size() != extended_state_size()) {
    throw std::invalid_argument("quadrotor: extended state has the wrong length");
  }
  if (command_rad_s.size() != rotor_count()) {
    throw std::invalid_argument("quadrotor: command vector has the wrong length");
  }

  const core::State state = core::State::from_vector(extended_state.head<core::kStateSize>());
  const Eigen::VectorXd speeds = extended_state.segment(rotor_state_offset(), rotor_count());

  const Eigen::Vector3d gravity_ned(0.0, 0.0, core::kStandardGravity);
  const core::StateVector rigid =
      sim::rigid_body_derivative(state, mass, wrench(state, speeds), gravity_ned);

  Eigen::VectorXd derivative = Eigen::VectorXd::Zero(extended_state_size());
  derivative.head<core::kStateSize>() = rigid;

  // The kernel rotated the AIR-RELATIVE velocity into NED, so what it produced
  // is the air-relative position rate. The ground position rate is that plus
  // the wind. This is the one place wind enters, and state.hpp says the caller
  // owns it.
  derivative(core::kPositionNorth) += wind_ned_m_s.x();
  derivative(core::kPositionEast) += wind_ned_m_s.y();
  derivative(core::kPositionDown) += wind_ned_m_s.z();

  // First-order rotor lag toward the command, clamped to what the rotor and the
  // battery allow. The clamp is on the TARGET, not on the state, so the lag
  // stays a smooth first-order system and the derivative has no kink at the
  // limit that a finite-difference Jacobian would average across.
  const double state_of_charge = has_battery() ? extended_state(battery_state_index()) : 1.0;
  const double drawn_shaft_w = has_battery() ? shaft_power_w(speeds) : 0.0;
  for (int index = 0; index < rotor_count(); ++index) {
    const Rotor& rotor = rotors[static_cast<std::size_t>(index)];
    // max() before clamp(): a depleted battery can drive the ceiling below the
    // rotor's own floor, and std::clamp with lo > hi is undefined behaviour.
    // The ceiling under the load the rotors are ACTUALLY drawing. The load is a
    // function of the current rotor states, not of the command, so there is no
    // algebraic loop here: what the pack can give at this instant is decided by
    // what it is already giving.
    const double ceiling = std::max(speed_ceiling_rad_s(index, state_of_charge, drawn_shaft_w),
                                    rotor.minimum_speed_rad_s);
    const double target = std::clamp(command_rad_s(index), rotor.minimum_speed_rad_s, ceiling);
    derivative(rotor_state_offset() + index) =
        (target - speeds(index)) / rotor.speed_time_constant_s;
  }

  if (has_battery()) {
    // Shaft power only: sum of rotor torque times rotor speed.
    //
    // This omits motor and ESC losses and any avionics load, so it UNDERSTATES
    // the discharge rate and OVERSTATES endurance. The omission is deliberate —
    // an efficiency this model cannot identify would be a fitted number wearing
    // a physical name — and it is why the header refuses to call this a battery
    // model in any electrochemical sense.
    // Under OpenCircuit this stays shaft power, which is what it always was and
    // what the comment above describes: it understates the draw and overstates
    // endurance, deliberately, because an efficiency this model cannot identify
    // would be a fitted number wearing a physical name.
    //
    // Under Resistive the study has DECLARED an efficiency and an auxiliary
    // load, so the electrical draw is known and used. It is still not a claim
    // about any aircraft: it is arithmetic on numbers the study supplied.
    double electrical_w = drawn_shaft_w;
    if (battery->sag == Battery::SagModel::Resistive) {
      electrical_w = drawn_shaft_w / battery->motor_and_esc_efficiency + battery->auxiliary_load_w;
    }
    derivative(battery_state_index()) = -electrical_w / battery->energy_j;
  }

  return derivative;
}

double Quadrotor::hover_speed_rad_s(double gravity_m_s2) const {
  if (rotors.empty()) {
    throw std::invalid_argument("quadrotor: hover requires at least one rotor");
  }
  const double coefficient = rotors.front().thrust_coefficient_n_s2;
  for (const Rotor& rotor : rotors) {
    if (rotor.thrust_coefficient_n_s2 != coefficient) {
      throw std::invalid_argument(
          "quadrotor: hover_speed_rad_s is defined only when every rotor shares one thrust "
          "coefficient; an averaged speed for mismatched rotors is not an equilibrium");
    }
  }
  if (!(coefficient > 0.0)) {
    throw std::invalid_argument("quadrotor: hover requires a positive thrust coefficient");
  }
  return std::sqrt(mass.mass_kg * gravity_m_s2
                   / (static_cast<double>(rotor_count()) * coefficient));
}

double Quadrotor::hover_speed_rad_s(int rotor_index, double gravity_m_s2) const {
  if (rotor_index < 0 || rotor_index >= rotor_count()) {
    throw std::invalid_argument("quadrotor: rotor index out of range");
  }
  const double coefficient = rotors[static_cast<std::size_t>(rotor_index)].thrust_coefficient_n_s2;
  if (!(coefficient > 0.0)) {
    throw std::invalid_argument("quadrotor: hover requires a positive thrust coefficient");
  }
  // Deliberately the same expression as the whole-vehicle form, in the same
  // order, so a homogeneous model produces bit-identical numbers here.
  return std::sqrt(mass.mass_kg * gravity_m_s2
                   / (static_cast<double>(rotor_count()) * coefficient));
}

void Quadrotor::project(Eigen::VectorXd& extended_state) const {
  core::State state = core::State::from_vector(extended_state.head<core::kStateSize>());
  state.renormalise_attitude();
  extended_state.head<core::kStateSize>() = state.to_vector();

  const double state_of_charge = has_battery() ? extended_state(battery_state_index()) : 1.0;
  for (int index = 0; index < rotor_count(); ++index) {
    const Rotor& rotor = rotors[static_cast<std::size_t>(index)];
    double& speed = extended_state(rotor_state_offset() + index);
    const double ceiling =
        std::max(speed_ceiling_rad_s(index, state_of_charge), rotor.minimum_speed_rad_s);
    speed = std::clamp(speed, rotor.minimum_speed_rad_s, ceiling);
  }
  if (has_battery()) {
    double& charge = extended_state(battery_state_index());
    charge = std::clamp(charge, 0.0, 1.0);
  }
}

Quadrotor parse_quadrotor(const std::string& bytes, const std::string& source_name) {
  const YAML::Node root = io::load_yaml(bytes, source_name);
  io::yaml_keys(
      root, source_name, {"description", "citation", "mass", "rotors", "drag", "battery"});

  Quadrotor model;
  model.description = optional_text(root, "description");
  model.citation = optional_text(root, "citation");

  const std::string mass_path = source_name + ".mass";
  const YAML::Node mass_node = root["mass"];
  if (!mass_node) {
    throw std::invalid_argument(source_name + ": missing required key 'mass'");
  }
  io::yaml_keys(mass_node,
                mass_path,
                {"mass_kg",
                 "inertia_xx_kg_m2",
                 "inertia_yy_kg_m2",
                 "inertia_zz_kg_m2",
                 "product_of_inertia_xy_kg_m2",
                 "product_of_inertia_xz_kg_m2",
                 "product_of_inertia_yz_kg_m2"});

  model.mass.mass_kg = scalar(mass_node, mass_path, "mass_kg");
  const double inertia_xx = scalar(mass_node, mass_path, "inertia_xx_kg_m2");
  const double inertia_yy = scalar(mass_node, mass_path, "inertia_yy_kg_m2");
  const double inertia_zz = scalar(mass_node, mass_path, "inertia_zz_kg_m2");
  // Quoted as the source prints them. The tensor's off-diagonal entries are the
  // NEGATIVE products of inertia, per sim::MassProperties, so the negation
  // happens here and exactly once.
  const double product_xy =
      optional_scalar(mass_node, mass_path, "product_of_inertia_xy_kg_m2", 0.0);
  const double product_xz =
      optional_scalar(mass_node, mass_path, "product_of_inertia_xz_kg_m2", 0.0);
  const double product_yz =
      optional_scalar(mass_node, mass_path, "product_of_inertia_yz_kg_m2", 0.0);

  Eigen::Matrix3d inertia;
  inertia << inertia_xx, -product_xy, -product_xz, -product_xy, inertia_yy, -product_yz,
      -product_xz, -product_yz, inertia_zz;
  model.mass.inertia_cg_body_kg_m2 = inertia;

  const YAML::Node rotor_node = root["rotors"];
  if (!rotor_node) {
    throw std::invalid_argument(source_name + ": missing required key 'rotors'");
  }
  if (!rotor_node.IsSequence() || rotor_node.size() == 0) {
    throw std::invalid_argument(source_name + ".rotors: expected a non-empty sequence");
  }
  for (std::size_t index = 0; index < rotor_node.size(); ++index) {
    const std::string path = source_name + ".rotors[" + std::to_string(index) + "]";
    const YAML::Node entry = rotor_node[index];
    io::yaml_keys(entry,
                  path,
                  {"position_cg_to_hub_body_m",
                   "spin_about_body_z",
                   "thrust_coefficient_n_s2",
                   "torque_coefficient_n_m_s2",
                   "speed_time_constant_s",
                   "minimum_speed_rad_s",
                   "maximum_speed_rad_s"});
    Rotor rotor;
    rotor.position_cg_to_hub_body_m = vector3(entry, path, "position_cg_to_hub_body_m");
    const double spin = scalar(entry, path, "spin_about_body_z");
    if (spin != 1.0 && spin != -1.0) {
      throw std::invalid_argument(path + ".spin_about_body_z: must be +1 or -1");
    }
    rotor.spin_about_body_z = static_cast<int>(spin);
    rotor.thrust_coefficient_n_s2 = scalar(entry, path, "thrust_coefficient_n_s2");
    rotor.torque_coefficient_n_m_s2 = scalar(entry, path, "torque_coefficient_n_m_s2");
    rotor.speed_time_constant_s = scalar(entry, path, "speed_time_constant_s");
    rotor.minimum_speed_rad_s = optional_scalar(entry, path, "minimum_speed_rad_s", 0.0);
    rotor.maximum_speed_rad_s = scalar(entry, path, "maximum_speed_rad_s");
    model.rotors.push_back(rotor);
  }

  const YAML::Node drag_node = root["drag"];
  if (!drag_node) {
    throw std::invalid_argument(source_name + ": missing required key 'drag'");
  }
  const std::string drag_path = source_name + ".drag";
  io::yaml_keys(drag_node, drag_path, {"linear_n_s_m", "quadratic_n_s2_m2", "angular_n_m_s"});
  model.drag_linear_n_s_m = vector3(drag_node, drag_path, "linear_n_s_m");
  model.drag_quadratic_n_s2_m2 = vector3(drag_node, drag_path, "quadratic_n_s2_m2");
  model.angular_drag_n_m_s = vector3(drag_node, drag_path, "angular_n_m_s");

  if (root["battery"]) {
    const std::string battery_path = source_name + ".battery";
    const YAML::Node battery_node = root["battery"];
    io::yaml_keys(battery_node,
                  battery_path,
                  {"energy_j",
                   "full_voltage_v",
                   "empty_voltage_v",
                   "internal_resistance_ohm",
                   "speed_at_full_voltage_rad_s",
                   "sag",
                   "motor_and_esc_efficiency",
                   "auxiliary_load_w"});
    Battery cell;
    cell.energy_j = scalar(battery_node, battery_path, "energy_j");
    cell.full_voltage_v = scalar(battery_node, battery_path, "full_voltage_v");
    cell.empty_voltage_v = scalar(battery_node, battery_path, "empty_voltage_v");
    cell.internal_resistance_ohm = scalar(battery_node, battery_path, "internal_resistance_ohm");
    cell.speed_at_full_voltage_rad_s =
        scalar(battery_node, battery_path, "speed_at_full_voltage_rad_s");
    // Absent means the model this file described before the resistive option
    // existed. A file written for the old behaviour keeps it, byte for byte;
    // opting in is a change the author makes on purpose.
    if (battery_node["sag"]) {
      const std::string name = battery_node["sag"].as<std::string>();
      if (name == "open_circuit") {
        cell.sag = Battery::SagModel::OpenCircuit;
      } else if (name == "resistive") {
        cell.sag = Battery::SagModel::Resistive;
      } else {
        throw std::invalid_argument(battery_path + ".sag must be `open_circuit` or `resistive`; '"
                                    + name + "' is neither");
      }
    }
    if (battery_node["motor_and_esc_efficiency"]) {
      cell.motor_and_esc_efficiency =
          scalar(battery_node, battery_path, "motor_and_esc_efficiency");
    }
    if (battery_node["auxiliary_load_w"]) {
      cell.auxiliary_load_w = scalar(battery_node, battery_path, "auxiliary_load_w");
    }
    model.battery = cell;
  }

  model.validate();
  return model;
}

Quadrotor load_quadrotor(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::invalid_argument("quadrotor: cannot open model file '" + path + "'");
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return parse_quadrotor(buffer.str(), path);
}

}  // namespace galata::model
