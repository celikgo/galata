// SPDX-License-Identifier: Apache-2.0
//
// The helicopter YAML contract. Unknown keys are errors, as everywhere else in
// this project, because a misspelled key that is silently ignored is a
// parameter the study thinks it set.
//
// Reference: the schema is documented key by key in docs/MODEL_FILES.md.
//
// UNITS ARE SI AND EACH KEY CARRIES ITS UNIT IN ITS NAME (ADR-0003). There is
// no degree, foot or knot anywhere in this contract: a source that publishes in
// those units is converted in the model's PROVENANCE.md, by hand, with the
// conversion recorded, and the YAML carries the SI result.

#include "galata/model/helicopter.hpp"

#include "../io/strict_yaml.hpp"
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace galata::model {
namespace {

double scalar(const YAML::Node& node, const std::string& path, const std::string& key) {
  if (!node[key]) {
    throw std::invalid_argument(path + ": missing required key '" + key + "'");
  }
  try {
    return node[key].as<double>();
  } catch (const YAML::Exception&) {
    throw std::invalid_argument(path + "." + key + ": expected a number");
  }
}

double optional_scalar(const YAML::Node& node,
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

int integer(const YAML::Node& node, const std::string& path, const std::string& key) {
  if (!node[key]) {
    throw std::invalid_argument(path + ": missing required key '" + key + "'");
  }
  try {
    return node[key].as<int>();
  } catch (const YAML::Exception&) {
    throw std::invalid_argument(path + "." + key + ": expected an integer");
  }
}

Eigen::Vector3d vector3(const YAML::Node& node, const std::string& path, const std::string& key) {
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

std::string optional_text(const YAML::Node& node, const std::string& key) {
  return node[key] ? node[key].as<std::string>() : std::string();
}

rotor::RotorGeometry parse_rotor(const YAML::Node& node,
                                 const std::string& path,
                                 const std::string& name) {
  io::yaml_keys(node,
                path,
                {"radius_m",
                 "chord_m",
                 "blade_count",
                 "lift_curve_slope",
                 "profile_drag_coefficient",
                 "induced_power_factor",
                 "blade_twist_rad",
                 "tip_loss_factor",
                 "hinge_offset_m",
                 "flap_stiffness_n_m_rad",
                 "blade_flap_inertia_kg_m2",
                 "polar_inertia_kg_m2",
                 "inflow_time_constant_s",
                 "position_cg_to_hub_body_m",
                 "spin_about_shaft",
                 "shaft_tilt_forward_rad",
                 "shaft_tilt_lateral_rad",
                 "thrust_towards_starboard",
                 "maximum_thrust_coefficient_solidity",
                 "maximum_advance_ratio"});

  rotor::RotorGeometry geometry;
  geometry.name = name;
  geometry.radius_m = scalar(node, path, "radius_m");
  geometry.chord_m = scalar(node, path, "chord_m");
  geometry.blade_count = integer(node, path, "blade_count");
  geometry.lift_curve_slope = scalar(node, path, "lift_curve_slope");
  geometry.profile_drag_coefficient = scalar(node, path, "profile_drag_coefficient");
  geometry.induced_power_factor = optional_scalar(node, path, "induced_power_factor", 1.0);
  geometry.blade_twist_rad = optional_scalar(node, path, "blade_twist_rad", 0.0);
  geometry.tip_loss_factor = optional_scalar(node, path, "tip_loss_factor", 0.97);
  geometry.hinge_offset_m = optional_scalar(node, path, "hinge_offset_m", 0.0);
  geometry.flap_stiffness_n_m_rad = optional_scalar(node, path, "flap_stiffness_n_m_rad", 0.0);
  geometry.blade_flap_inertia_kg_m2 = optional_scalar(node, path, "blade_flap_inertia_kg_m2", 0.0);
  geometry.polar_inertia_kg_m2 = scalar(node, path, "polar_inertia_kg_m2");
  geometry.inflow_time_constant_s = optional_scalar(node, path, "inflow_time_constant_s", 0.0);
  geometry.position_cg_to_hub_body_m = vector3(node, path, "position_cg_to_hub_body_m");
  geometry.spin_about_shaft = integer(node, path, "spin_about_shaft");
  geometry.maximum_thrust_coefficient_solidity =
      optional_scalar(node, path, "maximum_thrust_coefficient_solidity", 0.12);
  geometry.maximum_advance_ratio = optional_scalar(node, path, "maximum_advance_ratio", 0.35);

  // THE HUB ROTATION IS BUILT FROM DECLARED ANGLES, NEVER FROM A RAW MATRIX.
  //
  // A 3x3 in a YAML file is nine numbers that must be orthonormal, and the
  // usual way to get it wrong is to transpose it — which still validates and
  // still trims, with the aircraft yawing the wrong way. Angles cannot be
  // transposed.
  const bool is_tail = node["thrust_towards_starboard"].IsDefined();
  if (is_tail) {
    if (node["shaft_tilt_forward_rad"] || node["shaft_tilt_lateral_rad"]) {
      throw std::invalid_argument(
          path
          + ": a rotor declaring thrust_towards_starboard is a tail rotor and takes no shaft "
            "tilt. Declare one or the other, so the hub's orientation has a single source");
    }
    geometry.hub_to_body = rotor::tail_rotor_hub(node["thrust_towards_starboard"].as<bool>());
  } else {
    geometry.hub_to_body =
        rotor::main_rotor_hub(optional_scalar(node, path, "shaft_tilt_forward_rad", 0.0),
                              optional_scalar(node, path, "shaft_tilt_lateral_rad", 0.0));
  }
  return geometry;
}

ActuatorLimits parse_actuator(const YAML::Node& node, const std::string& path) {
  io::yaml_keys(node, path, {"minimum_rad", "maximum_rad", "rate_limit_rad_s", "time_constant_s"});
  ActuatorLimits limits;
  limits.minimum_rad = scalar(node, path, "minimum_rad");
  limits.maximum_rad = scalar(node, path, "maximum_rad");
  limits.rate_limit_rad_s = scalar(node, path, "rate_limit_rad_s");
  limits.time_constant_s = scalar(node, path, "time_constant_s");
  return limits;
}

std::optional<numerics::Table1D> parse_optional_table(const YAML::Node& parent,
                                                      const std::string& path,
                                                      const std::string& key,
                                                      const std::string& table_name) {
  const YAML::Node node = parent[key];
  if (!node) {
    return std::nullopt;
  }
  const std::string table_path = path + "." + key;
  io::yaml_keys(node, table_path, {"breakpoints", "values", "interpolation", "extrapolation"});

  const auto read_axis = [&](const std::string& which) {
    const YAML::Node entry = node[which];
    if (!entry || !entry.IsSequence()) {
      throw std::invalid_argument(table_path + "." + which + ": expected a sequence of numbers");
    }
    std::vector<double> out;
    out.reserve(entry.size());
    for (const auto& item : entry) {
      try {
        out.push_back(item.as<double>());
      } catch (const YAML::Exception&) {
        throw std::invalid_argument(table_path + "." + which + ": expected numbers");
      }
    }
    return out;
  };

  // BOTH RULES ARE REQUIRED, NOT DEFAULTED. There is no rule that is right for
  // every table, and a study that has not said which it wants has not decided.
  const auto interpolation_text =
      node["interpolation"] ? node["interpolation"].as<std::string>() : std::string();
  const auto extrapolation_text =
      node["extrapolation"] ? node["extrapolation"].as<std::string>() : std::string();
  if (interpolation_text.empty() || extrapolation_text.empty()) {
    throw std::invalid_argument(
        table_path
        + ": both 'interpolation' and 'extrapolation' are required. There is no default that is "
          "right for every table, and silent extrapolation of an aerodynamic table is how a "
          "component build-up produces a confidently wrong answer");
  }
  numerics::TableInterpolation interpolation{};
  if (interpolation_text == "linear") {
    interpolation = numerics::TableInterpolation::Linear;
  } else if (interpolation_text == "nearest_low") {
    interpolation = numerics::TableInterpolation::Nearest_Low;
  } else {
    throw std::invalid_argument(table_path + ".interpolation: expected 'linear' or 'nearest_low'");
  }
  numerics::TableExtrapolation extrapolation{};
  if (extrapolation_text == "hold") {
    extrapolation = numerics::TableExtrapolation::Hold;
  } else if (extrapolation_text == "refuse") {
    extrapolation = numerics::TableExtrapolation::Refuse;
  } else if (extrapolation_text == "linear") {
    extrapolation = numerics::TableExtrapolation::Linear;
  } else {
    throw std::invalid_argument(table_path
                                + ".extrapolation: expected 'hold', 'refuse' or 'linear'");
  }
  return numerics::Table1D(
      table_name, read_axis("breakpoints"), read_axis("values"), interpolation, extrapolation);
}

}  // namespace

HelicopterModel parse_helicopter(const std::string& bytes, const std::string& origin) {
  const YAML::Node root = io::load_yaml(bytes, origin);
  io::yaml_keys(root,
                origin,
                {"description",
                 "citation",
                 "mass",
                 "main_rotor",
                 "tail_rotor",
                 "airframe",
                 "drivetrain",
                 "actuators",
                 "tail_rotor_blockage_factor",
                 "pedal_to_tail_collective"});

  HelicopterModel model;
  model.description_text = optional_text(root, "description");
  model.citation = optional_text(root, "citation");
  if (model.citation.empty()) {
    throw std::invalid_argument(
        origin
        + ": missing required key 'citation'. Rule 7 of the charter: every physics model states "
          "where its numbers came from. A model without a citation is a model whose user cannot "
          "tell a measurement from a guess");
  }

  // ---- mass ------------------------------------------------------------
  const std::string mass_path = origin + ".mass";
  const YAML::Node mass_node = root["mass"];
  if (!mass_node) {
    throw std::invalid_argument(origin + ": missing required key 'mass'");
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
  const double ixx = scalar(mass_node, mass_path, "inertia_xx_kg_m2");
  const double iyy = scalar(mass_node, mass_path, "inertia_yy_kg_m2");
  const double izz = scalar(mass_node, mass_path, "inertia_zz_kg_m2");
  // Quoted as a source prints them. The tensor's off-diagonal entries are the
  // NEGATIVE products of inertia, per sim::MassProperties, so the negation
  // happens here and exactly once.
  const double pxy = optional_scalar(mass_node, mass_path, "product_of_inertia_xy_kg_m2", 0.0);
  const double pxz = optional_scalar(mass_node, mass_path, "product_of_inertia_xz_kg_m2", 0.0);
  const double pyz = optional_scalar(mass_node, mass_path, "product_of_inertia_yz_kg_m2", 0.0);
  Eigen::Matrix3d inertia;
  inertia << ixx, -pxy, -pxz, -pxy, iyy, -pyz, -pxz, -pyz, izz;
  model.mass.inertia_cg_body_kg_m2 = inertia;

  // ---- rotors ----------------------------------------------------------
  const YAML::Node main_node = root["main_rotor"];
  const YAML::Node tail_node = root["tail_rotor"];
  if (!main_node || !tail_node) {
    throw std::invalid_argument(origin + ": both 'main_rotor' and 'tail_rotor' are required");
  }
  model.main_rotor = parse_rotor(main_node, origin + ".main_rotor", "main");
  model.tail_rotor = parse_rotor(tail_node, origin + ".tail_rotor", "tail");
  if (!tail_node["thrust_towards_starboard"]) {
    throw std::invalid_argument(
        origin
        + ".tail_rotor: 'thrust_towards_starboard' is required. Which way the tail rotor pushes "
          "is the sign that decides whether the aircraft's anti-torque works or reinforces the "
          "torque it exists to oppose, so it is declared rather than inferred");
  }

  // ---- airframe --------------------------------------------------------
  const std::string airframe_path = origin + ".airframe";
  const YAML::Node airframe_node = root["airframe"];
  if (!airframe_node) {
    throw std::invalid_argument(origin + ": missing required key 'airframe'");
  }
  io::yaml_keys(airframe_node,
                airframe_path,
                {"flat_plate_area_m2",
                 "cg_to_fuselage_reference_body_m",
                 "fuselage_lift_vs_alpha",
                 "fuselage_pitching_moment_vs_alpha",
                 "horizontal_tail_area_m2",
                 "horizontal_tail_lift_slope",
                 "horizontal_tail_incidence_rad",
                 "cg_to_horizontal_tail_body_m",
                 "horizontal_tail_downwash_factor",
                 "vertical_tail_area_m2",
                 "vertical_tail_side_slope",
                 "vertical_tail_incidence_rad",
                 "cg_to_vertical_tail_body_m",
                 "surface_stall_angle_rad"});
  auto& airframe = model.airframe;
  airframe.flat_plate_area_m2 = scalar(airframe_node, airframe_path, "flat_plate_area_m2");
  airframe.cg_to_fuselage_reference_body_m =
      airframe_node["cg_to_fuselage_reference_body_m"]
          ? vector3(airframe_node, airframe_path, "cg_to_fuselage_reference_body_m")
          : Eigen::Vector3d::Zero();
  airframe.fuselage_lift_vs_alpha = parse_optional_table(
      airframe_node, airframe_path, "fuselage_lift_vs_alpha", "fuselage_lift_vs_alpha");
  airframe.fuselage_pitching_moment_vs_alpha =
      parse_optional_table(airframe_node,
                           airframe_path,
                           "fuselage_pitching_moment_vs_alpha",
                           "fuselage_pitching_moment_vs_alpha");
  airframe.horizontal_tail_area_m2 =
      optional_scalar(airframe_node, airframe_path, "horizontal_tail_area_m2", 0.0);
  airframe.horizontal_tail_lift_slope =
      optional_scalar(airframe_node, airframe_path, "horizontal_tail_lift_slope", 0.0);
  airframe.horizontal_tail_incidence_rad =
      optional_scalar(airframe_node, airframe_path, "horizontal_tail_incidence_rad", 0.0);
  airframe.cg_to_horizontal_tail_body_m =
      airframe_node["cg_to_horizontal_tail_body_m"]
          ? vector3(airframe_node, airframe_path, "cg_to_horizontal_tail_body_m")
          : Eigen::Vector3d::Zero();
  airframe.horizontal_tail_downwash_factor =
      optional_scalar(airframe_node, airframe_path, "horizontal_tail_downwash_factor", 0.0);
  airframe.vertical_tail_area_m2 =
      optional_scalar(airframe_node, airframe_path, "vertical_tail_area_m2", 0.0);
  airframe.vertical_tail_side_slope =
      optional_scalar(airframe_node, airframe_path, "vertical_tail_side_slope", 0.0);
  airframe.vertical_tail_incidence_rad =
      optional_scalar(airframe_node, airframe_path, "vertical_tail_incidence_rad", 0.0);
  airframe.cg_to_vertical_tail_body_m =
      airframe_node["cg_to_vertical_tail_body_m"]
          ? vector3(airframe_node, airframe_path, "cg_to_vertical_tail_body_m")
          : Eigen::Vector3d::Zero();
  airframe.surface_stall_angle_rad =
      optional_scalar(airframe_node, airframe_path, "surface_stall_angle_rad", 0.0);

  // ---- drivetrain ------------------------------------------------------
  const std::string drive_path = origin + ".drivetrain";
  const YAML::Node drive_node = root["drivetrain"];
  if (!drive_node) {
    throw std::invalid_argument(origin + ": missing required key 'drivetrain'");
  }
  io::yaml_keys(drive_node,
                drive_path,
                {"reference_rotor_speed_rad_s",
                 "tail_gear_ratio",
                 "governor_proportional_n_m_s",
                 "governor_time_constant_s",
                 "maximum_engine_torque_n_m",
                 "minimum_engine_torque_n_m",
                 "transmission_efficiency",
                 "accessory_torque_n_m",
                 "minimum_rotor_speed_rad_s",
                 "maximum_rotor_speed_rad_s"});
  auto& drive = model.drivetrain;
  drive.reference_rotor_speed_rad_s = scalar(drive_node, drive_path, "reference_rotor_speed_rad_s");
  drive.tail_gear_ratio = scalar(drive_node, drive_path, "tail_gear_ratio");
  drive.governor_proportional_n_m_s =
      optional_scalar(drive_node, drive_path, "governor_proportional_n_m_s", 0.0);
  drive.governor_time_constant_s =
      optional_scalar(drive_node, drive_path, "governor_time_constant_s", 0.0);
  drive.maximum_engine_torque_n_m = scalar(drive_node, drive_path, "maximum_engine_torque_n_m");
  drive.minimum_engine_torque_n_m =
      optional_scalar(drive_node, drive_path, "minimum_engine_torque_n_m", 0.0);
  drive.transmission_efficiency =
      optional_scalar(drive_node, drive_path, "transmission_efficiency", 1.0);
  drive.accessory_torque_n_m = optional_scalar(drive_node, drive_path, "accessory_torque_n_m", 0.0);
  drive.minimum_rotor_speed_rad_s =
      optional_scalar(drive_node, drive_path, "minimum_rotor_speed_rad_s", 0.0);
  drive.maximum_rotor_speed_rad_s =
      optional_scalar(drive_node, drive_path, "maximum_rotor_speed_rad_s", 0.0);

  // ---- actuators -------------------------------------------------------
  const std::string actuator_path = origin + ".actuators";
  const YAML::Node actuator_node = root["actuators"];
  if (!actuator_node) {
    throw std::invalid_argument(origin + ": missing required key 'actuators'");
  }
  io::yaml_keys(actuator_node,
                actuator_path,
                {"collective", "longitudinal_cyclic", "lateral_cyclic", "pedal"});
  const std::array<const char*, kHelicopterControlCount> actuator_keys{
      "collective", "longitudinal_cyclic", "lateral_cyclic", "pedal"};
  for (int i = 0; i < kHelicopterControlCount; ++i) {
    const YAML::Node entry = actuator_node[actuator_keys[static_cast<std::size_t>(i)]];
    if (!entry) {
      throw std::invalid_argument(actuator_path + ": missing required key '"
                                  + actuator_keys[static_cast<std::size_t>(i)] + "'");
    }
    model.actuators[static_cast<std::size_t>(i)] =
        parse_actuator(entry, actuator_path + "." + actuator_keys[static_cast<std::size_t>(i)]);
  }

  model.tail_rotor_blockage_factor =
      optional_scalar(root, origin, "tail_rotor_blockage_factor", 1.0);
  model.pedal_to_tail_collective = optional_scalar(root, origin, "pedal_to_tail_collective", 1.0);

  model.validate();
  return model;
}

HelicopterModel load_helicopter(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::invalid_argument("helicopter: cannot open '" + path + "'");
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return parse_helicopter(buffer.str(), path);
}

}  // namespace galata::model
