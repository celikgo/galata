// SPDX-License-Identifier: Apache-2.0
//
// Declared trim: Newton on a square residual system, with the rank and the
// conditioning reported rather than assumed.
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 3.
//   G. D. Padfield, "Helicopter Flight Dynamics", 2nd ed., Blackwell, 2007, §4.2.
//   G. H. Golub and C. F. Van Loan, "Matrix Computations", 4th ed., Johns
//   Hopkins, 2013, §5.5 — the SVD, its use for rank and conditioning, and the
//   null-space basis the unconstrained-unknown report is read from.
//
// Validity envelope: this finds a root of the residual system supplied. It says
// nothing about the equilibrium's stability, reachability or uniqueness, and
// nothing about whether the vehicle model it was solved against is valid at the
// point it found. The header's "WHAT THIS IS NOT" block states all three.
//
// DETERMINISM. A FIXED iteration count with no residual-based early exit
// (ADR-0004), a fixed finite-difference step, and an SVD whose result is a
// function of the matrix alone. The convergence test happens ONCE, at the end,
// as a gate on the answer rather than as a loop condition.

#include "galata/trim/problem.hpp"

#include "galata/core/quaternion.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

namespace galata::trim {
namespace {

std::string number(double value, int digits = 4) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(digits) << std::scientific << value;
  return out.str();
}

std::string plain(double value, int digits = 6) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(digits) << value;
  return out.str();
}

// Where an unknown lives: a state index, or a control index.
struct Slot {
  bool is_control = false;
  int index = 0;
};

Slot resolve(const model::VehicleModel& model, const std::string& name) {
  const auto states = model.state_names();
  for (std::size_t i = 0; i < states.size(); ++i) {
    if (states[i] == name) {
      return {false, static_cast<int>(i)};
    }
  }
  const auto controls = model.control_names();
  for (std::size_t i = 0; i < controls.size(); ++i) {
    if (controls[i] == name) {
      return {true, static_cast<int>(i)};
    }
  }
  std::ostringstream message;
  message << "trim: unknown '" << name
          << "' names neither a state nor a control of this model. It has states {";
  for (std::size_t i = 0; i < states.size(); ++i) {
    message << (i ? ", " : "") << states[i];
  }
  message << "} and controls {";
  for (std::size_t i = 0; i < controls.size(); ++i) {
    message << (i ? ", " : "") << controls[i];
  }
  message << "}";
  throw std::invalid_argument(message.str());
}

// Attitude unknowns are special: roll and pitch are not states, the QUATERNION
// is. A problem that names "roll_rad" is naming an Euler angle the solver has
// to compose into the quaternion, and that composition is here and only here.
constexpr const char* kRollUnknown = "roll_rad";
constexpr const char* kPitchUnknown = "pitch_rad";
constexpr const char* kYawUnknown = "yaw_rad";

bool is_attitude_unknown(const std::string& name) {
  return name == kRollUnknown || name == kPitchUnknown || name == kYawUnknown;
}

}  // namespace

void TrimProblem::validate(const model::VehicleModel& model) const {
  if (unknowns.empty()) {
    throw std::invalid_argument("trim: the problem declares no unknowns");
  }
  if (unknowns.size() != residuals.size()) {
    std::ostringstream message;
    message << "trim: the problem declares " << unknowns.size() << " unknowns and "
            << residuals.size()
            << " residuals. Newton solves a SQUARE system: more unknowns than residuals leaves a "
               "null space the iteration wanders in, and more residuals than unknowns has no root "
               "in general. Add or remove one, or declare a constraint";
    throw std::invalid_argument(message.str());
  }

  std::set<std::string> seen_unknowns;
  for (const auto& unknown : unknowns) {
    if (unknown.name.empty()) {
      throw std::invalid_argument("trim: an unknown has no name");
    }
    if (!seen_unknowns.insert(unknown.name).second) {
      throw std::invalid_argument("trim: unknown '" + unknown.name
                                  + "' is declared twice; the second declaration would silently "
                                    "overwrite the first");
    }
    if (!std::isfinite(unknown.initial_guess) || !std::isfinite(unknown.scale)
        || !(unknown.scale > 0.0)) {
      throw std::invalid_argument("trim: unknown '" + unknown.name
                                  + "' needs a finite guess and a positive scale");
    }
    if (!is_attitude_unknown(unknown.name)) {
      (void)resolve(model, unknown.name);
    }
  }

  std::set<std::string> seen_residuals;
  for (const auto& residual : residuals) {
    std::string key;
    switch (residual.kind) {
      case TrimResidualKind::BodyForceX:
        key = "force_x";
        break;
      case TrimResidualKind::BodyForceY:
        key = "force_y";
        break;
      case TrimResidualKind::BodyForceZ:
        key = "force_z";
        break;
      case TrimResidualKind::BodyMomentX:
        key = "moment_x";
        break;
      case TrimResidualKind::BodyMomentY:
        key = "moment_y";
        break;
      case TrimResidualKind::BodyMomentZ:
        key = "moment_z";
        break;
      case TrimResidualKind::AuxiliaryRate:
        key = "aux:" + residual.name;
        (void)resolve(model, residual.name);
        break;
      case TrimResidualKind::Custom:
        key = "custom:" + residual.name;
        if (!residual.evaluate) {
          throw std::invalid_argument(
              "trim: custom residual '" + residual.name
              + "' carries no evaluator. A residual that cannot be evaluated is a residual that "
                "is silently always zero");
        }
        break;
    }
    if (!seen_residuals.insert(key).second) {
      throw std::invalid_argument("trim: residual '" + key
                                  + "' is declared twice, making the system rank deficient by "
                                    "construction");
    }
    if (!std::isfinite(residual.scale) || !(residual.scale > 0.0)) {
      throw std::invalid_argument("trim: residual '" + key + "' needs a positive finite scale");
    }
  }
}

namespace {

// Apply the unknown vector to a copy of the condition, producing the extended
// state and the controls the residual is evaluated at.
void apply_unknowns(const model::VehicleModel& model,
                    const TrimProblem& problem,
                    const Eigen::VectorXd& values,
                    const TrimCondition& condition,
                    Eigen::VectorXd& extended_state,
                    Eigen::VectorXd& controls) {
  extended_state = condition.extended_state;
  controls = condition.controls;

  // Euler unknowns are collected first and composed into the quaternion once,
  // because composing them one at a time would apply each rotation to the
  // result of the last and give a different attitude.
  core::EulerAngles euler = core::euler_from_quaternion(
      core::from_wxyz(Eigen::Vector4d(extended_state.segment<4>(core::kQuaternionW))));
  bool attitude_touched = false;

  for (std::size_t i = 0; i < problem.unknowns.size(); ++i) {
    const auto& unknown = problem.unknowns[i];
    const double value = values(static_cast<Eigen::Index>(i));
    if (unknown.name == kRollUnknown) {
      euler.roll_rad = value;
      attitude_touched = true;
    } else if (unknown.name == kPitchUnknown) {
      euler.pitch_rad = value;
      attitude_touched = true;
    } else if (unknown.name == kYawUnknown) {
      euler.yaw_rad = value;
      attitude_touched = true;
    } else {
      const Slot slot = resolve(model, unknown.name);
      if (slot.is_control) {
        controls(slot.index) = value;
      } else {
        extended_state(slot.index) = value;
      }
    }
  }
  if (attitude_touched) {
    extended_state.segment<4>(core::kQuaternionW) =
        core::to_wxyz(core::quaternion_from_euler(euler));
  }
}

Eigen::VectorXd evaluate_residual(const model::VehicleModel& model,
                                  const TrimProblem& problem,
                                  const Eigen::VectorXd& values,
                                  const TrimCondition& condition) {
  Eigen::VectorXd extended_state;
  Eigen::VectorXd controls;
  apply_unknowns(model, problem, values, condition, extended_state, controls);

  const Eigen::VectorXd rate = model.derivative(extended_state, controls, condition.environment);
  Eigen::VectorXd residual(static_cast<Eigen::Index>(problem.residuals.size()));

  for (std::size_t i = 0; i < problem.residuals.size(); ++i) {
    const auto& declaration = problem.residuals[i];
    double value = 0.0;
    switch (declaration.kind) {
      case TrimResidualKind::BodyForceX:
        value = rate(core::kVelocityU);
        break;
      case TrimResidualKind::BodyForceY:
        value = rate(core::kVelocityV);
        break;
      case TrimResidualKind::BodyForceZ:
        value = rate(core::kVelocityW);
        break;
      case TrimResidualKind::BodyMomentX:
        value = rate(core::kRateP);
        break;
      case TrimResidualKind::BodyMomentY:
        value = rate(core::kRateQ);
        break;
      case TrimResidualKind::BodyMomentZ:
        value = rate(core::kRateR);
        break;
      case TrimResidualKind::AuxiliaryRate: {
        const Slot slot = resolve(model, declaration.name);
        value = rate(slot.index);
        break;
      }
      case TrimResidualKind::Custom:
        value = declaration.evaluate(extended_state, controls);
        break;
    }
    residual(static_cast<Eigen::Index>(i)) = value / declaration.scale;
  }
  return residual;
}

}  // namespace

TrimResult solve_trim(const model::VehicleModel& model,
                      const TrimProblem& problem,
                      const TrimCondition& condition,
                      const TrimOptions& options) {
  problem.validate(model);
  model.validate_vocabulary();
  condition.environment.validate();

  if (condition.extended_state.size() != model.extended_state_size()) {
    throw std::invalid_argument(
        "trim: the condition's state has " + std::to_string(condition.extended_state.size())
        + " entries, the model has " + std::to_string(model.extended_state_size()));
  }
  if (condition.controls.size() != model.control_count()) {
    throw std::invalid_argument(
        "trim: the condition's controls have " + std::to_string(condition.controls.size())
        + " entries, the model has " + std::to_string(model.control_count()));
  }
  if (options.iterations < 1) {
    throw std::invalid_argument("trim: iterations must be at least 1");
  }
  if (!(options.max_scaled_step > 0.0)) {
    throw std::invalid_argument(
        "trim: max_scaled_step must be positive; zero would freeze the iteration");
  }
  if (!(options.residual_tolerance > 0.0)) {
    throw std::invalid_argument(
        "trim: residual_tolerance must be positive; a budget of zero or less is a gate nothing "
        "can pass");
  }

  const auto n = static_cast<Eigen::Index>(problem.unknowns.size());
  Eigen::VectorXd values(n);
  Eigen::VectorXd scales(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    values(i) = problem.unknowns[static_cast<std::size_t>(i)].initial_guess;
    scales(i) = problem.unknowns[static_cast<std::size_t>(i)].scale;
  }

  TrimResult result;
  result.unknown_count = static_cast<int>(n);
  result.residual_tolerance = options.residual_tolerance;
  for (const auto& unknown : problem.unknowns) {
    result.unknown_names.push_back(unknown.name);
  }

  Eigen::MatrixXd jacobian(n, n);
  Eigen::VectorXd residual = evaluate_residual(model, problem, values, condition);

  // FIXED ITERATION COUNT. No residual test in the loop: ADR-0004.
  for (int iteration = 0; iteration < options.iterations; ++iteration) {
    // THE JACOBIAN IS TAKEN IN SCALED UNKNOWNS, not merely stepped by the scale.
    //
    // A first version used each unknown's scale for the finite-difference step
    // only, which left the MATRIX conditioned by its units: an engine torque of
    // order 1e4 N m against attitude angles of order 1e-1 rad gave a condition
    // number of 7.4e6, none of which was physics. Differentiating with respect
    // to y_j = x_j / scale_j instead multiplies column j by scale_j and removes
    // the unit disparity, which is what `TrimUnknown::scale` says it does.
    for (Eigen::Index j = 0; j < n; ++j) {
      const double step = options.jacobian_relative_step * scales(j);
      Eigen::VectorXd probe = values;
      probe(j) = values(j) + step;
      const double upper = probe(j);
      const Eigen::VectorXd forward = evaluate_residual(model, problem, probe, condition);
      probe(j) = values(j) - step;
      const double lower = probe(j);
      const Eigen::VectorXd backward = evaluate_residual(model, problem, probe, condition);
      jacobian.col(j) = (forward - backward) * (scales(j) / (upper - lower));
    }

    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian,
                                                Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd singular = svd.singularValues();
    const double largest = singular.size() > 0 ? singular(0) : 0.0;
    const double smallest = singular.size() > 0 ? singular(singular.size() - 1) : 0.0;
    result.jacobian_condition_number =
        smallest > 0.0 ? largest / smallest : std::numeric_limits<double>::infinity();
    result.jacobian_rank = 0;
    for (Eigen::Index i = 0; i < singular.size(); ++i) {
      if (singular(i) > options.rank_relative_floor * largest) {
        ++result.jacobian_rank;
      }
    }

    // A rank-deficient Jacobian has no unique Newton step. Rather than take a
    // pseudo-inverse step into a null space and report the point it reached,
    // stop and say which unknowns the residuals do not constrain.
    if (result.jacobian_rank < static_cast<int>(n)) {
      result.unconstrained_unknowns.clear();
      const Eigen::MatrixXd& v = svd.matrixV();
      for (Eigen::Index i = result.jacobian_rank; i < n; ++i) {
        // The null-space direction's largest component names the unknown most
        // implicated in the deficiency.
        Eigen::Index worst = 0;
        v.col(i).cwiseAbs().maxCoeff(&worst);
        const std::string& name = problem.unknowns[static_cast<std::size_t>(worst)].name;
        if (std::find(
                result.unconstrained_unknowns.begin(), result.unconstrained_unknowns.end(), name)
            == result.unconstrained_unknowns.end()) {
          result.unconstrained_unknowns.push_back(name);
        }
      }
      break;
    }

    // The step comes back in SCALED coordinates, so it is scaled back before it
    // is applied. Getting this wrong would move every unknown by its own scale
    // factor and diverge immediately, which is the reassuring failure mode.
    Eigen::VectorXd scaled_step = svd.solve(residual);
    if (!scaled_step.allFinite()) {
      break;
    }
    // The trust region, applied in scaled coordinates. See TrimOptions.
    if (options.max_scaled_step > 0.0) {
      const double largest_step = scaled_step.cwiseAbs().maxCoeff();
      if (largest_step > options.max_scaled_step) {
        scaled_step *= options.max_scaled_step / largest_step;
      }
    }
    values -= options.step_fraction * scaled_step.cwiseProduct(scales);
    residual = evaluate_residual(model, problem, values, condition);
    result.iterations = iteration + 1;
  }

  result.residual_norm = residual.norm();
  result.unknown_values = values;
  apply_unknowns(model, problem, values, condition, result.extended_state, result.controls);
  result.envelope = model.envelope(model::VehicleModel::rigid_body_part(result.extended_state),
                                   model.auxiliary_part(result.extended_state),
                                   result.controls,
                                   condition.environment);

  for (std::size_t i = 0; i < problem.unknowns.size(); ++i) {
    const auto& unknown = problem.unknowns[i];
    if (!unknown.bounded()) {
      continue;
    }
    const double value = values(static_cast<Eigen::Index>(i));
    if (value < unknown.minimum || value > unknown.maximum) {
      result.out_of_bounds_unknowns.push_back(unknown.name);
    }
  }

  result.converged = result.residual_norm <= options.residual_tolerance
                     && result.jacobian_rank == static_cast<int>(n)
                     && result.out_of_bounds_unknowns.empty() && result.extended_state.allFinite();

  if (!result.converged) {
    std::ostringstream message;
    message << "trim: no equilibrium found. Residual norm " << number(result.residual_norm)
            << " against the budget " << number(result.residual_tolerance) << " after "
            << result.iterations << " of " << options.iterations << " iterations; Jacobian rank "
            << result.jacobian_rank << " of " << n << ", condition number "
            << number(result.jacobian_condition_number) << ".";
    if (!result.unconstrained_unknowns.empty()) {
      message << " The residuals do not constrain: ";
      for (std::size_t i = 0; i < result.unconstrained_unknowns.size(); ++i) {
        message << (i ? ", " : "") << result.unconstrained_unknowns[i];
      }
      message << ". Either the problem is genuinely singular at this condition, or an unknown was "
                 "declared that nothing in the residual set depends on, or — most often — one of "
                 "them has SATURATED: an unknown that has reached a physical limit in the model "
                 "has a flat Jacobian column, because perturbing it changes nothing. Check "
                 "whether the named unknown is at a stop, a rating or a travel limit at this "
                 "condition; if it is, the condition is beyond the aircraft rather than beyond "
                 "the solver.";
    }
    if (!result.out_of_bounds_unknowns.empty()) {
      message << " Outside declared bounds: ";
      for (std::size_t i = 0; i < result.out_of_bounds_unknowns.size(); ++i) {
        const std::string& name = result.out_of_bounds_unknowns[i];
        const auto it = std::find_if(problem.unknowns.begin(),
                                     problem.unknowns.end(),
                                     [&](const TrimUnknown& u) { return u.name == name; });
        const Eigen::Index index = it - problem.unknowns.begin();
        message << (i ? ", " : "") << name << " = " << plain(values(index)) << " against ["
                << plain(it->minimum) << ", " << plain(it->maximum) << "]";
      }
      message << ". A trim outside the vehicle's own travel is refused rather than returned with a "
                 "note: it would be linearised, and a linearisation about a non-equilibrium "
                 "carries a constant term the A matrix cannot represent.";
    }
    throw std::runtime_error(message.str());
  }
  return result;
}

TrimProblem helicopter_trim_problem(const model::VehicleModel& model) {
  const auto controls = model.control_names();
  if (controls.size() != 4) {
    throw std::invalid_argument(
        "helicopter_trim_problem: this problem is declared for a four-control helicopter "
        "(collective, longitudinal cyclic, lateral cyclic, pedal); the model has "
        + std::to_string(controls.size()) + " controls");
  }
  const auto states = model.state_names();
  const auto has = [&states](const std::string& name) {
    return std::find(states.begin(), states.end(), name) != states.end();
  };
  for (const char* required :
       {"collective_rad", "longitudinal_cyclic_rad", "lateral_cyclic_rad", "pedal_rad"}) {
    if (!has(required)) {
      throw std::invalid_argument(
          std::string("helicopter_trim_problem: the model has no state '") + required
          + "'. This problem trims the ACTUATOR POSITIONS, not the commands, because an "
            "equilibrium is a statement about where the swashplate is and not about what was "
            "asked for");
    }
  }

  TrimProblem problem;
  // The inflow unknowns declare equal bounds, which means unbounded: an inflow
  // ratio has no mechanical travel to be outside of.
  // Attitude scales in radians; control scales in radians too, but the cyclics
  // move over a much smaller range than collective and pedal, so each gets its
  // own.
  problem.unknowns = {
      {kRollUnknown, 0.0, -0.5, 0.5, 0.1},
      {kPitchUnknown, 0.0, -0.5, 0.5, 0.1},
      {"collective_rad", 0.15, 0.0, 0.35, 0.05},
      {"longitudinal_cyclic_rad", 0.0, -0.18, 0.18, 0.02},
      {"lateral_cyclic_rad", 0.0, -0.14, 0.14, 0.02},
      {"pedal_rad", 0.05, -0.35, 0.35, 0.05},
  };

  // Forces and moments are already accelerations coming out of the kernel, so
  // the scales are 1 and the comparison is like for like.
  problem.residuals = {
      {TrimResidualKind::BodyForceX, "force_x", nullptr, 1.0},
      {TrimResidualKind::BodyForceY, "force_y", nullptr, 1.0},
      {TrimResidualKind::BodyForceZ, "force_z", nullptr, 1.0},
      {TrimResidualKind::BodyMomentX, "moment_x", nullptr, 1.0},
      {TrimResidualKind::BodyMomentY, "moment_y", nullptr, 1.0},
      {TrimResidualKind::BodyMomentZ, "moment_z", nullptr, 1.0},
  };

  // THE INFLOW STATES ARE TRIM UNKNOWNS, and leaving them out is a mistake this
  // project made once and caught with a measurement rather than by reasoning.
  //
  // A rotor with a declared dynamic-inflow lag carries its inflow as a STATE,
  // and that state has its own equilibrium: the value at which the momentum
  // balance is satisfied. Solving the six force-and-moment equations alone
  // leaves the inflow wherever the initial guess put it, so the point is an
  // equilibrium of the airframe and NOT of the rotor. The residual is invisible
  // in the forces — the trim converges to 1e-17 — and appears only when
  // something asks for the inflow's own rate. `linearize_vehicle` asks, which
  // is how this was found: it refused the point with `tail_inflow_ratio` named
  // and a residual of 0.66.
  //
  // So each inflow state that exists becomes an unknown, paired with its own
  // rate as a residual, and the system stays square.
  for (const char* inflow : {"main_inflow_ratio", "tail_inflow_ratio"}) {
    if (!has(inflow)) {
      continue;  // a rotor with no declared lag carries no inflow state
    }
    problem.unknowns.push_back({inflow, 0.05, 0.0, 0.0, 0.01});
    problem.residuals.push_back({TrimResidualKind::AuxiliaryRate, inflow, nullptr, 1.0});
  }

  // AND SO DOES THE ENGINE TORQUE, for exactly the same reason. It is a state
  // with a governor lag, so it has an equilibrium — the torque at which the
  // rotor neither accelerates nor decelerates — and leaving it out makes the
  // point an equilibrium of the airframe and not of the drivetrain. The
  // rotor-speed rate is added alongside it: with the torque free, holding the
  // rotor speed stationary is what pins it down.
  if (has("engine_torque_n_m") && has("main_rotor_speed_rad_s")) {
    // Scaled by its own magnitude: a torque of order 1e4 N m mixed with angles of
    // order 1e-1 rad gives a Jacobian conditioned by its UNITS rather than by the
    // physics, and the condition number rose to 7e6 before this scale was set.
    // Bounded, so a condition needing more torque than the drivetrain has is
    // refused naming the torque rather than returning a saturated answer.
    problem.unknowns.push_back({"engine_torque_n_m", 10000.0, 0.0, 1.0e7, 10000.0});
    problem.residuals.push_back(
        {TrimResidualKind::AuxiliaryRate, "main_rotor_speed_rad_s", nullptr, 1.0});
  }
  return problem;
}

double rotor_speed_residual(const model::VehicleModel& model,
                            const TrimResult& trim,
                            const model::Environment& environment,
                            const std::string& rotor_speed_state_name) {
  const Slot slot = resolve(model, rotor_speed_state_name);
  if (slot.is_control) {
    throw std::invalid_argument("rotor_speed_residual: '" + rotor_speed_state_name
                                + "' is a control, not a state");
  }
  const Eigen::VectorXd rate = model.derivative(trim.extended_state, trim.controls, environment);
  return rate(slot.index);
}

}  // namespace galata::trim
