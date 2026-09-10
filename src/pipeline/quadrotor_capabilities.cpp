// SPDX-License-Identifier: Apache-2.0
//
// Study adapters for the multirotor packages of RFC-0002: hover trim, the
// model-generic linearisation, and the named matrix export that closes the
// round trip with `model.linear.statespace`.
//
// The numerics are in galata::trim and galata::linearize. What lives here is
// the schema — which keys a stage may set, what they mean, and which unit each
// carries in its own name (ADR-0003) — and the evidence, because a matrix that
// reaches a file without the operating point it was taken about is a number
// with no routine behind it (charter rule 9).

#include "galata/core/frames.hpp"
#include "galata/linearize/extended.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/numerics/integrator.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/files.hpp"
#include "galata/sim/schedule.hpp"
#include "galata/trim/hover.hpp"
#include "galata/units.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace galata::pipeline {
namespace {

// --- schema helpers --------------------------------------------------------

// A three-component NED vector, written as a list of exactly three numbers.
// Absent means zero; the wrong length is an error rather than a pad or a
// truncation, because a two-entry wind is a caller who thinks this vehicle
// flies in a plane.
Eigen::Vector3d vector3_at(const StageContext& context, const std::string& key) {
  const ValuePtr value = context.input->get(key);
  if (!value || value->kind() == Value::Kind::Null) {
    return Eigen::Vector3d::Zero();
  }
  const std::vector<ValuePtr>& items = value->as_list();
  if (items.size() != 3) {
    std::ostringstream message;
    message << "'" << key << "' must be a list of exactly three numbers, north east down; "
            << items.size() << " were given";
    throw std::invalid_argument(message.str());
  }
  Eigen::Vector3d out;
  for (std::size_t i = 0; i < 3; ++i) {
    out(static_cast<Eigen::Index>(i)) = items[i]->as_number();
  }
  return out;
}

// A vector of a DECLARED length. The length is the model's, so a caller who
// gives four rotor commands to a six-rotor model is told so here rather than
// having the shorter list padded into a plausible trajectory.
Eigen::VectorXd vector_at(const StageContext& context, const std::string& key, int expected) {
  const ValuePtr value = context.input->get(key);
  if (!value || value->kind() == Value::Kind::Null) {
    std::ostringstream message;
    message << "'" << key << "' is required and must be a list of " << expected << " numbers";
    throw std::invalid_argument(message.str());
  }
  const std::vector<ValuePtr>& items = value->as_list();
  if (static_cast<int>(items.size()) != expected) {
    std::ostringstream message;
    message << "'" << key << "' must be a list of exactly " << expected << " numbers; "
            << items.size() << " were given";
    throw std::invalid_argument(message.str());
  }
  Eigen::VectorXd out(expected);
  for (std::size_t i = 0; i < items.size(); ++i) {
    out(static_cast<Eigen::Index>(i)) = items[i]->as_number();
  }
  return out;
}

int positive_integer_at(const StageContext& context, const std::string& key, int fallback) {
  const double number = context.input->number_at(key, static_cast<double>(fallback));
  if (!std::isfinite(number) || number < 1.0 || number > std::numeric_limits<int>::max()
      || std::floor(number) != number) {
    throw std::invalid_argument("'" + key + "' must be a positive whole number");
  }
  return static_cast<int>(number);
}

std::ostringstream evidence_stream() {
  std::ostringstream out;
  // ADR-0004: the classic locale, and the shortest precision that survives a
  // text round trip. An evidence file a comma-decimal machine writes must be
  // one every other machine can read.
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  return out;
}

void write_vector(std::ostream& out, const std::string& key, const Eigen::VectorXd& values) {
  out << key << ": [";
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << values(i);
  }
  out << "]\n";
}

// --- declared input histories ----------------------------------------------

// A schedule is a hold policy, an extrapolation policy and a list of samples.
// Both policies are REQUIRED. There is no default that is right for every
// channel — a rotor command that steps is held, a wind that builds is
// interpolated — and inferring one from the data picks a trajectory the study
// does not state.
//
//   command_schedule:
//     hold: zero_order          # or linear
//     extrapolation: refuse     # or hold
//     samples:
//       - {time_s: 0.0, values: [626.3, 626.3, 626.3, 626.3]}
//       - {time_s: 1.0, values: [645.1, 607.5, 607.5, 645.1]}
sim::InputSchedule schedule_at(const StageContext& context,
                               const std::string& key,
                               int expected_width) {
  const ValuePtr value = context.input->get(key);
  if (!value || value->kind() == Value::Kind::Null) {
    return {};
  }
  const std::string hold_name = value->string_at("hold");
  sim::HoldPolicy hold{};
  if (hold_name == "zero_order") {
    hold = sim::HoldPolicy::ZeroOrder;
  } else if (hold_name == "linear") {
    hold = sim::HoldPolicy::Linear;
  } else {
    throw std::invalid_argument("'" + key + ".hold' must be `zero_order` or `linear`; '" + hold_name
                                + "' is neither, and there is no default");
  }
  const std::string outside_name = value->string_at("extrapolation");
  sim::Extrapolation outside{};
  if (outside_name == "hold") {
    outside = sim::Extrapolation::Hold;
  } else if (outside_name == "refuse") {
    outside = sim::Extrapolation::Refuse;
  } else {
    throw std::invalid_argument("'" + key + ".extrapolation' must be `hold` or `refuse`; '"
                                + outside_name + "' is neither, and there is no default");
  }

  const ValuePtr samples = value->get("samples");
  if (!samples) {
    throw std::invalid_argument("'" + key + "' needs a `samples` list");
  }
  std::vector<double> times;
  std::vector<Eigen::VectorXd> rows;
  for (const ValuePtr& sample : samples->as_list()) {
    times.push_back(sample->number_at("time_s"));
    const ValuePtr values_node = sample->get("values");
    if (!values_node) {
      throw std::invalid_argument("'" + key + "' sample needs a `values` list");
    }
    const std::vector<ValuePtr>& items = values_node->as_list();
    if (static_cast<int>(items.size()) != expected_width) {
      std::ostringstream message;
      message << "'" << key << "' sample at t = " << times.back() << " s carries " << items.size()
              << " channel(s); this model needs " << expected_width;
      throw std::invalid_argument(message.str());
    }
    Eigen::VectorXd row(expected_width);
    for (std::size_t i = 0; i < items.size(); ++i) {
      row(static_cast<Eigen::Index>(i)) = items[i]->as_number();
    }
    rows.push_back(std::move(row));
  }
  return sim::InputSchedule(std::move(times), std::move(rows), hold, outside);
}

// A discontinuity has to land on a step boundary. Inside a step it is not
// representable: RK4's four stage evaluations straddle it, the method stops
// being fourth order there, and — for wind — the re-basing that keeps ground
// velocity continuous has no instant at which to happen. Refused rather than
// rounded to the nearest step, because rounding moves the event and says
// nothing about having done so.
int step_index_for_event(double time_s, double step_s, int steps, const std::string& key) {
  const double exact = time_s / step_s;
  const double nearest = std::round(exact);
  if (std::fabs(exact - nearest) > 1e-9 * std::fmax(1.0, std::fabs(exact))) {
    std::ostringstream message;
    message << "'" << key << "' changes at t = " << time_s << " s, which is not a whole number of "
            << step_s
            << " s steps from the start. A discontinuity inside a step is not representable: the "
               "integrator's stages would straddle it and, for wind, the re-basing that keeps "
               "ground velocity continuous has no instant to happen at. Align the schedule to the "
               "step, or choose a step that divides it";
    throw std::invalid_argument(message.str());
  }
  const int index = static_cast<int>(nearest);
  if (index < 0 || index > steps) {
    std::ostringstream message;
    message << "'" << key << "' changes at t = " << time_s
            << " s, outside the interval this run integrates";
    throw std::invalid_argument(message.str());
  }
  return index;
}

// --- sim.plant -------------------------------------------------------------
//
// The nonlinear plant, integrated, through a public capability.
//
// WHY THIS EXISTS. `sim.nonlinear` takes a `trim.level` point and actuators
// named elevator, aileron, rudder and thrust: it is the fixed-wing path and a
// multirotor cannot enter it. The multirotor plant was reachable only from C++,
// which meant the repository could integrate a quadrotor in its own tests and a
// user could not integrate one at all. This closes that, and the fixed-wing
// path is untouched.
//
// NO SECOND DYNAMICS. The derivative is `Quadrotor::derivative` and the
// integrator is `numerics::integrate_fixed_step` — the same pair the
// cross-implementation validation case has always used. Nothing here integrates
// anything; it assembles the call a caller could not otherwise make.
//
// THE NAME IS VEHICLE-NEUTRAL on purpose. What varies between vehicles is the
// MODEL, which is why the model-class name lives at `model.quadrotor`. A second
// simulation capability per airframe would be a taxonomy, not a contract.

Artifact simulate_plant_capability(const StageContext& context) {
  // Two ways in, and exactly one must be taken. A trim carries its own model and
  // its own equilibrium state; a bare model needs both declared. Accepting both
  // at once would leave it ambiguous which state was integrated.
  const bool from_trim = context.input->get("trim") != nullptr;
  const bool from_model = context.input->get("quadrotor") != nullptr;
  if (from_trim == from_model) {
    throw std::invalid_argument(
        "sim.plant: give exactly one of `trim` (an operating point, which carries its own "
        "model and state) or `quadrotor` (a model, whose initial state you then declare). "
        "Both together leaves it unsaid which state was integrated");
  }

  const model::Quadrotor* model = nullptr;
  Eigen::VectorXd initial;
  Eigen::VectorXd command;
  Eigen::Vector3d wind = Eigen::Vector3d::Zero();

  if (from_trim) {
    const auto& trimmed = context.upstream_at("trim").payload_as<HoverTrimArtifact>("hover_trim");
    model = &trimmed.model;
    initial = trimmed.point.extended_state;
    command = trimmed.point.command_rad_s;
    wind = trimmed.point.wind_ned_m_s;
  } else {
    model = &context.upstream_at("quadrotor").payload_as<model::Quadrotor>("quadrotor");
    initial = vector_at(context, "initial_extended_state", model->extended_state_size());
    command = vector_at(context, "command_rad_s", model->rotor_count());
  }

  // A declared value overrides what the trim carried, so a caller can hold the
  // trim's equilibrium and blow a different wind across it. Silence keeps the
  // trim's own.
  if (context.input->get("wind_ned_m_s") != nullptr) {
    wind = vector3_at(context, "wind_ned_m_s");
  }
  if (from_trim && context.input->get("command_rad_s") != nullptr) {
    command = vector_at(context, "command_rad_s", model->rotor_count());
  }
  if (from_trim && context.input->get("initial_extended_state") != nullptr) {
    initial = vector_at(context, "initial_extended_state", model->extended_state_size());
  }

  const double step_s = context.input->number_at("step_s");
  if (!(step_s > 0.0) || !std::isfinite(step_s)) {
    throw std::invalid_argument("sim.plant: step_s must be a positive finite number of seconds");
  }
  const int steps = positive_integer_at(context, "steps", 0);
  const int stride = positive_integer_at(context, "sample_stride", 1);

  // Declared histories. Absent means the constant already resolved above, which
  // is what every study written before schedules existed says.
  const sim::InputSchedule command_history =
      schedule_at(context, "command_schedule", model->rotor_count());
  const sim::InputSchedule wind_history = schedule_at(context, "wind_schedule", 3);

  const bool freeze_battery = context.input->bool_at("freeze_battery", false);
  if (freeze_battery && !model->has_battery()) {
    throw std::invalid_argument(
        "sim.plant: freeze_battery was asked for on a model that carries no battery; there is "
        "no state to freeze, and a request that does nothing is refused rather than ignored");
  }

  const int battery_index = model->has_battery() ? model->battery_state_index() : -1;

  // WIND AND THE STATE'S OWN VELOCITY COORDINATE.
  //
  // ADR-0002's velocity state is AIR-RELATIVE, and `Quadrotor::derivative`
  // takes the wind as steady: it carries no -R^T dw/dt term. That is correct
  // for constant wind and wrong for every other kind, in two different ways
  // that need two different treatments.
  //
  // A STEP is not a large derivative, it is an event. No force acts at the
  // instant the air mass changes speed, so the GROUND velocity is continuous
  // and the air-relative velocity must jump by exactly minus the wind change,
  // rotated into the body frame. Integrating through the step instead injects
  // the whole wind increment as a ground-velocity error, permanently and
  // silently — this is WP1's first finding, and validation case 6 re-bases at
  // the one wind step in its fixture for exactly this reason. Below, the run is
  // split at every wind discontinuity and the jump applied between segments.
  //
  // A RAMP is a genuine derivative. Under a linear hold the wind has a finite
  // dw/dt, and the air-relative velocity's rate gains -R^T dw/dt, which the
  // plant does not know about. It is added here, to the velocity rows only,
  // rather than inside the plant: the plant's contract is steady wind and this
  // is the simulator's coordinate bookkeeping, not new dynamics.
  const bool wind_ramps = !wind_history.empty() && wind_history.hold() == sim::HoldPolicy::Linear;

  // A ZERO-ORDER-HOLD VALUE IS RESOLVED ONCE PER SEGMENT, NOT PER STAGE, and
  // this is not an optimisation.
  //
  // RK4's last stage of the step ending at t_k sits exactly ON t_k. A schedule
  // asked for its value at t_k returns the value that STARTS there — the new
  // one — so the final stage of the preceding step integrates under a command
  // that has not taken effect yet. The step is a quarter wrong, at every jump.
  // Measured against the cross-implementation fixture it cost five orders of
  // magnitude: worst rotor 7.2e-01 rad/s against the 2.2e-05 the validation
  // case records.
  //
  // Within a zero-order segment the value is constant by construction, so
  // resolving it once at the segment's start is exact as well as correct. A
  // linear hold has no such boundary — it is continuous — and is evaluated per
  // stage.
  Eigen::VectorXd segment_command = command_history.empty() ? command : command_history.at(0.0);
  Eigen::Vector3d segment_wind =
      wind_history.empty() ? wind : Eigen::Vector3d(wind_history.at(0.0));
  const bool command_per_stage =
      !command_history.empty() && command_history.hold() == sim::HoldPolicy::Linear;
  const bool wind_per_stage =
      !wind_history.empty() && wind_history.hold() == sim::HoldPolicy::Linear;

  const numerics::DerivativeFunction derivative = [&](double time_s,
                                                      const Eigen::VectorXd& x) -> Eigen::VectorXd {
    const Eigen::VectorXd command_now =
        command_per_stage ? command_history.at(time_s) : segment_command;
    const Eigen::Vector3d wind_now =
        wind_per_stage ? Eigen::Vector3d(wind_history.at(time_s)) : segment_wind;
    Eigen::VectorXd rate = model->derivative(x, command_now, wind_now);
    if (wind_ramps) {
      const core::State state = core::State::from_vector(x.head<core::kStateSize>());
      const Eigen::Vector3d wind_rate_ned = wind_history.rate_at(time_s);
      rate.segment<3>(core::kVelocityU) -=
          core::dcm_ned_from_body(state.attitude_body_to_ned).transpose() * wind_rate_ned;
    }
    if (freeze_battery) {
      rate(battery_index) = 0.0;
    }
    return rate;
  };
  const numerics::ProjectionFunction projection = [&](Eigen::VectorXd& x) { model->project(x); };

  // Every discontinuity is a segment boundary, whether or not it moves the
  // state. A command step needs no re-basing, but integrating across it inside
  // one RK4 step would still cost the method its order; splitting there keeps
  // every step's four stages on one side of every jump.
  std::vector<int> boundaries;
  for (const double time_s : command_history.discontinuities()) {
    boundaries.push_back(step_index_for_event(time_s, step_s, steps, "command_schedule"));
  }
  std::vector<int> wind_events;
  for (const double time_s : wind_history.discontinuities()) {
    const int index = step_index_for_event(time_s, step_s, steps, "wind_schedule");
    boundaries.push_back(index);
    wind_events.push_back(index);
  }
  std::sort(boundaries.begin(), boundaries.end());
  boundaries.erase(std::unique(boundaries.begin(), boundaries.end()), boundaries.end());

  PlantRun run;
  run.state_names = model->extended_state_names();
  run.command_rad_s = command;
  run.wind_ned_m_s = wind;
  run.battery_present = model->has_battery();
  run.battery_frozen = freeze_battery;
  run.step_s = step_s;
  run.step_count = steps;

  // Integrate segment by segment at stride one, then take every `stride`-th
  // sample from the assembled whole. Sub-sampling globally rather than per
  // segment keeps the recorded times on one lattice however the events fall.
  Eigen::VectorXd state = initial;
  std::vector<Eigen::VectorXd> all_states;
  std::vector<double> all_times;
  all_states.reserve(static_cast<std::size_t>(steps) + 1);
  all_times.reserve(static_cast<std::size_t>(steps) + 1);
  int taken = 0;
  for (std::size_t segment = 0; segment <= boundaries.size(); ++segment) {
    const int stop = segment < boundaries.size() ? boundaries[segment] : steps;
    const int length = stop - taken;
    // The value in force ACROSS this segment, read at its start.
    const double segment_start_s = static_cast<double>(taken) * step_s;
    if (!command_history.empty() && !command_per_stage) {
      segment_command = command_history.at(segment_start_s);
    }
    if (!wind_history.empty() && !wind_per_stage) {
      segment_wind = Eigen::Vector3d(wind_history.at(segment_start_s));
    }
    if (length > 0) {
      const numerics::Trajectory piece = numerics::integrate_fixed_step(
          derivative, state, static_cast<double>(taken) * step_s, step_s, length, 1, projection);
      for (std::size_t i = 0; i + 1 < piece.states.size(); ++i) {
        all_times.push_back(piece.times_s[i]);
        all_states.push_back(piece.states[i]);
      }
      state = piece.states.back();
      taken = stop;
    }
    // The wind event, applied between segments: ground velocity is continuous,
    // so the air-relative velocity absorbs the whole change.
    if (segment < boundaries.size()
        && std::find(wind_events.begin(), wind_events.end(), stop) != wind_events.end()) {
      const core::State at_event = core::State::from_vector(state.head<core::kStateSize>());
      const Eigen::Vector3d jump = wind_history.jump_at(static_cast<double>(stop) * step_s);
      state.segment<3>(core::kVelocityU) -=
          core::dcm_ned_from_body(at_event.attitude_body_to_ned).transpose() * jump;
    }
  }
  all_times.push_back(static_cast<double>(steps) * step_s);
  all_states.push_back(state);

  for (std::size_t i = 0; i < all_states.size(); ++i) {
    if (i % static_cast<std::size_t>(stride) == 0 || i + 1 == all_states.size()) {
      const double at = all_times[i];
      run.trajectory.times_s.push_back(at);
      run.trajectory.states.push_back(all_states[i]);
      run.command_samples_rad_s.push_back(command_history.empty() ? command
                                                                  : command_history.at(at));
      // The wind AFTER the event at a boundary sample, which is the wind the
      // next step integrates under and the one that makes the recorded
      // air-relative velocity add up to the right ground velocity.
      run.wind_samples_ned_m_s.push_back(
          wind_history.empty() ? wind : Eigen::Vector3d(wind_history.at(at)));
    }
  }
  run.trajectory.step_s = step_s;
  run.trajectory.step_count = steps;
  run.trajectory.sample_stride = stride;

  std::ostringstream summary;
  summary << run.trajectory.states.size() << " samples over " << std::fixed << std::setprecision(3)
          << (static_cast<double>(steps) * step_s) << " s at " << std::scientific
          << std::setprecision(1) << step_s << " s";
  if (run.battery_present) {
    summary << "; battery " << (freeze_battery ? "frozen" : "evolving");
  }

  Artifact artifact;
  artifact.kind = "plant_trajectory";
  artifact.summary = summary.str();
  artifact.payload = std::move(run);
  return artifact;
}

// --- trim.hover ------------------------------------------------------------

Artifact trim_hover_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("quadrotor");
  const auto& model = upstream.payload_as<model::Quadrotor>("quadrotor");

  trim::HoverTrimRequest request;
  request.altitude_m = context.input->number_at("altitude_m", 0.0);
  request.wind_ned_m_s = vector3_at(context, "wind_ned_m_s");
  request.ground_velocity_ned_m_s = vector3_at(context, "ground_velocity_ned_m_s");
  // Degrees at the boundary and nowhere below it, converted by units.hpp alone
  // (ADR-0003). `trim.level` states its flight path angle the same way.
  request.heading_rad = units::degrees_to_radians(context.input->number_at("heading_deg", 0.0));
  request.battery_state_of_charge = context.input->number_at("battery_state_of_charge", 1.0);
  request.residual_tolerance = context.input->number_at("tolerance", 1e-10);
  request.iterations = positive_integer_at(context, "iterations", 40);

  HoverTrimArtifact trimmed{trim::trim_hover(model, request), model};

  std::ostringstream summary;
  summary << std::fixed << std::setprecision(3) << "roll "
          << units::radians_to_degrees(trimmed.point.roll_rad) << " deg, pitch "
          << units::radians_to_degrees(trimmed.point.pitch_rad) << " deg, airspeed "
          << trimmed.point.airspeed_m_s << " m/s; smallest rotor margin "
          << (100.0 * trimmed.point.smallest_rotor_margin_fraction) << " %; residual "
          << std::scientific << std::setprecision(1) << trimmed.point.residual_norm << " after "
          << trimmed.point.newton_iterations << " iterations";

  Artifact artifact;
  artifact.kind = "hover_trim";
  artifact.summary = summary.str();
  artifact.payload = trimmed;
  return artifact;
}

// --- linearize.extended ----------------------------------------------------

linearize::OutputKind output_kind_from_name(const std::string& name) {
  if (name == "specific_force") {
    return linearize::OutputKind::BodySpecificForce;
  }
  if (name == "body_rates") {
    return linearize::OutputKind::BodyRates;
  }
  if (name == "position_ned") {
    return linearize::OutputKind::PositionNed;
  }
  if (name == "altitude") {
    return linearize::OutputKind::Altitude;
  }
  if (name == "ground_velocity_ned") {
    return linearize::OutputKind::GroundVelocityNed;
  }
  throw std::invalid_argument(
      "'outputs' contains '" + name
      + "'; it must be one of specific_force, body_rates, position_ned, altitude, "
        "ground_velocity_ned");
}

std::vector<linearize::OutputKind> outputs_from(const StageContext& context) {
  const ValuePtr value = context.input->get("outputs");
  if (!value || value->kind() == Value::Kind::Null) {
    // The full observation model. Chosen as the default rather than C = I
    // because this programme's sensors do not measure the states, and a caller
    // who says nothing should get the honest set rather than the flattering one.
    return {linearize::OutputKind::BodySpecificForce,
            linearize::OutputKind::BodyRates,
            linearize::OutputKind::PositionNed,
            linearize::OutputKind::Altitude,
            linearize::OutputKind::GroundVelocityNed};
  }
  std::vector<linearize::OutputKind> kinds;
  for (const ValuePtr& item : value->as_list()) {
    kinds.push_back(output_kind_from_name(item->as_string()));
  }
  if (kinds.empty()) {
    throw std::invalid_argument("'outputs' is an empty list; omit the key for the full set");
  }
  return kinds;
}

std::string evidence_for(const trim::HoverTrim& point,
                         const linearize::ExtendedLinearisation& linearisation,
                         const std::string& stage_id) {
  std::ostringstream out = evidence_stream();
  out << "schema: galata.operating-point.v1\n";
  out << "stage: \"" << stage_id << "\"\n";
  out << "# The point the matrices beside this file were taken about, and every\n";
  out << "# diagnostic of the taking. A matrix without this is a number with no\n";
  out << "# routine behind it.\n";
  out << "trim:\n";
  out << "  altitude_m: " << point.altitude_m << "\n";
  out << "  roll_rad: " << point.roll_rad << "\n";
  out << "  pitch_rad: " << point.pitch_rad << "\n";
  out << "  yaw_rad: " << point.yaw_rad << "\n";
  out << "  airspeed_m_s: " << point.airspeed_m_s << "\n";
  out << "  battery_state_of_charge: " << point.battery_state_of_charge << "\n";
  out << "  residual_norm: " << point.residual_norm << "\n";
  // Which coordinates this residual DOES NOT COVER. A trim that is an
  // equilibrium in six coordinates and a held value in a seventh must say so
  // here, or a reader takes the residual as covering every row of the state.
  out << "  frozen_states:\n";
  if (!point.battery_state_of_charge_frozen) {
    out << "    []\n";
  } else {
    out << "    - name: \"battery_soc\"\n";
    out << "      held_at: " << point.battery_state_of_charge << "\n";
    out << "      excluded_from_residual: true\n";
    out << "      reason: \"a powered battery has no zero-energy-derivative equilibrium\"\n";
  }
  out << "  residual_tolerance: " << point.residual_tolerance << "\n";
  out << "  newton_iterations: " << point.newton_iterations << "\n";
  out << "  jacobian_condition_number: " << point.jacobian_condition_number << "\n";
  out << "  smallest_rotor_margin_fraction: " << point.smallest_rotor_margin_fraction << "\n";
  out << "  ";
  write_vector(out, "extended_state", point.extended_state);
  out << "  ";
  write_vector(out, "command_rad_s", point.command_rad_s);
  out << "linearisation:\n";
  out << "  equilibrium_residual_norm: " << linearisation.equilibrium_residual_norm << "\n";
  out << "  equilibrium_tolerance: " << linearisation.equilibrium_tolerance << "\n";
  out << "  worst_relative_truncation: " << linearisation.worst_relative_truncation << "\n";
  out << "  worst_relative_truncation_a: " << linearisation.worst_relative_truncation_a << "\n";
  out << "  worst_relative_truncation_b: " << linearisation.worst_relative_truncation_b << "\n";
  out << "  worst_relative_truncation_c: " << linearisation.worst_relative_truncation_c << "\n";
  out << "  worst_relative_truncation_d: " << linearisation.worst_relative_truncation_d << "\n";
  out << "  ";
  write_vector(out, "state_steps", linearisation.state_steps);
  out << "  ";
  write_vector(out, "input_steps", linearisation.input_steps);
  out << "  frozen_states:\n";
  if (linearisation.frozen_state_names.empty()) {
    out << "    []\n";
  }
  for (std::size_t i = 0; i < linearisation.frozen_state_names.size(); ++i) {
    // The rate that was declared away, reported so the declaration is auditable.
    out << "    - name: \"" << linearisation.frozen_state_names[i] << "\"\n";
    out << "      declared_zero_actual_rate: " << linearisation.frozen_state_rates[i] << "\n";
  }
  return out.str();
}

Artifact linearize_extended_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("trim");
  const auto& trimmed = upstream.payload_as<HoverTrimArtifact>("hover_trim");
  const model::Quadrotor& model = trimmed.model;
  const trim::HoverTrim& point = trimmed.point;

  const bool wind_inputs = context.input->bool_at("wind_inputs", true);
  const bool freeze_battery = context.input->bool_at("freeze_battery", true);

  linearize::ExtendedLinearisationOptions options;
  options.appended_state_names = model.extended_state_names();
  options.appended_state_names.erase(options.appended_state_names.begin(),
                                     options.appended_state_names.begin() + core::kStateSize);
  options.input_names = model.input_names();
  if (wind_inputs) {
    options.wind_input_offset = static_cast<int>(options.input_names.size());
    options.input_names.push_back("wind_north_m_s");
    options.input_names.push_back("wind_east_m_s");
    options.input_names.push_back("wind_down_m_s");
  }
  options.outputs = outputs_from(context);
  options.report_truncation_error = context.input->bool_at("report_truncation_error", true);
  options.equilibrium_tolerance = context.input->number_at("equilibrium_tolerance", 1e-10);
  if (freeze_battery && model.has_battery()) {
    // The battery index within the APPENDED block: one per rotor comes first.
    options.frozen_appended_states = {model.rotor_count()};
  }

  const int rotor_count = model.rotor_count();
  const linearize::ExtendedDynamics dynamics = [&](const Eigen::VectorXd& x,
                                                   const Eigen::VectorXd& u) {
    const Eigen::Vector3d wind =
        wind_inputs ? Eigen::Vector3d(u.segment<3>(rotor_count)) : Eigen::Vector3d::Zero();
    return model.derivative(x, u.head(rotor_count), wind);
  };

  // The nominal input is the trim's own: its steady rotor commands, and the
  // wind it was solved at. Neither is re-read from this stage, so the point the
  // matrices describe cannot drift from the point that was trimmed.
  Eigen::VectorXd input =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(options.input_names.size()));
  input.head(rotor_count) = point.command_rad_s;
  if (wind_inputs) {
    input.segment<3>(rotor_count) = point.wind_ned_m_s;
  }

  const linearize::ExtendedLinearisation linearisation =
      linearize::linearize_extended(dynamics, point.extended_state, input, options);

  std::ostringstream description;
  description << (model.description.empty() ? std::string("multirotor") : model.description)
              << " — linearised about " << point.altitude_m << " m, airspeed " << point.airspeed_m_s
              << " m/s";

  std::ostringstream summary;
  summary << linearisation.a.rows() << " states, " << linearisation.b.cols() << " inputs, "
          << linearisation.c.rows() << " outputs; equilibrium residual " << std::scientific
          << std::setprecision(1) << linearisation.equilibrium_residual_norm;
  if (options.report_truncation_error) {
    summary << ", worst truncation " << linearisation.worst_relative_truncation;
  } else {
    summary << ", truncation not estimated";
  }
  if (!linearisation.frozen_state_names.empty()) {
    summary << "; frozen " << linearisation.frozen_state_names.size() << " state(s)";
  }

  if (const ValuePtr evidence_path = context.input->get("evidence_path")) {
    context.write_output(evidence_path->as_string(),
                         evidence_for(point, linearisation, context.stage_id));
  }

  Artifact artifact;
  artifact.kind = "linear_system";
  artifact.summary = summary.str();
  artifact.payload = linearisation.to_linear_system(description.str(), model.citation);
  return artifact;
}

// --- model.linear.export ---------------------------------------------------

Artifact export_state_space(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("system");
  const auto& system = upstream.payload_as<model::LinearSystem>("linear_system");

  const std::string path = context.input->string_at("path");
  context.write_output(path, model::serialize_linear_system(system));

  std::ostringstream summary;
  summary << "wrote " << system.state_count() << " states, " << system.input_count() << " inputs, "
          << system.output_count() << " outputs to " << path;

  Artifact artifact;
  artifact.kind = "linear_system";
  artifact.summary = summary.str();
  // The system itself, unchanged, so an export can sit mid-chain rather than
  // only at the end of one.
  artifact.payload = system;
  artifact.linearization_evidence = upstream.linearization_evidence;
  return artifact;
}

}  // namespace

void register_quadrotor_capabilities(Registry& registry) {
  // ImplementedUnvalidated, for the reason `model.quadrotor` carries: the cases
  // behind these are exact invariants of the equations and a cross-check
  // against an independent implementation. Neither is a published reference.
  registry.add(Capability{
      "sim.plant",
      "Integrate a nonlinear plant model with fixed-step RK4 from a declared state or a trim, "
      "carrying its appended rotor and battery states",
      "plant_trajectory",
      Capability::State::ImplementedUnvalidated,
      simulate_plant_capability,
      {"quadrotor",
       "trim",
       "initial_extended_state",
       "command_rad_s",
       "command_schedule",
       "wind_ned_m_s",
       "wind_schedule",
       "step_s",
       "steps",
       "sample_stride",
       "freeze_battery"}});

  registry.add(Capability{
      "trim.hover",
      "Solve multirotor equilibrium — still-air hover, hover in a crosswind, or cruise as a "
      "relative equilibrium — for attitude and rotor speeds, reporting each rotor's margin",
      "hover_trim",
      Capability::State::ImplementedUnvalidated,
      trim_hover_capability,
      {"quadrotor",
       "altitude_m",
       "wind_ned_m_s",
       "ground_velocity_ned_m_s",
       "heading_deg",
       "battery_state_of_charge",
       "tolerance",
       "iterations"}});

  registry.add(Capability{
      "linearize.extended",
      "Linearise a multirotor about a hover trim on a local attitude-error chart, with named "
      "wind disturbance columns and a declared observation model",
      "linear_system",
      Capability::State::ImplementedUnvalidated,
      linearize_extended_capability,
      {"trim",
       "outputs",
       "wind_inputs",
       "freeze_battery",
       "report_truncation_error",
       "equilibrium_tolerance",
       "evidence_path"},
      {},
      {"evidence_path"}});

  registry.add(
      Capability{"model.linear.export",
                 "Write a linear model as the named-matrix YAML that model.linear.statespace reads",
                 "linear_system",
                 Capability::State::ImplementedUnvalidated,
                 export_state_space,
                 {"system", "path"},
                 {},
                 {"path"}});
}

}  // namespace galata::pipeline
