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
#include "galata/sim/discrete.hpp"
#include "galata/sim/schedule.hpp"
#include "galata/synth/control.hpp"
#include "galata/trim/hover.hpp"
#include "galata/units.hpp"
#include "galata/version.hpp"

#include "input_schedule_parse.hpp"
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string_view>

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
//
// The schema is shared with `sim.linear` and lives in input_schedule_parse.cpp.

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
    model = &context.upstream_at("quadrotor").payload_as<QuadrotorArtifact>("quadrotor").model;
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

// --- sim.sampled -----------------------------------------------------------
//
// A controller executed at a DECLARED RATE against the nonlinear plant.
//
// THE ORDERING, written down because a sampled loop is defined by it and two
// implementations that disagree about it produce different aircraft. At every
// controller tick k, at time t_k = k * period:
//
//   1. MEASURE.    The plant state at t_k, sampled instantaneously. There is no
//                  sensor model here: this is the ideal-measurement case, and a
//                  sensor's own dynamics belong in the plant, not in the loop.
//   2. COORDINATES. The error in the chart the gain was designed in, taken
//                  against the reference AT t_k — see the reference motion note
//                  below, which is where cruise differs from hover.
//   3. LAW.        u_requested = u_trim - K e. Static state feedback: no
//                  controller state, and therefore NO ANTI-WINDUP, because
//                  there is no integrator to wind up. That is a property of
//                  this law and not a general claim; an integrating law added
//                  here later must bring its own anti-windup and say so.
//   4. ALLOCATION. The identity, stated rather than skipped. This law's outputs
//                  ARE rotor-speed commands, because the gain was designed
//                  against a B whose columns are rotor commands. A law that
//                  produced collective and moments would need a mixer here, and
//                  that mixer would be a declared part of the contract.
//   5. SATURATION. Clamped to each rotor's own [minimum, maximum]. The
//                  requested and the clamped command are BOTH recorded.
//   6. DELAY.      The clamped command enters a queue of `delay_periods`
//                  entries; what the plant receives at t_k is the command
//                  computed `delay_periods` ticks ago. At t = 0 the queue holds
//                  the TRIM command, declared rather than zero, because a
//                  multirotor commanded to zero falls.
//   7. HOLD.       That command is held until t_{k+1}. Zero-order, exactly.
//
// TIMING IS AN INTEGER SCHEDULE. The controller period must be a whole number
// of integration steps and the horizon a whole number of periods; anything else
// is refused rather than rounded. A tick that landed inside a step would have
// the integrator's four stages straddling a command change, which costs the
// method its order and — unlike a wind step — cannot be fixed by splitting,
// because the tick is where the command is DEFINED to change.
//
// TWO KINDS OF LAW, and they are not interchangeable. A `control_law` from
// `synth.lqr` was designed on the continuous linearisation; executing it here
// at a rate is emulation, which this capability has always done and says so. A
// `sampled_control_law` from `synth.sampled_lqr` was designed FOR one sample
// time, one hold and one plant, and is optimal at that period and no other. So
// for a discrete design the execution period must EQUAL the design period, the
// hold and the delay must both be declared rather than defaulted, and the run
// carries the design's own prediction of the loop beside what the nonlinear
// plant did. No transformation between rates is supported: a mismatch is
// refused, and the remedy is a redesign at the execution period.
//
// WHAT THIS IS NOT. Not a claim about margins: gain and phase margins computed
// from the continuous linearisation describe the continuous loop, and this loop
// samples, holds and delays. The sampled loop's own robustness is a separate
// question this capability does not answer — for a discrete design as much as
// for a continuous one, since a converging run and a stabilising Riccati
// solution are both statements about the nominal loop. Not an ESC model either
// — commands are rotor speeds in rad/s, and the map to an electrical command is
// out of scope by RFC-0002's own boundary.

// A gain can be applied to this vehicle only if it was designed on this
// vehicle's chart: one row per rotor command in the model's order, one column
// per chart coordinate, and the appended coordinates the model's own. Checked
// for both kinds of law, because a gain of the wrong shape would otherwise
// reach Eigen as a size mismatch rather than a sentence.
void require_law_basis(const model::Quadrotor& model,
                       const Eigen::MatrixXd& gain,
                       const std::vector<std::string>& state_names,
                       const std::vector<std::string>& input_names) {
  const int chart_width =
      linearize::kRigidChartSize + model.extended_state_size() - core::kStateSize;
  if (gain.rows() != model.rotor_count() || gain.cols() != chart_width
      || static_cast<int>(state_names.size()) != chart_width) {
    std::ostringstream message;
    message << "sim.sampled: the law's gain is " << gain.rows() << "x" << gain.cols() << " over "
            << state_names.size() << " named states; this vehicle needs " << model.rotor_count()
            << "x" << chart_width
            << " — one row per rotor command and one column per attitude-error chart "
               "coordinate. A gain designed on another vehicle, or on a selection that dropped "
               "states, cannot be applied to this one";
    throw std::invalid_argument(message.str());
  }
  const std::vector<std::string> commands = model.input_names();
  if (input_names != commands) {
    std::ostringstream message;
    message << "sim.sampled: the law commands [";
    for (std::size_t i = 0; i < input_names.size(); ++i) {
      message << (i == 0 ? "" : ", ") << input_names[i];
    }
    message << "] and this vehicle's rotor commands are [";
    for (std::size_t i = 0; i < commands.size(); ++i) {
      message << (i == 0 ? "" : ", ") << commands[i];
    }
    message << "]. The allocation here is the identity, so the law's inputs must BE the rotor "
               "commands, in the model's order";
    throw std::invalid_argument(message.str());
  }
  const std::vector<std::string> extended = model.extended_state_names();
  for (int i = linearize::kRigidChartSize; i < chart_width; ++i) {
    const std::string& expected =
        extended[static_cast<std::size_t>(i - linearize::kRigidChartSize + core::kStateSize)];
    if (state_names[static_cast<std::size_t>(i)] != expected) {
      throw std::invalid_argument("sim.sampled: the law's chart coordinate " + std::to_string(i)
                                  + " is '" + state_names[static_cast<std::size_t>(i)]
                                  + "' and this vehicle's is '" + expected + "'");
    }
  }
}

Artifact simulate_sampled_capability(const StageContext& context) {
  const auto& trimmed = context.upstream_at("trim").payload_as<HoverTrimArtifact>("hover_trim");
  const model::Quadrotor& model = trimmed.model;
  const Eigen::VectorXd reference_state = trimmed.point.extended_state;
  const Eigen::VectorXd trim_command = trimmed.point.command_rad_s;

  const Artifact& law_artifact = context.upstream_at("law");
  const synth::SampledLqrDesign* discrete_law = nullptr;
  Eigen::MatrixXd gain;
  std::vector<std::string> law_state_names;
  std::vector<std::string> law_input_names;
  if (law_artifact.kind == "sampled_control_law") {
    discrete_law = &law_artifact.payload_as<synth::SampledLqrDesign>("sampled_control_law");
    gain = discrete_law->riccati.k;
    law_state_names = discrete_law->discretisation.system.state_names;
    law_input_names = discrete_law->discretisation.system.input_names;
  } else if (law_artifact.kind == "control_law") {
    const synth::LqrDesign* continuous_law =
        &law_artifact.payload_as<synth::LqrDesign>("control_law");
    gain = continuous_law->riccati.k;
    law_state_names = continuous_law->plant.state_names;
    law_input_names = continuous_law->plant.input_names;
  } else {
    throw std::invalid_argument(
        "sim.sampled: `law` must be a `control_law` from synth.lqr — a continuous design this "
        "capability executes at a rate — or a `sampled_control_law` from synth.sampled_lqr, "
        "designed for the period it runs at. Stage produced a '"
        + law_artifact.kind + "'");
  }
  require_law_basis(model, gain, law_state_names, law_input_names);

  const double step_s = context.input->number_at("step_s");
  if (!(step_s > 0.0) || !std::isfinite(step_s)) {
    throw std::invalid_argument("sim.sampled: step_s must be a positive finite number of seconds");
  }
  const int steps = positive_integer_at(context, "steps", 0);
  const double period_s = context.input->number_at("controller_period_s");
  if (!(period_s > 0.0) || !std::isfinite(period_s)) {
    throw std::invalid_argument(
        "sim.sampled: controller_period_s must be a positive finite number of seconds");
  }
  const double periods_per_step = period_s / step_s;
  const double nearest = std::round(periods_per_step);
  if (std::fabs(periods_per_step - nearest) > 1e-9 * std::fmax(1.0, periods_per_step)
      || nearest < 1.0) {
    std::ostringstream message;
    message << "sim.sampled: the controller period " << period_s << " s is not a whole number of "
            << step_s
            << " s integration steps. A tick inside a step would put the integrator's stages "
               "either side of a command change, and unlike a wind step it cannot be split "
               "around, because the tick is where the command is defined to change. Choose a "
               "period that the step divides";
    throw std::invalid_argument(message.str());
  }
  const int steps_per_tick = static_cast<int>(nearest);
  if (steps % steps_per_tick != 0) {
    std::ostringstream message;
    message << "sim.sampled: " << steps << " step(s) is not a whole number of " << steps_per_tick
            << "-step controller periods; the run would end partway through a hold";
    throw std::invalid_argument(message.str());
  }
  const int tick_count = steps / steps_per_tick;

  // THE HOLD. Zero-order is the only one this schedule executes, so a study
  // running a continuous law may omit it, as every study written before
  // discrete designs existed does. A discrete design must state it: the design
  // was discretised under a hold, and this run is the loop the design describes
  // only if it holds the same way.
  const ValuePtr hold_value = context.input->get("hold");
  if (hold_value && hold_value->as_string() != "zero_order") {
    throw std::invalid_argument("sim.sampled: `hold: " + hold_value->as_string()
                                + "` is not executed here; `zero_order` is the only hold this "
                                  "schedule applies");
  }
  if (discrete_law != nullptr) {
    if (!hold_value) {
      throw std::invalid_argument(
          "sim.sampled: a discrete design must declare `hold: zero_order`. It was discretised "
          "under a hold, and this run is the loop it describes only if it holds the same way; "
          "a study that flies it should say so rather than inherit it");
    }
    if (context.input->get("delay_periods") == nullptr) {
      throw std::invalid_argument(
          "sim.sampled: a discrete design must declare `delay_periods`, zero included. The "
          "design modelled no delay at all, so the delay it is flown with is the study's "
          "statement about the implementation and not something to default");
    }
    // THE PERIOD, CHECKED AND NOT ASSUMED. Exact equality, because both numbers
    // are read from the same kind of decimal text and a design at 0.004 s is
    // executed at 0.004 s or it is not.
    if (period_s != discrete_law->sample_time_s) {
      std::ostringstream message;
      message << std::setprecision(std::numeric_limits<double>::max_digits10)
              << "sim.sampled: the law was designed at a sample time of "
              << discrete_law->sample_time_s << " s and this run executes it every " << period_s
              << " s. A discrete gain is optimal for the period its plant and cost were "
                 "discretised at and for no other: at another period the same matrix acts on a "
                 "different plant, minimises a cost that is not this loop's, and has closed-loop "
                 "eigenvalues on a different circle. No transformation between rates is "
                 "supported, so the mismatch is refused. Redesign with synth.sampled_lqr at "
                 "`sample_time_s: "
              << period_s << "`, or execute at the design's period";
      throw std::invalid_argument(message.str());
    }
  }

  const double delay_number = context.input->number_at("delay_periods", 0.0);
  if (!std::isfinite(delay_number) || delay_number < 0.0
      || std::floor(delay_number) != delay_number) {
    throw std::invalid_argument(
        "sim.sampled: delay_periods must be a whole number of controller periods, zero or more. "
        "A delay that is not a whole number of periods is not representable by this schedule");
  }
  const int delay_periods = static_cast<int>(delay_number);

  // THE REFERENCE, AND WHY CRUISE IS NOT HOVER WITH A NUMBER CHANGED.
  //
  // At hover the reference state is constant. At a relative equilibrium it is
  // not: the position rate is nonzero by construction, so a reference held at
  // the trim's initial position becomes, one second later, a demand to fly back
  // to where the aircraft started. The position error then grows without bound
  // and the controller fights the very equilibrium it was designed about.
  //
  // So a trim with a nonzero ground velocity must SAY which it means, and gets
  // no default. `follow_trim_velocity` translates the reference position at the
  // trim's ground velocity; `hold_position` keeps it fixed, which is station
  // keeping and a legitimate but different task.
  const bool cruising = trimmed.point.ground_velocity_ned_m_s.norm() > 0.0;
  const ValuePtr motion = context.input->get("reference_motion");
  if (cruising && !motion) {
    throw std::invalid_argument(
        "sim.sampled: this trim has a nonzero ground velocity, so `reference_motion` must be "
        "declared: `follow_trim_velocity` translates the reference position along the "
        "equilibrium, `hold_position` keeps it fixed. There is no default, because a hover "
        "reference silently reused at cruise becomes a demand to fly back to the start");
  }
  bool reference_moves = false;
  if (motion) {
    const std::string name = motion->as_string();
    if (name == "follow_trim_velocity") {
      reference_moves = true;
    } else if (name == "hold_position") {
      reference_moves = false;
    } else {
      throw std::invalid_argument(
          "sim.sampled: `reference_motion` must be `follow_trim_velocity` or `hold_position`; '"
          + name + "' is neither");
    }
  }

  const sim::InputSchedule wind_history = schedule_at(context, "wind_schedule", 3);
  const bool freeze_battery = context.input->bool_at("freeze_battery", false);
  if (freeze_battery && !model.has_battery()) {
    throw std::invalid_argument(
        "sim.sampled: freeze_battery was asked for on a model that carries no battery");
  }
  const int battery_index = model.has_battery() ? model.battery_state_index() : -1;

  // THE DESIGN'S OWN PREDICTION, and when there is none to compare. The
  // prediction is in deviations from the equilibrium the design was linearised
  // about, so it is withheld — not reported — whenever the measured chart
  // coordinates would be deviations from something else.
  std::string prediction_blocker;
  if (discrete_law == nullptr) {
    prediction_blocker = "the law is a continuous design and has no discrete prediction";
  } else if (cruising && !reference_moves) {
    prediction_blocker =
        "the reference holds position at a relative equilibrium, so the measured coordinates "
        "are not deviations from the equilibrium the design was linearised about";
  } else if (!wind_history.empty()) {
    prediction_blocker = "a declared wind history acts on the plant and not on the prediction";
  }
  const ValuePtr budget_value = context.input->get("linear_agreement_budget");
  double agreement_budget = 0.0;
  if (budget_value) {
    if (!prediction_blocker.empty()) {
      throw std::invalid_argument(
          "sim.sampled: `linear_agreement_budget` was declared, but there is nothing on this run "
          "to hold it against: "
          + prediction_blocker + ". A budget on nothing is refused rather than ignored");
    }
    agreement_budget = budget_value->as_number();
    if (!(agreement_budget > 0.0) || !std::isfinite(agreement_budget)) {
      throw std::invalid_argument(
          "sim.sampled: `linear_agreement_budget` must be a positive finite fraction");
    }
  }

  Eigen::VectorXd state = reference_state;
  if (context.input->get("initial_chart_perturbation") != nullptr) {
    const int chart_width =
        linearize::kRigidChartSize + model.extended_state_size() - core::kStateSize;
    const Eigen::VectorXd delta = vector_at(context, "initial_chart_perturbation", chart_width);
    state = linearize::extended_from_chart(delta, reference_state);
  }

  PlantRun run;
  run.state_names = model.extended_state_names();
  run.command_rad_s = trim_command;
  run.wind_ned_m_s = trimmed.point.wind_ned_m_s;
  run.battery_present = model.has_battery();
  run.battery_frozen = freeze_battery;
  run.step_s = step_s;
  run.step_count = steps;

  SampledControlRecord record;
  record.controller_period_s = period_s;
  record.delay_periods = delay_periods;
  record.reference_follows_trim_velocity = reference_moves;

  // The delay line, initialised to the TRIM command. Zero would be a multirotor
  // switched off for the first `delay_periods` ticks.
  std::vector<Eigen::VectorXd> queue(static_cast<std::size_t>(delay_periods) + 1, trim_command);
  std::size_t queue_head = 0;

  const auto record_sample =
      [&](double time_s, const Eigen::VectorXd& x, const Eigen::VectorXd& applied) {
        run.trajectory.times_s.push_back(time_s);
        run.trajectory.states.push_back(x);
        run.command_samples_rad_s.push_back(applied);
        run.wind_samples_ned_m_s.push_back(
            wind_history.empty() ? run.wind_ned_m_s : Eigen::Vector3d(wind_history.at(time_s)));
      };

  Eigen::VectorXd applied = trim_command;
  for (int tick = 0; tick < tick_count; ++tick) {
    const double tick_time_s = static_cast<double>(tick) * period_s;

    // 2. Coordinates, against the reference AT THIS TICK.
    Eigen::VectorXd reference_now = reference_state;
    if (reference_moves) {
      reference_now.segment<3>(core::kPositionNorth) +=
          trimmed.point.ground_velocity_ned_m_s * tick_time_s;
    }
    const Eigen::VectorXd error = linearize::chart_from_extended(state, reference_now);

    // 3. Law. 4. Allocation is the identity. 5. Saturation.
    const Eigen::VectorXd requested = trim_command - gain * error;
    Eigen::VectorXd clamped = requested;
    bool clipped = false;
    for (int rotor = 0; rotor < model.rotor_count(); ++rotor) {
      const auto& description = model.rotors[static_cast<std::size_t>(rotor)];
      const double ceiling =
          std::fmax(model.speed_ceiling_rad_s(rotor, trimmed.point.battery_state_of_charge),
                    description.minimum_speed_rad_s);
      clamped(rotor) = std::clamp(requested(rotor), description.minimum_speed_rad_s, ceiling);
      const double residual = std::fabs(clamped(rotor) - requested(rotor));
      if (residual > 0.0) {
        clipped = true;
        record.worst_saturation_residual_rad_s =
            std::fmax(record.worst_saturation_residual_rad_s, residual);
      }
    }
    if (clipped) {
      ++record.saturated_tick_count;
    }

    // 6. Delay: what the plant receives now was computed `delay_periods` ago.
    queue[queue_head] = clamped;
    queue_head = (queue_head + 1) % queue.size();
    applied = queue[queue_head];

    record.tick_times_s.push_back(tick_time_s);
    record.requested_rad_s.push_back(requested);
    record.saturated_rad_s.push_back(clamped);
    record.applied_rad_s.push_back(applied);
    record.chart_error.push_back(error);

    record_sample(tick_time_s, state, applied);

    // 7. Hold, and integrate one period under it. The wind is the plant's, not
    // the controller's: it is read per stage exactly as `sim.plant` reads it.
    const numerics::DerivativeFunction derivative =
        [&](double time_s, const Eigen::VectorXd& x) -> Eigen::VectorXd {
      const Eigen::Vector3d wind_now =
          wind_history.empty() ? run.wind_ned_m_s : Eigen::Vector3d(wind_history.at(time_s));
      Eigen::VectorXd rate = model.derivative(x, applied, wind_now);
      if (freeze_battery) {
        rate(battery_index) = 0.0;
      }
      return rate;
    };
    const numerics::ProjectionFunction projection = [&](Eigen::VectorXd& x) { model.project(x); };
    const numerics::Trajectory piece = numerics::integrate_fixed_step(
        derivative, state, tick_time_s, step_s, steps_per_tick, steps_per_tick, projection);
    state = piece.states.back();
  }
  record_sample(static_cast<double>(steps) * step_s, state, applied);

  run.trajectory.step_s = step_s;
  run.trajectory.step_count = steps;
  run.trajectory.sample_stride = steps_per_tick;

  if (discrete_law != nullptr) {
    record.law_time_domain = "discrete_design";
    record.design_sample_time_s = discrete_law->sample_time_s;
    LinearPredictionRecord& prediction = record.prediction;
    prediction.chart_names = law_state_names;
    prediction.unavailable_reason = prediction_blocker;
    if (prediction_blocker.empty()) {
      // The same initial chart state the run started from, the same gain, the
      // same whole-period delay and the same hold — only the nonlinearity and
      // the actuator limits are missing from the prediction.
      const sim::SampledLoopPrediction predicted =
          sim::predict_sampled_loop(discrete_law->discretisation.system,
                                    gain,
                                    delay_periods,
                                    record.chart_error.front(),
                                    tick_count);
      Eigen::VectorXd reference_end = reference_state;
      if (reference_moves) {
        reference_end.segment<3>(core::kPositionNorth) +=
            trimmed.point.ground_velocity_ned_m_s * (static_cast<double>(tick_count) * period_s);
      }
      prediction.available = true;
      prediction.predicted_chart = predicted.states;
      prediction.measured_chart = record.chart_error;
      prediction.measured_chart.push_back(linearize::chart_from_extended(state, reference_end));

      // In the design's own cost-to-go norm; see `LinearPredictionRecord`.
      const Eigen::MatrixXd& cost_to_go = discrete_law->riccati.x;
      double worst = 0.0;
      double peak = 0.0;
      for (std::size_t k = 0; k < prediction.predicted_chart.size(); ++k) {
        const Eigen::VectorXd miss = prediction.measured_chart[k] - prediction.predicted_chart[k];
        const Eigen::VectorXd& expected = prediction.predicted_chart[k];
        const double miss_size = std::sqrt(std::fmax(0.0, miss.dot(cost_to_go * miss)));
        const double expected_size = std::sqrt(std::fmax(0.0, expected.dot(cost_to_go * expected)));
        if (miss_size > worst) {
          worst = miss_size;
          prediction.worst_tick = static_cast<int>(k);
        }
        peak = std::fmax(peak, expected_size);
      }
      prediction.relative_discrepancy_defined = peak > 0.0;
      prediction.relative_discrepancy = peak > 0.0 ? worst / peak : 0.0;
      const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(cost_to_go,
                                                                    Eigen::EigenvaluesOnly);
      const double largest = spectrum.eigenvalues().maxCoeff();
      prediction.cost_to_go_eigenvalue_ratio =
          largest > 0.0 ? spectrum.eigenvalues().minCoeff() / largest : 0.0;
      prediction.premise_violated_by_saturation = record.saturated_tick_count > 0;
      if (budget_value) {
        prediction.budget_declared = true;
        prediction.budget = agreement_budget;
        prediction.within_budget = prediction.relative_discrepancy_defined
                                   && prediction.relative_discrepancy <= agreement_budget;
      }
    }
  }

  std::ostringstream summary;
  summary << tick_count << " ticks at " << std::fixed << std::setprecision(1) << (1.0 / period_s)
          << " Hz, delay " << delay_periods << " period(s), "
          << (discrete_law != nullptr ? "discrete design at its own period"
                                      : "continuous design executed at a rate");
  if (record.saturated_tick_count > 0) {
    summary << "; saturated at " << record.saturated_tick_count << " tick(s), worst residual "
            << std::scientific << std::setprecision(2) << record.worst_saturation_residual_rad_s
            << " rad/s";
  } else {
    summary << "; no saturation";
  }
  if (run.battery_present) {
    summary << "; battery " << (freeze_battery ? "frozen" : "evolving");
  }
  const LinearPredictionRecord& prediction = record.prediction;
  if (prediction.available && prediction.relative_discrepancy_defined) {
    summary << "; nonlinear run differs from the discrete prediction by " << std::scientific
            << std::setprecision(2) << prediction.relative_discrepancy
            << " of its peak in the design's cost-to-go norm";
    if (prediction.budget_declared) {
      summary << (prediction.within_budget ? ", within" : ", OUTSIDE") << " the declared budget "
              << prediction.budget;
    }
    if (prediction.premise_violated_by_saturation) {
      summary << " (the run saturated, which the prediction does not model)";
    }
  }

  Artifact artifact;
  artifact.kind = "sampled_trajectory";
  artifact.summary = summary.str();
  artifact.payload = SampledRun{std::move(run), std::move(record)};
  return artifact;
}

// --- trim.hover ------------------------------------------------------------

Artifact trim_hover_capability(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("quadrotor");
  const auto& quadrotor_artifact = upstream.payload_as<QuadrotorArtifact>("quadrotor");
  const auto& model = quadrotor_artifact.model;

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

  HoverTrimArtifact trimmed{trim::trim_hover(model, request), model, quadrotor_artifact.identity};

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

// --- model.quadrotor.export ------------------------------------------------
//
// A model leaving galata as a file the loader reads back, with the record of
// where it came from beside it.
//
// THE EVIDENCE FILE IS NOT OPTIONAL, and that is the decision ADR-0018 records.
// `model::serialize_quadrotor` writes a file that is byte-for-byte the same KIND
// of object whether the numbers in it were measured on a bench or estimated by
// an optimiser, and nothing in the format can distinguish them. The repository
// already answers this for hand-written models — every directory under `models/`
// ships a `PROVENANCE.md` beside its YAML — and a model this tool writes is held
// to the same rule rather than a weaker one. So both paths are required, for a
// fitted model and for a loaded one alike: a uniform rule has no branch for a
// caller to take, and "no parameter was fitted; these are the bytes it came
// from" is itself worth recording.

void write_yaml_text(std::ostream& out, const std::string& key, std::string_view text) {
  if (text.empty()) {
    return;
  }
  out << key << ": \"";
  for (const char character : text) {
    if (character == '"' || character == '\\') {
      out << '\\';
    }
    // A control character cannot appear in a provenance field; the serialiser
    // refuses the same thing for the model file itself.
    out << (static_cast<unsigned char>(character) < 0x20 ? ' ' : character);
  }
  out << "\"\n";
}

std::string fitted_model_evidence(const QuadrotorArtifact& subject, const std::string& model_path) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10);

  out << "# SPDX-License-Identifier: Apache-2.0\n";
  out << "#\n";
  out << "# Written by `model.quadrotor.export`. This is the record that says what the model\n";
  out << "# file beside it is; the file itself cannot say, because a fitted model and a\n";
  out << "# measured one are the same kind of document. Not a study input: no capability\n";
  out << "# reads this back, and nothing downstream depends on its shape.\n\n";

  out << "model_file: \"" << model_path << "\"\n";
  out << "origin: \"" << subject.identity.origin << "\"\n";
  write_yaml_text(out, "produced_by", galata::build_identification());

  if (!subject.identity.is_fitted()) {
    out << "\n# Loaded from a file and not modified. Recorded so that an exported model always\n";
    out << "# carries an account of itself, whether or not anything was estimated.\n";
    out << "source:\n";
    write_yaml_text(out, "  path", subject.identity.path);
    write_yaml_text(out, "  sha256", subject.identity.sha256);
    out << "fitted_parameter_count: 0\n";
    return out.str();
  }

  const FittedModelProvenance& fit = *subject.identity.fit;
  out << "\nbase_model:\n";
  write_yaml_text(out, "  path", fit.base_model_path);
  write_yaml_text(out, "  sha256", fit.base_model_sha256);
  write_yaml_text(out, "  description", fit.base_model_description);
  out << "  # Not modified and not replaced. The file above is still what it was; the model\n";
  out << "  # beside this record is a new one whose unnamed parameters are that file's.\n";

  out << "\nestimation_record:\n";
  write_yaml_text(out, "  path", fit.estimation_record_path);
  write_yaml_text(out, "  sha256", fit.estimation_record_sha256);
  out << "  sample_count: " << fit.estimation_sample_count << "\n";
  out << "  first_sample_s: " << fit.estimation_first_sample_s << "\n";
  out << "  last_sample_s: " << fit.estimation_last_sample_s << "\n";
  out << "  is_window: " << (fit.estimation_record_is_window ? "true" : "false") << "\n";
  if (fit.estimation_record_is_window) {
    out << "  window_start_s: " << fit.estimation_window_start_s << "\n";
    out << "  window_end_s: " << fit.estimation_window_end_s << "\n";
  }

  out << "\nobjective:\n";
  write_yaml_text(out, "  definition", fit.objective_definition);
  out << "  step_s: " << fit.step_s << "\n";
  out << "  outputs:\n";
  for (const std::string& match : fit.output_matches) {
    out << "    - \"" << match << "\"\n";
  }
  out << "  command_channels: [";
  for (std::size_t k = 0; k < fit.command_channels.size(); ++k) {
    out << (k == 0 ? "" : ", ") << "\"" << fit.command_channels[k] << "\"";
  }
  out << "]\n";

  out << "\nfitted_parameters:\n";
  for (const FittedParameter& parameter : fit.fitted) {
    out << "  - path: \"" << parameter.path << "\"\n";
    out << "    unit: \"" << parameter.unit << "\"\n";
    out << "    value: " << parameter.value << "\n";
    out << "    initial: " << parameter.initial << "\n";
    out << "    lower: " << parameter.lower << "\n";
    out << "    upper: " << parameter.upper << "\n";
    if (parameter.standard_error_is_estimable) {
      out << "    standard_error: " << parameter.standard_error << "\n";
    } else {
      out << "    standard_error: null   # none could be estimated; see uncertainty below\n";
    }
    out << "    at_bound: " << (parameter.at_bound ? "true" : "false");
    if (parameter.at_bound) {
      out << "   # resting on a declared limit, which is not an interior estimate";
    }
    out << "\n";
  }

  // Written out in full rather than as "everything else": a reader asking which
  // numbers in the model file are measurements and which are inherited must be
  // able to answer it from this record alone.
  out << "\npreserved_parameters:   # carried from the base model, untouched by the fit\n";
  for (const std::string& path : fit.preserved_parameter_paths) {
    out << "  - \"" << path << "\"\n";
  }

  // The five questions, under five headings. A single `diagnostics` block with
  // them interleaved is how they get read as one verdict.
  out << "\nexecution:\n";
  out << "  iterations_declared: " << fit.iterations_declared << "\n";
  out << "  iterations_run: " << fit.iterations_run << "\n";
  write_yaml_text(out, "  stop_reason", fit.stop_reason);
  out << "  # One stopping rule exists. A tolerance-based exit is not missing here;\n";
  out << "  # ADR-0004 forbids it, because it stops after a different number of\n";
  out << "  # steps on a different machine.\n";

  out << "\nobjective_progress:\n";
  out << "  initial: " << fit.initial_objective << "\n";
  out << "  final: " << fit.objective << "\n";
  out << "  improved: " << (fit.objective_improved ? "true" : "false") << "\n";
  out << "  accepted_steps: " << fit.accepted_steps << "\n";
  out << "  residual_rms_scaled: " << fit.residual_rms << "\n";
  out << "  residual_count: " << fit.residual_count << "\n";

  out << "\nconvergence_evidence:   # evidence, NOT a verdict — there is no `converged` field\n";
  out << "  last_step_norm: " << fit.last_step_norm << "\n";
  out << "  last_accepted_iteration: " << fit.last_accepted_iteration << "\n";
  out << "  gradient_infinity_norm: " << fit.gradient_infinity_norm << "\n";
  out << "  gradient_over_bound_span_infinity_norm: " << fit.gradient_over_bound_span_infinity_norm
      << "\n";
  out << "  # J^T r at the FINAL point, with J recomputed there rather than reused\n";
  out << "  # from the last iteration's start. How near zero is near enough is a\n";
  out << "  # question about this fit's use, which nothing here answers.\n";

  out << "\nidentifiability:\n";
  out << "  jacobian_condition_number: " << fit.jacobian_condition_number << "\n";
  out << "  ratio_required: " << fit.identifiability_ratio << "\n";
  out << "  # A fit failing this test does not return at all; the refusal is the\n";
  out << "  # report. These figures describe a fit that passed it.\n";

  out << "\nuncertainty:\n";
  out << "  estimable: " << (fit.uncertainty_is_estimable ? "true" : "false") << "\n";
  write_yaml_text(out, "  assumptions", fit.uncertainty_assumptions);

  out << "\nwhat_this_is_not: \"A completed fit is not a validation and this model is not "
         "measured aircraft data. Whether the residual is small enough for any use is an "
         "engineering judgement nothing here makes. Run identify.validate against a record "
         "this model was not fitted to before treating it as predictive.\"\n";
  return out.str();
}

Artifact export_quadrotor(const StageContext& context) {
  const Artifact& upstream = context.upstream_at("model");
  const auto& subject = upstream.payload_as<QuadrotorArtifact>("quadrotor");

  const std::string path = context.input->string_at("path");
  const std::string evidence_path = context.input->string_at("evidence_path");
  if (path == evidence_path) {
    throw std::invalid_argument(
        "model.quadrotor.export: `path` and `evidence_path` name the same file. The model and "
        "the record of where it came from are two documents and one would overwrite the other");
  }
  context.write_output(path, model::serialize_quadrotor(subject.model));
  context.write_output(evidence_path, fitted_model_evidence(subject, path));

  std::ostringstream summary;
  summary << "wrote " << subject.model.rotor_count() << " rotors, "
          << subject.model.extended_state_size() << " states to " << path << ", with its "
          << (subject.identity.is_fitted() ? "fit record" : "source record") << " in "
          << evidence_path;

  Artifact artifact;
  artifact.kind = "quadrotor";
  artifact.summary = summary.str();
  // The model itself, unchanged and with its identity intact, so an export can
  // sit mid-chain rather than only at the end of one. `model.linear.export`
  // passes its system through for the same reason.
  artifact.payload = subject;
  return artifact;
}

}  // namespace

void register_quadrotor_capabilities(Registry& registry) {
  // ImplementedUnvalidated, for the reason `model.quadrotor` carries: the cases
  // behind these are exact invariants of the equations and a cross-check
  // against an independent implementation. Neither is a published reference.
  registry.add(Capability{
      "sim.sampled",
      "Execute a state-feedback law against the nonlinear plant — a continuous design at a "
      "declared rate, or a discrete design only at its own period — with zero-order hold, "
      "whole-period delay and per-rotor saturation",
      "sampled_trajectory",
      Capability::State::ImplementedUnvalidated,
      simulate_sampled_capability,
      {"trim",
       "law",
       "controller_period_s",
       "hold",
       "delay_periods",
       "linear_agreement_budget",
       "reference_motion",
       "initial_chart_perturbation",
       "wind_schedule",
       "step_s",
       "steps",
       "freeze_battery"}});

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

  registry.add(Capability{
      "model.quadrotor.export",
      "Write a multirotor as the YAML model.quadrotor reads, with a required record of where "
      "its numbers came from — which parameters were fitted, from what, and which were "
      "carried over untouched",
      "quadrotor",
      Capability::State::ImplementedUnvalidated,
      export_quadrotor,
      {"model", "path", "evidence_path"},
      {},
      {"path", "evidence_path"}});
}

}  // namespace galata::pipeline
