// SPDX-License-Identifier: Apache-2.0
//
// Trim as a DECLARED problem: a set of unknowns, a set of residuals, and
// optional constraints — solved by one Newton on a square system.
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 3 — trim as the solution of the
//   steady-state equations, and the constrained formulations for turns and
//   climbs.
//   G. D. Padfield, "Helicopter Flight Dynamics", 2nd ed., Blackwell, 2007,
//   section 4.2 — the helicopter trim problem, its six unknowns and why the
//   fixed-wing unknown set cannot express it.
//
// WHY THIS EXISTS. `trim/level.hpp` solves angle of attack, elevator and thrust;
// `trim/hover.hpp` solves two attitude angles and N rotor speeds. Both
// HARD-CODE their unknowns, and that is why a two-rotor helicopter could not be
// trimmed: the refusal "this solve needs exactly 4 rotors" is not a statement
// about helicopters, it is a statement about a fixed unknown vector. A
// helicopter's unknowns are roll, pitch, collective, longitudinal cyclic,
// lateral cyclic and pedal — six for six equations, a perfectly well-posed
// problem that neither existing solver can express at any effort.
//
// Declaring the problem also makes climb, descent, coordinated turn and
// autorotation the same solver with a different declaration, rather than four
// more hard-coded solvers.
//
// WHAT IS SOLVED, AND WHAT IS DECLARED. The unknowns are what Newton moves. The
// residuals are what must vanish. Everything else — heading, altitude, the
// position rate, a commanded turn rate — is DECLARED and is an input to the
// problem, not an output of it. Solving for a quantity the equations do not
// constrain is solving a singular system and reporting whichever point the
// iteration wandered into, and this refuses instead.
//
// RANK IS CHECKED BEFORE THE ANSWER IS BELIEVED. A square system is not
// necessarily a solvable one. The Jacobian's rank and condition number are
// computed and reported, and a rank-deficient problem is refused with the
// unknowns the residuals do not constrain NAMED — the diagnostic pattern
// `identify.greybox` already uses for a parameter the data does not constrain.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not an optimiser. This finds a ROOT of a square residual system. Bounds are
//   checked after the solve and a trim outside them is REFUSED, not projected
//   onto them: a best effort reported as a trim gets linearised, and a
//   linearisation about a non-equilibrium carries a constant term the A matrix
//   cannot represent. That is the same choice trim_level and trim_hover make.
//
// * Not a global solver. Newton from one initial guess finds one root. A
//   helicopter's trim map has more than one solution in some conditions — high
//   speed, where two collective settings give the same thrust — and this
//   returns whichever the guess was nearest. The guess is the caller's.
//
// * Not a claim that the equilibrium is stable, reachable or unique. It is a
//   root of the steady-state equations. What happens when it is disturbed is
//   what the linearisation is for.
//
// * Not constrained in the optimisation sense. `TrimConstraint` adds an
//   equation and a matching unknown must be removed, keeping the system square.
//   It is not a Lagrange multiplier and it is not an inequality.

#ifndef GALATA_TRIM_PROBLEM_HPP
#define GALATA_TRIM_PROBLEM_HPP

#include "galata/model/vehicle.hpp"

#include <Eigen/Core>

#include <functional>
#include <string>
#include <vector>

namespace galata::trim {

// What the solver is allowed to move.
//
// An unknown names a STATE or a CONTROL of the vehicle, by the vehicle's own
// vocabulary. Resolving by name rather than by index is what lets a problem
// declaration be written once and applied to any model that has those names.
struct TrimUnknown {
  std::string name;          // must appear in state_names() or control_names()
  double initial_guess = 0.0;
  double minimum = 0.0;      // checked AFTER the solve; equal bounds mean unbounded
  double maximum = 0.0;
  // Scale for the Newton step and the Jacobian's conditioning. An unknown in
  // radians and one in rad/s differ by two orders of magnitude, and a Jacobian
  // mixing them is ill-conditioned for reasons that have nothing to do with the
  // physics. Defaults to 1.
  double scale = 1.0;

  [[nodiscard]] bool bounded() const noexcept {
    return maximum > minimum;
  }
};

// What must vanish.
enum class TrimResidualKind {
  // The body-axis acceleration components, from the rigid-body kernel.
  BodyForceX,
  BodyForceY,
  BodyForceZ,
  BodyMomentX,
  BodyMomentY,
  BodyMomentZ,
  // The rate of one of the model's auxiliary states, by name. A helicopter's
  // rotor speed must be stationary in trim, and this is how that is said.
  AuxiliaryRate,
  // A caller-supplied scalar of the trim point. The escape hatch for a
  // condition the enumeration does not cover, used by the turn and climb
  // constraints below.
  Custom,
};

struct TrimResidual {
  TrimResidualKind kind = TrimResidualKind::BodyForceX;
  std::string name;  // for AuxiliaryRate: the state's name. For Custom: a label.

  // Required for Custom, ignored otherwise. Takes the extended state and the
  // controls, returns the quantity that must be zero.
  std::function<double(const Eigen::VectorXd& extended_state,
                       const Eigen::VectorXd& controls)>
      evaluate;

  // Scale, for the same reason TrimUnknown has one: a force residual in newtons
  // and a moment residual in newton-metres are not comparable, and a Newton
  // step that treats them as such is dominated by whichever has the larger
  // units. Defaults to 1.
  double scale = 1.0;
};

// A declared flight condition. Everything here is an INPUT: the solver does not
// move any of it.
struct TrimCondition {
  // The extended state the unknowns are applied to. Entries the unknowns do not
  // name are held exactly as supplied, which is how heading, altitude and the
  // commanded airspeed enter the problem.
  Eigen::VectorXd extended_state;
  Eigen::VectorXd controls;
  model::Environment environment;
};

struct TrimOptions {
  // FIXED, not a budget. ADR-0004: an exit condition read off the residual is
  // an exit condition read off the last bits of the state.
  int iterations = 60;
  double residual_tolerance = 1.0e-8;  // the GATE, checked once at the end
  // Relative step for the finite-difference Jacobian, scaled by each unknown's
  // own `scale`. eps^(1/3) for binary64, as numerics/jacobian.hpp derives.
  double jacobian_relative_step = 6.055454452393343e-06;
  // Rank is declared deficient when a singular value falls below this fraction
  // of the largest. 1e-9 matches analyze/gramians.
  double rank_relative_floor = 1.0e-9;
  // Newton step damping. 1.0 is full Newton; below 1 trades convergence rate
  // for robustness on a stiff residual map. Fixed, never line-searched.
  double step_fraction = 1.0;
};

struct TrimProblem {
  std::vector<TrimUnknown> unknowns;
  std::vector<TrimResidual> residuals;

  // Throws std::invalid_argument unless the problem is square, every unknown
  // names something the model has, no unknown or residual is named twice, and
  // every Custom residual carries an evaluator.
  void validate(const model::VehicleModel& model) const;
};

struct TrimResult {
  bool converged = false;
  Eigen::VectorXd extended_state;  // the trimmed state
  Eigen::VectorXd controls;        // the trimmed controls

  // Solved values, in the problem's own unknown order and by name, so a report
  // does not have to know the layout.
  std::vector<std::string> unknown_names;
  Eigen::VectorXd unknown_values;

  double residual_norm = 0.0;
  double residual_tolerance = 0.0;
  int iterations = 0;

  // JACOBIAN DIAGNOSTICS, reported whether or not the solve converged, because
  // they are what says WHY it did not.
  int jacobian_rank = 0;
  int unknown_count = 0;
  double jacobian_condition_number = 0.0;
  // Unknowns lying in the Jacobian's null space: the ones the residuals do not
  // constrain. Empty on a full-rank problem.
  std::vector<std::string> unconstrained_unknowns;

  // Unknowns that finished outside their declared bounds. A trim with any of
  // these is REFUSED rather than returned with a note.
  std::vector<std::string> out_of_bounds_unknowns;

  model::EnvelopeStatus envelope;
};

// Solve. Throws std::invalid_argument for a malformed problem, and
// std::runtime_error with a diagnostic naming the cause when the solve does not
// produce a usable trim — a residual over budget, a rank-deficient Jacobian, or
// an unknown outside its bounds.
//
// The refusal message carries the residual norm, the budget, the iteration
// count, the Jacobian rank and its condition number, because "did not converge"
// is not a diagnosis and those five numbers usually are.
[[nodiscard]] TrimResult solve_trim(const model::VehicleModel& model,
                                    const TrimProblem& problem,
                                    const TrimCondition& condition,
                                    const TrimOptions& options = {});

// ---------------------------------------------------------------------------
// Ready-made problems. Each is a DECLARATION, not a solver: they build a
// TrimProblem and hand it to solve_trim above.
// ---------------------------------------------------------------------------

// A helicopter in equilibrium: six unknowns, six residuals.
//
//   unknowns  = [ roll, pitch, collective, longitudinal cyclic, lateral cyclic, pedal ]
//   residuals = [ force x, y, z, moment x, y, z ]
//
// Yaw is DECLARED, not solved: a helicopter in still air is in equilibrium at
// any heading, so solving for it would be solving a singular system. The
// rotor-speed and inflow states are held at the values the condition supplies
// and their rates are NOT in the residual — the governor holds rotor speed and
// the inflow settles, and including them would make the system non-square.
[[nodiscard]] TrimProblem helicopter_trim_problem(const model::VehicleModel& model);

// The same six unknowns, with the rotor-speed rate added as a seventh residual
// and the engine torque... no: rotor speed is held by the governor, so the
// seventh equation would need a seventh unknown. Provided instead as an
// explicit check: after a trim, is the rotor-speed rate acceptably small?
[[nodiscard]] double rotor_speed_residual(const model::VehicleModel& model,
                                          const TrimResult& trim,
                                          const model::Environment& environment,
                                          const std::string& rotor_speed_state_name);

}  // namespace galata::trim

#endif  // GALATA_TRIM_PROBLEM_HPP
