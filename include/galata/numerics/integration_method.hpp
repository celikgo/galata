// SPDX-License-Identifier: Apache-2.0
//
// A chosen, deterministic, fixed-step integration method — and a bound on the
// states it produces.
//
// Reference:
//   E. Hairer, S. P. Norsett and G. Wanner, "Solving Ordinary Differential
//   Equations I: Nonstiff Problems", 2nd revised ed., Springer, 1993.
//   E. Hairer and G. Wanner, "Solving Ordinary Differential Equations II:
//   Stiff and Differential-Algebraic Problems", 2nd revised ed., Springer,
//   1996, chapters IV.3 and IV.7 — A-stability, the implicit Euler and
//   trapezoidal methods, and simplified Newton iteration on the stage equation.
//   J. C. Butcher, "Numerical Methods for Ordinary Differential Equations",
//   3rd ed., Wiley, 2016.
//
// WHY THIS EXISTS ALONGSIDE integrator.hpp, WHICH IS UNCHANGED.
//
// `integrator.hpp` holds classical RK4 and its exact summation expression, and
// ADR-0004's bit-identity tier is a claim about that expression. Nothing here
// touches it: `Rk4Fixed` below CALLS it, so a study that does not ask for
// another method gets the same bits it got before this file existed.
//
// What this adds is a method CHOICE, for one reason. RK4's stability region on
// the negative real axis reaches h*lambda = -2.78, so a mode at 100 rad/s needs
// h < 28 ms merely to remain stable. A helicopter carries rigid-body modes at
// O(0.1-1) rad/s, rotor speed and governor at O(1-10), dynamic inflow and
// actuators at O(10-100) and flapping at O(100) — three decades, which
// `integrator.hpp` names as the point where an explicit method becomes
// "impractical rather than merely slow". The methods here are A-stable: their
// step is chosen for ACCURACY, not for stability.
//
// DETERMINISM IS PRESERVED, AND ADR-0004 IS NOT WEAKENED. The prohibition there
// is on ADAPTIVE stepping, because a step sequence chosen from an error
// estimate is a function of the last bits of the state. An IMPLICIT method with
// a FIXED step and a FIXED iteration count is not adaptive: it performs exactly
// the same arithmetic in the same order on every run. There is no tolerance
// exit anywhere in this file, and `newton_iterations` is a count, not a budget.
// ADR-0021 records the distinction.
//
// A BOUND ON THE STATE, AND WHY IT IS NOT OPTIONAL IN PRACTICE.
//
// `rk4_step` throws only on a NON-FINITE state. A stiff instability that grows
// to 1e246 is finite, so it is returned as a trajectory and the capability
// layer reports "completed". That is the failure this file's `StateBounds`
// exists to catch: a declared per-state magnitude, checked after every step,
// which terminates the run with the OFFENDING STATE NAMED rather than handing
// back a plausible CSV of a divergence.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not adaptive, and not error-controlled. Neither method estimates its own
//   error. `step_size_study()` in integrator.hpp remains the only honest error
//   information on offer, and it works on these methods too.
//
// * Not symplectic, and the trapezoidal rule's energy behaviour is not a
//   conservation property. It is the implicit midpoint rule that conserves
//   quadratic invariants; the trapezoidal rule does not, and over an orbit
//   these are the wrong tools. Over the tens of seconds a flight simulation
//   runs, the difference is far below every other error.
//
// * Not L-stable in the trapezoidal case. Trapezoidal is A-stable but its
//   stability function tends to -1 as h*lambda -> -infinity, so a very stiff
//   mode RINGS at the step frequency instead of decaying. Implicit Euler is
//   L-stable and damps it, at the cost of first-order accuracy. Both are
//   offered because the right answer depends on whether the stiff mode carries
//   information or only stability, and only the model's author knows that.
//
// * Not a DAE solver, and not an algebraic-loop solver. The stage equation is
//   solved for the state, never for a constraint.
//
// * Not Jacobian-free. Each implicit step forms a numerical Jacobian, which for
//   a state of dimension n costs n extra derivative evaluations. For the tens
//   of states a rotorcraft model carries this is the right trade; for thousands
//   it is not.

#ifndef GALATA_NUMERICS_INTEGRATION_METHOD_HPP
#define GALATA_NUMERICS_INTEGRATION_METHOD_HPP

#include "galata/numerics/integrator.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace galata::numerics {

enum class IntegrationMethod {
  // Classical fourth-order Runge-Kutta. The default, and the only method used
  // for gated results unless a study says otherwise. Explicit, not stiff-capable.
  Rk4Fixed,
  // Implicit Euler. First order, A-stable AND L-stable: a stiff mode is damped
  // rather than rung. The method to choose when the fast dynamics carry no
  // information you need and you want them out of the way.
  ImplicitEulerFixed,
  // Trapezoidal rule (implicit, second order, A-stable). More accurate than
  // implicit Euler and the usual choice when the fast modes matter.
  TrapezoidalFixed,
};

[[nodiscard]] std::string to_string(IntegrationMethod method);

// The nominal order of the method, for a step-size study to compare against.
[[nodiscard]] int method_order(IntegrationMethod method);

[[nodiscard]] bool method_is_implicit(IntegrationMethod method);

// A declared magnitude bound per state, checked after every completed step.
//
// EMPTY MEANS "FINITE ONLY", WHICH IS THE OLD BEHAVIOUR. A default-constructed
// StateBounds checks that the state is finite and nothing else, so an existing
// caller that does not supply one gets exactly what it got before.
class StateBounds {
 public:
  StateBounds() = default;

  // One magnitude per state, in the state's own SI units. A non-positive or
  // non-finite entry means "unbounded" for that state, which is how a caller
  // says it wants a bound on the rotor speed and none on the position.
  StateBounds(std::vector<std::string> names, Eigen::VectorXd magnitudes);

  [[nodiscard]] bool empty() const noexcept {
    return magnitudes_.size() == 0;
  }

  [[nodiscard]] Eigen::Index size() const noexcept {
    return magnitudes_.size();
  }

  [[nodiscard]] const std::vector<std::string>& names() const noexcept {
    return names_;
  }

  [[nodiscard]] const Eigen::VectorXd& magnitudes() const noexcept {
    return magnitudes_;
  }

  // Empty when the state is acceptable. Otherwise a sentence naming the state,
  // its value and its bound — which is the whole point: "diverged" is not a
  // diagnosis, "rotor_speed_0 reached 4.1e+03 rad/s against a bound of 6.0e+01"
  // is.
  [[nodiscard]] std::string violation(const Eigen::VectorXd& state) const;

 private:
  std::vector<std::string> names_;
  Eigen::VectorXd magnitudes_;
};

struct IntegrationOptions {
  IntegrationMethod method = IntegrationMethod::Rk4Fixed;
  double step_s = 0.0;    // s, positive
  int step_count = 0;     // number of fixed steps, non-negative
  int sample_stride = 1;  // steps between recorded samples

  // FIXED, not a budget, and not a tolerance. Every implicit step performs
  // exactly this many Newton iterations whatever the residual, because an exit
  // condition read off the residual is an exit condition read off the last bits
  // of the state. Three is the usual choice for a simplified Newton iteration
  // on a stage equation; two is enough when h is small relative to the stiff
  // time constant, and more than four indicates the step is too large.
  int newton_iterations = 3;

  // Refresh the stage Jacobian every this many steps. 1 is a full Newton
  // iteration; larger values are the "simplified Newton" of Hairer and Wanner
  // II.IV.8, which reuses a factorisation while the Jacobian changes slowly.
  // Deterministic either way: the schedule is by step INDEX, never by residual.
  int jacobian_refresh_steps = 1;
};

// Why a run stopped. `Completed` is the only outcome that means the trajectory
// covers the requested span.
enum class TerminationReason {
  Completed,
  // A declared magnitude bound was exceeded. `detail` names the state.
  BoundExceeded,
  // The state or a derivative stopped being finite.
  NonFinite,
  // The derivative function itself threw. `detail` carries its message.
  DerivativeFailed,
};

[[nodiscard]] std::string to_string(TerminationReason reason);

struct IntegrationResult {
  Trajectory trajectory;
  IntegrationMethod method = IntegrationMethod::Rk4Fixed;
  TerminationReason reason = TerminationReason::Completed;

  // Empty when `reason` is Completed. Otherwise the sentence that goes in the
  // report, the manifest and the CLI diagnostic, unchanged, so the three cannot
  // disagree about why a run stopped.
  std::string detail;

  // The step at which it stopped, and the time there. Equal to step_count and
  // the final time on a completed run.
  int steps_taken = 0;
  double termination_time_s = 0.0;  // s

  // Total implicit Newton iterations performed. Zero for an explicit method.
  // Reported rather than hidden because it is the cost of the method choice.
  long long newton_iterations_performed = 0;

  [[nodiscard]] bool completed() const noexcept {
    return reason == TerminationReason::Completed;
  }
};

// Integrate for exactly `step_count` steps, or until a bound is exceeded.
//
// The step COUNT is the argument, not a duration, for the reason
// integrator.hpp gives: deriving a count from duration/step is a floating-point
// division whose result can land either side of an integer boundary.
//
// On a bound violation or a non-finite state the trajectory returned holds the
// samples up to and INCLUDING the last good one, and `reason` says why it
// stopped. It does not throw: a divergence is a result about the model, and a
// caller that wants an exception can check `completed()`.
//
// Throws std::invalid_argument for a malformed request — a non-positive step, a
// negative count, a stride below one, an empty or non-finite initial state, a
// bounds vector whose length does not match the state, or a non-positive
// Newton iteration count on an implicit method.
[[nodiscard]] IntegrationResult integrate(const DerivativeFunction& derivative,
                                          const Eigen::VectorXd& initial_state,
                                          double initial_time_s,
                                          const IntegrationOptions& options,
                                          const ProjectionFunction& projection = nullptr,
                                          const StateBounds& bounds = {});

// One step of the chosen method. Exposed because a sampled controller drives
// the plant one step at a time and must be able to choose the same method.
//
// For Rk4Fixed this forwards to rk4_step() unchanged, so the bits are the same.
[[nodiscard]] Eigen::VectorXd integration_step(IntegrationMethod method,
                                               const DerivativeFunction& derivative,
                                               double time_s,
                                               const Eigen::VectorXd& state,
                                               double step_s,
                                               int newton_iterations,
                                               long long* iterations_performed = nullptr);

}  // namespace galata::numerics

#endif  // GALATA_NUMERICS_INTEGRATION_METHOD_HPP
