// SPDX-License-Identifier: Apache-2.0
//
// Continuous linear state response using fixed-step classical RK4, under a
// constant input or a declared input history.
// Reference: Hairer, Norsett & Wanner, Solving Ordinary Differential Equations I,
// 2nd revised ed., Springer, 1993, stability functions of Runge-Kutta methods.
// WHAT THIS IS NOT: no adaptive error control, no discrete system support, no
// guarantee against nonnormal transient amplification, and no resampling or
// smoothing of a declared history. A step outside RK4's stability region for any
// stable mode is rejected; accuracy still requires a step-halving study.
// Unstable modes retain their physical growth.
#ifndef GALATA_SIM_LINEAR_HPP
#define GALATA_SIM_LINEAR_HPP

#include "galata/model/linear_system.hpp"
#include "galata/numerics/integrator.hpp"
#include "galata/sim/schedule.hpp"

#include <vector>

namespace galata::sim {

[[nodiscard]] numerics::Trajectory simulate_linear(const model::LinearSystem& system,
                                                   const Eigen::VectorXd& initial_state,
                                                   const Eigen::VectorXd& constant_input,
                                                   double step_s,   // s
                                                   int step_count,  // steps
                                                   int sample_stride = 1);

// A linear response driven by a DECLARED INPUT HISTORY — the `lsim` case. The
// semantics are the ones `sim.plant` already applies to the nonlinear plant, so
// a schedule's TIMING means the same thing on either path. Its values are each
// model's own inputs: absolute rotor speeds for the plant, deviations from the
// trim for a linearisation about it.
//
//   TIMESTAMPS    Seconds from the start of the run: the initial state is at
//                 t = 0 and the run ends at t = step_s * step_count. A history
//                 may start before 0 or end after the horizon; only the part
//                 the run crosses is read.
//   ZERO-ORDER    Each value is held from its sample until the next. A change of
//                 value is an EVENT: it must fall on an integration step boundary
//                 (to 1e-9 of a whole number of steps, relative) and is refused
//                 otherwise, because an RK4 step straddling a jump is first order
//                 across it and rounding the event to a step moves it silently.
//                 The value in force across a segment is resolved ONCE, at the
//                 event's own time, so all four RK4 stages of every step see it.
//   LINEAR        Piecewise linear between samples, evaluated at every RK4 stage.
//                 A change of SLOPE inside a step costs RK4 its order across that
//                 step, exactly as in `sim.plant`; it is not refused, and a caller
//                 who needs the full order aligns the sample times to the step.
//   OUTSIDE       The schedule's declared extrapolation. Under `refuse`, a run
//                 whose horizon leaves the sampled span is refused before any
//                 step is taken, so there is no partial trajectory.
//   RECORDING     The input recorded at each sample is the value IN FORCE AFTER
//                 any event at that instant — the one the next step integrates
//                 under. Output feedthrough D u must use it, not a constant.
//
// A history that holds one value throughout reproduces the constant-input
// overload bit for bit.
struct ScheduledLinearRun {
  numerics::Trajectory trajectory;
  // One entry per recorded sample, right-continuous at events; see above.
  std::vector<Eigen::VectorXd> input_samples;
  // Integration step indices at which a zero-order value changed inside the
  // run. Each is a segment boundary; empty under a linear hold.
  std::vector<int> event_steps;
};

// Refuses everything the constant-input overload refuses, and in addition: an
// empty history; a system with no inputs; a history whose width is not the
// system's input count; a zero-order event strictly inside the run that does
// not fall on a step boundary; and, under `refuse`, a horizon outside the span.
[[nodiscard]] ScheduledLinearRun simulate_linear(const model::LinearSystem& system,
                                                 const Eigen::VectorXd& initial_state,
                                                 const InputSchedule& input,
                                                 double step_s,   // s
                                                 int step_count,  // steps
                                                 int sample_stride = 1);

}  // namespace galata::sim
#endif
