// SPDX-License-Identifier: Apache-2.0
//
// Fixed-step numerical integration.
//
// Reference:
//   E. Hairer, S. P. Norsett and G. Wanner, "Solving Ordinary Differential
//   Equations I: Nonstiff Problems", 2nd revised ed., Springer Series in
//   Computational Mathematics 8, Springer, 1993 — the classical Runge-Kutta
//   methods, their order conditions and their error behaviour.
//   J. C. Butcher, "Numerical Methods for Ordinary Differential Equations",
//   3rd ed., Wiley, 2016.
//
// WHY FIXED STEP IS THE DEFAULT, and the only integrator used for gated
// results: an adaptive method chooses its step sequence from an error estimate,
// which is a function of the last bits of the state. Two runs that differ by
// one ulp can therefore take different numbers of steps and diverge visibly.
// Reproducibility beats efficiency here, and ADR-0004 records the reasoning.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not adaptive, and not error-controlled. RK4 at a fixed step gives you no
//   estimate of its own error. The step size is the user's responsibility, and
//   a step too large for the fastest mode in the system produces a smooth,
//   plausible, wrong trajectory. Halving the step and comparing is the only
//   check this interface offers, and step_size_study() exists to make it easy.
//
// * Not stiff-capable. RK4's stability region on the negative real axis extends
//   to about h*lambda = -2.78. An actuator with a 100 rad/s pole therefore needs
//   h < 28 ms merely to remain stable, well before accuracy is considered. A
//   system with modes separated by more than about three decades will be
//   impractical rather than merely slow.
//
// * Not symplectic. Energy in a conservative system drifts secularly rather
//   than oscillating about the true value. Over the tens of seconds a flight
//   simulation runs this is far below every other error; over an orbit it is
//   not, and this integrator is the wrong tool for that.

#ifndef GALATA_NUMERICS_INTEGRATOR_HPP
#define GALATA_NUMERICS_INTEGRATOR_HPP

#include <Eigen/Core>

#include <functional>
#include <vector>

namespace galata::numerics {

// dx/dt = f(t, x). Time is passed even though most galata systems are
// autonomous, because disturbances and commanded inputs are not.
using DerivativeFunction = std::function<Eigen::VectorXd(double t, const Eigen::VectorXd& x)>;

// Applied to the state after every completed step, never between stages.
//
// This exists for the quaternion: RK4 integrates the four components
// independently and the result is not exactly a unit quaternion, so it is
// renormalised once the step is complete. Applying it between stages would
// change the method's order conditions and silently make RK4 something other
// than fourth order.
using ProjectionFunction = std::function<void(Eigen::VectorXd&)>;

// One classical RK4 step. The Butcher tableau is the standard one:
//
//   0   |
//   1/2 | 1/2
//   1/2 | 0    1/2
//   1   | 0    0    1
//   ----+----------------------
//       | 1/6  1/3  1/3  1/6
//
[[nodiscard]] Eigen::VectorXd rk4_step(const DerivativeFunction& derivative,
                                       double time_s,
                                       const Eigen::VectorXd& state,
                                       double step_s);

struct Trajectory {
  std::vector<double> times_s;          // s
  std::vector<Eigen::VectorXd> states;  // one entry per recorded sample
  double step_s = 0.0;                  // s, the integration step used
  int step_count = 0;                   // total steps taken
  int sample_stride = 1;                // steps between recorded samples
};

// Integrate for exactly `step_count` steps.
//
// The step COUNT is the argument, not a duration. Deriving a count from
// duration/step is a floating-point division whose result can land either side
// of an integer boundary depending on the values involved, which would make the
// number of steps — and therefore the answer — depend on rounding. ADR-0004
// requires that it not.
//
// Sample times are computed as t0 + k*step rather than accumulated, so the
// reported time does not drift over a long run.
[[nodiscard]] Trajectory integrate_fixed_step(const DerivativeFunction& derivative,
                                              const Eigen::VectorXd& initial_state,
                                              double initial_time_s,
                                              double step_s,
                                              int step_count,
                                              int sample_stride = 1,
                                              const ProjectionFunction& projection = nullptr);

struct StepSizeStudy {
  double step_s = 0.0;  // s, the coarse step h

  Eigen::VectorXd final_at_h;    // final state integrated at h
  Eigen::VectorXd final_at_h_2;  // ... at h/2
  Eigen::VectorXd final_at_h_4;  // ... at h/4

  double difference_h_to_h2 = 0.0;   // ||x_h  - x_h/2||
  double difference_h2_to_h4 = 0.0;  // ||x_h/2 - x_h/4||

  // Richardson extrapolation, with p = 4.
  //
  // For a method of order p, x_h - exact = C h^p, so
  //   x_h - x_h/2 = C h^p (1 - 2^-p)
  // and therefore
  //   error(x_h/2) = ||x_h - x_h/2|| / (2^p - 1)
  //   error(x_h)   = ||x_h - x_h/2|| * 2^p / (2^p - 1)
  //
  // The factor differs between the two by 2^p; conflating them is the usual way
  // to be sixteen times wrong about your own accuracy, so both are reported
  // with their step size named.
  double estimated_error_at_h = 0.0;
  double estimated_error_at_h_2 = 0.0;

  // log2(||x_h - x_h/2|| / ||x_h/2 - x_h/4||). Near 4 when RK4 is in its
  // asymptotic regime.
  //
  // Well below 4 means the step is too large for the error expansion to hold
  // and the estimates above should not be believed. Well ABOVE 4 usually means
  // the differences have reached rounding noise and the study needs a larger
  // step, not a smaller one.
  double observed_order = 0.0;
};

// Integrate the same problem at h, h/2 and h/4 and report whether the method is
// behaving like a fourth-order method on this problem.
//
// This is the only error information a fixed-step integrator can honestly
// offer, so the interface makes it cheap rather than leaving each caller to
// improvise it.
[[nodiscard]] StepSizeStudy step_size_study(const DerivativeFunction& derivative,
                                            const Eigen::VectorXd& initial_state,
                                            double initial_time_s,
                                            double coarse_step_s,
                                            int coarse_step_count,
                                            const ProjectionFunction& projection = nullptr);

}  // namespace galata::numerics

#endif  // GALATA_NUMERICS_INTEGRATOR_HPP
