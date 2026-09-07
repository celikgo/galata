// SPDX-License-Identifier: Apache-2.0
//
// Continuous linear state response using fixed-step classical RK4.
// Reference: Hairer, Norsett & Wanner, Solving Ordinary Differential Equations I,
// 2nd revised ed., Springer, 1993, stability functions of Runge-Kutta methods.
// WHAT THIS IS NOT: no adaptive error control, input interpolation, discrete
// system support or guarantee against nonnormal transient amplification. A step
// outside RK4's stability region for any stable mode is rejected; accuracy still
// requires a step-halving study. Unstable modes retain their physical growth.
#ifndef GALATA_SIM_LINEAR_HPP
#define GALATA_SIM_LINEAR_HPP

#include "galata/model/linear_system.hpp"
#include "galata/numerics/integrator.hpp"

namespace galata::sim {

[[nodiscard]] numerics::Trajectory simulate_linear(const model::LinearSystem& system,
                                                   const Eigen::VectorXd& initial_state,
                                                   const Eigen::VectorXd& constant_input,
                                                   double step_s,   // s
                                                   int step_count,  // steps
                                                   int sample_stride = 1);

}  // namespace galata::sim
#endif
