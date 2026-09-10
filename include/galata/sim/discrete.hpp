// SPDX-License-Identifier: Apache-2.0
//
// The response a discrete design PREDICTS for its own sampled loop: the
// discrete plant, the static gain, and a whole-period delay line between them,
// iterated tick by tick from a declared initial state.
//
//   u[k]     = -K x[k]                      the law, at tick k
//   v[k]     = u[k - d]   for k >= d        what the plant receives
//            = 0          for k <  d        the delay line starts at trim
//   x[k + 1] = A x[k] + B v[k]              the plant, held across the tick
//
// Every quantity is a DEVIATION from the operating point the plant was
// linearised about, so "zero" in the delay line is the trim command and not a
// switched-off actuator.
//
// References:
//   K. J. Astrom and B. Wittenmark, "Computer-Controlled Systems: Theory and
//   Design", 3rd ed., 1997, section 2.3 — a delay that is a whole number of
//   sample periods is represented EXACTLY by d extra states holding the past
//   inputs, which is what the queue below is.
//   G. F. Franklin, J. D. Powell and M. L. Workman, "Digital Control of
//   Dynamic Systems", 3rd ed., 1998, chapter 4.
//
// WHY THIS EXISTS. `sim.sampled` flies a discrete design against the NONLINEAR
// plant. The comparison worth making is against what the design itself says
// the loop will do — the same hold, the same period and the same delay, with
// only the nonlinearity missing — because a disagreement between those two is
// then attributable to the one thing that differs. Comparing against a
// continuous simulation instead would mix the sampling into the discrepancy.
//
// WHAT THIS IS NOT
// * NOT A MODEL OF SATURATION. The loop is linear; an actuator limit reached in
//   the real run is a premise this prediction does not share, and a comparison
//   across a saturated tick is outside it.
// * NOT A FRACTIONAL DELAY. Only whole periods are representable here, exactly
//   as in the schedule `sim.sampled` executes. A delay of 1.5 periods is a
//   different plant, not a rounding of this one.
// * NOT THE INTER-SAMPLE RESPONSE. The states are at the ticks and nowhere else.
// * NOT A STABILITY OR ROBUSTNESS STATEMENT. A trajectory that decays is one
//   trajectory from one initial state; it is neither a margin nor a proof.
#ifndef GALATA_SIM_DISCRETE_HPP
#define GALATA_SIM_DISCRETE_HPP

#include "galata/model/discrete_system.hpp"

#include <Eigen/Core>

#include <vector>

namespace galata::sim {

struct SampledLoopPrediction {
  // x[0] .. x[tick_count]: one entry per tick and one after the last hold.
  std::vector<Eigen::VectorXd> states;
  // v[0] .. v[tick_count - 1]: the input the plant received across each hold.
  std::vector<Eigen::VectorXd> applied_inputs;
  int delay_periods = 0;
  double sample_time_s = 0.0;  // s, the plant's own
};

// Refuses a plant that does not validate, a gain whose shape is not
// inputs x states, a negative delay, a tick count below one, and an initial
// state of the wrong length or with a non-finite entry.
[[nodiscard]] SampledLoopPrediction predict_sampled_loop(const model::DiscreteLinearSystem& plant,
                                                         const Eigen::MatrixXd& gain,
                                                         int delay_periods,
                                                         const Eigen::VectorXd& initial_state,
                                                         int tick_count);

}  // namespace galata::sim

#endif  // GALATA_SIM_DISCRETE_HPP
