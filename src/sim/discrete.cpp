// SPDX-License-Identifier: Apache-2.0
//
// Astrom and Wittenmark (1997), section 2.3: a whole-period delay is a queue
// of past inputs. The queue here is the same circular buffer `sim.sampled`
// executes, written independently of it, so that the prediction and the run
// agree on the ordering of law, delay and hold by construction of the
// schedule and not by sharing code.

#include "galata/sim/discrete.hpp"

#include <sstream>
#include <stdexcept>

namespace galata::sim {

SampledLoopPrediction predict_sampled_loop(const model::DiscreteLinearSystem& plant,
                                           const Eigen::MatrixXd& gain,
                                           int delay_periods,
                                           const Eigen::VectorXd& initial_state,
                                           int tick_count) {
  plant.validate();
  const Eigen::Index states = plant.state_count();
  const Eigen::Index inputs = plant.input_count();
  if (inputs == 0) {
    throw std::invalid_argument(
        "predict_sampled_loop: the plant has no inputs, so there is no loop to predict");
  }
  if (gain.rows() != inputs || gain.cols() != states || !gain.allFinite()) {
    std::ostringstream message;
    message << "predict_sampled_loop: the gain is " << gain.rows() << "x" << gain.cols()
            << " and must be a finite " << inputs << "x" << states
            << " matrix, one row per plant input and one column per state";
    throw std::invalid_argument(message.str());
  }
  if (delay_periods < 0) {
    throw std::invalid_argument(
        "predict_sampled_loop: the delay must be a whole number of periods, zero or more");
  }
  if (tick_count < 1) {
    throw std::invalid_argument("predict_sampled_loop: at least one tick is required");
  }
  if (initial_state.size() != states || !initial_state.allFinite()) {
    std::ostringstream message;
    message << "predict_sampled_loop: the initial state has " << initial_state.size()
            << " entries and must be a finite vector of " << states;
    throw std::invalid_argument(message.str());
  }

  SampledLoopPrediction prediction;
  prediction.delay_periods = delay_periods;
  prediction.sample_time_s = plant.sample_time_s;
  prediction.states.reserve(static_cast<std::size_t>(tick_count) + 1);
  prediction.applied_inputs.reserve(static_cast<std::size_t>(tick_count));

  // The delay line holds `delay_periods + 1` entries so that the slot written
  // at this tick is read back exactly `delay_periods` ticks later. Zero is the
  // trim command in deviation coordinates.
  std::vector<Eigen::VectorXd> queue(static_cast<std::size_t>(delay_periods) + 1,
                                     Eigen::VectorXd::Zero(inputs));
  std::size_t head = 0;

  Eigen::VectorXd state = initial_state;
  for (int tick = 0; tick < tick_count; ++tick) {
    prediction.states.push_back(state);
    queue[head] = -gain * state;
    head = (head + 1) % queue.size();
    const Eigen::VectorXd applied = queue[head];
    prediction.applied_inputs.push_back(applied);
    state = plant.a * state + plant.b * applied;
  }
  prediction.states.push_back(state);
  return prediction;
}

}  // namespace galata::sim
