// SPDX-License-Identifier: Apache-2.0
//
// Local nonlinear aircraft simulation with bounded first-order actuators.
// References: Stevens, Lewis & Johnson, Aircraft Control and Simulation,
// 3rd ed., Wiley, 2016, chapters 2-3; Hairer, Norsett & Wanner, Solving Ordinary
// Differential Equations I, 2nd revised ed., Springer, 1993 (fixed-step RK4).
//
// WHAT THIS IS NOT: no wind, sensors, sampled controller, propulsion map,
// structural flexibility or flight-envelope expansion. The aircraft retains
// its local derivative-model limitations. Feedback assumes exact full states
// and continuous evaluation at each RK stage. Saturation introduces nonsmooth
// dynamics: fourth-order convergence is only expected away from limit events.
// A completed trajectory is not a validation of the supplied aircraft model.
#ifndef GALATA_SIM_NONLINEAR_HPP
#define GALATA_SIM_NONLINEAR_HPP

#include "galata/model/aircraft.hpp"
#include "galata/trim/level.hpp"

#include <array>
#include <string>
#include <vector>

namespace galata::sim {

// Channel order is elevator, aileron, rudder, thrust. All entries are required:
// zero defaults intentionally fail validation rather than inventing hardware.
struct ActuatorLimits {
  double minimum = 0.0;           // rad for surfaces, N for thrust
  double maximum = 0.0;           // rad for surfaces, N for thrust
  double rate_limit_per_s = 0.0;  // rad/s for surfaces, N/s for thrust
  double time_constant_s = 0.0;   // s, strictly positive first-order lag
};

struct StateFeedback {
  Eigen::MatrixXd k;                     // input_i / state_j; empty means open loop
  std::vector<std::string> state_names;  // physical Euler names from linearisation
  std::vector<std::string> input_names;  // elevator, aileron, rudder, thrust
};

struct NonlinearRequest {
  double step_s = 0.0;    // s, positive and no larger than any actuator time constant
  int step_count = 0;     // number of fixed integration steps
  int sample_stride = 1;  // steps; final accepted state is always recorded
  std::array<ActuatorLimits, model::Controls::kSize> actuators;
  StateFeedback feedback;
  std::vector<std::string> perturbation_state_names;  // unique physical Euler state names
  Eigen::VectorXd initial_perturbation;               // m, m/s, rad, rad/s by named state
  model::Controls command_increment;                  // rad, N; constant offset from trim
  bool stop_outside_envelope = true;                  // stop at the first violating RK stage
};

struct NonlinearSample {
  double time_s = 0.0;  // s
  core::State state;
  model::Controls controls;  // actual actuator positions, rad/N
  model::Controls command;   // requested positions before clipping, rad/N
  std::array<bool, model::Controls::kSize> position_limited{};
  std::array<bool, model::Controls::kSize> rate_limited{};
  model::EnvelopeWarning envelope;
};

struct NonlinearResult {
  std::vector<NonlinearSample> samples;
  double step_s = 0.0;      // s
  int requested_steps = 0;  // steps
  int completed_steps = 0;  // accepted fixed steps
  bool completed = false;   // all requested steps completed
  std::string termination_reason;
  double termination_time_s = 0.0;  // s; rejected RK stage time on domain failure
  bool outside_envelope_encountered = false;
  double max_alpha_departure_rad = 0.0;  // rad, across samples and RK stages
  double max_mach_departure = 0.0;       // dimensionless, across samples and RK stages
  std::array<int, model::Controls::kSize> position_limited_steps{};
  std::array<int, model::Controls::kSize> rate_limited_steps{};
};

// gamma must be zero: feedback tracks the translating reference position while
// trim velocity, attitude and controls stay constant. Initial perturbations use
// the physical Euler coordinates p_n,p_e,p_d,u,v,w,phi,theta,psi,p,q,r; the
// integrated aircraft state remains the 13-component quaternion state.
// Invalid configuration throws. Encountering the model domain/envelope returns
// an incomplete result with its reason and the last accepted state.
[[nodiscard]] NonlinearResult simulate_nonlinear(const model::Aircraft& aircraft,
                                                 const trim::TrimPoint& trim,
                                                 const NonlinearRequest& request);

}  // namespace galata::sim
#endif
