// SPDX-License-Identifier: Apache-2.0
// Deterministic typed state-space graph lowering; see ADR-0013.
#ifndef GALATA_MODELING_LINEAR_ADAPTER_HPP
#define GALATA_MODELING_LINEAR_ADAPTER_HPP

#include "galata/model/linear_system.hpp"
#include "galata/modeling/model.hpp"

namespace galata::modeling {

// Thirty-two, and the figure is derived rather than chosen. A state row of the
// lowered graph carries one term per state and one per input, so its width is
// n + m, and `kMaxLinearTerms` caps a row at 64. Setting this cap to half that
// makes every admissible channel combination produce a row the executor
// accepts, with no combination left to discover at run time.
//
// It was 16, which is where ADR-0013 first set it. Sixteen is exactly the width
// of the fixed-voltage quadrotor's hover linearisation — twelve chart
// coordinates and four rotor states — so that model sat on the cap and the
// seventeen-state battery variant sat one over it, losing the typed
// linear-graph path for one state. RFC-0002 raised that as a question rather
// than absorbing it; ADR-0013 answers it here.
inline constexpr std::size_t kMaxLinearChannels = 32;

// Explicit types in the source matrix order. Units are canonical SI, with no
// inference from LinearSystem's free-text names or units description.
struct LinearChannels {
  std::vector<SignalType> states;
  std::vector<SignalType> inputs;
  std::vector<SignalType> outputs;
};

struct LinearGraphOptions {
  Eigen::VectorXd initial_state;
  Eigen::VectorXd command;
  Eigen::MatrixXd feedback_gain;  // empty: open loop; otherwise m x n, u = command - K x
};

struct LinearGraph {
  Model model;
  // Each mapping follows source matrix channel order. control_output_ids are
  // additional Output blocks for actual u. Compiled outputs are sorted by ID;
  // use these mappings rather than assuming their vector positions coincide.
  std::vector<std::string> state_ids;
  std::vector<std::string> command_ids;
  std::vector<std::string> control_ids;
  std::vector<std::string> output_ids;
  std::vector<std::string> control_output_ids;
};

// Lowers x_dot = A x + B u, y = C x + D u and optional u = command - K x
// (Astrom & Murray, Feedback Systems, 2nd ed., state-space/feedback equations).
// Each channel count must be in [1,32]; initial state, constant command and
// channel metadata must have exact sizes. C/D retain LinearSystem defaults.
// Channel names are nonempty and unique within each list, with a combined
// one-MiB byte limit. Matrix axes, including empty matrix axes, are bounded.
// Rows append state terms before actual-control terms in matrix column order.
// Control rows append the command before negative-feedback state terms.
// Only exact zero matrix coefficients (including -0) are omitted; authored
// graph terms are never pruned. Empty rows become typed +0 constants. Products
// and additions evaluate as a finite-checked left fold from +0, so agreement
// with a matrix solver is numerical, not a promise of bitwise Eigen identity.
// This uses the existing graph RK4 executor; it does not add a numerical solver
// or establish the physical validity of user-declared coordinate couplings.
[[nodiscard]] LinearGraph lower_linear_system(const model::LinearSystem& system,
                                              const LinearChannels& channels,
                                              const LinearGraphOptions& options);

}  // namespace galata::modeling
#endif
