// SPDX-License-Identifier: Apache-2.0
//
// A linear time-invariant state-space model, and its on-disk form.
//
//   x_dot = A x + B u
//   y     = C x + D u
//
// State ordering is whatever the model declares, not ADR-0002's thirteen-state
// ordering: a model may be longitudinal-only, lateral-only, or in wind axes,
// and the state NAMES are what let the analysis layer know which is which. A
// system produced by galata's own linearisation will carry ADR-0002's ordering
// and names; one loaded from a file carries whatever the file says.
//
// WHAT THIS IS NOT
// * Not a descriptor system. E x_dot = A x is not represented; a model whose
//   equations are implicit must be rearranged before it gets here.
// * Not time-varying, and not parameter-scheduled. One matrix set, one
//   operating point.
// * Carries no units. The entries of an A matrix have mixed dimensions by
//   nature — a single row can hold 1/s, rad/s and dimensionless entries — so
//   ADR-0003's "every field carries its unit" rule cannot apply entry by entry.
//   What it requires instead is that the STATES be documented, which is what
//   state_names and the file's `units` field are for.

#ifndef GALATA_MODEL_LINEAR_SYSTEM_HPP
#define GALATA_MODEL_LINEAR_SYSTEM_HPP

#include <Eigen/Core>

#include <string>
#include <vector>

namespace galata::model {

struct LinearSystem {
  Eigen::MatrixXd a;  // n x n
  Eigen::MatrixXd b;  // n x m, empty when the model has no inputs

  std::vector<std::string> state_names;  // n entries
  std::vector<std::string> input_names;  // m entries

  // Free text from the file, carried through to reports so a result can name
  // the aircraft and condition it describes.
  std::string description;
  // The publication the model came from, if the file declares one. Reports
  // print it, so a number cannot be quoted without its source travelling with
  // it (charter rule 9).
  std::string citation;
  // What the states are measured in, as free text — see the note above about
  // why this is not per-entry.
  std::string units;

  // Throws std::invalid_argument describing the first inconsistency found.
  void validate() const;

  [[nodiscard]] Eigen::Index state_count() const {
    return a.rows();
  }

  [[nodiscard]] Eigen::Index input_count() const {
    return b.cols();
  }
};

// Reads a linear system from a YAML file:
//
//   description: NT-33A lateral-directional, flight condition 1
//   citation: NASA CR-2144 ...
//   units: beta rad, p rad/s, r rad/s, phi rad
//   states: [beta, p, r, phi]
//   inputs: [delta_a, delta_r]
//   a:
//     - [-0.125, 0.0384, -0.9993, 0.1411]
//     ...
//   b:
//     - [0.0, 0.0295]
//     ...
//
// `b` and `inputs` are optional. Throws with the file path in the message on
// any structural problem.
[[nodiscard]] LinearSystem load_linear_system(const std::string& path);

}  // namespace galata::model

#endif  // GALATA_MODEL_LINEAR_SYSTEM_HPP
