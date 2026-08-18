// SPDX-License-Identifier: Apache-2.0
//
// The NT-33A state matrices assembled BY HAND from NASA CR-2144's published
// dimensional derivatives.
//
// Shared between the validation test that gates them and the V&V report that
// quotes them, for the same reason the determinism battery is shared: a second
// implementation would be a second answer, and the report would be describing a
// matrix nobody tests.
//
// These are deliberately NOT how galata computes a state matrix. galata trims a
// nonlinear model and linearises it. This assembly exists only so the two
// routes can be compared, and the comparison is the whole point: the phugoid
// damping disagrees here and does not disagree there, which is what localises
// the discrepancy to this assembly.
//
// Equations: NASA CR-2144 Appendix C, pages C-1 (longitudinal) and C-3
// (lateral). Both are printed as Laplace matrix equations and are rearranged
// here into state-space form; the derivations are written out in the
// implementation.

#ifndef GALATA_TOOLS_VALIDATION_NT33A_HAND_ASSEMBLY_HPP
#define GALATA_TOOLS_VALIDATION_NT33A_HAND_ASSEMBLY_HPP

#include <Eigen/Core>

#include <map>
#include <string>
#include <vector>

namespace galata::validation {

// Inputs in the units the report prints, i.e. feet and seconds. Converted
// inside, at the boundary, per ADR-0003.
struct HandAssemblyInputs {
  double x_u_star = 0.0;
  double z_u_star = 0.0;
  double m_u_star = 0.0;  // per second-FOOT
  double x_w = 0.0;
  double z_w = 0.0;
  double m_w = 0.0;  // per second-foot
  double z_w_dot = 0.0;
  double m_w_dot = 0.0;  // per foot
  double z_q = 0.0;      // ft/s
  double m_q = 0.0;

  double y_v = 0.0;
  double l_beta_prime = 0.0;
  double n_beta_prime = 0.0;
  double l_p_prime = 0.0;
  double n_p_prime = 0.0;
  double l_r_prime = 0.0;
  double n_r_prime = 0.0;

  double true_airspeed_ft_s = 0.0;
  double trim_alpha_deg = 0.0;
  double trim_gamma_deg = 0.0;

  // Reads every field from a reference table's quantity->value map.
  [[nodiscard]] static HandAssemblyInputs from_published(
      const std::map<std::string, double>& published);
};

// State order [u, w, q, theta], SI.
[[nodiscard]] Eigen::MatrixXd hand_assembled_longitudinal(const HandAssemblyInputs& inputs);
[[nodiscard]] std::vector<std::string> longitudinal_state_names();

// State order [beta, p, r, phi]. Dimensionless beta rather than v, matching
// the report's own presentation.
[[nodiscard]] Eigen::MatrixXd hand_assembled_lateral(const HandAssemblyInputs& inputs);
[[nodiscard]] std::vector<std::string> lateral_state_names();

}  // namespace galata::validation

#endif  // GALATA_TOOLS_VALIDATION_NT33A_HAND_ASSEMBLY_HPP
