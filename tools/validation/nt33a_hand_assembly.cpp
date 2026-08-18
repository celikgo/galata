// SPDX-License-Identifier: Apache-2.0

#include "nt33a_hand_assembly.hpp"

#include "galata/core/constants.hpp"
#include "galata/units.hpp"

#include <cmath>

namespace galata::validation {
namespace {

// The moment derivatives are per FOOT in the source, so unlike the lateral
// matrix the longitudinal one is not invariant under a change of length unit.
// Getting this wrong is a factor of 3.28 on the whole pitching-moment row.
constexpr double per_foot_to_per_metre() {
  return 1.0 / units::kMetresPerFoot;
}

}  // namespace

HandAssemblyInputs HandAssemblyInputs::from_published(
    const std::map<std::string, double>& published) {
  HandAssemblyInputs inputs;
  inputs.x_u_star = published.at("X_u_star");
  inputs.z_u_star = published.at("Z_u_star");
  inputs.m_u_star = published.at("M_u_star");
  inputs.x_w = published.at("X_w");
  inputs.z_w = published.at("Z_w");
  inputs.m_w = published.at("M_w");
  inputs.z_w_dot = published.at("Z_w_dot");
  inputs.m_w_dot = published.at("M_w_dot");
  inputs.z_q = published.at("Z_q");
  inputs.m_q = published.at("M_q");
  inputs.y_v = published.at("Y_v");
  inputs.l_beta_prime = published.at("L_beta_prime");
  inputs.n_beta_prime = published.at("N_beta_prime");
  inputs.l_p_prime = published.at("L_p_prime");
  inputs.n_p_prime = published.at("N_p_prime");
  inputs.l_r_prime = published.at("L_r_prime");
  inputs.n_r_prime = published.at("N_r_prime");
  inputs.true_airspeed_ft_s = published.at("true_airspeed");
  inputs.trim_alpha_deg = published.at("trim_alpha");
  inputs.trim_gamma_deg = published.at("trim_gamma");
  return inputs;
}

std::vector<std::string> longitudinal_state_names() {
  return {"u", "w", "q", "theta"};
}

std::vector<std::string> lateral_state_names() {
  return {"beta", "p", "r", "phi"};
}

Eigen::MatrixXd hand_assembled_longitudinal(const HandAssemblyInputs& inputs) {
  // NASA CR-2144 Appendix C p.C-1, a descriptor form in [u, w, theta] with
  // q supplied by q = s theta. Rearranged:
  //
  //   u_dot = X_u* u + X_w w - W_o q - g cos(th_o) th
  //   w_dot = [Z_u* u + Z_w w + (Z_q + U_o) q - g sin(th_o) th] / (1 - Z_wdot)
  //   q_dot = M_u* u + M_w w + M_q q + M_wdot w_dot
  //
  // Derivatives the report leaves blank for this aircraft — X_udot, X_wdot,
  // X_q, Z_udot, M_udot — are taken as zero. That is an ASSUMPTION this
  // assembly makes and the document does not, and it is the leading candidate
  // for the phugoid-damping discrepancy the report describes.
  const double scale_to_per_metre = per_foot_to_per_metre();
  const double speed = units::feet_to_metres(inputs.true_airspeed_ft_s);
  const double alpha = units::degrees_to_radians(inputs.trim_alpha_deg);
  const double theta = alpha + units::degrees_to_radians(inputs.trim_gamma_deg);
  const double gravity = core::kStandardGravity;

  const double u_o = speed * std::cos(alpha);
  const double w_o = speed * std::sin(alpha);

  const double m_u = inputs.m_u_star * scale_to_per_metre;
  const double m_w = inputs.m_w * scale_to_per_metre;
  const double m_w_dot = inputs.m_w_dot * scale_to_per_metre;
  const double z_q = units::feet_to_metres(inputs.z_q);

  const double inverse = 1.0 / (1.0 - inputs.z_w_dot);
  const double w_row_u = inputs.z_u_star * inverse;
  const double w_row_w = inputs.z_w * inverse;
  const double w_row_q = (z_q + u_o) * inverse;
  const double w_row_theta = -gravity * std::sin(theta) * inverse;

  Eigen::MatrixXd a(4, 4);
  a << inputs.x_u_star, inputs.x_w, -w_o, -gravity * std::cos(theta),  //
      w_row_u, w_row_w, w_row_q, w_row_theta,                          //
      m_u + m_w_dot * w_row_u, m_w + m_w_dot * w_row_w,                //
      inputs.m_q + m_w_dot * w_row_q, m_w_dot * w_row_theta,           //
      0.0, 0.0, 1.0, 0.0;
  return a;
}

Eigen::MatrixXd hand_assembled_lateral(const HandAssemblyInputs& inputs) {
  // NASA CR-2144 Appendix C p.C-3, with phi promoted from the auxiliary
  // relation to a fourth state:
  //
  //   beta_dot = Y_v beta + (W_o/V) p - (U_o/V) r + (g cos(th_o)/V) phi
  //   p_dot    = L_beta' beta + L_p' p + L_r' r
  //   r_dot    = N_beta' beta + N_p' p + N_r' r
  //   phi_dot  = p + r tan(th_o)
  //
  // W_o/V is sin(alpha) and U_o/V is cos(alpha), so only g/V carries a unit and
  // this matrix is the same in feet as in metres.
  const double speed = units::feet_to_metres(inputs.true_airspeed_ft_s);
  const double alpha = units::degrees_to_radians(inputs.trim_alpha_deg);
  const double theta = alpha + units::degrees_to_radians(inputs.trim_gamma_deg);
  const double gravity_term = core::kStandardGravity * std::cos(theta) / speed;

  Eigen::MatrixXd a(4, 4);
  a << inputs.y_v, std::sin(alpha), -std::cos(alpha), gravity_term,  //
      inputs.l_beta_prime, inputs.l_p_prime, inputs.l_r_prime, 0.0,  //
      inputs.n_beta_prime, inputs.n_p_prime, inputs.n_r_prime, 0.0,  //
      0.0, 1.0, std::tan(theta), 0.0;
  return a;
}

}  // namespace galata::validation
