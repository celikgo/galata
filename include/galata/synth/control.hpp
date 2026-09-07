// SPDX-License-Identifier: Apache-2.0
//
// Continuous-time quadratic state feedback and filtered PID realisations.
// References: A. J. Laub, "A Schur Method for Solving Algebraic Riccati
// Equations", IEEE TAC 24(6), 1979, pp. 913-921; B. D. O. Anderson and
// J. B. Moore, "Optimal Control: Linear Quadratic Methods", 1990, ch. 3.
// K. J. Astrom and R. M. Murray, "Feedback Systems", 2nd ed., 2021, ch. 11.
//
// WHAT THIS IS NOT: no estimator, automatic tuning, discrete-time controller,
// or constraint handling. Full state feedback assumes measured states. The
// dense Schur solver rejects poorly separated/conditioned problems instead of
// returning an uncertified gain. Numerical residuals do not validate a plant.
#ifndef GALATA_SYNTH_CONTROL_HPP
#define GALATA_SYNTH_CONTROL_HPP

#include "galata/model/linear_system.hpp"

#include <complex>
#include <vector>

namespace galata::synth {

struct CareSolution {
  Eigen::MatrixXd x;                    // cost / (state_i state_j)
  Eigen::MatrixXd k;                    // input_i / state_j
  double relative_residual = 0.0;       // dimensionless
  double residual_budget = 0.0;         // dimensionless, conditioning-scaled roundoff
  double symmetry_defect = 0.0;         // dimensionless
  double subspace_condition = 0.0;      // dimensionless
  double hamiltonian_separation = 0.0;  // 1/s, distance from imaginary axis
  std::vector<std::complex<double>> closed_loop_eigenvalues;  // 1/s
};

// Q and R must be symmetric; R positive definite; the block cost [Q N; N' R]
// positive semidefinite. Empty N means zero. Throws on invalid input or when
// the stabilising solution cannot satisfy the residual and stability checks.
[[nodiscard]] CareSolution solve_care(const Eigen::MatrixXd& a,
                                      const Eigen::MatrixXd& b,
                                      const Eigen::MatrixXd& q,
                                      const Eigen::MatrixXd& r,
                                      const Eigen::MatrixXd& n = {});

struct LqrDesign {
  CareSolution riccati;
  Eigen::MatrixXd q;  // state-cost weights, mixed units
  Eigen::MatrixXd r;  // input-cost weights, mixed units
  Eigen::MatrixXd n;  // state/input cross-cost weights, mixed units
  model::LinearSystem plant;
  model::LinearSystem closed_loop;
  // L(s) = K (sI-A)^-1 B, the loop broken at the plant input. Negative feedback.
  model::LinearSystem broken_loop;
};

// Additionally requires numerical detectability of the completed-square cost:
// unpenalised nonstable modes are refused. Stable unpenalised modes are allowed.
[[nodiscard]] LqrDesign design_lqr(const model::LinearSystem& plant,
                                   const Eigen::MatrixXd& q,
                                   const Eigen::MatrixXd& r,
                                   const Eigen::MatrixXd& n = {});

// Kp + Ki/s + Kd*s/(tau*s+1). tau must be strictly positive, including when
// Kd is zero; it is an explicit modelling decision. Zero terms add no states.
[[nodiscard]] model::LinearSystem filtered_pid(double kp,                    // output/input
                                               double ki,                    // output/(input s)
                                               double kd,                    // output s/input
                                               double derivative_filter_s);  // s

// Cascade: input -> first -> second -> output. Internal states are prefixed
// first./second. to retain identity. Throws on incompatible channel counts.
[[nodiscard]] model::LinearSystem series(const model::LinearSystem& first,
                                         const model::LinearSystem& second);
// Square loop with negative identity feedback. Input/output channel order is
// the connection contract. Rejects singular algebraic feedthrough loops.
[[nodiscard]] model::LinearSystem negative_feedback(const model::LinearSystem& loop);

}  // namespace galata::synth
#endif
