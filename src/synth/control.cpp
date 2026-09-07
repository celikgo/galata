// SPDX-License-Identifier: Apache-2.0
// Laub (1979), IEEE TAC 24(6), pp. 913-921: stable Hamiltonian Schur subspace.
// WHAT THIS IS NOT: a generalised-pencil solver for singular/indefinite costs.
// R is solved by Cholesky, never explicitly inverted. Ill-conditioned stable
// subspaces, nonreal/nonsymmetric solutions, and non-Hurwitz results are refused.
#include "galata/synth/control.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace galata::synth {
namespace {
constexpr double kEps = std::numeric_limits<double>::epsilon();

void require_symmetric(const Eigen::MatrixXd& matrix, const char* name) {
  if (!matrix.allFinite() || matrix.rows() != matrix.cols()
      || (matrix - matrix.transpose()).norm() > 64.0 * kEps * matrix.norm()) {
    throw std::invalid_argument(std::string("solve_care: ") + name
                                + " must be finite and symmetric");
  }
}

// Move one stable eigenvalue left by adjacent unitary Schur exchanges. For
// [a b;0 d], [b,d-a] is an eigenvector for d. Its orthogonal complement gives
// a unitary basis without a matrix inverse. The number of exchanges is bounded
// by the matrix dimension; ties within each half keep their Schur order.
void exchange(Eigen::MatrixXcd& t, Eigen::MatrixXcd& z, Eigen::Index i) {
  const std::complex<double> b = t(i, i + 1);
  const std::complex<double> delta = t(i + 1, i + 1) - t(i, i);
  const double scale = std::max(std::abs(b), std::abs(delta));
  if (!(scale > 0.0) || !std::isfinite(scale)) {
    throw std::runtime_error("solve_care: inseparable Hamiltonian eigenvalues");
  }
  Eigen::Vector2cd v;
  v << b / scale, delta / scale;
  v.normalize();
  Eigen::Matrix2cd u;
  u << v(0), -std::conj(v(1)), v(1), std::conj(v(0));
  t.middleCols(i, 2) = (t.middleCols(i, 2) * u).eval();
  t.middleRows(i, 2) = (u.adjoint() * t.middleRows(i, 2)).eval();
  z.middleCols(i, 2) = (z.middleCols(i, 2) * u).eval();
  t(i + 1, i) = 0.0;
}

// Anderson and Moore (1990), ch. 3: detectability after completing the square
// ensures that the stabilising CARE solution is the infinite-horizon optimum.
// The PBH test requires [lambda I-Abar; sqrt(Qbar)] to have full column rank at
// every nonstable eigenvalue. Stable unobserved modes are permitted. We reject
// numerically unresolved rank, rather than infer detectability from a tiny pivot.
// Inputs have already passed solve_care's dimension, cost and finiteness checks.
void require_detectable(const Eigen::MatrixXd& a,
                        const Eigen::MatrixXd& b,
                        const Eigen::MatrixXd& q,
                        const Eigen::MatrixXd& r,
                        const Eigen::MatrixXd& n) {
  const Eigen::Index count = a.rows();
  const Eigen::MatrixXd rn = r.llt().solve(n.transpose());
  const Eigen::MatrixXd abar = a - b * rn;
  const Eigen::MatrixXd qbar = q - n * rn;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> cost(qbar);
  Eigen::EigenSolver<Eigen::MatrixXd> modes(abar, false);
  if (cost.info() != Eigen::Success || modes.info() != Eigen::Success) {
    throw std::runtime_error("design_lqr: detectability decomposition failed");
  }
  Eigen::MatrixXd root =
      cost.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() * cost.eigenvectors().transpose();
  const double root_scale = root.stableNorm();
  if (root_scale > 0.0) {
    root /= root_scale;
  }
  const double state_scale = std::max(abar.stableNorm(), std::numeric_limits<double>::min());
  const double rank_budget = 256.0 * static_cast<double>(count) * kEps;
  for (Eigen::Index i = 0; i < count; ++i) {
    const std::complex<double> pole = modes.eigenvalues()(i);
    if (!std::isfinite(pole.real()) || !std::isfinite(pole.imag())) {
      throw std::runtime_error("design_lqr: nonfinite pole in detectability check");
    }
    if (pole.real() < -rank_budget * state_scale) {
      continue;
    }
    Eigen::MatrixXcd pbh(2 * count, count);
    pbh.topRows(count) =
        (pole * Eigen::MatrixXcd::Identity(count, count) - abar.cast<std::complex<double>>())
        / state_scale;
    pbh.bottomRows(count) = root.cast<std::complex<double>>();
    if (!pbh.allFinite()) {
      throw std::runtime_error("design_lqr: overflow in detectability check; rescale the problem");
    }
    Eigen::JacobiSVD<Eigen::MatrixXcd> rank(pbh);
    if (rank.info() != Eigen::Success
        || rank.singularValues()(count - 1) <= rank_budget * rank.singularValues()(0)) {
      throw std::invalid_argument(
          "design_lqr: completed-square state cost is not detectably weighted, or the PBH rank "
          "is numerically unresolved; penalise the nonstable modes or rescale the problem");
    }
  }
}
}  // namespace

CareSolution solve_care(const Eigen::MatrixXd& a,
                        const Eigen::MatrixXd& b,
                        const Eigen::MatrixXd& q,
                        const Eigen::MatrixXd& r,
                        const Eigen::MatrixXd& cross) {
  const Eigen::Index states = a.rows();
  const Eigen::Index inputs = b.cols();
  if (states < 1 || states > 256 || a.cols() != states || b.rows() != states || inputs < 1
      || q.rows() != states || q.cols() != states || r.rows() != inputs || r.cols() != inputs
      || !a.allFinite() || !b.allFinite()) {
    throw std::invalid_argument(
        "solve_care: finite compatible matrices and 1..256 states required");
  }
  require_symmetric(q, "Q");
  require_symmetric(r, "R");
  const Eigen::MatrixXd n = cross.size() == 0 ? Eigen::MatrixXd::Zero(states, inputs) : cross;
  if (n.rows() != states || n.cols() != inputs || !n.allFinite()) {
    throw std::invalid_argument(
        "solve_care: N must be finite with one row per state and column per input");
  }
  Eigen::LLT<Eigen::MatrixXd> r_factor(r);
  if (r_factor.info() != Eigen::Success) {
    throw std::invalid_argument("solve_care: R must be positive definite");
  }
  const Eigen::MatrixXd rb = r_factor.solve(b.transpose());
  const Eigen::MatrixXd rn = r_factor.solve(n.transpose());
  const Eigen::MatrixXd abar = a - b * rn;
  const Eigen::MatrixXd qbar = q - n * rn;
  const Eigen::MatrixXd g = b * rb;
  if (!rb.allFinite() || !rn.allFinite() || !abar.allFinite() || !qbar.allFinite()
      || !g.allFinite()) {
    throw std::runtime_error("solve_care: transformed cost overflow; rescale the problem");
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> cost(qbar);
  if (cost.info() != Eigen::Success
      || cost.eigenvalues().minCoeff() < -128.0 * kEps * std::max(q.norm(), qbar.norm())) {
    throw std::invalid_argument("solve_care: block cost [Q N; N' R] must be positive semidefinite");
  }
  Eigen::MatrixXd h(2 * states, 2 * states);
  h << abar, -g, -qbar, -abar.transpose();
  if (!h.allFinite()) {
    throw std::runtime_error("solve_care: Hamiltonian overflow; rescale the problem");
  }
  Eigen::ComplexSchur<Eigen::MatrixXd> schur(h);
  if (schur.info() != Eigen::Success) {
    throw std::runtime_error("solve_care: Hamiltonian Schur decomposition failed");
  }
  Eigen::MatrixXcd t = schur.matrixT();
  Eigen::MatrixXcd z = schur.matrixU();
  CareSolution result;
  result.hamiltonian_separation = std::numeric_limits<double>::infinity();
  Eigen::Index stable = 0;
  const double separation_floor = 128.0 * kEps * h.norm();
  for (Eigen::Index i = 0; i < 2 * states; ++i) {
    const double real = t(i, i).real();
    result.hamiltonian_separation = std::min(result.hamiltonian_separation, std::abs(real));
    if (!std::isfinite(real) || std::abs(real) <= separation_floor) {
      throw std::runtime_error(
          "solve_care: Hamiltonian spectrum touches the imaginary axis; solution is not separated");
    }
    if (real < 0.0) {
      for (Eigen::Index j = i; j > stable; --j) {
        exchange(t, z, j - 1);
      }
      ++stable;
    }
  }
  if (stable != states) {
    throw std::runtime_error("solve_care: Hamiltonian does not have the required stable subspace");
  }
  const Eigen::MatrixXcd u1 = z.topLeftCorner(states, states);
  const Eigen::MatrixXcd u2 = z.bottomLeftCorner(states, states);
  Eigen::JacobiSVD<Eigen::MatrixXcd> svd(u1);
  result.subspace_condition = svd.singularValues()(0) / svd.singularValues()(states - 1);
  if (!std::isfinite(result.subspace_condition)
      || result.subspace_condition > 1.0 / std::sqrt(kEps)) {
    throw std::runtime_error("solve_care: stable subspace is ill-conditioned; rescale the problem");
  }
  const Eigen::MatrixXcd xc = u1.transpose().fullPivLu().solve(u2.transpose()).transpose();
  const double scale = std::max(1.0, xc.norm());
  const double roundoff = 512.0 * static_cast<double>(states) * kEps * result.subspace_condition;
  if (!xc.allFinite() || xc.imag().norm() > roundoff * scale) {
    throw std::runtime_error("solve_care: stable subspace did not produce a real solution");
  }
  const Eigen::MatrixXd raw = xc.real();
  result.symmetry_defect = (raw - raw.transpose()).norm() / scale;
  if (result.symmetry_defect > roundoff) {
    throw std::runtime_error("solve_care: Riccati solution is not symmetric within roundoff");
  }
  result.x = 0.5 * (raw + raw.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solution_eigen(result.x);
  if (solution_eigen.info() != Eigen::Success
      || solution_eigen.eigenvalues().minCoeff() < -roundoff * scale) {
    throw std::runtime_error("solve_care: stabilising solution is not positive semidefinite");
  }
  result.k = r_factor.solve(b.transpose() * result.x + n.transpose());
  const Eigen::MatrixXd left = a.transpose() * result.x;
  const Eigen::MatrixXd right = result.x * a;
  const Eigen::MatrixXd quadratic = (result.x * b + n) * result.k;
  const double denominator = std::max(std::numeric_limits<double>::min(),
                                      left.norm() + right.norm() + q.norm() + quadratic.norm());
  result.relative_residual = (left + right - quadratic + q).norm() / denominator;
  // Cap the permitted backward error. Poor conditioning must not grant an
  // arbitrarily weak gate. No user option can relax this acceptance budget.
  result.residual_budget = std::min(1e-8, roundoff);
  if (!std::isfinite(result.relative_residual)
      || result.relative_residual > result.residual_budget) {
    throw std::runtime_error(
        "solve_care: residual exceeds the conditioning-scaled roundoff budget");
  }
  Eigen::EigenSolver<Eigen::MatrixXd> closed(a - b * result.k, false);
  if (closed.info() != Eigen::Success) {
    throw std::runtime_error("solve_care: closed-loop eigenvalue check failed");
  }
  for (Eigen::Index i = 0; i < states; ++i) {
    const auto pole = closed.eigenvalues()(i);
    if (!std::isfinite(pole.real()) || !std::isfinite(pole.imag()) || pole.real() >= 0.0) {
      throw std::runtime_error("solve_care: computed state feedback is not stabilising");
    }
    result.closed_loop_eigenvalues.push_back(pole);
  }
  std::sort(result.closed_loop_eigenvalues.begin(),
            result.closed_loop_eigenvalues.end(),
            [](auto x, auto y) {
              return x.real() != y.real() ? x.real() < y.real() : x.imag() < y.imag();
            });
  return result;
}

LqrDesign design_lqr(const model::LinearSystem& plant,
                     const Eigen::MatrixXd& q,
                     const Eigen::MatrixXd& r,
                     const Eigen::MatrixXd& n) {
  plant.validate();
  LqrDesign design;
  design.plant = plant;
  design.q = q;
  design.r = r;
  design.n = n.size() == 0 ? Eigen::MatrixXd::Zero(plant.state_count(), plant.input_count()) : n;
  design.riccati = solve_care(plant.a, plant.b, q, r, design.n);
  // solve_care first validates all matrix dimensions and costs. A stabilising
  // algebraic solution alone is insufficient to label this an optimal design.
  require_detectable(plant.a, plant.b, q, r, design.n);
  design.closed_loop = plant;
  design.closed_loop.a = plant.a - plant.b * design.riccati.k;
  design.closed_loop.c = plant.output_matrix() - plant.feedthrough_matrix() * design.riccati.k;
  design.closed_loop.description = "LQR closed loop: " + plant.description;
  design.broken_loop = plant;
  design.broken_loop.c = design.riccati.k;
  design.broken_loop.d = Eigen::MatrixXd::Zero(plant.input_count(), plant.input_count());
  design.broken_loop.output_names = plant.input_names;
  design.broken_loop.description = "LQR return ratio at plant input: " + plant.description;
  design.closed_loop.validate();
  design.broken_loop.validate();
  return design;
}

model::LinearSystem filtered_pid(double kp, double ki, double kd, double tau) {
  if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd) || !std::isfinite(tau)
      || tau <= 0.0) {
    throw std::invalid_argument(
        "filtered_pid: finite gains and a positive derivative_filter_s required");
  }
  // Retain a stable zero-output state for a pure proportional controller,
  // because LinearSystem intentionally requires a nonempty state matrix.
  const int count = (ki != 0.0 ? 1 : 0) + (kd != 0.0 ? 1 : 0);
  const int size = std::max(count, 1);
  model::LinearSystem system;
  system.a = Eigen::MatrixXd::Zero(size, size);
  system.b = Eigen::MatrixXd::Zero(size, 1);
  system.c = Eigen::MatrixXd::Zero(1, size);
  system.d = Eigen::MatrixXd::Constant(1, 1, kp + kd / tau);
  int index = 0;
  if (ki != 0.0) {
    system.b(index, 0) = 1.0;
    system.c(0, index) = ki;
    system.state_names.push_back("integrated_error");
    ++index;
  }
  if (kd != 0.0) {
    system.a(index, index) = -1.0 / tau;
    system.b(index, 0) = 1.0 / tau;
    system.c(0, index) = -kd / tau;
    system.state_names.push_back("filtered_error");
  }
  if (count == 0) {
    system.a(0, 0) = -1.0;
    system.state_names.push_back("unused_stable_state");
  }
  system.input_names = {"error"};
  system.output_names = {"command"};
  system.description = "Filtered PID controller";
  system.citation = "Astrom and Murray, Feedback Systems, 2nd ed., 2021, chapter 11";
  system.units = "error and command units supplied by the connected plant; time s";
  system.validate();
  return system;
}

model::LinearSystem series(const model::LinearSystem& first, const model::LinearSystem& second) {
  first.validate();
  second.validate();
  if (first.output_count() != second.input_count()) {
    throw std::invalid_argument(
        "series: first outputs must match second inputs in count and order");
  }
  const Eigen::Index n1 = first.state_count(), n2 = second.state_count();
  const Eigen::MatrixXd c1 = first.output_matrix(), c2 = second.output_matrix();
  const Eigen::MatrixXd d1 = first.feedthrough_matrix(), d2 = second.feedthrough_matrix();
  model::LinearSystem result;
  result.a = Eigen::MatrixXd::Zero(n1 + n2, n1 + n2);
  result.a.topLeftCorner(n1, n1) = first.a;
  result.a.bottomLeftCorner(n2, n1) = second.b * c1;
  result.a.bottomRightCorner(n2, n2) = second.a;
  result.b.resize(n1 + n2, first.input_count());
  result.b << first.b, second.b * d1;
  result.c.resize(second.output_count(), n1 + n2);
  result.c << d2 * c1, c2;
  result.d = d2 * d1;
  for (const auto& name : first.state_names) {
    result.state_names.push_back("first." + name);
  }
  for (const auto& name : second.state_names) {
    result.state_names.push_back("second." + name);
  }
  result.input_names = first.input_names;
  result.output_names = second.output_labels();
  result.description = "Cascade: " + first.description + " -> " + second.description;
  result.citation = first.citation + "; " + second.citation;
  result.units = "first: " + first.units + "; second: " + second.units;
  result.validate();
  return result;
}

model::LinearSystem negative_feedback(const model::LinearSystem& loop) {
  loop.validate();
  if (loop.input_count() < 1 || loop.input_count() != loop.output_count()) {
    throw std::invalid_argument("negative_feedback: a nonempty square loop is required");
  }
  const auto count = loop.input_count();
  const Eigen::MatrixXd denominator =
      Eigen::MatrixXd::Identity(count, count) + loop.feedthrough_matrix();
  Eigen::FullPivLU<Eigen::MatrixXd> solve(denominator);
  if (!solve.isInvertible() || solve.rcond() < 128.0 * kEps) {
    throw std::invalid_argument("negative_feedback: I+D is singular or ill-conditioned");
  }
  const Eigen::MatrixXd solved_c = solve.solve(loop.output_matrix());
  model::LinearSystem result = loop;
  result.a = loop.a - loop.b * solved_c;
  result.b = loop.b * solve.solve(Eigen::MatrixXd::Identity(count, count));
  result.c = solved_c;
  result.d = solve.solve(loop.feedthrough_matrix());
  result.description = "Negative identity feedback: " + loop.description;
  result.validate();
  return result;
}
}  // namespace galata::synth
