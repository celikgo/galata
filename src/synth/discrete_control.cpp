// SPDX-License-Identifier: Apache-2.0
//
// Pappas, Laub and Sandell (1980): the stabilising DARE solution is the
// deflating subspace of the symplectic pencil for the eigenvalues inside the
// unit circle. The ordered-Schur construction is the one `control.cpp` uses for
// the Hamiltonian, with the imaginary axis replaced by the unit circle
// throughout — that substitution IS the discrete-time difference, and keeping
// the two files structurally parallel is deliberate so that a reader who knows
// one can check the other.

#include "galata/synth/discrete_control.hpp"

#include "galata/numerics/matrix_exponential.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace galata::synth {
namespace {

constexpr double kEps = std::numeric_limits<double>::epsilon();

void require_symmetric(const Eigen::MatrixXd& matrix, const char* name, const char* routine) {
  if (!matrix.allFinite() || matrix.rows() != matrix.cols()
      || (matrix - matrix.transpose()).norm() > 64.0 * kEps * matrix.norm()) {
    throw std::invalid_argument(std::string(routine) + ": " + name
                                + " must be finite and symmetric");
  }
}

// Move one eigenvalue INSIDE THE UNIT CIRCLE left by adjacent unitary Schur
// exchanges. Identical in construction to `control.cpp`'s `exchange`: for
// [a b; 0 d], [b, d-a] is an eigenvector for d, and its orthogonal complement
// gives a unitary basis with no matrix inverse. The number of exchanges is
// bounded by the matrix dimension, and ties keep their Schur order.
void exchange(Eigen::MatrixXcd& t, Eigen::MatrixXcd& z, Eigen::Index i) {
  const std::complex<double> b = t(i, i + 1);
  const std::complex<double> delta = t(i + 1, i + 1) - t(i, i);
  const double scale = std::max(std::abs(b), std::abs(delta));
  if (!(scale > 0.0) || !std::isfinite(scale)) {
    throw std::runtime_error("solve_dare: inseparable symplectic eigenvalues");
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

// Anderson and Moore (1990), ch. 6: the discrete analogue of the continuous
// detectability requirement. The PBH test needs [lambda I - Abar; sqrt(Qbar)]
// at full column rank at every NONSTABLE eigenvalue, which in discrete time
// means every eigenvalue with |lambda| >= 1 rather than Re(lambda) >= 0.
// Unpenalised modes strictly inside the circle are permitted.
void require_detectable(const Eigen::MatrixXd& abar,
                        const Eigen::MatrixXd& qbar,
                        const Eigen::MatrixXd& reference_norm) {
  const Eigen::Index count = abar.rows();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> cost(qbar);
  Eigen::EigenSolver<Eigen::MatrixXd> modes(abar, false);
  if (cost.info() != Eigen::Success || modes.info() != Eigen::Success) {
    throw std::runtime_error("solve_dare: detectability spectrum failed");
  }
  const Eigen::MatrixXd root =
      cost.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() * cost.eigenvectors().transpose();
  const double root_scale = root.stableNorm();
  const double state_scale =
      std::max(reference_norm.stableNorm(), std::numeric_limits<double>::min());

  for (Eigen::Index i = 0; i < count; ++i) {
    const std::complex<double> pole = modes.eigenvalues()(i);
    // The nonstable half of the discrete plane is the closed exterior of the
    // unit circle. The floor keeps a mode that sits on the circle to within
    // roundoff on the nonstable side, where it must be penalised.
    if (std::abs(pole) < 1.0 - 128.0 * kEps) {
      continue;
    }
    Eigen::MatrixXcd pbh(2 * count, count);
    pbh.topRows(count) =
        (pole * Eigen::MatrixXcd::Identity(count, count) - abar.cast<std::complex<double>>());
    pbh.bottomRows(count) = root.cast<std::complex<double>>();
    Eigen::JacobiSVD<Eigen::MatrixXcd> rank(pbh);
    if (rank.info() != Eigen::Success
        || rank.singularValues()(count - 1)
               <= 128.0 * kEps * static_cast<double>(count) * std::max(root_scale, state_scale)) {
      throw std::invalid_argument(
          "solve_dare: a mode on or outside the unit circle is unpenalised, so the "
          "stabilising solution is numerically unresolved; penalise the nonstable modes "
          "or rescale the problem");
    }
  }
}

}  // namespace

DareSolution solve_dare(const Eigen::MatrixXd& a,
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
        "solve_dare: finite compatible matrices and 1..256 states required");
  }
  require_symmetric(q, "Q", "solve_dare");
  require_symmetric(r, "R", "solve_dare");
  const Eigen::MatrixXd n = cross.size() == 0 ? Eigen::MatrixXd::Zero(states, inputs) : cross;
  if (n.rows() != states || n.cols() != inputs || !n.allFinite()) {
    throw std::invalid_argument(
        "solve_dare: N must be finite with one row per state and column per input");
  }
  Eigen::LLT<Eigen::MatrixXd> r_factor(r);
  if (r_factor.info() != Eigen::Success) {
    throw std::invalid_argument("solve_dare: R must be positive definite");
  }

  // COMPLETE THE SQUARE ON THE CROSS TERM. Abar = A - B R^-1 N' and
  // Qbar = Q - N R^-1 N' turn the cross-term equation into one without a cross
  // term whose solution X is THE SAME. The gain then comes back as
  // K = Kbar + R^-1 N', and the closed loop A - B K equals Abar - B Kbar.
  // Verified at the end against the residual of the ORIGINAL equation, so this
  // algebra is checked rather than trusted.
  const Eigen::MatrixXd rb = r_factor.solve(b.transpose());
  const Eigen::MatrixXd rn = r_factor.solve(n.transpose());
  const Eigen::MatrixXd abar = a - b * rn;
  const Eigen::MatrixXd qbar = q - n * rn;
  const Eigen::MatrixXd g = b * rb;
  if (!rb.allFinite() || !rn.allFinite() || !abar.allFinite() || !qbar.allFinite()
      || !g.allFinite()) {
    throw std::runtime_error("solve_dare: transformed cost overflow; rescale the problem");
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> block_cost(qbar);
  if (block_cost.info() != Eigen::Success
      || block_cost.eigenvalues().minCoeff() < -128.0 * kEps * std::max(q.norm(), qbar.norm())) {
    throw std::invalid_argument("solve_dare: block cost [Q N; N' R] must be positive semidefinite");
  }
  require_detectable(abar, qbar, abar);

  DareSolution result;

  // THE SYMPLECTIC MATRIX. The pencil is L = [[Abar, 0], [-Qbar, I]] against
  // M = [[I, G], [0, Abar']], and S = M^-1 L is
  //
  //   [[Abar + G Abar^-T Qbar,  -G Abar^-T],
  //    [        -Abar^-T Qbar,   Abar^-T  ]]
  //
  // whose eigenvalues come in reciprocal pairs about the unit circle, exactly as
  // the Hamiltonian's come in pairs about the imaginary axis. This is the one
  // place the construction needs Abar invertible; the alternative is a QZ
  // decomposition of the pencil, which avoids the inverse and is the right
  // answer for a genuinely singular Abar. It is not implemented, so a singular
  // Abar is refused by name rather than answered badly.
  const Eigen::FullPivLU<Eigen::MatrixXd> transition(abar.transpose());
  Eigen::JacobiSVD<Eigen::MatrixXd> transition_svd(abar);
  if (transition_svd.info() != Eigen::Success
      || transition_svd.singularValues()(states - 1) <= 0.0) {
    throw std::runtime_error("solve_dare: state transition singular value decomposition failed");
  }
  result.transition_condition =
      transition_svd.singularValues()(0) / transition_svd.singularValues()(states - 1);
  if (!std::isfinite(result.transition_condition)
      || result.transition_condition > 1.0 / std::sqrt(kEps)) {
    std::ostringstream message;
    message << "solve_dare: A - B R^-1 N' has condition number " << result.transition_condition
            << ", which the symplectic construction cannot invert reliably. A discrete plant "
               "from a zero-order-hold discretisation is always invertible, so this usually "
               "means the cross term is large enough to cancel it; rescale the cost or use a "
               "pencil-based solver, which this routine is not.";
    throw std::runtime_error(message.str());
  }

  const Eigen::MatrixXd inverse_transition =
      transition.solve(Eigen::MatrixXd::Identity(states, states));
  Eigen::MatrixXd symplectic(2 * states, 2 * states);
  symplectic << abar + g * inverse_transition * qbar, -g * inverse_transition,
      -inverse_transition * qbar, inverse_transition;
  if (!symplectic.allFinite()) {
    throw std::runtime_error("solve_dare: symplectic overflow; rescale the problem");
  }

  Eigen::ComplexSchur<Eigen::MatrixXd> schur(symplectic);
  if (schur.info() != Eigen::Success) {
    throw std::runtime_error("solve_dare: symplectic Schur decomposition failed");
  }
  Eigen::MatrixXcd t = schur.matrixT();
  Eigen::MatrixXcd z = schur.matrixU();

  result.symplectic_separation = std::numeric_limits<double>::infinity();
  Eigen::Index inside = 0;
  const double separation_floor = 128.0 * kEps * symplectic.norm();
  for (Eigen::Index i = 0; i < 2 * states; ++i) {
    const double magnitude = std::abs(t(i, i));
    // Distance from the UNIT CIRCLE, which is the discrete separation measure.
    result.symplectic_separation =
        std::min(result.symplectic_separation, std::abs(magnitude - 1.0));
    if (!std::isfinite(magnitude) || std::abs(magnitude - 1.0) <= separation_floor) {
      throw std::runtime_error(
          "solve_dare: symplectic spectrum touches the unit circle; solution is not separated");
    }
    if (magnitude < 1.0) {
      for (Eigen::Index j = i; j > inside; --j) {
        exchange(t, z, j - 1);
      }
      ++inside;
    }
  }
  if (inside != states) {
    std::ostringstream message;
    message << "solve_dare: the symplectic matrix has " << inside << " eigenvalues inside the "
            << "unit circle, not the " << states << " a stabilising solution requires";
    throw std::runtime_error(message.str());
  }

  const Eigen::MatrixXcd u1 = z.topLeftCorner(states, states);
  const Eigen::MatrixXcd u2 = z.bottomLeftCorner(states, states);
  Eigen::JacobiSVD<Eigen::MatrixXcd> svd(u1);
  result.subspace_condition = svd.singularValues()(0) / svd.singularValues()(states - 1);
  if (!std::isfinite(result.subspace_condition)
      || result.subspace_condition > 1.0 / std::sqrt(kEps)) {
    throw std::runtime_error(
        "solve_dare: deflating subspace is ill-conditioned; rescale the problem");
  }
  const Eigen::MatrixXcd xc = u1.transpose().fullPivLu().solve(u2.transpose()).transpose();
  const double scale = std::max(1.0, xc.norm());
  const double roundoff = 512.0 * static_cast<double>(states) * kEps * result.subspace_condition;
  if (!xc.allFinite() || xc.imag().norm() > roundoff * scale) {
    throw std::runtime_error("solve_dare: deflating subspace did not produce a real solution");
  }
  const Eigen::MatrixXd raw = xc.real();
  result.symmetry_defect = (raw - raw.transpose()).norm() / scale;
  if (result.symmetry_defect > roundoff) {
    throw std::runtime_error("solve_dare: Riccati solution is not symmetric within roundoff");
  }
  result.x = 0.5 * (raw + raw.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solution_eigen(result.x);
  if (solution_eigen.info() != Eigen::Success
      || solution_eigen.eigenvalues().minCoeff() < -roundoff * scale) {
    throw std::runtime_error("solve_dare: stabilising solution is not positive semidefinite");
  }

  // K = (R + B' X B)^-1 (B' X A + N'), the discrete gain. The B' X B term is
  // what distinguishes this from the continuous formula and is why R alone
  // cannot be reused as the factor.
  const Eigen::MatrixXd weighted = r + b.transpose() * result.x * b;
  Eigen::LLT<Eigen::MatrixXd> weighted_factor(weighted);
  if (weighted_factor.info() != Eigen::Success) {
    throw std::runtime_error(
        "solve_dare: R + B' X B is not positive definite, so the discrete gain does not exist");
  }
  result.k = weighted_factor.solve(b.transpose() * result.x * a + n.transpose());

  // RESIDUAL OF THE EQUATION AS POSED, cross term and all. This is what checks
  // the completing-the-square algebra above: if the reduction or the gain
  // formula were wrong, X would satisfy some other equation and this would fail.
  const Eigen::MatrixXd closed = a - b * result.k;
  const Eigen::MatrixXd transported = a.transpose() * result.x * a;
  const Eigen::MatrixXd quadratic = (a.transpose() * result.x * b + n) * result.k;
  const double denominator =
      std::max(std::numeric_limits<double>::min(),
               result.x.norm() + transported.norm() + q.norm() + quadratic.norm());
  result.relative_residual = (result.x - transported - q + quadratic).norm() / denominator;
  // The same cap the continuous solver applies: poor conditioning must not buy
  // an arbitrarily weak gate, and no caller option can relax it.
  result.residual_budget = std::min(1e-8, roundoff);
  if (!std::isfinite(result.relative_residual)
      || result.relative_residual > result.residual_budget) {
    std::ostringstream message;
    message << "solve_dare: residual " << result.relative_residual
            << " exceeds the conditioning-scaled roundoff budget " << result.residual_budget;
    throw std::runtime_error(message.str());
  }

  Eigen::EigenSolver<Eigen::MatrixXd> closed_spectrum(closed, false);
  if (closed_spectrum.info() != Eigen::Success) {
    throw std::runtime_error("solve_dare: closed-loop spectrum failed");
  }
  double radius = 0.0;
  for (Eigen::Index i = 0; i < closed_spectrum.eigenvalues().size(); ++i) {
    const std::complex<double> value = closed_spectrum.eigenvalues()(i);
    result.closed_loop_eigenvalues.push_back(value);
    radius = std::max(radius, std::abs(value));
  }
  result.spectral_radius = radius;
  if (!(radius < 1.0)) {
    std::ostringstream message;
    message << "solve_dare: the closed loop has spectral radius " << radius
            << ", which is not inside the unit circle, so the returned solution is not "
               "stabilising";
    throw std::runtime_error(message.str());
  }
  return result;
}

DiscreteCost discretize_cost(const model::LinearSystem& plant,
                             const Eigen::MatrixXd& q,
                             const Eigen::MatrixXd& r,
                             const Eigen::MatrixXd& n,
                             double sample_time_s) {
  plant.validate();
  if (!(sample_time_s > 0.0) || !std::isfinite(sample_time_s)) {
    std::ostringstream message;
    message << "discretize_cost: the sample time is " << sample_time_s
            << ", which is not a positive finite number of seconds";
    throw std::invalid_argument(message.str());
  }
  const Eigen::Index states = plant.state_count();
  const Eigen::Index inputs = plant.input_count();
  if (inputs == 0) {
    throw std::invalid_argument(
        "discretize_cost: the plant has no inputs, so there is no input cost to discretise");
  }
  if (q.rows() != states || q.cols() != states || r.rows() != inputs || r.cols() != inputs) {
    throw std::invalid_argument("discretize_cost: Q must be n x n and R must be m x m");
  }
  require_symmetric(q, "Q", "discretize_cost");
  require_symmetric(r, "R", "discretize_cost");
  const Eigen::MatrixXd cross = n.size() == 0 ? Eigen::MatrixXd::Zero(states, inputs) : n;
  if (cross.rows() != states || cross.cols() != inputs || !cross.allFinite()) {
    throw std::invalid_argument(
        "discretize_cost: N must be finite with one row per state and column per input");
  }

  const Eigen::Index size = states + inputs;
  // Z propagates [x; u] with u held: d/dt [x; u] = [[A, B], [0, 0]] [x; u].
  Eigen::MatrixXd z = Eigen::MatrixXd::Zero(size, size);
  z.topLeftCorner(states, states) = plant.a;
  z.topRightCorner(states, inputs) = plant.b;

  // W is the continuous block cost, so that the instantaneous rate is
  // [x; u]' W [x; u].
  Eigen::MatrixXd w(size, size);
  w << q, cross, cross.transpose(), r;

  // VAN LOAN'S THEOREM 1. For C = [[A1, B1], [0, A2]],
  //   expm(C t) = [[expm(A1 t), F(t)], [0, expm(A2 t)]]
  //   F(t) = integral from 0 to t of expm(A1 (t-s)) B1 expm(A2 s) ds.
  // With A1 = -Z', B1 = W and A2 = Z, F(T) = expm(-Z'T) * Wd, where
  //   Wd = integral from 0 to T of expm(Z' s) W expm(Z s) ds
  // is the exact cost of one held interval. So Wd = expm(Z T)' F(T), and the
  // bottom-right block of the same exponential IS expm(Z T). One exponential
  // gives the whole discretised cost, with no truncated series anywhere.
  Eigen::MatrixXd block = Eigen::MatrixXd::Zero(2 * size, 2 * size);
  block.topLeftCorner(size, size) = -z.transpose();
  block.topRightCorner(size, size) = w;
  block.bottomRightCorner(size, size) = z;
  block *= sample_time_s;

  const auto exponential = numerics::matrix_exponential(block);
  const Eigen::MatrixXd f12 = exponential.value.topRightCorner(size, size);
  const Eigen::MatrixXd f22 = exponential.value.bottomRightCorner(size, size);
  const Eigen::MatrixXd discretised = f22.transpose() * f12;

  DiscreteCost result;
  result.sample_time_s = sample_time_s;
  result.exponential_error_bound = exponential.backward_error_bound;
  result.exponential_squarings = exponential.squarings;

  const double norm = std::max(discretised.norm(), std::numeric_limits<double>::min());
  result.symmetry_defect = (discretised - discretised.transpose()).norm() / norm;
  const Eigen::MatrixXd symmetric = 0.5 * (discretised + discretised.transpose());

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(symmetric);
  if (spectrum.info() != Eigen::Success) {
    throw std::runtime_error("discretize_cost: discretised block cost spectrum failed");
  }
  result.minimum_block_eigenvalue = spectrum.eigenvalues().minCoeff() / norm;
  // The integral of a positive semidefinite form is positive semidefinite, so a
  // materially negative eigenvalue here is a numerical failure rather than a
  // property of the problem, and it is refused rather than clipped.
  if (result.minimum_block_eigenvalue < -128.0 * kEps * static_cast<double>(2 * size)) {
    std::ostringstream message;
    message << "discretize_cost: the discretised block cost has a negative eigenvalue at "
            << result.minimum_block_eigenvalue
            << " of its norm, so the integral lost definiteness. The continuous block cost "
               "[Q N; N' R] must be positive semidefinite, and a very long sample time or a "
               "badly scaled cost can still lose it here.";
    throw std::runtime_error(message.str());
  }

  result.q = symmetric.topLeftCorner(states, states);
  result.n = symmetric.topRightCorner(states, inputs);
  result.r = symmetric.bottomRightCorner(inputs, inputs);

  std::ostringstream assumptions;
  assumptions << "Continuous quadratic cost integrated exactly over one " << sample_time_s
              << " s interval under a zero-order hold, by the exponential of Van Loan's block "
                 "matrix. The hold produces a state-input cross term N even where the "
                 "continuous cost had none, and dropping it would minimise a different "
                 "objective. The weights belong to this sample time alone.";
  result.assumptions = assumptions.str();
  return result;
}

SampledLqrDesign design_sampled_lqr(const model::LinearSystem& plant,
                                    const Eigen::MatrixXd& q,
                                    const Eigen::MatrixXd& r,
                                    const Eigen::MatrixXd& n,
                                    double sample_time_s) {
  SampledLqrDesign design;
  design.sample_time_s = sample_time_s;
  design.continuous_q = q;
  design.continuous_r = r;
  design.continuous_n =
      n.size() == 0 ? Eigen::MatrixXd::Zero(plant.state_count(), plant.input_count()) : n;

  // Order matters and is the point: the PLANT and the COST are both discretised
  // under the same declared hold, and only then is the discrete problem solved.
  design.discretisation = model::discretize_zoh(plant, sample_time_s);
  design.cost = discretize_cost(plant, q, r, design.continuous_n, sample_time_s);
  design.riccati = solve_dare(design.discretisation.system.a,
                              design.discretisation.system.b,
                              design.cost.q,
                              design.cost.r,
                              design.cost.n);

  design.closed_loop = design.discretisation.system;
  design.closed_loop.a =
      design.discretisation.system.a - design.discretisation.system.b * design.riccati.k;
  design.closed_loop.b = Eigen::MatrixXd::Zero(plant.state_count(), plant.input_count());
  design.closed_loop.description = plant.description.empty()
                                       ? std::string("Sampled closed loop")
                                       : plant.description + " — sampled closed loop";
  design.closed_loop.validate();
  return design;
}

}  // namespace galata::synth
