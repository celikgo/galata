// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the reachability and observability analysis declared in
// include/galata/analyze/gramians.hpp. The references and the validity envelope
// are there; this file carries only what the arithmetic does.

#include "galata/analyze/gramians.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace galata::analyze {
namespace {

// An orthonormal basis for the column space of `m`, and the singular values the
// decision was made on.
//
// WHY NOT THE KRYLOV MATRIX. The textbook reachability test forms
// [B, AB, ..., A^{n-1}B] and takes its rank. For n = 16 that matrix contains
// A^15, whose entries span the fifteenth power of the spectrum's spread — on a
// hover linearisation with rotor-lag poles near -29 and integrators at zero,
// that is a range no floating-point rank test can resolve. The staircase below
// re-orthonormalises at every step instead, so no power of A is ever formed and
// the condition number of what is decomposed stays bounded by the basis itself.
struct OrthogonalBasis {
  Eigen::MatrixXd basis;
  std::vector<double> singular_values;
};

OrthogonalBasis orthonormal_columns(const Eigen::MatrixXd& m, double relative_tolerance) {
  OrthogonalBasis out;
  if (m.size() == 0) {
    out.basis = Eigen::MatrixXd::Zero(m.rows(), 0);
    return out;
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeThinU);
  if (svd.info() != Eigen::Success) {
    throw std::runtime_error("analyse_gramians: the singular value decomposition failed");
  }
  const Eigen::VectorXd values = svd.singularValues();
  out.singular_values.assign(values.data(), values.data() + values.size());
  const double largest = values.size() > 0 ? values(0) : 0.0;
  const double floor = relative_tolerance * largest;
  Eigen::Index kept = 0;
  while (kept < values.size() && values(kept) > floor) {
    ++kept;
  }
  out.basis = svd.matrixU().leftCols(kept);
  return out;
}

// The subspace reachable from `seed` under `a`, by repeated orthogonalisation.
// Terminates in at most `n` rounds because each round either grows the basis or
// closes it, and a basis of an n-dimensional space cannot grow past n.
SubspaceAnalysis staircase(const Eigen::MatrixXd& a,
                           const Eigen::MatrixXd& seed,
                           const std::vector<std::string>& state_names,
                           double relative_tolerance) {
  SubspaceAnalysis out;
  const Eigen::Index n = a.rows();
  out.state_count = static_cast<int>(n);

  OrthogonalBasis current = orthonormal_columns(seed, relative_tolerance);
  out.singular_values = current.singular_values;
  for (Eigen::Index round = 0; round < n; ++round) {
    if (current.basis.cols() == 0 || current.basis.cols() == n) {
      break;
    }
    Eigen::MatrixXd grown(n, current.basis.cols() * 2);
    grown << current.basis, a * current.basis;
    OrthogonalBasis next = orthonormal_columns(grown, relative_tolerance);
    if (next.basis.cols() <= current.basis.cols()) {
      // Closed: another application of A adds no direction the basis lacks.
      out.singular_values = next.singular_values;
      break;
    }
    current = std::move(next);
    out.singular_values = current.singular_values;
  }
  out.rank = static_cast<int>(current.basis.cols());

  double smallest_retained = std::numeric_limits<double>::infinity();
  double largest_retained = 0.0;
  for (int k = 0; k < out.rank && k < static_cast<int>(out.singular_values.size()); ++k) {
    smallest_retained =
        std::fmin(smallest_retained, out.singular_values[static_cast<std::size_t>(k)]);
    largest_retained =
        std::fmax(largest_retained, out.singular_values[static_cast<std::size_t>(k)]);
  }
  out.retained_condition_number = (out.rank > 0 && smallest_retained > 0.0)
                                      ? largest_retained / smallest_retained
                                      : std::numeric_limits<double>::infinity();

  if (out.rank >= static_cast<int>(n)) {
    return out;
  }

  // THE DIRECTIONS THAT ARE MISSING, named in the model's own coordinates. The
  // orthogonal complement of the basis is what the inputs cannot move (or the
  // outputs cannot see), and it is far more useful than the deficiency count:
  // "you cannot reach a direction that is mostly the battery state of charge"
  // is actionable and "the rank is 15 of 16" is not.
  Eigen::MatrixXd padded(n, n);
  padded.leftCols(current.basis.cols()) = current.basis;
  padded.rightCols(n - current.basis.cols()).setZero();
  const Eigen::JacobiSVD<Eigen::MatrixXd> complement(padded, Eigen::ComputeFullU);
  for (Eigen::Index column = current.basis.cols(); column < n; ++column) {
    StateDirection direction;
    direction.coordinates = complement.matrixU().col(column);
    std::vector<std::pair<double, std::size_t>> shares;
    for (Eigen::Index k = 0; k < n; ++k) {
      const double share = direction.coordinates(k) * direction.coordinates(k);
      if (share > 0.01) {
        shares.emplace_back(share, static_cast<std::size_t>(k));
      }
    }
    std::sort(shares.begin(), shares.end(), [](const auto& left, const auto& right) {
      return left.first != right.first ? left.first > right.first : left.second < right.second;
    });
    for (const auto& [share, index] : shares) {
      std::ostringstream entry;
      entry << (index < state_names.size() ? state_names[index] : "state " + std::to_string(index))
            << " (" << std::fixed << std::setprecision(2) << share << ")";
      direction.dominant_states.push_back(entry.str());
    }
    out.missing_directions.push_back(std::move(direction));
  }
  return out;
}

// The finite-horizon Gramian, by integrating the matrix differential equation
// rather than by a Lyapunov solve.
//
// WHY THIS WAY. The infinite-horizon Gramian solves A X + X A' + B B' = 0, which
// has a unique solution only for a strictly stable A — and the models this is
// for are not strictly stable, so that equation has no unique solution and a
// solver would return something that is not a Gramian. The finite-horizon
// integral exists for every A, and integrating
//
//     dX/dt = A X + X A' + B B',    X(0) = 0
//
// gives it directly through the fixed-step RK4 already in the tree, with the
// same determinism properties: a declared number of steps and no early exit.
Eigen::MatrixXd finite_horizon_gramian(const Eigen::MatrixXd& a,
                                       const Eigen::MatrixXd& source,
                                       double horizon_s,
                                       int steps) {
  const Eigen::MatrixXd constant = source * source.transpose();
  const double step = horizon_s / static_cast<double>(steps);
  Eigen::MatrixXd x = Eigen::MatrixXd::Zero(a.rows(), a.rows());
  const auto derivative = [&](const Eigen::MatrixXd& state) {
    return (a * state + state * a.transpose() + constant).eval();
  };
  for (int k = 0; k < steps; ++k) {
    const Eigen::MatrixXd k1 = derivative(x);
    const Eigen::MatrixXd k2 = derivative(x + 0.5 * step * k1);
    const Eigen::MatrixXd k3 = derivative(x + 0.5 * step * k2);
    const Eigen::MatrixXd k4 = derivative(x + step * k3);
    x += (step / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }
  // Symmetric by construction; symmetrised anyway, because the round-off that
  // breaks the symmetry would otherwise reach a self-adjoint eigensolver that
  // is entitled to assume it.
  return (0.5 * (x + x.transpose())).eval();
}

std::vector<double> descending_eigenvalues(const Eigen::MatrixXd& symmetric,
                                           double& condition_number) {
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("analyse_gramians: the Gramian eigendecomposition failed");
  }
  std::vector<double> values(solver.eigenvalues().data(),
                             solver.eigenvalues().data() + solver.eigenvalues().size());
  std::sort(values.begin(), values.end(), std::greater<double>());
  const double largest = values.empty() ? 0.0 : values.front();
  const double smallest = values.empty() ? 0.0 : values.back();
  condition_number =
      (smallest > 0.0) ? largest / smallest : std::numeric_limits<double>::infinity();
  return values;
}

}  // namespace

GramianAnalysis analyse_gramians(const model::LinearSystem& system, const GramianOptions& options) {
  system.validate();
  if (!(options.horizon_s > 0.0) || !std::isfinite(options.horizon_s)) {
    throw std::invalid_argument(
        "analyze.gramians: `horizon_s` must be positive and finite, and it is required rather "
        "than defaulted. The answer depends on it: a finite-horizon Gramian measures how far "
        "the inputs can move each direction in that many seconds, and a default would put a "
        "horizon nobody chose into a figure a reader will quote");
  }
  if (options.steps < 1) {
    throw std::invalid_argument("analyze.gramians: `steps` must be at least one");
  }
  if (!(options.rank_tolerance > 0.0) || !(options.rank_tolerance < 1.0)) {
    throw std::invalid_argument(
        "analyze.gramians: `rank_tolerance` must lie strictly between zero and one; it is a "
        "relative singular-value floor, not an absolute one");
  }

  GramianAnalysis out;
  out.horizon_s = options.horizon_s;
  out.steps = options.steps;
  out.rank_tolerance = options.rank_tolerance;

  const Eigen::MatrixXd& a = system.a;
  const Eigen::EigenSolver<Eigen::MatrixXd> spectrum(a, false);
  if (spectrum.info() != Eigen::Success || !spectrum.eigenvalues().allFinite()) {
    throw std::runtime_error(
        "analyze.gramians: the eigenvalues of A could not be computed, so whether the "
        "infinite-horizon Gramians exist cannot be reported either way");
  }
  out.rightmost_eigenvalue_real_part = spectrum.eigenvalues().real().maxCoeff();
  // Strictly stable, with the same scale-relative floor the stability checks in
  // this directory use: an eigenvalue this close to the axis does not support a
  // claim that a T -> infinity limit converges.
  const double axis_floor = 1024.0 * static_cast<double>(a.rows())
                            * std::numeric_limits<double>::epsilon()
                            * std::max(a.stableNorm(), std::numeric_limits<double>::min());
  out.spectrum_is_strictly_stable = out.rightmost_eigenvalue_real_part < -axis_floor;

  out.reachability = staircase(a, system.b, system.state_names, options.rank_tolerance);
  const Eigen::MatrixXd c = system.output_matrix();
  out.observability =
      staircase(a.transpose(), c.transpose(), system.state_names, options.rank_tolerance);

  out.controllability = finite_horizon_gramian(a, system.b, options.horizon_s, options.steps);
  out.observability_gramian =
      finite_horizon_gramian(a.transpose(), c.transpose(), options.horizon_s, options.steps);
  out.controllability_eigenvalues =
      descending_eigenvalues(out.controllability, out.controllability_condition_number);
  out.observability_eigenvalues =
      descending_eigenvalues(out.observability_gramian, out.observability_condition_number);

  std::ostringstream assumptions;
  assumptions << "ranks are numerical decisions at a relative singular-value floor of "
              << options.rank_tolerance
              << ", taken from an orthogonal staircase rather than from a Krylov matrix, so no "
                 "power of A is formed; the Gramians are integrals over a declared horizon of "
              << options.horizon_s << " s in " << options.steps
              << " fixed RK4 steps and are NOT the infinite-horizon Gramians, which ";
  if (out.spectrum_is_strictly_stable) {
    assumptions << "do exist for this model but are not computed here";
  } else {
    assumptions << "do not exist for this model at all — its rightmost eigenvalue has real part "
                << out.rightmost_eigenvalue_real_part;
  }
  assumptions << ". The Gramians are in the model's own mixed units and their smallest "
                 "eigenvalues reflect that spread as much as any near-unreachable direction; "
                 "the rank results do not, being relative. Every actuator is unbounded here, so "
                 "a direction reported as reachable may be reachable only through a command no "
                 "actuator can produce";
  out.assumptions = assumptions.str();
  return out;
}

}  // namespace galata::analyze
