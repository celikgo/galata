// SPDX-License-Identifier: Apache-2.0

#include "galata/numerics/matrix_exponential.hpp"

#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace galata::numerics {
namespace {

struct PadeOrder {
  int order;
  double theta;
};

// Higham 2005, Table 2.3: the largest ||A||_1 for which the diagonal Pade
// approximant of the given order still meets the double-precision backward
// error bound. Published constants, transcribed, not values fitted here. Each
// is a threshold on a DIMENSIONLESS matrix one-norm; none converts a unit.
constexpr double kThetaThree = 1.495585217958292e-2;    // GALATA_SI_EXEMPT: one-norm threshold
constexpr double kThetaFive = 2.539398330063230e-1;     // GALATA_SI_EXEMPT: one-norm threshold
constexpr double kThetaSeven = 9.504178996162932e-1;    // GALATA_SI_EXEMPT: one-norm threshold
constexpr double kThetaNine = 2.097847961257068e0;      // GALATA_SI_EXEMPT: one-norm threshold
constexpr double kThetaThirteen = 5.371920351148152e0;  // GALATA_SI_EXEMPT: one-norm threshold

// The backward error each threshold was chosen to meet: unit roundoff in IEEE
// double, 2^-53. Higham picks theta so that ||dA||_1 / ||A||_1 is bounded by it.
constexpr double kBackwardErrorBound = 1.1102230246251565e-16;  // GALATA_SI_EXEMPT: unit roundoff

// Pade numerator/denominator coefficients b_k, k ascending, for the diagonal
// approximants. Higham 2005, equations (2.2)-(2.3).
const std::vector<double>& coefficients(int order) {
  static const std::vector<double> three{120.0, 60.0, 12.0, 1.0};
  static const std::vector<double> five{30240.0, 15120.0, 3360.0, 420.0, 30.0, 1.0};
  static const std::vector<double> seven{
      17297280.0, 8648640.0, 1995840.0, 277200.0, 25200.0, 1512.0, 56.0, 1.0};
  static const std::vector<double> nine{17643225600.0,
                                        8821612800.0,
                                        2075673600.0,
                                        302702400.0,
                                        30270240.0,
                                        2162160.0,
                                        110880.0,
                                        3960.0,
                                        90.0,
                                        1.0};
  static const std::vector<double> thirteen{64764752532480000.0,
                                            32382376266240000.0,
                                            7771770303897600.0,
                                            1187353796428800.0,
                                            129060195264000.0,
                                            10559470521600.0,
                                            670442572800.0,
                                            33522128640.0,
                                            1323241920.0,
                                            40840800.0,
                                            960960.0,
                                            16380.0,
                                            182.0,
                                            1.0};
  switch (order) {
    case 3:
      return three;
    case 5:
      return five;
    case 7:
      return seven;
    case 9:
      return nine;
    default:
      return thirteen;
  }
}

// Split the Pade polynomial into its odd part U = A * (sum over odd k) and its
// even part V = sum over even k, then solve (V - U) X = (V + U). The SPLIT is
// Higham's and is what makes one solve serve both numerator and denominator.
//
// The powers are accumulated straightforwardly rather than by his flop-optimal
// nesting for order 13, which would build A^2, A^4 and A^6 and nest the rest.
// That costs a few more matrix products at order 13 and is no less accurate;
// the state dimensions this repository linearises are small enough that
// legibility is worth more than the products, and the operation count stays a
// fixed function of the order either way.
Eigen::MatrixXd pade(const Eigen::MatrixXd& a, int order) {
  const auto& b = coefficients(order);
  const Eigen::Index n = a.rows();
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(n, n);
  const Eigen::MatrixXd a2 = a * a;

  Eigen::MatrixXd odd = Eigen::MatrixXd::Zero(n, n);
  Eigen::MatrixXd even = Eigen::MatrixXd::Zero(n, n);

  // Powers of A^2 built once and reused, ascending, so the loop below never
  // recomputes one. A fixed number of products for a fixed order: no
  // data-dependent work, per the header's determinism note.
  Eigen::MatrixXd power = identity;
  for (std::size_t k = 0; k + 1 < b.size(); k += 2) {
    even += b[k] * power;
    odd += b[k + 1] * power;
    if (k + 3 < b.size()) {
      power = power * a2;
    }
  }

  const Eigen::MatrixXd u = a * odd;
  const Eigen::MatrixXd v = even;
  const Eigen::MatrixXd left = v - u;
  const Eigen::MatrixXd right = v + u;

  const Eigen::FullPivLU<Eigen::MatrixXd> lu(left);
  if (!lu.isInvertible()) {
    // The Pade denominator is singular only when the scaled matrix sits on a
    // pole of the approximant, which the thresholds are chosen to avoid. If it
    // happens the result would be meaningless, so it is refused rather than
    // returned.
    throw std::invalid_argument(
        "matrix_exponential: the Pade denominator is singular for this matrix, so no "
        "approximant of the selected order exists; the matrix is far outside the regime "
        "the published thresholds cover");
  }
  return lu.solve(right);
}

}  // namespace

MatrixExponential matrix_exponential(const Eigen::MatrixXd& m) {
  if (m.rows() != m.cols()) {
    throw std::invalid_argument("matrix_exponential: the matrix must be square, but it is "
                                + std::to_string(m.rows()) + " by " + std::to_string(m.cols()));
  }
  if (m.rows() == 0) {
    throw std::invalid_argument("matrix_exponential: the matrix is empty");
  }
  if (!m.allFinite()) {
    throw std::invalid_argument(
        "matrix_exponential: the matrix has a non-finite entry, so its exponential is not "
        "defined; a caller reaching here has usually built a model from an unconverged solve");
  }

  MatrixExponential result;
  result.one_norm = m.cwiseAbs().colwise().sum().maxCoeff();
  result.backward_error_bound = kBackwardErrorBound;

  const PadeOrder direct[] = {{3, kThetaThree}, {5, kThetaFive}, {7, kThetaSeven}, {9, kThetaNine}};
  for (const auto& candidate : direct) {
    if (result.one_norm <= candidate.theta) {
      result.pade_order = candidate.order;
      result.squarings = 0;
      result.value = pade(m, candidate.order);
      return result;
    }
  }

  // Scale so the one-norm falls inside the order-13 threshold, approximate, and
  // square back up. `ceil(log2(.))` is a closed form, not a search.
  const int squarings =
      std::max(0, static_cast<int>(std::ceil(std::log2(result.one_norm / kThetaThirteen))));
  result.pade_order = 13;
  result.squarings = squarings;

  Eigen::MatrixXd scaled = m / std::pow(2.0, squarings);
  Eigen::MatrixXd value = pade(scaled, 13);
  for (int i = 0; i < squarings; ++i) {
    value = value * value;
  }
  result.value = value;
  return result;
}

}  // namespace galata::numerics
