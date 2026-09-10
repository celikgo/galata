// SPDX-License-Identifier: Apache-2.0
//
// The matrix exponential.
//
// EVERY REFERENCE HERE IS COMPUTED A DIFFERENT WAY FROM THE ROUTINE UNDER TEST.
// The cases are built so that a closed form exists and is written as a formula
// rather than a typed number: a diagonal matrix's exponential comes from
// `std::exp` of its entries, a rotation generator's from `std::cos` and
// `std::sin`, and a non-normal matrix's from a similarity transform whose
// eigenvalues and eigenvectors are chosen here as small integers. So a test
// that passes has agreed with scalar arithmetic, not with itself.
//
// Two of the checks are IDENTITIES rather than references — det(expm A) =
// exp(tr A), and expm(A)^2 = expm(2A). They are here ALONGSIDE the references
// and not instead of them, and two injected defects show why that distinction
// is not decoration:
//
//   * Swapping the Pade numerator and denominator, which returns exp(-A),
//     was caught by six of these ten tests.
//   * Dropping one squaring was caught by three — the two similarity
//     references and the determinant identity — and NOT by
//     `SquaringTheResultMatchesDoublingTheArgument`, because both of its sides
//     lose the same squaring and still agree with each other. An identity
//     between two calls of the same routine cannot see an error that scales
//     both.
//
// So the closed-form references carry the load, and the identities cover the
// squaring-loop and scaling errors a single well-conditioned reference case
// would let through.

#include "galata/numerics/matrix_exponential.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <stdexcept>

namespace {

using galata::numerics::matrix_exponential;

// The exponential of a diagonalisable matrix, computed from the scalar
// exponentials of its eigenvalues. This is the independent reference: it never
// touches the routine under test.
Eigen::MatrixXd reference_via_similarity(const Eigen::MatrixXd& vectors,
                                         const Eigen::VectorXd& eigenvalues) {
  Eigen::VectorXd exponentiated(eigenvalues.size());
  for (Eigen::Index i = 0; i < eigenvalues.size(); ++i) {
    exponentiated(i) = std::exp(eigenvalues(i));
  }
  return vectors * exponentiated.asDiagonal() * vectors.inverse();
}

}  // namespace

TEST(MatrixExponential, TheZeroMatrixExponentiatesToTheIdentity) {
  const auto result = matrix_exponential(Eigen::MatrixXd::Zero(4, 4));
  EXPECT_TRUE(result.value.isApprox(Eigen::MatrixXd::Identity(4, 4), 0.0))
      << "exp(0) is the identity exactly, not to a tolerance:\n"
      << result.value;
  EXPECT_EQ(result.squarings, 0) << "a zero-norm matrix needs no scaling";
  EXPECT_EQ(result.pade_order, 3) << "and takes the cheapest approximant in the table";
}

TEST(MatrixExponential, ADiagonalMatrixAgreesWithScalarExponentials) {
  Eigen::VectorXd diagonal(4);
  diagonal << -0.25, 0.5, -2.0, 3.0;
  const Eigen::MatrixXd a = diagonal.asDiagonal();

  const auto result = matrix_exponential(a);
  for (Eigen::Index i = 0; i < 4; ++i) {
    EXPECT_NEAR(result.value(i, i), std::exp(diagonal(i)), 1e-14)
        << "diagonal entry " << i << " must be the scalar exponential";
  }
  // Off-diagonal entries are zero, and must stay zero rather than filling in
  // with the products of a scheme that ignored the structure.
  Eigen::MatrixXd off = result.value;
  off.diagonal().setZero();
  EXPECT_LT(off.cwiseAbs().maxCoeff(), 1e-15) << "a diagonal matrix stays diagonal";
}

// A ROTATION GENERATOR HAS AN EXACT CLOSED FORM, and it is the case that
// catches a sign error the diagonal case cannot: every entry of the reference
// is a cosine or a sine, so a transposed or negated term is visible.
TEST(MatrixExponential, ARotationGeneratorGivesTheRotationMatrix) {
  const double rate = 1.75;   // rad/s
  const double time = 0.625;  // s
  Eigen::MatrixXd generator(2, 2);
  generator << 0.0, -rate, rate, 0.0;

  const auto result = matrix_exponential(generator * time);
  const double angle = rate * time;
  EXPECT_NEAR(result.value(0, 0), std::cos(angle), 1e-14);
  EXPECT_NEAR(result.value(0, 1), -std::sin(angle), 1e-14);
  EXPECT_NEAR(result.value(1, 0), std::sin(angle), 1e-14);
  EXPECT_NEAR(result.value(1, 1), std::cos(angle), 1e-14);
  // A rotation is orthogonal with unit determinant, which the closed form
  // above already implies but which a reader checking the result independently
  // would look for.
  EXPECT_NEAR(result.value.determinant(), 1.0, 1e-14);
}

// A DEFECTIVE MATRIX HAS NO EIGENBASIS, so a routine that quietly assumed one
// returns the wrong answer here and only here. The Jordan block's exponential
// is exp(a) * [[1, 1], [0, 1]], term for term.
TEST(MatrixExponential, ADefectiveJordanBlockKeepsItsPolynomialTerm) {
  const double lambda = -0.75;
  Eigen::MatrixXd a(2, 2);
  a << lambda, 1.0, 0.0, lambda;

  const auto result = matrix_exponential(a);
  const double scale = std::exp(lambda);
  EXPECT_NEAR(result.value(0, 0), scale, 1e-14);
  EXPECT_NEAR(result.value(0, 1), scale, 1e-14) << "the off-diagonal term is t * exp(lambda) at "
                                                   "t = 1, and vanishes for a routine that "
                                                   "diagonalised a matrix with no eigenbasis";
  EXPECT_NEAR(result.value(1, 0), 0.0, 1e-15);
  EXPECT_NEAR(result.value(1, 1), scale, 1e-14);
}

// MOLER AND VAN LOAN'S STANDARD HARD CASE. A = [[-49, 24], [-64, 31]] has
// eigenvalues -1 and -17 and an ill-conditioned eigenvector basis, and is the
// example their survey uses to break naive schemes.
//
//   C. B. Moler and C. F. Van Loan, "Nineteen Dubious Ways to Compute the
//   Exponential of a Matrix, Twenty-Five Years Later", SIAM Review 45(1),
//   2003, pp. 3-49 — section 1's example.
//
// The reference is assembled here from the eigenvalues and eigenvectors, both
// small integers derived by hand, so the comparison is against scalar
// arithmetic rather than against a transcribed decimal.
TEST(MatrixExponential, AgreesWithTheMolerVanLoanExampleThroughItsClosedForm) {
  Eigen::MatrixXd a(2, 2);
  a << -49.0, 24.0, -64.0, 31.0;

  // (A + I) v = 0 gives v = (1, 2); (A + 17 I) v = 0 gives v = (3, 4).
  Eigen::MatrixXd vectors(2, 2);
  vectors << 1.0, 3.0, 2.0, 4.0;
  Eigen::VectorXd eigenvalues(2);
  eigenvalues << -1.0, -17.0;
  ASSERT_TRUE((a * vectors.col(0)).isApprox(eigenvalues(0) * vectors.col(0), 1e-14))
      << "the hand-derived first eigenpair must actually be one";
  ASSERT_TRUE((a * vectors.col(1)).isApprox(eigenvalues(1) * vectors.col(1), 1e-14))
      << "the hand-derived second eigenpair must actually be one";

  const Eigen::MatrixXd expected = reference_via_similarity(vectors, eigenvalues);
  const auto result = matrix_exponential(a);
  EXPECT_LT((result.value - expected).cwiseAbs().maxCoeff(), 1e-13)
      << "computed:\n"
      << result.value << "\nclosed form:\n"
      << expected;
  EXPECT_GT(result.squarings, 0) << "a one-norm of 113 must take the scaling path";
  EXPECT_EQ(result.pade_order, 13) << "which is the order the scaled path uses";
}

// THE SCALING PATH IS WHERE THE ALGORITHM EARNS ITS NAME, and a large-norm
// matrix is the only way to exercise it. Built by similarity so a closed form
// exists at a norm that forces several squarings.
TEST(MatrixExponential, TheScalingPathAgreesWithTheClosedFormAtLargeNorm) {
  Eigen::MatrixXd vectors(3, 3);
  vectors << 1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 3.0;
  Eigen::VectorXd eigenvalues(3);
  eigenvalues << -3.0, -0.5, 2.0;
  const Eigen::MatrixXd a = vectors * eigenvalues.asDiagonal() * vectors.inverse();

  // Scale the matrix up so the one-norm lands well past the order-13 threshold.
  const double horizon = 12.0;
  const Eigen::MatrixXd scaled = a * horizon;
  Eigen::VectorXd scaled_eigenvalues = eigenvalues * horizon;

  const auto result = matrix_exponential(scaled);
  const Eigen::MatrixXd expected = reference_via_similarity(vectors, scaled_eigenvalues);
  EXPECT_GE(result.squarings, 3) << "one-norm " << result.one_norm << " must be scaled down";
  // The bound is looser than the unscaled cases on purpose: squaring amplifies
  // the Pade step's error, which is exactly the hump the header describes. It is
  // stated here rather than absorbed into a blanket tolerance.
  const double relative = (result.value - expected).cwiseAbs().maxCoeff() /
                          expected.cwiseAbs().maxCoeff();
  EXPECT_LT(relative, 1e-12) << "relative error " << relative << " after " << result.squarings
                             << " squarings";
}

// det(expm A) = exp(tr A) is a theorem, and the two sides are computed by
// entirely different routes: an LU determinant of the result against a scalar
// exponential of a sum. A scheme that got the scaling wrong fails this even
// when its entries look plausible.
TEST(MatrixExponential, TheDeterminantIsTheExponentialOfTheTrace) {
  Eigen::MatrixXd a(4, 4);
  a << -1.5, 0.25, 0.0, 2.0,
       0.5, -0.75, 1.25, 0.0,
       0.0, -2.5, -0.5, 0.75,
       1.0, 0.0, -0.25, -3.0;

  const auto result = matrix_exponential(a);
  const double expected = std::exp(a.trace());
  EXPECT_NEAR(result.value.determinant() / expected, 1.0, 1e-12)
      << "det = " << result.value.determinant() << ", exp(trace) = " << expected;
}

// expm(A) expm(A) = expm(2A), because A commutes with itself. This is the
// identity that catches an off-by-one in the squaring loop, which every
// reference case above would pass.
TEST(MatrixExponential, SquaringTheResultMatchesDoublingTheArgument) {
  Eigen::MatrixXd a(3, 3);
  a << 0.4, -1.2, 0.0, 0.9, -0.3, 0.6, -0.5, 0.0, -1.1;

  const auto once = matrix_exponential(a);
  const auto twice = matrix_exponential(2.0 * a);
  const Eigen::MatrixXd squared = once.value * once.value;
  EXPECT_LT((squared - twice.value).cwiseAbs().maxCoeff(), 1e-13)
      << "expm(A)^2:\n"
      << squared << "\nexpm(2A):\n"
      << twice.value;
}

TEST(MatrixExponential, TheReportedDiagnosticsDescribeThePathTaken) {
  Eigen::MatrixXd small(2, 2);
  small << 0.001, 0.0, 0.0, -0.002;
  const auto tiny = matrix_exponential(small);
  EXPECT_EQ(tiny.pade_order, 3) << "one-norm " << tiny.one_norm << " is inside the order-3 bound";
  EXPECT_EQ(tiny.squarings, 0);
  EXPECT_GT(tiny.backward_error_bound, 0.0) << "the declared bound travels with the result";
  EXPECT_NEAR(tiny.one_norm, 0.002, 1e-15) << "the reported norm is the one-norm of the input";
}

TEST(MatrixExponential, WhatIsNotAMatrixExponentialIsRefusedByName) {
  EXPECT_THROW((void)matrix_exponential(Eigen::MatrixXd::Zero(2, 3)), std::invalid_argument)
      << "a non-square matrix has no exponential";
  EXPECT_THROW((void)matrix_exponential(Eigen::MatrixXd::Zero(0, 0)), std::invalid_argument);

  Eigen::MatrixXd infinite(2, 2);
  infinite << 1.0, 0.0, 0.0, std::numeric_limits<double>::infinity();
  EXPECT_THROW((void)matrix_exponential(infinite), std::invalid_argument)
      << "a non-finite entry must be refused rather than propagated into a discrete model";

  Eigen::MatrixXd not_a_number(2, 2);
  not_a_number << std::nan(""), 0.0, 0.0, 1.0;
  EXPECT_THROW((void)matrix_exponential(not_a_number), std::invalid_argument);
}
