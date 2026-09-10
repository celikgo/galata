// SPDX-License-Identifier: Apache-2.0
//
// The matrix exponential, by scaling and squaring with a Pade approximant.
//
// Reference:
//   N. J. Higham, "The Scaling and Squaring Method for the Matrix Exponential
//   Revisited", SIAM J. Matrix Anal. Appl. 26(4), 2005, pp. 1179-1193 — the
//   order/threshold table this file uses, and the backward-error analysis the
//   thresholds come from.
//   N. J. Higham, "Functions of Matrices: Theory and Computation", SIAM, 2008,
//   chapter 10 — the same algorithm with its conditioning discussion.
//   C. F. Van Loan, "Computing Integrals Involving the Matrix Exponential",
//   IEEE Trans. Automatic Control 23(3), 1978, pp. 395-404 — why a state-space
//   discretisation wants this primitive rather than a series of its own.
//   G. H. Golub and C. F. Van Loan, "Matrix Computations", 4th ed., 2013,
//   section 9.3.
//
// WHY THIS EXISTS. A zero-order-hold discretisation is an exponential of a
// block matrix and nothing else; so is the exact discretisation of a quadratic
// cost. Writing either from a truncated series would put an unstated truncation
// error inside every discrete model this repository produces. This is the one
// place that error is bounded, and the bound is reported rather than assumed.
//
// ===========================================================================
// WHY IT IS DETERMINISTIC, WHICH A MATRIX EXPONENTIAL IS NOT OBLIGED TO BE
// ===========================================================================
//
// ADR-0004 forbids a tolerance-based early exit. This algorithm has none: the
// Pade order is selected from the one-norm by a table lookup, and the squaring
// count is `ceil(log2(norm / theta))`. Both are closed-form functions of the
// input, so the same matrix takes the same path and the same number of flops
// every run. Nothing here iterates until something is small enough.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * NOT EXACT, and the error is BACKWARD rather than forward. Higham's analysis
//   bounds the result as the exact exponential of a PERTURBED matrix, A + dA
//   with ||dA|| <= `backward_error_bound` * ||A||. It does not bound the
//   distance to expm(A) itself. Turning one into the other needs the condition
//   number of the exponential at A, which this does not compute, so a caller
//   who needs a forward bound does not have one here.
// * NOT IMMUNE TO THE HUMP. Squaring amplifies whatever error the Pade step
//   left, and for a matrix whose exponential decays while intermediate powers
//   grow — a stiff or highly non-normal A — the amplification can be severe.
//   `squarings` and `one_norm` are reported so a caller can see when the
//   algorithm was pushed into that regime rather than discovering it from a
//   result that looks plausible.
// * NOT A SUBSTITUTE FOR A SPECTRAL METHOD on a matrix known to be normal or
//   diagonalisable with a well-conditioned basis, where an eigendecomposition
//   is both cheaper and better conditioned. This is the general-purpose choice.
// * NOT FOR A SINGULAR OR DEFECTIVE PENCIL, and not a solver. It exponentiates
//   the matrix it is handed. It makes no statement about any system that matrix
//   came from, and a finite result is not evidence that a model is sound.
#ifndef GALATA_NUMERICS_MATRIX_EXPONENTIAL_HPP
#define GALATA_NUMERICS_MATRIX_EXPONENTIAL_HPP

#include <Eigen/Core>

namespace galata::numerics {

struct MatrixExponential {
  Eigen::MatrixXd value;  // n x n, mixed units — the exponential is dimensionless per entry
  // The one-norm of the matrix handed in, before any scaling. Reported because
  // it is what selected the path below, and because a large value is the
  // warning sign the header's hump note describes.
  double one_norm = 0.0;
  // The diagonal Pade order used: 3, 5, 7, 9 or 13, per Higham's table.
  int pade_order = 0;
  // How many times the Pade result was squared back up. Zero means the matrix
  // was small enough in norm to approximate directly.
  int squarings = 0;
  // Higham's backward-error bound for the order that was used: the result is
  // the exact exponential of A + dA with ||dA||_1 <= this times ||A||_1. It is
  // a DECLARED bound from the cited table, not a measurement of this run.
  double backward_error_bound = 0.0;
};

// Refuses a non-square matrix, an empty matrix, and any non-finite entry.
//
// A zero matrix returns the identity exactly, by the same path as any other
// small-norm input rather than as a special case.
[[nodiscard]] MatrixExponential matrix_exponential(const Eigen::MatrixXd& m);

}  // namespace galata::numerics

#endif  // GALATA_NUMERICS_MATRIX_EXPONENTIAL_HPP
