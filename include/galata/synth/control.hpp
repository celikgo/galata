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

// THE SINGLE LOOP SEEN AT ONE PLANT INPUT WITH THE OTHER LOOPS STILL CLOSED.
//
// WHY `broken_loop` IS NOT ENOUGH, AND WHY THIS IS NOT A CONVENIENCE. `broken_loop`
// is the full MIMO return ratio L(s) = K(sI-A)^-1 B. Handing one of its channels
// to a SISO margin routine breaks that channel AND leaves the other m-1 channels
// OPEN, so the loop whose Nyquist plot is being read is the loop of a vehicle
// with most of its controller disconnected. On a plant that needs every channel
// to be stabilised — a multirotor at hover needs all four — that closure is not
// internally stable, its encirclement count means nothing, and
// `analyze.margins` correctly refuses it rather than reporting a number.
//
// The measure a control engineer actually means by "the gain margin of my
// design" is the loop-at-a-time margin with the OTHER LOOPS CLOSED: break
// channel k, leave every other channel connected, and ask how much gain or
// phase that one channel tolerates. That loop is
//
//     A_k = A - B K + b_k k_k^T,     b = b_k,     c = k_k^T,     D = 0
//
// so that closing unit negative feedback around it returns exactly A - BK, the
// design's own closed loop. Internal stability of the Nyquist test is then the
// stability of the design, which an LQR solution guarantees — which is what
// makes the margin well posed here and ill posed for the other-loops-open
// reading.
//
// WHAT THIS IS NOT. Not a MIMO robustness measure, and THREE DIFFERENT THINGS
// ARE EASY TO CONFLATE HERE. An earlier version of this comment conflated the
// second and the third, which is the error it now exists to prevent:
//
//   1. A GAIN OR PHASE MARGIN on this loop bounds a pure gain change, or a pure
//      phase change, in THIS ONE CHANNEL with the others held at nominal.
//   2. A DISK MARGIN on this loop — `analyze.diskmargin` — bounds SIMULTANEOUS
//      GAIN AND PHASE variation, still in THIS ONE CHANNEL. It is strictly
//      stronger than (1) and it is a SISO condition;
//      `include/galata/analyze/disk_margin.hpp` says so in its own words: "Not
//      a MIMO disk margin. This is the SISO condition. The multi-loop case
//      needs a structured singular value, which galata does not have."
//   3. SIMULTANEOUS VARIATION ACROSS CHANNELS — every input perturbed at once —
//      is bounded by NEITHER of the above, at any number of channels. The
//      classic counterexample perturbs two channels together while every
//      loop-at-a-time figure, disk margins included, stays comfortable.
//      GALATA HAS NO CAPABILITY FOR THIS. Running `analyze.diskmargin` on each
//      single loop in turn does not add up to it.
//
// `analyze.sensitivity` and `analyze.sigma` give MIMO PEAKS — norms of S, T and
// the principal gains — which qualify the design and are not a structured
// robustness margin either.
//
// Not a sampled-loop margin either. This is the continuous design's loop; a
// controller executed at a rate with a hold and a delay has different margins,
// and nothing here computes them.
//
// `channel` indexes the plant's inputs. Refuses an index the plant does not
// have, and refuses a design whose own closed loop is not the one this
// construction closes back to — which cannot happen for a design this file
// produced and is checked rather than assumed.
[[nodiscard]] model::LinearSystem single_loop_others_closed(const LqrDesign& design, int channel);

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
