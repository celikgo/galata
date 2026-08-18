// SPDX-License-Identifier: Apache-2.0
//
// Modal decomposition of a linear system, with automatic classification of the
// classical aircraft modes.
//
// Reference:
//   B. Etkin and L. D. Reid, "Dynamics of Flight: Stability and Control",
//   3rd ed., Wiley, 1996, chapters 6 and 7 — the longitudinal and lateral
//   modes and their physical character.
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 4.
//   I. J. Perez-Arriaga, G. C. Verghese and F. C. Schweppe, "Selective Modal
//   Analysis with Applications to Electric Power Systems, Part I: Heuristic
//   Introduction", IEEE Transactions on Power Apparatus and Systems, vol.
//   PAS-101, no. 9, pp. 3117-3125, 1982 — participation factors, which is
//   where the classification's measure of "how much does this state take part
//   in this mode" comes from. The technique originates in power systems, not
//   in flight dynamics, and it transfers directly.
//
// A plain eigenvalue list is what every tool gives. A LABELLED modal table is
// what an engineer actually wants, and the labelling is done by eigenvector
// participation rather than by frequency thresholds, because thresholds stop
// working on exactly the configurations worth studying.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * The classifier does not know your aircraft. It labels modes by which
//   states participate in them. It cannot tell you that a label is
//   meaningless — it can only tell you that its evidence was weak, which it
//   does through `label_score`. ALWAYS look at the score before quoting a
//   label.
//
// * The classical five modes are a description of a conventional aeroplane in
//   ordinary flight. They are not a law. At an aft CG the short period and
//   phugoid can merge into a pair of real roots, one of which is the "tuck"
//   divergence; on a tailless or highly augmented configuration the modes may
//   not be separable at all. In those cases the participation evidence is
//   genuinely ambiguous, the scores will be low, and the honest output is a
//   low-confidence label — which is what you get, rather than a confident
//   wrong one.
//
// * Participation factors are defined for a system with distinct eigenvalues.
//   At a repeated eigenvalue the eigenvector matrix is singular or nearly so,
//   participation is not defined, and this reports the conditioning rather
//   than quietly producing numbers. Repeated eigenvalues are not a pathological
//   case in flight dynamics: they occur at the exact CG where two real roots
//   coalesce into a complex pair, and a CG sweep will walk straight through
//   one.
//
// * Nothing here says whether a mode is ACCEPTABLE. Level 1/2/3 boundaries are
//   a handling-qualities judgement against a specification, and that is a
//   separate capability with its own citations.

#ifndef GALATA_ANALYZE_MODES_HPP
#define GALATA_ANALYZE_MODES_HPP

#include <Eigen/Core>

#include <complex>
#include <string>
#include <vector>

namespace galata::analyze {

enum class ModeLabel {
  Unclassified,
  ShortPeriod,
  Phugoid,
  DutchRoll,
  RollSubsidence,
  Spiral,
};

[[nodiscard]] std::string to_string(ModeLabel label);

// Which column of the A matrix plays which physical role.
//
// Supplied by the caller rather than guessed from state names, because a
// classifier that guesses is a classifier that silently mislabels a model whose
// author spelled a state differently. -1 means the role is absent from this
// system, which is normal: a purely longitudinal model has no roll rate.
struct StateRoles {
  int axial_speed = -1;      // u, or V
  int angle_of_attack = -1;  // alpha, or w
  int pitch_rate = -1;       // q
  int pitch_attitude = -1;   // theta
  int sideslip = -1;         // beta, or v
  int roll_rate = -1;        // p
  int yaw_rate = -1;         // r
  int bank_angle = -1;       // phi

  // Fills the roles by matching common names, case-insensitively:
  //   u, v, w, p, q, r, alpha, beta, theta, phi, V, speed, ...
  // Convenience only. A model with unusual names should set the fields
  // directly; a role left at -1 simply takes no part in classification.
  [[nodiscard]] static StateRoles from_names(const std::vector<std::string>& state_names);

  [[nodiscard]] bool has_longitudinal() const noexcept;
  [[nodiscard]] bool has_lateral() const noexcept;
};

struct Mode {
  // The eigenvalue. For an oscillatory mode this is the member of the
  // conjugate pair with POSITIVE imaginary part; the pair is reported once.
  std::complex<double> eigenvalue{0.0, 0.0};  // 1/s

  bool is_oscillatory = false;

  // Undamped natural frequency, |lambda|. For a real root this equals
  // |lambda| too, which is the convention, though it is rarely quoted.
  double natural_frequency_rad_s = 0.0;  // rad/s

  // zeta = -Re(lambda) / |lambda|. Negative for a divergent mode. For a real
  // root this is +1 (convergent) or -1 (divergent).
  double damping_ratio = 0.0;  // dimensionless

  // Damped frequency and period. NaN for a non-oscillatory mode: a real root
  // has no period, and reporting 0 or infinity invites a plot to draw it.
  double damped_frequency_rad_s = 0.0;  // rad/s, NaN if not oscillatory
  double period_s = 0.0;                // s,     NaN if not oscillatory

  // -1/Re(lambda). Positive for a convergent mode, negative for a divergent
  // one. NaN when Re(lambda) is zero.
  double time_constant_s = 0.0;  // s

  // Time for the envelope to halve, or to double. Exactly one of these is a
  // number and the other is NaN, decided by the sign of Re(lambda). Both are
  // NaN for a neutrally stable mode.
  double time_to_half_amplitude_s = 0.0;    // s
  double time_to_double_amplitude_s = 0.0;  // s

  // Participation of each state in this mode, normalised to sum to 1 across
  // states. Real and non-negative: the magnitude of the complex participation
  // factor. Index matches the A matrix's ordering.
  std::vector<double> participation;

  ModeLabel label = ModeLabel::Unclassified;

  // How much of this mode's participation sits in the states that define its
  // label, in [0, 1]. This is the number to check before quoting the label.
  // Below about 0.5 the label is a guess.
  double label_score = 0.0;

  // Human-readable justification, e.g. "0.87 of participation in alpha and q".
  std::string label_reason;
};

struct ModalDecomposition {
  std::vector<Mode> modes;
  std::vector<std::string> state_names;

  // Condition number of the eigenvector matrix, in the 2-norm.
  //
  // Large means the eigenvectors are nearly linearly dependent, participation
  // factors are ill-defined, and the classification should not be trusted. A
  // CG sweep that walks through the point where two real roots become a
  // complex pair will show this spiking, which is a physical event and not a
  // numerical failure.
  double eigenvector_condition_number = 0.0;

  // False when the eigenvector matrix was too ill-conditioned for
  // participation factors to mean anything. Modes are still reported; their
  // participation vectors and labels are not.
  bool participation_is_meaningful = false;

  [[nodiscard]] const Mode* find(ModeLabel label) const;
};

// Eigenvalues, modal metrics, participation factors and — when `roles`
// identifies enough states — classification.
//
// `state_names` must have one entry per row of `a`; it is carried through to
// the result so a report can label rows without the caller re-supplying them.
[[nodiscard]] ModalDecomposition analyze_modes(const Eigen::MatrixXd& a,
                                               const std::vector<std::string>& state_names,
                                               const StateRoles& roles);

// Without roles: metrics and participation, no classification.
[[nodiscard]] ModalDecomposition analyze_modes(const Eigen::MatrixXd& a,
                                               const std::vector<std::string>& state_names);

}  // namespace galata::analyze

#endif  // GALATA_ANALYZE_MODES_HPP
