// SPDX-License-Identifier: Apache-2.0
//
// Modal decomposition and classification.
//
// Reference:
//   B. Etkin and L. D. Reid, "Dynamics of Flight: Stability and Control",
//   3rd ed., Wiley, 1996, chapters 6 and 7.
//   I. J. Perez-Arriaga, G. C. Verghese and F. C. Schweppe, "Selective Modal
//   Analysis with Applications to Electric Power Systems, Part I", IEEE Trans.
//   Power Apparatus and Systems, PAS-101(9), 1982 — participation factors.
//
// Validity envelope and the classifier's failure modes are in the header's
// "WHAT THIS IS NOT" block.

#include "galata/analyze/modes.hpp"

#include "galata/core/constants.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace galata::analyze {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Above this, the eigenvector matrix is too close to singular for
// participation factors to carry information.
constexpr double kConditionLimit = 1e10;

// An eigenvalue with |Im| below this fraction of |lambda| is treated as real.
// Relative rather than absolute so it behaves the same for a spiral root at
// 0.03 rad/s and a structural mode at 300 rad/s.
constexpr double kRealThreshold = 1e-9;

std::string lower(std::string text) {
  for (char& c : text) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return text;
}

// Deterministic total order over modes, so that two runs — and two platforms —
// report the same table in the same order. Eigen's eigenvalue ordering is
// deterministic for identical input but is not specified, and a report whose
// row order depends on the solver's internals is a report that diffs noisily.
bool mode_precedes(const Mode& left, const Mode& right) {
  // Non-oscillatory first, then by natural frequency, then by real part, then
  // by imaginary part. Every comparison is on a value the caller can see.
  if (left.is_oscillatory != right.is_oscillatory) {
    return !left.is_oscillatory;
  }
  if (left.natural_frequency_rad_s != right.natural_frequency_rad_s) {
    return left.natural_frequency_rad_s < right.natural_frequency_rad_s;
  }
  if (left.eigenvalue.real() != right.eigenvalue.real()) {
    return left.eigenvalue.real() < right.eigenvalue.real();
  }
  return left.eigenvalue.imag() < right.eigenvalue.imag();
}

void fill_metrics(Mode& mode) {
  const double real_part = mode.eigenvalue.real();
  const double imaginary_part = mode.eigenvalue.imag();
  const double magnitude = std::abs(mode.eigenvalue);

  mode.natural_frequency_rad_s = magnitude;
  mode.is_oscillatory = std::fabs(imaginary_part) > kRealThreshold * std::fmax(magnitude, 1e-300);

  if (magnitude > 0.0) {
    mode.damping_ratio = -real_part / magnitude;
  } else {
    // A zero eigenvalue: a pure integrator, such as heading in a model with no
    // heading feedback. Neither damped nor undamped.
    mode.damping_ratio = kNaN;
  }

  if (mode.is_oscillatory) {
    mode.damped_frequency_rad_s = std::fabs(imaginary_part);
    mode.period_s = core::kTwoPi / mode.damped_frequency_rad_s;
  } else {
    mode.damped_frequency_rad_s = kNaN;
    mode.period_s = kNaN;
  }

  if (real_part != 0.0) {
    mode.time_constant_s = -1.0 / real_part;
  } else {
    mode.time_constant_s = kNaN;
  }

  // Exactly one of half/double is a number. Reporting both, or reporting a
  // "time to half" for a divergent mode, is how a divergence gets quoted as if
  // it were a convergence in a report table.
  const double log_two = std::log(2.0);
  if (real_part < 0.0) {
    mode.time_to_half_amplitude_s = log_two / (-real_part);
    mode.time_to_double_amplitude_s = kNaN;
  } else if (real_part > 0.0) {
    mode.time_to_half_amplitude_s = kNaN;
    mode.time_to_double_amplitude_s = log_two / real_part;
  } else {
    mode.time_to_half_amplitude_s = kNaN;
    mode.time_to_double_amplitude_s = kNaN;
  }
}

// Signature states for each label, and whether the label describes an
// oscillatory mode. This table IS the classifier; everything else is
// bookkeeping.
struct Signature {
  ModeLabel label;
  bool oscillatory;
  // Offsets of the StateRoles members that define this label.
  std::array<int StateRoles::*, 2> roles;
};

const std::array<Signature, 5> kSignatures = {{
    // Short period: a rapid pitching oscillation at nearly constant speed.
    // alpha and q take part; u barely moves.
    {ModeLabel::ShortPeriod, true, {&StateRoles::angle_of_attack, &StateRoles::pitch_rate}},
    // Phugoid: a slow exchange of speed and height at nearly constant alpha.
    {ModeLabel::Phugoid, true, {&StateRoles::axial_speed, &StateRoles::pitch_attitude}},
    // Dutch roll: a yaw-sideslip oscillation with rolling.
    {ModeLabel::DutchRoll, true, {&StateRoles::sideslip, &StateRoles::yaw_rate}},
    // Roll subsidence: a fast, heavily damped pure roll convergence.
    {ModeLabel::RollSubsidence, false, {&StateRoles::roll_rate, &StateRoles::roll_rate}},
    // Spiral: a slow bank-angle divergence or convergence.
    {ModeLabel::Spiral, false, {&StateRoles::bank_angle, &StateRoles::bank_angle}},
}};

double signature_score(const Mode& mode, const StateRoles& roles, const Signature& signature) {
  double score = 0.0;
  int previous = -1;
  for (int StateRoles::* member : signature.roles) {
    const int index = roles.*member;
    if (index < 0 || index == previous) {
      continue;  // absent, or the same role listed twice for a single-state signature
    }
    previous = index;
    if (static_cast<std::size_t>(index) < mode.participation.size()) {
      score += mode.participation[static_cast<std::size_t>(index)];
    }
  }
  return score;
}

}  // namespace

std::string to_string(ModeLabel label) {
  switch (label) {
    case ModeLabel::ShortPeriod:
      return "short period";
    case ModeLabel::Phugoid:
      return "phugoid";
    case ModeLabel::DutchRoll:
      return "Dutch roll";
    case ModeLabel::RollSubsidence:
      return "roll subsidence";
    case ModeLabel::Spiral:
      return "spiral";
    case ModeLabel::Unclassified:
      break;
  }
  return "unclassified";
}

StateRoles StateRoles::from_names(const std::vector<std::string>& state_names) {
  StateRoles roles;
  for (std::size_t i = 0; i < state_names.size(); ++i) {
    const std::string name = lower(state_names[i]);
    const int index = static_cast<int>(i);

    if (name == "u" || name == "v_t" || name == "vt" || name == "speed" || name == "airspeed") {
      roles.axial_speed = index;
    } else if (name == "w" || name == "alpha" || name == "aoa") {
      roles.angle_of_attack = index;
    } else if (name == "q" || name == "pitch_rate") {
      roles.pitch_rate = index;
    } else if (name == "theta" || name == "pitch" || name == "pitch_attitude") {
      roles.pitch_attitude = index;
    } else if (name == "v" || name == "beta" || name == "sideslip") {
      roles.sideslip = index;
    } else if (name == "p" || name == "roll_rate") {
      roles.roll_rate = index;
    } else if (name == "r" || name == "yaw_rate") {
      roles.yaw_rate = index;
    } else if (name == "phi" || name == "bank" || name == "roll" || name == "bank_angle") {
      roles.bank_angle = index;
    }
  }
  return roles;
}

bool StateRoles::has_longitudinal() const noexcept {
  return angle_of_attack >= 0 && pitch_rate >= 0;
}

bool StateRoles::has_lateral() const noexcept {
  return sideslip >= 0 && yaw_rate >= 0;
}

const Mode* ModalDecomposition::find(ModeLabel label) const {
  for (const Mode& mode : modes) {
    if (mode.label == label) {
      return &mode;
    }
  }
  return nullptr;
}

ModalDecomposition analyze_modes(const Eigen::MatrixXd& a,
                                 const std::vector<std::string>& state_names,
                                 const StateRoles& roles) {
  if (a.rows() != a.cols()) {
    throw std::invalid_argument("analyze_modes: A is not square");
  }
  if (static_cast<Eigen::Index>(state_names.size()) != a.rows()) {
    std::ostringstream message;
    message << "analyze_modes: " << state_names.size() << " state names for a " << a.rows() << "x"
            << a.cols() << " matrix";
    throw std::invalid_argument(message.str());
  }

  ModalDecomposition result;
  result.state_names = state_names;

  Eigen::EigenSolver<Eigen::MatrixXd> solver(a, /*computeEigenvectors=*/true);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("analyze_modes: eigenvalue decomposition failed to converge");
  }

  const Eigen::MatrixXcd right = solver.eigenvectors();
  const Eigen::VectorXcd eigenvalues = solver.eigenvalues();

  // Conditioning of the eigenvector matrix, from its singular values. This is
  // the honest measure of whether a modal decomposition means anything: a
  // defective or near-defective A has eigenvectors that are nearly linearly
  // dependent, and participation factors computed from them are noise.
  Eigen::JacobiSVD<Eigen::MatrixXcd> svd(right);
  const double largest = svd.singularValues()(0);
  const double smallest = svd.singularValues()(svd.singularValues().size() - 1);
  result.eigenvector_condition_number =
      (smallest > 0.0) ? (largest / smallest) : std::numeric_limits<double>::infinity();
  result.participation_is_meaningful = result.eigenvector_condition_number < kConditionLimit;

  // Left eigenvectors are the rows of the inverse of the right eigenvector
  // matrix. Participation factor P_ki = V(k,i) * Vinv(i,k); by construction
  // sum_k P_ki = (Vinv V)(i,i) = 1 exactly, before taking magnitudes.
  Eigen::MatrixXcd left;
  if (result.participation_is_meaningful) {
    left = right.inverse();
  }

  const Eigen::Index size = a.rows();
  std::vector<bool> consumed(static_cast<std::size_t>(size), false);

  for (Eigen::Index i = 0; i < size; ++i) {
    if (consumed[static_cast<std::size_t>(i)]) {
      continue;
    }
    const std::complex<double> lambda = eigenvalues(i);

    Mode mode;
    // Report the member of a conjugate pair with positive imaginary part, and
    // consume its partner, so an oscillation appears once rather than twice.
    mode.eigenvalue = (lambda.imag() < 0.0) ? std::conj(lambda) : lambda;
    fill_metrics(mode);

    if (mode.is_oscillatory) {
      // Find the conjugate partner and mark it consumed. Matched by closest
      // conjugate rather than by index, because the solver does not promise
      // adjacency.
      Eigen::Index best = -1;
      double best_distance = std::numeric_limits<double>::infinity();
      for (Eigen::Index j = 0; j < size; ++j) {
        if (j == i || consumed[static_cast<std::size_t>(j)]) {
          continue;
        }
        const double distance = std::abs(eigenvalues(j) - std::conj(lambda));
        if (distance < best_distance) {
          best_distance = distance;
          best = j;
        }
      }
      if (best >= 0 && best_distance <= 1e-8 * std::fmax(std::abs(lambda), 1.0)) {
        consumed[static_cast<std::size_t>(best)] = true;
      }
    }

    if (result.participation_is_meaningful) {
      mode.participation.assign(static_cast<std::size_t>(size), 0.0);
      double total = 0.0;
      for (Eigen::Index k = 0; k < size; ++k) {
        const double factor = std::abs(right(k, i) * left(i, k));
        mode.participation[static_cast<std::size_t>(k)] = factor;
        total += factor;
      }
      // Renormalise: the complex factors sum to 1, but their magnitudes do not.
      if (total > 0.0) {
        for (double& value : mode.participation) {
          value /= total;
        }
      }
    }

    consumed[static_cast<std::size_t>(i)] = true;
    result.modes.push_back(std::move(mode));
  }

  std::sort(result.modes.begin(), result.modes.end(), mode_precedes);

  // --- Classification ------------------------------------------------------
  //
  // Greedy assignment over (mode, label) pairs ordered by signature score, with
  // each label used at most once and each mode labelled at most once.
  //
  // Greedy rather than a global optimum: with at most five labels the two
  // agree in every case that is not already ambiguous, and a greedy rule is one
  // a reader can follow when a label surprises them. The alternative — an
  // assignment problem whose answer depends on scores the user cannot see — is
  // harder to argue with when it is wrong.
  if (!result.participation_is_meaningful) {
    return result;
  }

  struct Candidate {
    std::size_t mode_index;
    std::size_t signature_index;
    double score;
  };

  std::vector<Candidate> candidates;
  for (std::size_t m = 0; m < result.modes.size(); ++m) {
    for (std::size_t s = 0; s < kSignatures.size(); ++s) {
      if (kSignatures[s].oscillatory != result.modes[m].is_oscillatory) {
        continue;
      }
      const double score = signature_score(result.modes[m], roles, kSignatures[s]);
      if (score > 0.0) {
        candidates.push_back({m, s, score});
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate& x, const Candidate& y) {
    if (x.score != y.score) {
      return x.score > y.score;
    }
    // Deterministic tie-break, so the table does not depend on sort stability.
    if (x.mode_index != y.mode_index) {
      return x.mode_index < y.mode_index;
    }
    return x.signature_index < y.signature_index;
  });

  std::vector<bool> mode_taken(result.modes.size(), false);
  std::vector<bool> label_taken(kSignatures.size(), false);
  for (const Candidate& candidate : candidates) {
    if (mode_taken[candidate.mode_index] || label_taken[candidate.signature_index]) {
      continue;
    }
    mode_taken[candidate.mode_index] = true;
    label_taken[candidate.signature_index] = true;

    Mode& mode = result.modes[candidate.mode_index];
    mode.label = kSignatures[candidate.signature_index].label;
    mode.label_score = candidate.score;

    std::ostringstream reason;
    reason.setf(std::ios::fixed);
    reason.precision(3);
    reason << candidate.score << " of participation in ";
    bool first = true;
    int previous = -1;
    for (int StateRoles::* member : kSignatures[candidate.signature_index].roles) {
      const int index = roles.*member;
      if (index < 0 || index == previous) {
        continue;
      }
      previous = index;
      if (!first) {
        reason << " and ";
      }
      reason << state_names[static_cast<std::size_t>(index)];
      first = false;
    }
    mode.label_reason = reason.str();
  }

  return result;
}

ModalDecomposition analyze_modes(const Eigen::MatrixXd& a,
                                 const std::vector<std::string>& state_names) {
  return analyze_modes(a, state_names, StateRoles{});
}

}  // namespace galata::analyze
