// SPDX-License-Identifier: Apache-2.0
//
// Modal decomposition, against systems whose eigenvalues are known by
// construction.
//
// The classification tests build block-diagonal systems whose blocks have
// deliberately UNHELPFUL frequencies — a phugoid faster than the short period,
// for instance — because a classifier that keys on frequency would pass a test
// built from realistic numbers while failing on the aft-CG configurations that
// are the entire reason to run a modal analysis.

#include "galata/analyze/modes.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using galata::analyze::analyze_modes;
using galata::analyze::ModeLabel;
using galata::analyze::StateRoles;

// Companion form of s^2 + 2*zeta*omega*s + omega^2.
Eigen::Matrix2d oscillator(double omega, double zeta) {
  Eigen::Matrix2d block;
  block << 0.0, 1.0, -omega * omega, -2.0 * zeta * omega;
  return block;
}

TEST(Modes, RecoversFrequencyAndDampingOfASecondOrderSystem) {
  const double omega = 2.7;
  const double zeta = 0.35;
  const auto result = analyze_modes(oscillator(omega, zeta), {"x", "xdot"});

  ASSERT_EQ(result.modes.size(), 1U) << "a conjugate pair must be reported once, not twice";
  const auto& mode = result.modes[0];
  EXPECT_TRUE(mode.is_oscillatory);
  EXPECT_NEAR(mode.natural_frequency_rad_s, omega, 1e-12);
  EXPECT_NEAR(mode.damping_ratio, zeta, 1e-12);
  EXPECT_NEAR(mode.damped_frequency_rad_s, omega * std::sqrt(1.0 - zeta * zeta), 1e-12);
  EXPECT_NEAR(mode.period_s, 2.0 * M_PI / (omega * std::sqrt(1.0 - zeta * zeta)), 1e-12);
  EXPECT_GT(mode.eigenvalue.imag(), 0.0) << "the reported member of the pair must be the one "
                                            "with positive imaginary part";
  EXPECT_NEAR(mode.time_to_half_amplitude_s, std::log(2.0) / (zeta * omega), 1e-12);
  EXPECT_TRUE(std::isnan(mode.time_to_double_amplitude_s));
}

TEST(Modes, DivergentOscillationReportsTimeToDoubleAndNegativeDamping) {
  const auto result = analyze_modes(oscillator(1.5, -0.08), {"x", "xdot"});
  ASSERT_EQ(result.modes.size(), 1U);
  const auto& mode = result.modes[0];
  EXPECT_LT(mode.damping_ratio, 0.0);
  EXPECT_TRUE(std::isnan(mode.time_to_half_amplitude_s))
      << "a divergent mode has no time to half amplitude, and reporting one is how a "
         "divergence gets quoted as a convergence";
  EXPECT_GT(mode.time_to_double_amplitude_s, 0.0);
  EXPECT_NEAR(mode.time_to_double_amplitude_s, std::log(2.0) / (0.08 * 1.5), 1e-12);
}

TEST(Modes, RealRootsReportTimeConstantsAndNoPeriod) {
  Eigen::MatrixXd a(2, 2);
  a << -2.0, 0.0, 0.0, -0.5;
  const auto result = analyze_modes(a, {"fast", "slow"});

  ASSERT_EQ(result.modes.size(), 2U);
  for (const auto& mode : result.modes) {
    EXPECT_FALSE(mode.is_oscillatory);
    EXPECT_TRUE(std::isnan(mode.period_s));
    EXPECT_TRUE(std::isnan(mode.damped_frequency_rad_s));
    EXPECT_NEAR(mode.damping_ratio, 1.0, 1e-12);
  }
  // Sorted by natural frequency, so the slow root comes first.
  EXPECT_NEAR(result.modes[0].time_constant_s, 2.0, 1e-12);
  EXPECT_NEAR(result.modes[1].time_constant_s, 0.5, 1e-12);
}

TEST(Modes, ParticipationIsUnityOnTheOwningStateForADecoupledSystem) {
  Eigen::MatrixXd a(3, 3);
  a << -1.0, 0.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, -3.0;
  const auto result = analyze_modes(a, {"a", "b", "c"});

  ASSERT_TRUE(result.participation_is_meaningful);
  ASSERT_EQ(result.modes.size(), 3U);
  for (const auto& mode : result.modes) {
    double largest = 0.0;
    double total = 0.0;
    for (const double value : mode.participation) {
      largest = std::fmax(largest, value);
      total += value;
    }
    EXPECT_NEAR(total, 1.0, 1e-12) << "participation must be normalised";
    EXPECT_NEAR(largest, 1.0, 1e-12)
        << "in a decoupled system each mode belongs entirely to one state";
  }
}

TEST(Modes, ParticipationSumsToOneForACoupledSystem) {
  Eigen::MatrixXd a(4, 4);
  a << -0.05, 0.3, 0.0, -9.81,  //
      -0.4, -1.2, 1.0, 0.0,     //
      0.1, -3.4, -2.1, 0.0,     //
      0.0, 0.0, 1.0, 0.0;
  const auto result = analyze_modes(a, {"u", "alpha", "q", "theta"});
  ASSERT_TRUE(result.participation_is_meaningful);
  for (const auto& mode : result.modes) {
    double total = 0.0;
    for (const double value : mode.participation) {
      EXPECT_GE(value, 0.0);
      total += value;
    }
    EXPECT_NEAR(total, 1.0, 1e-12);
  }
}

TEST(Modes, ClassifiesLongitudinalModesByParticipationNotByFrequency) {
  // Deliberately perverse: the PHUGOID block is given a HIGHER frequency than
  // the short-period block. A classifier keying on frequency gets both labels
  // backwards here; one keying on participation does not.
  //
  // States: [u, theta, alpha, q]. The first block couples u and theta, the
  // second alpha and q.
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(4, 4);
  a.block<2, 2>(0, 0) = oscillator(3.0, 0.05);  // "phugoid" states, fast
  a.block<2, 2>(2, 2) = oscillator(0.4, 0.6);   // "short period" states, slow

  const std::vector<std::string> names = {"u", "theta", "alpha", "q"};
  const auto roles = StateRoles::from_names(names);
  ASSERT_GE(roles.axial_speed, 0);
  ASSERT_GE(roles.pitch_attitude, 0);
  ASSERT_GE(roles.angle_of_attack, 0);
  ASSERT_GE(roles.pitch_rate, 0);

  const auto result = analyze_modes(a, names, roles);

  const auto* phugoid = result.find(ModeLabel::Phugoid);
  const auto* short_period = result.find(ModeLabel::ShortPeriod);
  ASSERT_NE(phugoid, nullptr);
  ASSERT_NE(short_period, nullptr);

  EXPECT_NEAR(phugoid->natural_frequency_rad_s, 3.0, 1e-9)
      << "the phugoid label followed the u/theta states, not the low frequency";
  EXPECT_NEAR(short_period->natural_frequency_rad_s, 0.4, 1e-9);
  EXPECT_GT(phugoid->label_score, 0.9);
  EXPECT_GT(short_period->label_score, 0.9);
}

TEST(Modes, ClassifiesTheFullLateralSet) {
  // States: [beta, r, p, phi]. Dutch roll couples beta and r; roll subsidence
  // is a fast real root in p; spiral is a slow real root in phi.
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(4, 4);
  a.block<2, 2>(0, 0) = oscillator(1.13, 0.0609);  // Dutch roll
  a(2, 2) = -2.2;                                  // roll subsidence
  a(3, 3) = 0.0318;                                // spiral, divergent

  const std::vector<std::string> names = {"beta", "r", "p", "phi"};
  const auto result = analyze_modes(a, names, StateRoles::from_names(names));

  const auto* dutch = result.find(ModeLabel::DutchRoll);
  const auto* roll = result.find(ModeLabel::RollSubsidence);
  const auto* spiral = result.find(ModeLabel::Spiral);
  ASSERT_NE(dutch, nullptr);
  ASSERT_NE(roll, nullptr);
  ASSERT_NE(spiral, nullptr);

  EXPECT_NEAR(dutch->natural_frequency_rad_s, 1.13, 1e-9);
  EXPECT_NEAR(dutch->damping_ratio, 0.0609, 1e-9);
  EXPECT_NEAR(roll->time_constant_s, 1.0 / 2.2, 1e-9);
  EXPECT_LT(spiral->damping_ratio, 0.0) << "this spiral is divergent";
  EXPECT_GT(spiral->time_to_double_amplitude_s, 0.0);
}

TEST(Modes, ReportsIllConditionedEigenvectorsRatherThanInventingParticipation) {
  // A Jordan block is defective: it has one eigenvector for a repeated
  // eigenvalue, so participation factors are not defined. This is not an exotic
  // case — a CG sweep walks through one wherever two real roots coalesce into a
  // complex pair.
  Eigen::MatrixXd a(2, 2);
  a << -1.0, 1.0, 0.0, -1.0;
  const auto result = analyze_modes(a, {"x", "y"});

  EXPECT_FALSE(result.participation_is_meaningful);
  EXPECT_GT(result.eigenvector_condition_number, 1e10);
  // The eigenvalues are still reported: they are well defined even when the
  // eigenvectors are not.
  ASSERT_FALSE(result.modes.empty());
  for (const auto& mode : result.modes) {
    EXPECT_NEAR(mode.eigenvalue.real(), -1.0, 1e-6);
    EXPECT_EQ(mode.label, ModeLabel::Unclassified);
  }
}

TEST(Modes, LabelsAreNeverAssignedTwice) {
  // Two similar oscillations both scoring on the same signature must not both
  // be labelled. The second one has to go unlabelled rather than duplicate.
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(4, 4);
  a.block<2, 2>(0, 0) = oscillator(1.0, 0.3);
  a.block<2, 2>(2, 2) = oscillator(1.1, 0.3);

  const std::vector<std::string> names = {"alpha", "q", "alpha2", "q2"};
  const auto result = analyze_modes(a, names, StateRoles::from_names(names));

  int short_period_count = 0;
  for (const auto& mode : result.modes) {
    if (mode.label == ModeLabel::ShortPeriod) {
      ++short_period_count;
    }
  }
  EXPECT_EQ(short_period_count, 1);
}

TEST(Modes, RejectsMismatchedStateNames) {
  Eigen::MatrixXd a(2, 2);
  a << -1.0, 0.0, 0.0, -2.0;
  EXPECT_THROW((void)analyze_modes(a, {"only_one"}), std::invalid_argument);

  Eigen::MatrixXd rectangular(2, 3);
  rectangular.setZero();
  EXPECT_THROW((void)analyze_modes(rectangular, {"a", "b"}), std::invalid_argument);
}

TEST(Modes, ModeOrderIsDeterministic) {
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(6, 6);
  a.block<2, 2>(0, 0) = oscillator(2.0, 0.2);
  a.block<2, 2>(2, 2) = oscillator(0.5, 0.1);
  a(4, 4) = -3.0;
  a(5, 5) = -0.02;

  const auto first = analyze_modes(a, {"a", "b", "c", "d", "e", "f"});
  const auto second = analyze_modes(a, {"a", "b", "c", "d", "e", "f"});
  ASSERT_EQ(first.modes.size(), second.modes.size());
  for (std::size_t i = 0; i < first.modes.size(); ++i) {
    EXPECT_EQ(first.modes[i].eigenvalue, second.modes[i].eigenvalue);
  }
  // Non-oscillatory modes sort first, then by natural frequency.
  EXPECT_FALSE(first.modes[0].is_oscillatory);
  EXPECT_FALSE(first.modes[1].is_oscillatory);
  EXPECT_LT(first.modes[0].natural_frequency_rad_s, first.modes[1].natural_frequency_rad_s);
}

}  // namespace
