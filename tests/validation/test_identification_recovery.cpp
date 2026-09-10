// SPDX-License-Identifier: Apache-2.0
//
// Identification acceptance: recovery of DECLARED parameters from a record
// carrying DECLARED NOISE, to within the uncertainty the fit itself reports.
//
// WHY A NOISELESS SELF-TEST IS NOT THIS TEST. `examples/quadrotor-identification`
// fits against a record that is the truth model's own output to round-off. That
// demonstrates the machinery recovers a parameter it can SEE, and that the path
// from import through export to trim is wired correctly. It demonstrates
// nothing about behaviour against a sensor: the residual sits at the arithmetic
// floor, and the standard errors it reports are correct and useless in
// magnitude — they say the record shows no scatter, which is true and carries
// no information. RFC-0002's WP4 verification asks for the other test, and this
// is it.
//
// THE ACCEPTANCE CRITERION IS THE FIT'S OWN INTERVAL, not a tolerance chosen
// here. A fit that reports a standard error is making a checkable claim: that
// the truth lies within a few of them of the estimate. So the budget is stated
// as a multiple of the REPORTED standard error, fixed before any number is
// read, and a fit whose intervals are too narrow for its own error fails —
// which is the failure mode a fixed absolute tolerance cannot see, because a
// fit can be accurate and overconfident at the same time.
//
// WHAT IS NOT CLAIMED. Nothing here is evidence about an aircraft. The records
// are galata's own output with pseudo-random numbers added, the noise is
// independent and Gaussian because that is what the reported covariance assumes,
// and real sensor error is none of those things — it is coloured, quantised,
// occasionally missing, and correlated with the manoeuvre. A fit that behaves
// here has been shown to behave under the noise model its own uncertainty
// arithmetic assumes, and no further.

#include "galata/core/constants.hpp"
#include "galata/core/state.hpp"
#include "galata/data/record.hpp"
#include "galata/data/window.hpp"
#include "galata/identify/greybox.hpp"
#include "galata/identify/validate.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/numerics/integrator.hpp"

#include "validation_config.hpp"
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace {

using galata::data::Channel;
using galata::data::Record;
using galata::identify::fit_greybox;
using galata::identify::GreyboxRequest;
using galata::identify::RecordSeparation;
using galata::identify::validate_model;
using galata::identify::ValidationRequest;
using galata::model::Quadrotor;

// A DECLARED, PLATFORM-INDEPENDENT NORMAL SEQUENCE.
//
// `std::mt19937` is specified bit for bit by the standard and produces the same
// sequence everywhere. `std::normal_distribution` is NOT — its algorithm is the
// implementation's choice, so two standard libraries hand back different
// numbers from the same engine and the same seed. A test whose fixture depends
// on which standard library built it is not reproducible, and ADR-0004 asks for
// better than that. So the transform is written out: Box-Muller on two uniform
// deviates taken from the engine's own 32-bit output, in a fixed order.
//
// The generator is a test fixture and deliberately not a shipped capability.
// Adding noise to a measured record is not an operation this repository wants a
// study to be able to perform.
class DeclaredNormal {
 public:
  explicit DeclaredNormal(std::uint32_t seed) : engine_(seed) {}

  double next() {
    // Box-Muller: two uniforms on (0, 1], one cosine. The pair's second value
    // is discarded rather than cached, so the k-th draw depends only on k and
    // the seed, whatever order the caller asks in.
    const double u1 = uniform();
    const double u2 = uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * galata::core::kPi * u2);
  }

 private:
  double uniform() {
    // (0, 1]: the engine's output plus one over its range plus one, so a zero
    // draw cannot reach the logarithm.
    const double raw = static_cast<double>(engine_() - std::mt19937::min());
    const double span = static_cast<double>(std::mt19937::max() - std::mt19937::min()) + 1.0;
    return (raw + 1.0) / (span + 1.0);
  }

  std::mt19937 engine_;
};

Quadrotor shipped() {
  return galata::model::load_quadrotor(std::string(GALATA_MODELS_DIR)
                                       + "/souxmar-quad/souxmar-quad.yaml");
}

Eigen::VectorXd hover_state(const Quadrotor& model, double altitude_m) {
  Eigen::VectorXd x = Eigen::VectorXd::Zero(model.extended_state_size());
  x(galata::core::kPositionDown) = -altitude_m;
  x(galata::core::kQuaternionW) = 1.0;
  const double hover = model.hover_speed_rad_s(galata::core::kStandardGravity);
  x.segment(model.rotor_state_offset(), model.rotor_count()).setConstant(hover);
  return x;
}

// TWO COMMAND HISTORIES, and the difference between them is the point of the
// unidentifiable case below.
//
// IDENTIFIABILITY IS A PROPERTY OF THE RECORD, NOT OF THE MODEL. The same two
// parameters are recoverable from one of these manoeuvres and are refused
// outright on the other, and no property of `model::Quadrotor` changed between
// the two runs. A test that only showed a refusal would leave a reader unable to
// tell a strict routine from a broken one; showing both halves on one parameter
// set is what makes the refusal informative.
enum class Excitation {
  // Pitch up, pitch down, roll one way, roll back. The body pitch rate moves,
  // so the moment that damps it is visible in the record.
  PitchAndRoll,
  // Four equal rotors, stepped. The vehicle climbs and never rotates: p, q and
  // r are identically zero for the whole record, because the thrusts are equal
  // and the reaction torques of the opposite-spin diagonals cancel exactly. A
  // moment proportional to a body rate that is identically zero contributes
  // identically nothing, so the pitch-damping column of the sensitivity matrix
  // is EXACTLY zero rather than merely small.
  VerticalOnly,
};

Eigen::VectorXd command_at(const Quadrotor& model, double time_s, Excitation excitation) {
  const double hover = model.hover_speed_rad_s(galata::core::kStandardGravity);
  Eigen::VectorXd u = Eigen::VectorXd::Constant(model.rotor_count(), hover + 6.0);
  if (excitation == Excitation::VerticalOnly) {
    // A collective step, equal on every rotor, so nothing can rotate.
    if (time_s >= 0.6 && time_s < 1.4) {
      u.array() += 14.0;
    } else if (time_s >= 1.4) {
      u.array() -= 9.0;
    }
    return u;
  }
  const double d = 25.0;
  if (time_s >= 0.2 && time_s < 0.6) {
    u(0) -= d;
    u(1) += d;
    u(2) += d;
    u(3) -= d;  // nose up
  } else if (time_s >= 0.6 && time_s < 1.2) {
    u(0) += d;
    u(1) -= d;
    u(2) -= d;
    u(3) += d;  // nose down
  } else if (time_s >= 1.2 && time_s < 1.6) {
    u(0) += d;
    u(1) += d;
    u(2) -= d;
    u(3) -= d;  // roll
  } else if (time_s >= 1.6) {
    u(0) -= d;
    u(1) -= d;
    u(2) += d;
    u(3) += d;  // roll back
  }
  return u;
}

// The noise levels the records carry, one per fitted channel, in that channel's
// own SI unit. Declared here, before any fit, and quoted in the acceptance
// criteria: 5 cm of position, 2 cm/s of velocity, 5 mrad/s of rate. They are
// chosen to be a visible fraction of each channel's excursion over the
// manoeuvre and are not measurements of any sensor.
constexpr double kSigmaDownM = 0.05;
constexpr double kSigmaVerticalSpeed = 0.02;
constexpr double kSigmaPitchRate = 0.005;

struct SyntheticRecord {
  Record record;
  double sigma_down_m = 0.0;
  double sigma_w_m_s = 0.0;
  double sigma_q_rad_s = 0.0;
};

// Simulates the truth model over `duration_s` at `sample_s`, then adds
// independent Gaussian noise of the declared size to the three fitted channels
// and to nothing else. The command channels are recorded EXACTLY: a commanded
// rotor speed is what the study asked for, not something a sensor measured, so
// adding noise to it would be modelling a different experiment.
SyntheticRecord synthesise(const Quadrotor& truth,
                           double noise_scale,
                           std::uint32_t seed,
                           const std::string& digest,
                           Excitation excitation = Excitation::PitchAndRoll,
                           double duration_s = 2.4,
                           double sample_s = 0.02) {
  SyntheticRecord out;
  out.sigma_down_m = kSigmaDownM * noise_scale;
  out.sigma_w_m_s = kSigmaVerticalSpeed * noise_scale;
  out.sigma_q_rad_s = kSigmaPitchRate * noise_scale;

  Record& record = out.record;
  record.source_path = "synthetic-noisy.csv";
  record.source_sha256 = digest;
  record.description = "synthetic manoeuvre with declared Gaussian noise";

  const std::vector<std::string> names = truth.extended_state_names();
  // Every state is recorded, so a window can start where it starts. The three
  // NOISY channels are the ones the fit compares against; the rest are exact and
  // are used only to seed an initial state.
  std::vector<Channel> states;
  states.reserve(names.size());
  for (const std::string& name : names) {
    states.push_back({name, name, "si", "none", "", 1.0, 0.0, {}});
  }
  std::vector<Channel> commands;
  for (int r = 0; r < truth.rotor_count(); ++r) {
    commands.push_back({"cmd_" + std::to_string(r), "cmd", "rad/s", "none", "", 1.0, 0.0, {}});
  }

  DeclaredNormal noise(seed);
  Eigen::VectorXd state = hover_state(truth, 100.0);
  const auto steps = static_cast<int>(std::llround(duration_s / sample_s));
  for (int k = 0; k <= steps; ++k) {
    const double time_s = sample_s * static_cast<double>(k);
    record.times_s.push_back(time_s);
    for (std::size_t i = 0; i < names.size(); ++i) {
      states[i].samples.push_back(state(static_cast<Eigen::Index>(i)));
    }
    const Eigen::VectorXd u = command_at(truth, time_s, excitation);
    for (int r = 0; r < truth.rotor_count(); ++r) {
      commands[static_cast<std::size_t>(r)].samples.push_back(u(r));
    }
    if (k == steps) {
      break;
    }
    const galata::numerics::DerivativeFunction derivative = [&](double, const Eigen::VectorXd& x) {
      return truth.derivative(x, u);
    };
    const galata::numerics::ProjectionFunction projection = [&](Eigen::VectorXd& x) {
      truth.project(x);
    };
    state = galata::numerics::integrate_fixed_step(derivative, state, 0.0, 0.004, 5, 5, projection)
                .states.back();
  }

  // The noisy observation channels, added beside the exact states rather than
  // replacing them, so the record carries both what happened and what was
  // "measured". Drawn in one pass per channel, in a fixed channel order.
  const std::size_t count = record.times_s.size();
  const auto noisy = [&](const std::string& source, const std::string& name, double sigma) {
    Channel channel{name, source, "si", "none", "", 1.0, 0.0, {}};
    const std::vector<double>& exact =
        states[static_cast<std::size_t>(std::find(names.begin(), names.end(), source)
                                        - names.begin())]
            .samples;
    for (std::size_t k = 0; k < count; ++k) {
      channel.samples.push_back(exact[k] + sigma * noise.next());
    }
    return channel;
  };
  const Channel measured_down = noisy("p_d", "down_m", out.sigma_down_m);
  const Channel measured_w = noisy("w", "w_m_s", out.sigma_w_m_s);
  const Channel measured_q = noisy("q", "q_rad_s", out.sigma_q_rad_s);

  for (const Channel& channel : states) {
    record.channels.push_back(channel);
  }
  for (const Channel& channel : commands) {
    record.channels.push_back(channel);
  }
  record.channels.push_back(measured_down);
  record.channels.push_back(measured_w);
  record.channels.push_back(measured_q);
  record.rows_read = static_cast<std::int64_t>(count);
  return out;
}

// The base model: the truth with two parameters deliberately wrong.
Quadrotor base_from(const Quadrotor& truth) {
  Quadrotor base = truth;
  base.mass.mass_kg = truth.mass.mass_kg * 0.9;
  base.angular_drag_n_m_s(1) = truth.angular_drag_n_m_s(1) * 2.0;
  base.validate();
  return base;
}

GreyboxRequest two_parameter_request(const Quadrotor& base, const Record& record) {
  GreyboxRequest request;
  request.parameters = {{"mass.mass_kg", 1.0, 3.0, base.mass.mass_kg},
                        {"drag.angular_n_m_s[1]", 5.0e-4, 2.0e-2, base.angular_drag_n_m_s(1)}};
  // The scales are the study's weighting decision, declared on purpose. They are
  // the DECLARED NOISE SIGMAS, which is the choice that makes the objective a
  // properly weighted least-squares problem and therefore the choice under which
  // the reported covariance means what it says.
  request.outputs = {{"down_m", "p_d", kSigmaDownM},
                     {"w_m_s", "w", kSigmaVerticalSpeed},
                     {"q_rad_s", "q", kSigmaPitchRate}};
  for (int r = 0; r < base.rotor_count(); ++r) {
    request.command_channels.push_back("cmd_" + std::to_string(r));
  }
  request.initial_extended_state = Eigen::VectorXd::Zero(base.extended_state_size());
  const std::vector<std::string> names = base.extended_state_names();
  for (std::size_t i = 0; i < names.size(); ++i) {
    request.initial_extended_state(static_cast<Eigen::Index>(i)) =
        record.find(names[i])->samples.front();
  }
  base.project(request.initial_extended_state);
  request.step_s = 0.004;
  request.iterations = 40;
  return request;
}

// THE ACCEPTANCE BUDGET, FIXED BEFORE ANY NUMBER IS READ.
//
// Three standard errors. For an estimator whose reported covariance is
// trustworthy and whose errors are Gaussian, the truth falls inside that
// interval about 99.7 times in a hundred; the seeds below are fixed, so this is
// a statement about these records rather than a coverage claim over an ensemble.
// The multiple is not tightened afterwards to fit a result, and it is not
// widened either: a fit whose interval is too narrow for its own error must
// fail here, because "accurate" and "honestly uncertain" are two claims and
// this test is about the second one.
constexpr double kStandardErrorsAllowed = 3.0;

// A floor on the interval, so the criterion cannot be satisfied by an estimator
// that reports an absurdly wide one. Expressed as a fraction of each
// parameter's true value: an interval wider than this is not a useful estimate
// even when it contains the answer, and a fit that widened its way to passing
// would be caught here.
constexpr double kRelativeIntervalCeiling = 0.25;

}  // namespace

// ---------------------------------------------------------------------------
// 1. Recovery under declared noise, against the fit's own reported interval
// ---------------------------------------------------------------------------

TEST(IdentificationRecovery, RecoversDeclaredParametersFromANoisyRecordWithinItsOwnIntervals) {
  const Quadrotor truth = shipped();
  const Quadrotor base = base_from(truth);
  const SyntheticRecord synthetic = synthesise(truth, 1.0, 20260910u, std::string(64, '1'));

  RecordProperty("noise_sigma_down_m", std::to_string(synthetic.sigma_down_m));
  RecordProperty("noise_sigma_w_m_s", std::to_string(synthetic.sigma_w_m_s));
  RecordProperty("noise_sigma_q_rad_s", std::to_string(synthetic.sigma_q_rad_s));
  RecordProperty("standard_errors_allowed", std::to_string(kStandardErrorsAllowed));

  const auto fit =
      fit_greybox(base, synthetic.record, two_parameter_request(base, synthetic.record));

  ASSERT_EQ(fit.value.size(), 2);
  ASSERT_TRUE(fit.uncertainty_is_estimable)
      << "a noisy record demonstrates scatter, so an uncertainty must be estimable: " << fit.note;
  // The assumptions the interval rests on are stated in the result, and the
  // study's scales were chosen to make them hold: the objective is weighted by
  // the declared sigmas, so "equal variance in the scaled units" is true here by
  // construction rather than by hope.
  EXPECT_NE(fit.note.find("Gauss-Newton"), std::string::npos) << fit.note;
  EXPECT_NE(fit.note.find("SCALED units"), std::string::npos) << fit.note;

  const double truth_values[2] = {truth.mass.mass_kg, truth.angular_drag_n_m_s(1)};
  for (Eigen::Index p = 0; p < 2; ++p) {
    const double estimate = fit.value(p);
    const double interval = fit.standard_error(p);
    const double error = std::fabs(estimate - truth_values[p]);
    RecordProperty(fit.names[static_cast<std::size_t>(p)] + "_error", std::to_string(error));
    RecordProperty(fit.names[static_cast<std::size_t>(p)] + "_standard_error",
                   std::to_string(interval));

    EXPECT_GT(interval, 0.0) << fit.names[static_cast<std::size_t>(p)]
                             << ": a noisy record must support a nonzero interval";
    EXPECT_LE(error, kStandardErrorsAllowed * interval)
        << fit.names[static_cast<std::size_t>(p)] << ": the truth is " << error
        << " away and the fit claims a standard error of " << interval
        << ", so the estimate is outside its own " << kStandardErrorsAllowed
        << "-sigma interval. Either the estimate is wrong or the interval is too narrow, and "
           "both are failures of the same claim";
    EXPECT_LT(interval, kRelativeIntervalCeiling * std::fabs(truth_values[p]))
        << fit.names[static_cast<std::size_t>(p)]
        << ": the reported interval is wide enough to contain almost anything, so containing "
           "the answer establishes nothing";
  }

  // The fit must not have driven the residual below the noise it was given. A
  // residual RMS well under one, in units of the declared sigmas, would mean the
  // optimiser had fitted the noise rather than the dynamics.
  RecordProperty("residual_rms_in_sigma_units", std::to_string(fit.residual_rms));
  EXPECT_GT(fit.residual_rms, 0.3)
      << "the residual is far below the noise floor the record carries, which means the fit "
         "absorbed noise into its parameters";
  EXPECT_LT(fit.residual_rms, 3.0)
      << "the residual is far above the noise floor, so something other than noise is left";

  // Convergence evidence, which is evidence and not a verdict: the objective
  // improved from a deliberately wrong start, and the first-order measure is
  // small against the parameters' own declared ranges.
  EXPECT_TRUE(fit.objective_improved);
  EXPECT_LT(fit.objective, fit.initial_objective);
  EXPECT_GE(fit.convergence.last_accepted_iteration, 0);
}

// ---------------------------------------------------------------------------
// 2. The reported interval tracks the noise the record carries
// ---------------------------------------------------------------------------
//
// This is the property that makes the interval worth quoting. An estimator that
// reported the same uncertainty whatever the data would pass case 1 whenever it
// happened to be accurate, and would be reporting a constant. The Gauss-Newton
// covariance scales with the residual variance, so a four-fold noise increase
// must widen the interval by roughly four — and "roughly" is bounded here rather
// than left vague.

TEST(IdentificationRecovery, TheReportedIntervalWidensWithTheNoiseTheRecordCarries) {
  const Quadrotor truth = shipped();
  const Quadrotor base = base_from(truth);
  const double factor = 4.0;

  const SyntheticRecord quiet = synthesise(truth, 1.0, 20260910u, std::string(64, '1'));
  const SyntheticRecord loud = synthesise(truth, factor, 20260910u, std::string(64, '2'));

  const auto quiet_fit = fit_greybox(base, quiet.record, two_parameter_request(base, quiet.record));
  const auto loud_fit = fit_greybox(base, loud.record, two_parameter_request(base, loud.record));
  ASSERT_TRUE(quiet_fit.uncertainty_is_estimable);
  ASSERT_TRUE(loud_fit.uncertainty_is_estimable);

  // The budget: the ratio must lie between half and twice the noise factor. That
  // band is wide because the two records are different realisations and the
  // covariance is an approximation at a point, and it is narrow enough to fail
  // an estimator that reported a constant (ratio 1) or one that scaled with the
  // square of the noise (ratio 16).
  const double lower = factor / 2.0;
  const double upper = factor * 2.0;
  RecordProperty("noise_factor", std::to_string(factor));
  RecordProperty("interval_ratio_band", std::to_string(lower) + " to " + std::to_string(upper));

  for (Eigen::Index p = 0; p < 2; ++p) {
    const double ratio = loud_fit.standard_error(p) / quiet_fit.standard_error(p);
    RecordProperty(quiet_fit.names[static_cast<std::size_t>(p)] + "_interval_ratio",
                   std::to_string(ratio));
    EXPECT_GT(ratio, lower) << quiet_fit.names[static_cast<std::size_t>(p)]
                            << ": the interval barely moved when the noise quadrupled, so it is "
                               "not reporting the data's scatter";
    EXPECT_LT(ratio, upper) << quiet_fit.names[static_cast<std::size_t>(p)]
                            << ": the interval grew far faster than the noise";
  }
}

// ---------------------------------------------------------------------------
// 3. A parameter set the record cannot separate is refused, with the reason
// ---------------------------------------------------------------------------

TEST(IdentificationRecovery, TheSameParameterSetIsRefusedOnARecordThatCannotSeparateIt) {
  const Quadrotor truth = shipped();
  const Quadrotor base = base_from(truth);

  // A purely vertical manoeuvre. The body rates are identically zero for the
  // whole record, so the moment that damps pitch acts on nothing and the fit is
  // being asked for a number the data does not contain.
  const SyntheticRecord vertical =
      synthesise(truth, 1.0, 20260910u, std::string(64, '3'), Excitation::VerticalOnly);
  {
    double worst_rate = 0.0;
    for (const char* name : {"p", "q", "r"}) {
      for (const double sample : vertical.record.find(name)->samples) {
        worst_rate = std::fmax(worst_rate, std::fabs(sample));
      }
    }
    RecordProperty("vertical_worst_body_rate_rad_s", std::to_string(worst_rate));
    // Not "small": zero. Four equal thrusts produce no moment and the
    // opposite-spin diagonals cancel exactly, so the sensitivity column is
    // exactly zero and the refusal is structural rather than a threshold.
    ASSERT_EQ(worst_rate, 0.0)
        << "the vertical case rests on the body rates being identically zero; if this record "
           "rotates at all, the refusal below is about a threshold rather than about a "
           "direction the data cannot see";
  }

  try {
    (void)fit_greybox(base, vertical.record, two_parameter_request(base, vertical.record));
    FAIL() << "a parameter direction the record cannot see must be refused, not estimated";
  } catch (const std::invalid_argument& error) {
    const std::string message = error.what();
    RecordProperty("refusal", message);
    EXPECT_NE(message.find("does not constrain every declared parameter"), std::string::npos)
        << message;
    EXPECT_NE(message.find("condition number"), std::string::npos) << message;
    // The distinction ADR-0008 requires, made in the refusal itself: finishing
    // is not measuring, and the optimiser's willingness to return a number is
    // not evidence that the number means anything.
    EXPECT_NE(message.find("The optimiser finished"), std::string::npos) << message;
    EXPECT_NE(message.find("where it happened to stop"), std::string::npos) << message;
    // And it says what to do about it, which a refusal that only reports a
    // number does not.
    EXPECT_NE(message.find("Excite the"), std::string::npos) << message;
  }

  // THE OTHER HALF. The identical parameter set, the identical bounds, the
  // identical starting values and the identical noise level, on a record that
  // pitches and rolls — accepted, and recovered. Identifiability moved because
  // the manoeuvre moved.
  const SyntheticRecord excited =
      synthesise(truth, 1.0, 20260910u, std::string(64, '1'), Excitation::PitchAndRoll);
  const auto fit = fit_greybox(base, excited.record, two_parameter_request(base, excited.record));
  RecordProperty("excited_condition_number", std::to_string(fit.jacobian_condition_number));
  EXPECT_TRUE(fit.uncertainty_is_estimable);
  EXPECT_LE(std::fabs(fit.value(1) - truth.angular_drag_n_m_s(1)),
            kStandardErrorsAllowed * fit.standard_error(1))
      << "the parameter refused on the vertical record must be recovered on the one that "
         "excites it, or the refusal was about something else";
}

// A parameter whose effect the record can barely see is NOT the same case, and
// is not refused: the routine's test is about a direction the data cannot see at
// all. What happens instead is that the estimate walks to a declared bound and
// is reported as resting on one — which is the honest outcome and is why
// `at_bound` exists.
TEST(IdentificationRecovery, AWeaklyExcitedParameterIsReportedAsRestingOnItsBoundNotRefused) {
  const Quadrotor truth = shipped();
  const Quadrotor base = base_from(truth);
  const SyntheticRecord excited = synthesise(truth, 1.0, 20260910u, std::string(64, '1'));

  // Yaw angular drag. The commands produce no net yaw torque, but the rotor lag
  // makes the four speeds differ transiently through a roll reversal, so a yaw
  // rate of a few ten-thousandths of a radian per second does appear and decays
  // through this coefficient. The record sees it — barely.
  auto request = two_parameter_request(base, excited.record);
  request.parameters.push_back(
      {"drag.angular_n_m_s[2]", 5.0e-4, 2.0e-2, base.angular_drag_n_m_s(2)});

  const auto fit = fit_greybox(base, excited.record, request);
  RecordProperty("weakly_excited_condition_number", std::to_string(fit.jacobian_condition_number));
  RecordProperty("yaw_drag_estimate", std::to_string(fit.value(2)));
  // Not refused, because the direction is visible. Poorly determined, which the
  // condition number says and the bound flag confirms.
  EXPECT_GT(fit.jacobian_condition_number, 1.0e3)
      << "a barely-excited parameter should show as an ill-conditioned sensitivity matrix";
  EXPECT_TRUE(fit.at_bound[2])
      << "an estimate the data barely constrains drifts to a declared limit, and reporting it "
         "as an interior estimate would be the dishonest outcome this flag exists to prevent";
  // The two parameters the record DOES see are still recovered, so the presence
  // of a poorly determined third has not corrupted them.
  EXPECT_LE(std::fabs(fit.value(0) - truth.mass.mass_kg),
            kStandardErrorsAllowed * fit.standard_error(0));
}

// ---------------------------------------------------------------------------
// 4. A wrong model is caught by the SHAPE of its held-out residual
// ---------------------------------------------------------------------------
//
// The interesting failure is not a large residual — a large residual could be a
// noisy sensor. It is a residual with STRUCTURE in it, which noise does not
// have. A model missing a physical effect leaves a residual correlated with
// itself; a model that is right leaves one that looks like the noise it was
// given. This case requires both halves, on the same held-out window, so that
// the discriminator is demonstrated rather than assumed.

TEST(IdentificationRecovery, AWrongModelIsCaughtByTheShapeOfItsHeldOutResidualNotOnlyItsSize) {
  const Quadrotor truth = shipped();
  const SyntheticRecord synthetic = synthesise(truth, 1.0, 20260910u, std::string(64, '1'));

  const Record estimation = galata::data::window_record(synthetic.record, 0.0, 1.4);
  const Record heldout = galata::data::window_record(synthetic.record, 1.4, 2.5);

  const auto validation_request = [&](const Quadrotor& model, const Record& record) {
    ValidationRequest request;
    request.estimation_record = &estimation;
    request.outputs = {{"down_m", "p_d"}, {"q_rad_s", "q"}};
    for (int r = 0; r < model.rotor_count(); ++r) {
      request.command_channels.push_back("cmd_" + std::to_string(r));
    }
    request.initial_extended_state = Eigen::VectorXd::Zero(model.extended_state_size());
    const std::vector<std::string> names = model.extended_state_names();
    for (std::size_t i = 0; i < names.size(); ++i) {
      request.initial_extended_state(static_cast<Eigen::Index>(i)) =
          record.find(names[i])->samples.front();
    }
    model.project(request.initial_extended_state);
    request.step_s = 0.004;
    return request;
  };

  // THE WRONG MODEL. Not merely mis-parameterised: its pitch angular drag is
  // removed entirely, so the body-rate axis it governs has no decay at all and
  // no value of the remaining parameters can supply one. This is a structural
  // error, which is the kind held-out validation exists to find.
  Quadrotor wrong = truth;
  wrong.angular_drag_n_m_s(1) = 1.0e-6;
  wrong.validate();

  const auto right_result = validate_model(truth, heldout, validation_request(truth, heldout));
  const auto wrong_result = validate_model(wrong, heldout, validation_request(wrong, heldout));

  ASSERT_EQ(right_result.separation, RecordSeparation::VerifiedDisjoint)
      << right_result.separation_basis;
  ASSERT_EQ(wrong_result.separation, RecordSeparation::VerifiedDisjoint);
  ASSERT_EQ(right_result.outputs.size(), 2u);

  // The pitch-rate channel is where the removed effect acts, so that is the
  // channel the discriminator is read on. The budget: the wrong model's residual
  // must be at least five times the right model's on that channel, and its
  // lag-one autocorrelation must be high where the right model's is not.
  const auto& right_q = right_result.outputs[1];
  const auto& wrong_q = wrong_result.outputs[1];
  RecordProperty("right_model_q_rmse", std::to_string(right_q.rmse));
  RecordProperty("wrong_model_q_rmse", std::to_string(wrong_q.rmse));
  RecordProperty("right_model_q_autocorrelation",
                 std::to_string(right_q.residual_lag_one_autocorrelation));
  RecordProperty("wrong_model_q_autocorrelation",
                 std::to_string(wrong_q.residual_lag_one_autocorrelation));

  EXPECT_GT(wrong_q.rmse, 5.0 * right_q.rmse)
      << "removing the pitch damping entirely must not pass unnoticed on the pitch-rate channel";
  ASSERT_TRUE(wrong_q.autocorrelation_is_defined);
  ASSERT_TRUE(right_q.autocorrelation_is_defined);
  EXPECT_GT(wrong_q.residual_lag_one_autocorrelation, 0.5)
      << "a residual left by a missing dynamic effect is correlated with itself; this is the "
         "half of the discriminator that a large noisy residual cannot fake";
  EXPECT_LT(right_q.residual_lag_one_autocorrelation, 0.5)
      << "the correct model's residual should look like the independent noise it was given, "
         "which is what makes the wrong model's structure a finding rather than a baseline";

  // And the fit fraction, which is defined here because the channel moves,
  // separates them the same way.
  ASSERT_TRUE(right_q.fit_fraction_is_defined);
  ASSERT_TRUE(wrong_q.fit_fraction_is_defined);
  EXPECT_GT(right_q.fit_fraction, wrong_q.fit_fraction);
}

// ---------------------------------------------------------------------------
// 5. The held-out window is scored, and the label says what it does not mean
// ---------------------------------------------------------------------------

TEST(IdentificationRecovery, AFitFromOneWindowIsScoredOnAnotherAndTheLabelIsNotIndependence) {
  const Quadrotor truth = shipped();
  const Quadrotor base = base_from(truth);
  const SyntheticRecord synthetic = synthesise(truth, 1.0, 20260910u, std::string(64, '1'));

  const Record estimation = galata::data::window_record(synthetic.record, 0.0, 1.4);
  const Record heldout = galata::data::window_record(synthetic.record, 1.4, 2.5);

  const auto fit = fit_greybox(base, estimation, two_parameter_request(base, estimation));
  ASSERT_TRUE(fit.uncertainty_is_estimable);

  // The fit saw only the first window and must still recover the parameters, to
  // the same criterion as case 1.
  const double truth_values[2] = {truth.mass.mass_kg, truth.angular_drag_n_m_s(1)};
  for (Eigen::Index p = 0; p < 2; ++p) {
    EXPECT_LE(std::fabs(fit.value(p) - truth_values[p]),
              kStandardErrorsAllowed * fit.standard_error(p))
        << fit.names[static_cast<std::size_t>(p)]
        << ": fitted on one window, outside its own interval";
  }

  ValidationRequest request;
  request.estimation_record = &estimation;
  request.outputs = {{"down_m", "p_d"}, {"q_rad_s", "q"}};
  for (int r = 0; r < truth.rotor_count(); ++r) {
    request.command_channels.push_back("cmd_" + std::to_string(r));
  }
  request.initial_extended_state = Eigen::VectorXd::Zero(truth.extended_state_size());
  const std::vector<std::string> names = truth.extended_state_names();
  for (std::size_t i = 0; i < names.size(); ++i) {
    request.initial_extended_state(static_cast<Eigen::Index>(i)) =
        heldout.find(names[i])->samples.front();
  }
  fit.fitted_model.project(request.initial_extended_state);
  request.step_s = 0.004;

  const auto scored = validate_model(fit.fitted_model, heldout, request);
  EXPECT_EQ(scored.separation, RecordSeparation::VerifiedDisjoint) << scored.separation_basis;
  RecordProperty("separation_basis", scored.separation_basis);

  // THE SCOPE OF THE LABEL, asserted rather than trusted to a comment. The basis
  // sentence a reader will quote must itself say that this is sample separation
  // and not statistical independence — two windows of one manoeuvre share the
  // aircraft, the trim and the air mass, and no scan in this repository can
  // establish otherwise.
  EXPECT_NE(scored.separation_basis.find("not statistical independence"), std::string::npos)
      << scored.separation_basis;
  EXPECT_NE(scored.assumptions.find("not a claim of statistical independence"), std::string::npos)
      << scored.assumptions;

  // The prediction on unseen data must be of the order of the noise, not of the
  // order of the signal: the fitted model predicts the held-out window about as
  // well as the noise permits.
  const auto& down = scored.outputs.front();
  RecordProperty("heldout_down_rmse_m", std::to_string(down.rmse));
  EXPECT_LT(down.rmse, 4.0 * synthetic.sigma_down_m)
      << "a model fitted on one window should predict the next to within a few times the noise "
         "the record carries";
  ASSERT_TRUE(down.fit_fraction_is_defined);
  EXPECT_GT(down.fit_fraction, 0.9);
}
