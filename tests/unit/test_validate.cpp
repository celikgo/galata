// SPDX-License-Identifier: Apache-2.0
//
// Held-out validation.
//
// THE TEST THAT MATTERS IS THE THIRD ONE. The first two check that a correct
// model scores well and a wrong one scores badly, which any metric does. The
// third checks that running a model on the record it was FITTED to is labelled
// as not held out — because that is the claim this capability exists to keep
// people from making by accident, and no arithmetic prevents it.

#include "galata/core/constants.hpp"
#include "galata/core/state.hpp"
#include "galata/data/record.hpp"
#include "galata/data/window.hpp"
#include "galata/identify/validate.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/numerics/integrator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace {

using galata::data::Channel;
using galata::data::Record;
using galata::identify::Independence;
using galata::identify::validate_model;
using galata::identify::ValidationRequest;
using galata::model::Quadrotor;

Quadrotor shipped() {
  Quadrotor model;
  model.mass.mass_kg = 1.6;
  model.mass.inertia_cg_body_kg_m2 = Eigen::Matrix3d::Identity();
  model.mass.inertia_cg_body_kg_m2(0, 0) = 0.023;
  model.mass.inertia_cg_body_kg_m2(1, 1) = 0.023;
  model.mass.inertia_cg_body_kg_m2(2, 2) = 0.042;
  const double arm = 0.16263455967290593;
  const int spin[4] = {1, -1, 1, -1};
  const double x[4] = {arm, -arm, -arm, arm};
  const double y[4] = {-arm, -arm, arm, arm};
  for (int r = 0; r < 4; ++r) {
    galata::model::Rotor rotor;
    rotor.position_cg_to_hub_body_m = Eigen::Vector3d(x[r], y[r], 0.0);
    rotor.spin_about_body_z = spin[r];
    rotor.thrust_coefficient_n_s2 = 1.0e-5;
    rotor.torque_coefficient_n_m_s2 = 1.7e-7;
    rotor.speed_time_constant_s = 0.035;
    rotor.minimum_speed_rad_s = 0.0;
    rotor.maximum_speed_rad_s = 1102.4200493562662;
    model.rotors.push_back(rotor);
  }
  model.drag_linear_n_s_m = Eigen::Vector3d(0.12, 0.12, 0.18);
  model.drag_quadratic_n_s2_m2 = Eigen::Vector3d(0.025, 0.025, 0.035);
  model.angular_drag_n_m_s = Eigen::Vector3d(0.002, 0.002, 0.003);
  model.validate();
  return model;
}

Eigen::VectorXd start(const Quadrotor& model, double hover) {
  Eigen::VectorXd x = Eigen::VectorXd::Zero(model.extended_state_size());
  x(galata::core::kQuaternionW) = 1.0;
  for (int r = 0; r < model.rotor_count(); ++r) {
    x(galata::core::kStateSize + r) = hover;
  }
  return x;
}

Record synthesise(const Quadrotor& truth, const std::string& digest, double excitation) {
  const double hover = shipped().hover_speed_rad_s(galata::core::kStandardGravity);
  Record record;
  record.source_path = "synthetic";
  record.source_sha256 = digest;
  Channel down{"down_m", "d", "m", "ned", "", 1.0, 0.0, {}};
  std::vector<Channel> command;
  for (int r = 0; r < truth.rotor_count(); ++r) {
    command.push_back({"cmd_" + std::to_string(r), "c", "rad/s", "none", "", 1.0, 0.0, {}});
  }
  Eigen::VectorXd state = start(truth, hover);
  Eigen::VectorXd u(truth.rotor_count());
  for (int k = 0; k < 50; ++k) {
    for (int r = 0; r < truth.rotor_count(); ++r) {
      u(r) = hover + excitation;
      command[static_cast<std::size_t>(r)].samples.push_back(u(r));
    }
    record.times_s.push_back(0.02 * static_cast<double>(k));
    down.samples.push_back(state(galata::core::kPositionDown));
    const galata::numerics::DerivativeFunction derivative = [&](double, const Eigen::VectorXd& x) {
      return truth.derivative(x, u);
    };
    const galata::numerics::ProjectionFunction projection = [&](Eigen::VectorXd& x) {
      truth.project(x);
    };
    state =
        galata::numerics::integrate_fixed_step(derivative, state, 0.0, 0.002, 10, 10, projection)
            .states.back();
  }
  record.channels.push_back(down);
  for (const Channel& c : command) {
    record.channels.push_back(c);
  }
  return record;
}

ValidationRequest request_for(const Quadrotor& model, const std::string& estimation_digest) {
  ValidationRequest request;
  request.estimation_record_sha256 = estimation_digest;
  request.outputs = {{"down_m", "p_d"}};
  for (int r = 0; r < model.rotor_count(); ++r) {
    request.command_channels.push_back("cmd_" + std::to_string(r));
  }
  request.initial_extended_state =
      start(model, shipped().hover_speed_rad_s(galata::core::kStandardGravity));
  request.step_s = 0.002;
  return request;
}

const std::string kEstimation(64, 'a');
const std::string kValidation(64, 'b');

}  // namespace

TEST(Validate, TheRightModelPredictsARecordItNeverSaw) {
  const Quadrotor truth = shipped();
  // A DIFFERENT excitation from anything a fit would have seen, and a different
  // digest. The digest alone establishes nothing, so with no estimation record
  // to compare against and no claim from the caller, the label is `Unknown` —
  // the numbers below are what this test is about.
  const Record held_out = synthesise(truth, kValidation, 30.0);
  const auto result = validate_model(truth, held_out, request_for(truth, kEstimation));

  EXPECT_EQ(result.independence, Independence::Unknown)
      << "a digest inequality is an inequality of bytes and must not be read as independence";
  ASSERT_EQ(result.outputs.size(), 1u);
  const auto& down = result.outputs.front();
  EXPECT_LT(down.rmse, 1e-9) << "the model that generated the record must predict it";
  ASSERT_TRUE(down.fit_fraction_is_defined);
  EXPECT_GT(down.fit_fraction, 0.999);
}

// A model with the wrong mass must be CAUGHT. A validation that cannot fail is
// not a validation, and the residual has to be visible in more than one way:
// large, biased, and correlated with itself.
TEST(Validate, AWrongModelIsDetectedAndTheResidualShowsItsShape) {
  Quadrotor truth = shipped();
  truth.mass.mass_kg = 2.1;
  truth.validate();
  const Record held_out = synthesise(truth, kValidation, 30.0);

  // Validate the SHIPPED model — 1.6 kg — against a record made by a 2.1 kg
  // aircraft.
  const auto result = validate_model(shipped(), held_out, request_for(shipped(), kEstimation));
  const auto& down = result.outputs.front();
  EXPECT_GT(down.rmse, 0.1) << "a 30 percent mass error must not pass unnoticed";
  EXPECT_GT(std::fabs(down.mean_error), 0.05)
      << "a systematic mass error must show as a bias, not as symmetric scatter";
  ASSERT_TRUE(down.autocorrelation_is_defined);
  EXPECT_GT(down.residual_lag_one_autocorrelation, 0.5)
      << "a residual left by a missing dynamic effect is correlated with itself; white noise "
         "would mean the model had captured the structure and only scatter remained";
  // Worse than predicting the channel's own mean is a finding, and it is
  // reported as a negative number rather than clamped at zero.
  EXPECT_TRUE(down.fit_fraction_is_defined);
}

// THE ONE THAT MATTERS. Running a model on the record it was fitted to is a
// legitimate diagnostic and is not validation. The result says so — a good
// estimation fit presented as held-out validation is the specific dishonesty
// this design exists to make awkward.
TEST(Validate, ReusingTheEstimationRecordIsLabelledNotHeldOut) {
  const Quadrotor truth = shipped();
  const Record estimation = synthesise(truth, kEstimation, 30.0);

  const auto result = validate_model(truth, estimation, request_for(truth, kEstimation));
  EXPECT_EQ(result.independence, Independence::NotHeldOut)
      << "the same record cannot be both what a model was fitted to and what it was validated on";
  EXPECT_NE(result.independence_basis.find("same bytes"), std::string::npos)
      << result.independence_basis;
  EXPECT_EQ(result.validation_record_sha256, kEstimation);
  EXPECT_EQ(result.estimation_record_sha256, kEstimation);
  // The numbers are still computed and still excellent — which is exactly why
  // the label has to be there.
  EXPECT_LT(result.outputs.front().rmse, 1e-9);
}

TEST(Validate, AChannelThatNeverMovesSupportsNoFitFraction) {
  const Quadrotor truth = shipped();
  Record flat = synthesise(truth, kValidation, 30.0);
  for (double& sample : flat.channels[0].samples) {
    sample = -50.0;
  }
  const auto result = validate_model(truth, flat, request_for(truth, kEstimation));
  const auto& down = result.outputs.front();
  EXPECT_FALSE(down.fit_fraction_is_defined)
      << "a channel with no variation gives a prediction nothing to explain";
  EXPECT_FALSE(down.undefined_reason.empty());
  // The error itself is still reported: the absence is of a RATIO, not of a
  // measurement.
  EXPECT_GT(down.rmse, 0.0);
}

TEST(Validate, AClaimWithoutAnEstimationIdentityIsRefused) {
  const Quadrotor truth = shipped();
  const Record record = synthesise(truth, kValidation, 30.0);
  auto request = request_for(truth, "");
  EXPECT_THROW((void)validate_model(truth, record, request), std::invalid_argument)
      << "a caller who cannot say which record trained the model cannot claim anything was "
         "held out from it";
}

// ===========================================================================
// THE INDEPENDENCE LADDER
// ===========================================================================
//
// These are the tests that the old `bool is_held_out` could not have. Each one
// is a case where two records have DIFFERENT digests and are NOT independent,
// or the same digest and are, and the digest comparison gets the answer wrong.
// A regression that collapsed the classification back to `a != b` fails every
// one of them.

namespace {

using galata::data::window_record;

// One import, cut two ways. This is the only shape galata can PROVE disjoint,
// and it is the shape a real estimation/validation split has.
Record whole_flight() {
  return synthesise(shipped(), kEstimation, 30.0);
}

ValidationRequest request_with(const Quadrotor& model,
                               const Record* estimation,
                               const std::string& digest,
                               bool declared) {
  ValidationRequest request = request_for(model, digest);
  request.estimation_record = estimation;
  request.caller_declares_independent = declared;
  return request;
}

}  // namespace

TEST(Independence, TwoWindowsOfOneImportOverDisjointIntervalsAreVerifiedDisjoint) {
  const Quadrotor truth = shipped();
  const Record flight = whole_flight();
  const Record estimation = window_record(flight, 0.0, 0.5);
  const Record validation = window_record(flight, 0.5, 1.01);

  const auto result =
      validate_model(truth, validation, request_with(truth, &estimation, "", false));
  EXPECT_EQ(result.independence, Independence::VerifiedDisjoint);
  EXPECT_NE(result.independence_basis.find("windows of one imported file"), std::string::npos)
      << result.independence_basis;
  EXPECT_NE(result.independence_basis.find("not by an inequality of hashes"), std::string::npos)
      << result.independence_basis;
  EXPECT_FALSE(result.intervals_overlap);
  EXPECT_EQ(result.shared_sample_count, 0);
  EXPECT_GT(result.shared_channel_count, 0);
  // The lineage is preserved on both sides, so a reader can check the verdict
  // rather than take it.
  ASSERT_TRUE(result.estimation_lineage_is_known);
  EXPECT_TRUE(result.estimation_lineage.is_window);
  EXPECT_TRUE(result.validation_lineage.is_window);
  EXPECT_EQ(result.estimation_lineage.source_sha256, result.validation_lineage.source_sha256);
}

// THE CASE A DIGEST COMPARISON CANNOT SEE. Two windows of one file whose
// intervals overlap hold the same seconds of the same run. Their digests are
// equal here, so the old rule happens to get this one right; the next test is
// the one it gets wrong.
TEST(Independence, OverlappingWindowsOfOneImportAreNotHeldOut) {
  const Quadrotor truth = shipped();
  const Record flight = whole_flight();
  const Record estimation = window_record(flight, 0.0, 0.6);
  const Record validation = window_record(flight, 0.4, 1.01);

  const auto result = validate_model(truth, validation, request_with(truth, &estimation, "", true));
  EXPECT_EQ(result.independence, Independence::NotHeldOut);
  EXPECT_TRUE(result.intervals_overlap);
  // The study declared independence and was overruled, and it is told so
  // without having to parse the sentence.
  EXPECT_TRUE(result.caller_declaration_was_contradicted);
  EXPECT_NE(result.independence_basis.find("intervals overlap"), std::string::npos)
      << result.independence_basis;
}

// THE CASE THE OLD RULE GOT WRONG. A segment copied from one file into another:
// different digests, shared observations. `a != b` called this held out.
TEST(Independence, SamplesCopiedIntoAFileWithADifferentDigestAreFoundAndRefuseTheClaim) {
  const Quadrotor truth = shipped();
  const Record flight = whole_flight();
  const Record estimation = window_record(flight, 0.0, 0.5);

  // A second "file" — its own digest, its own path — that happens to contain
  // the estimation window's own samples.
  Record reissued = estimation;
  reissued.source_sha256 = kValidation;
  reissued.source_path = "second-export.csv";
  reissued.is_window = false;

  const auto result = validate_model(truth, reissued, request_with(truth, &estimation, "", true));
  EXPECT_EQ(result.independence, Independence::NotHeldOut)
      << "unequal digests must not be allowed to launder shared observations";
  EXPECT_EQ(result.shared_sample_count, static_cast<int>(estimation.sample_count()));
  EXPECT_TRUE(result.caller_declaration_was_contradicted);
  EXPECT_NE(result.independence_basis.find("different digests"), std::string::npos)
      << result.independence_basis;
}

// Two genuinely different files. galata cannot prove this either way — exact
// comparison cannot see the same run resampled into a second file — so the
// caller's claim is recorded AS the caller's claim.
TEST(Independence, TwoDifferentFilesWithAClaimAreCallerDeclaredAndNotVerified) {
  const Quadrotor truth = shipped();
  const Record estimation = synthesise(truth, kEstimation, 30.0);
  Record other = synthesise(truth, kValidation, 12.0);
  other.source_path = "second-flight.csv";

  const auto result = validate_model(truth, other, request_with(truth, &estimation, "", true));
  EXPECT_EQ(result.independence, Independence::CallerDeclared);
  EXPECT_FALSE(result.caller_declaration_was_contradicted);
  EXPECT_NE(result.independence_basis.find("the study declared"), std::string::npos)
      << result.independence_basis;
  EXPECT_NE(result.independence_basis.find("not a proof"), std::string::npos)
      << result.independence_basis;
}

TEST(Independence, TwoDifferentFilesWithNoClaimAreUnknown) {
  const Quadrotor truth = shipped();
  const Record estimation = synthesise(truth, kEstimation, 30.0);
  Record other = synthesise(truth, kValidation, 12.0);

  const auto result = validate_model(truth, other, request_with(truth, &estimation, "", false));
  EXPECT_EQ(result.independence, Independence::Unknown);
  EXPECT_NE(result.independence_basis.find("data.window"), std::string::npos)
      << "the refusal should say how to make the split checkable: " << result.independence_basis;
}

// A digest alone, with no record to compare against, can establish nothing
// positive — and the two silences are told apart.
TEST(Independence, ADigestAloneYieldsUnknownOrCallerDeclaredAndNeverVerified) {
  const Quadrotor truth = shipped();
  const Record validation = synthesise(truth, kValidation, 12.0);

  const auto silent = validate_model(truth, validation, request_for(truth, kEstimation));
  EXPECT_EQ(silent.independence, Independence::Unknown);
  EXPECT_FALSE(silent.estimation_lineage_is_known);

  const auto claimed =
      validate_model(truth, validation, request_with(truth, nullptr, kEstimation, true));
  EXPECT_EQ(claimed.independence, Independence::CallerDeclared);
  EXPECT_NE(claimed.independence_basis.find("was NOT treated as evidence"), std::string::npos)
      << claimed.independence_basis;
}

// Two identities that disagree cannot both be the training data, and choosing
// between them is not this routine's to do.
TEST(Independence, ADeclaredDigestThatContradictsTheSuppliedRecordIsRefused) {
  const Quadrotor truth = shipped();
  const Record estimation = synthesise(truth, kEstimation, 30.0);
  const Record validation = synthesise(truth, kValidation, 12.0);

  EXPECT_THROW(
      (void)validate_model(truth, validation, request_with(truth, &estimation, kValidation, false)),
      std::invalid_argument);
  // Agreeing is fine.
  EXPECT_NO_THROW(
      (void)validate_model(truth, validation, request_with(truth, &estimation, kEstimation, true)));
}

TEST(Independence, NeitherARecordNorADigestIsRefused) {
  const Quadrotor truth = shipped();
  const Record validation = synthesise(truth, kValidation, 12.0);
  EXPECT_THROW((void)validate_model(truth, validation, request_with(truth, nullptr, "", true)),
               std::invalid_argument);
}
