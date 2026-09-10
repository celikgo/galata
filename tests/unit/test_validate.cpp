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
  // digest: this is the held-out case.
  const Record held_out = synthesise(truth, kValidation, 30.0);
  const auto result = validate_model(truth, held_out, request_for(truth, kEstimation));

  EXPECT_TRUE(result.is_held_out);
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
// legitimate diagnostic and is not validation. The digests decide, and the
// result says so — a good estimation fit presented as held-out validation is
// the specific dishonesty this design exists to make awkward.
TEST(Validate, ReusingTheEstimationRecordIsLabelledNotHeldOut) {
  const Quadrotor truth = shipped();
  const Record estimation = synthesise(truth, kEstimation, 30.0);

  const auto result = validate_model(truth, estimation, request_for(truth, kEstimation));
  EXPECT_FALSE(result.is_held_out)
      << "the same record cannot be both what a model was fitted to and what it was validated on";
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
