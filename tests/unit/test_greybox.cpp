// SPDX-License-Identifier: Apache-2.0
//
// Grey-box identification against the nonlinear plant.
//
// THE RECORD IS SYNTHESISED BY THE SAME PLANT THE FIT USES, from a parameter
// value chosen here. That makes these self-tests rather than validation, and it
// is deliberate: the question an estimator has to answer first is whether it
// returns the value it was given. A record from a real aircraft would answer a
// different and later question, and none exists.
//
// AND IT IS WHY THE UNIDENTIFIABLE CASE MATTERS MORE. A fit that recovers a
// parameter from its own simulator has shown it can walk downhill. A fit that
// REFUSES a parameter its data cannot see has shown it knows the difference
// between finishing and measuring.

#include "galata/core/constants.hpp"
#include "galata/core/state.hpp"
#include "galata/data/record.hpp"
#include "galata/identify/greybox.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/numerics/integrator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace {

using galata::data::Channel;
using galata::data::Record;
using galata::identify::fit_greybox;
using galata::identify::GreyboxRequest;
using galata::model::Quadrotor;

// Built here rather than loaded from models/: a unit test that reads a file is
// a unit test that fails for a reason unrelated to what it checks. The numbers
// are the shipped model's shape, and none of them is measured data.
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

// The nominal hover speed of the HOMOGENEOUS model, used as the starting state
// for every case. Taken from the unmodified model on purpose: a variant whose
// rotors differ has no single hover speed, and asking for one is refused —
// correctly — by `hover_speed_rad_s`. The state a record starts from is a
// declared initial condition, not a property of the parameter being fitted.
double nominal_hover_speed() {
  const Quadrotor homogeneous = shipped();
  return homogeneous.hover_speed_rad_s(galata::core::kStandardGravity);
}

Eigen::VectorXd hover_state(const Quadrotor& model) {
  Eigen::VectorXd x = Eigen::VectorXd::Zero(model.extended_state_size());
  x(galata::core::kQuaternionW) = 1.0;
  const double hover = nominal_hover_speed();
  for (int r = 0; r < model.rotor_count(); ++r) {
    x(galata::core::kStateSize + r) = hover;
  }
  return x;
}

// A record made by running the plant with a DECLARED excitation: the rotors are
// stepped away from hover so the vertical channel actually moves. A record in
// which nothing happens identifies nothing, which is the point of the
// unidentifiable case below.
Record synthesise(const Quadrotor& truth, double excitation, int samples = 60) {
  const double step_s = 0.002;
  const double period_s = 0.02;
  Record record;
  record.source_path = "synthetic";
  Channel down{"down_m", "d", "m", "ned", "", 1.0, 0.0, {}};
  std::vector<Channel> command;
  for (int r = 0; r < truth.rotor_count(); ++r) {
    command.push_back({"cmd_" + std::to_string(r), "c", "rad/s", "none", "", 1.0, 0.0, {}});
  }

  Eigen::VectorXd state = hover_state(truth);
  const double hover = nominal_hover_speed();
  Eigen::VectorXd u(truth.rotor_count());
  for (int k = 0; k < samples; ++k) {
    for (int r = 0; r < truth.rotor_count(); ++r) {
      u(r) = hover + excitation;
      command[static_cast<std::size_t>(r)].samples.push_back(u(r));
    }
    record.times_s.push_back(period_s * static_cast<double>(k));
    down.samples.push_back(state(galata::core::kPositionDown));
    const galata::numerics::DerivativeFunction derivative = [&](double, const Eigen::VectorXd& x) {
      return truth.derivative(x, u);
    };
    const galata::numerics::ProjectionFunction projection = [&](Eigen::VectorXd& x) {
      truth.project(x);
    };
    state =
        galata::numerics::integrate_fixed_step(derivative, state, 0.0, step_s, 10, 10, projection)
            .states.back();
  }
  record.channels.push_back(down);
  for (const Channel& c : command) {
    record.channels.push_back(c);
  }
  return record;
}

// Mass, from vertical motion. Chosen deliberately over a single rotor's thrust
// coefficient, and the reason is worth recording: with only `down_m` observed,
// the objective in ONE rotor's k_T is not unimodal. A weaker rotor also rolls
// the vehicle, so the vertical channel sees a curved trajectory whose
// projection happens to pass near the truth from the wrong side — measured, at
// k_T = 0.9e-5, 1.0e-5, 1.14e-5(truth): objective 2.08, 29.85, 0. A descent
// method started below the bump walks away from the answer, correctly, because
// downhill is away from it.
//
// That is a statement about the EXPERIMENT, not about the estimator: a per-rotor
// coefficient needs attitude or rate channels observed, or an excitation that
// separates the rotors. Fitting it from vertical position alone is a badly posed
// question, and this test does not paper over that by pretending otherwise.
GreyboxRequest thrust_request(const Quadrotor& model, double initial) {
  GreyboxRequest request;
  request.parameters = {{"mass.mass_kg", 0.8, 3.0, initial}};
  request.outputs = {{"down_m", "p_d", 1.0}};
  for (int r = 0; r < model.rotor_count(); ++r) {
    request.command_channels.push_back("cmd_" + std::to_string(r));
  }
  request.initial_extended_state = hover_state(model);
  request.step_s = 0.002;
  request.iterations = 30;
  return request;
}

}  // namespace

TEST(Greybox, RecoversAKnownParameterFromAnExcitedRecord) {
  Quadrotor truth = shipped();
  const double actual = 1.82;
  truth.mass.mass_kg = actual;
  truth.validate();
  const Record record = synthesise(truth, 20.0);

  // Start deliberately away from the answer.
  const auto result = fit_greybox(shipped(), record, thrust_request(shipped(), 1.55));
  ASSERT_EQ(result.value.size(), 1);
  EXPECT_TRUE(result.optimiser_finished);
  EXPECT_NEAR(result.value(0), actual, 1e-3)
      << "the fit did not recover the coefficient the record was generated from";
  EXPECT_FALSE(result.at_bound.front()) << "an estimate resting on a bound is not an interior one";
  EXPECT_EQ(result.iterations, 30) << "the iteration count is declared, not discovered";
}

// THE CASE THAT MATTERS. A fit that recovers a parameter from its own simulator
// has shown it can walk downhill. A fit that REFUSES a parameter its data cannot
// see has shown it knows the difference between finishing and measuring.
//
// The record here is a purely VERTICAL manoeuvre: four equal rotors stepped
// together, no lateral motion at any instant. The x-axis quadratic drag
// coefficient acts on body-x velocity, which is zero throughout, so it changes
// the prediction by exactly nothing. The optimiser still finishes — it always
// does, it runs a declared count — and its answer is wherever it happened to
// stop, which is not an estimate of anything.
TEST(Greybox, AParameterTheRecordCannotSeeIsRefusedWithTheReason) {
  const Quadrotor truth = shipped();
  const Record vertical = synthesise(truth, 20.0);

  auto request = thrust_request(shipped(), 1.6);
  request.parameters = {{"drag.quadratic_n_s2_m2[0]", 0.001, 0.5, 0.025}};

  try {
    (void)fit_greybox(shipped(), vertical, request);
    FAIL() << "a parameter the data does not constrain must not be reported";
  } catch (const std::invalid_argument& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("does not constrain"), std::string::npos) << message;
    // The distinction is the point: the optimiser finishing is not the same as
    // the parameter being measured, and the message has to say so.
    EXPECT_NE(message.find("optimiser finished"), std::string::npos)
        << "the refusal must separate finishing from measuring: " << message;
    EXPECT_NE(message.find("Excite"), std::string::npos)
        << "the refusal must say what to do about it: " << message;
  }
}

TEST(Greybox, WhatTheModelAndRecordDoNotHaveIsRefused) {
  const Quadrotor model = shipped();
  const Record record = synthesise(model, 20.0);

  auto request = thrust_request(model, 1.6);
  request.parameters.front().path = "rotors[0].no_such_parameter";
  EXPECT_THROW((void)fit_greybox(model, record, request), std::invalid_argument)
      << "a parameter silently not fitted is a run reporting success having estimated less";

  request = thrust_request(model, 1.6);
  request.parameters.front().initial = 99.0;  // outside its own bounds
  EXPECT_THROW((void)fit_greybox(model, record, request), std::invalid_argument);

  request = thrust_request(model, 1.6);
  request.parameters.front().lower = 3.0;
  request.parameters.front().upper = 1.0;
  EXPECT_THROW((void)fit_greybox(model, record, request), std::invalid_argument);

  request = thrust_request(model, 1.6);
  request.outputs.front().channel = "no_such_channel";
  EXPECT_THROW((void)fit_greybox(model, record, request), std::invalid_argument);

  // An output with no scale: channels in different units would be added as
  // though a metre and a radian per second were the same size.
  request = thrust_request(model, 1.6);
  request.outputs.front().scale = 0.0;
  EXPECT_THROW((void)fit_greybox(model, record, request), std::invalid_argument);
}

// Bounds hold at every trial point, not only at the end: an optimiser that
// walked through an invalid model on its way somewhere evaluated an objective
// that means nothing.
TEST(Greybox, BoundsAreEnforcedThroughoutAndAnEstimateOnOneIsReportedAsSuch) {
  Quadrotor truth = shipped();
  truth.mass.mass_kg = 2.4;
  truth.validate();
  const Record record = synthesise(truth, 20.0);

  auto request = thrust_request(shipped(), 1.6);
  // A ceiling below the true value: the fit must stop at the bound and say so
  // rather than reporting an interior estimate it never reached.
  request.parameters.front().upper = 2.0;
  const auto result = fit_greybox(shipped(), record, request);
  EXPECT_LE(result.value(0), 2.0 + 1e-12) << "a bound must not be crossed";
  EXPECT_TRUE(result.at_bound.front())
      << "an estimate resting on its bound must be reported as resting on it";
}
