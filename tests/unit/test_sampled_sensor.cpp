// SPDX-License-Identifier: Apache-2.0
// Direct contracts for the reusable sampled-loop and deterministic sensor
// primitives. The helicopter integration tests exercise the pipeline adapter;
// these tests keep the timing and stream guarantees local and cheap.

#include "galata/sim/sampled_loop.hpp"
#include "galata/sim/sensor.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

TEST(DeterministicSensor, LatencyDropoutAndQuantisationAreDeclared) {
  galata::sim::SensorConfiguration configuration;
  configuration.name = "test";
  configuration.channel_names = {"x"};
  configuration.seed = 7;
  configuration.sample_period_s = 0.1;
  configuration.latency_s = 0.1;
  configuration.bias = Eigen::VectorXd::Constant(1, 0.23);
  configuration.white_noise_stddev = Eigen::VectorXd::Zero(1);
  configuration.quantization_step = 0.1;
  configuration.dropout_samples = {1};
  configuration.dropout_policy = "unavailable";

  galata::sim::DeterministicSensor sensor(configuration, 0.1);
  const Eigen::VectorXd truth = Eigen::VectorXd::Constant(1, 1.01);
  const auto initial = sensor.sample_if_due(0, 0.0, truth);
  EXPECT_FALSE(initial.available);
  const auto delivered = sensor.sample_if_due(1, 0.1, truth);
  ASSERT_TRUE(delivered.available);
  EXPECT_DOUBLE_EQ(delivered.values(0), 1.2);
  const auto dropped = sensor.sample_if_due(2, 0.2, truth);
  EXPECT_FALSE(dropped.available);
  EXPECT_TRUE(dropped.stale);
  EXPECT_DOUBLE_EQ(dropped.values(0), 1.2);
  ASSERT_EQ(sensor.stream_ids().size(), 1U);
  EXPECT_EQ(sensor.stream_ids().front(), "test/x");
}

TEST(SampledLoop, AppliesBoundaryTickSaturationDelayAndHoldInOrder) {
  galata::sim::SampledLoopOptions options;
  options.step_s = 0.1;
  options.controller_period_s = 0.2;
  options.steps = 4;
  options.delay_periods = 1;
  options.initial_state = Eigen::VectorXd::Zero(1);
  options.trim_controls = Eigen::VectorXd::Zero(1);
  std::vector<std::string> order;
  std::vector<double> applied;
  options.on_boundary = [&](int step, double, Eigen::VectorXd&) {
    order.push_back("boundary" + std::to_string(step));
  };
  options.on_tick = [&](int tick, double, const Eigen::VectorXd&) {
    galata::sim::SampledTick result;
    result.requested_controls = Eigen::VectorXd::Constant(1, tick + 1.0);
    order.push_back("tick" + std::to_string(tick));
    return result;
  };
  options.saturate = [&](const Eigen::VectorXd& requested) {
    order.push_back("saturate");
    return requested;
  };
  options.on_command_applied = [&](const Eigen::VectorXd& command) {
    order.push_back("applied");
    applied.push_back(command(0));
  };
  options.derivative = [&](double, const Eigen::VectorXd&, const Eigen::VectorXd& command) {
    return command;
  };

  const auto result = galata::sim::run_sampled_loop(options);
  ASSERT_EQ(result.ticks.size(), 2U);
  ASSERT_EQ(applied.size(), 2U);
  EXPECT_DOUBLE_EQ(applied[0], 0.0);
  EXPECT_DOUBLE_EQ(applied[1], 1.0);
  EXPECT_DOUBLE_EQ(result.integration.trajectory.states.back()(0), 0.2);
  ASSERT_EQ(order.front(), "boundary0");
  ASSERT_EQ(order[1], "tick0");
  ASSERT_EQ(order[2], "saturate");
  ASSERT_EQ(order[3], "applied");
  EXPECT_EQ(order.back(), "boundary4");
}
