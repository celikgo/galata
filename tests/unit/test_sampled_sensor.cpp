// SPDX-License-Identifier: Apache-2.0
// Direct contracts for the reusable sampled-loop and deterministic sensor
// primitives. The helicopter integration tests exercise the pipeline adapter;
// these tests keep the timing and stream guarantees local and cheap.

#include "galata/sim/sampled_loop.hpp"
#include "galata/sim/sensor.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace {

std::uint64_t reference_splitmix64(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

std::uint64_t reference_hash_text(const std::string& text) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const char character : text) {
    hash ^= static_cast<std::uint64_t>(static_cast<unsigned char>(character));
    hash *= 1099511628211ULL;
  }
  return hash;
}

double reference_unit_open(std::mt19937_64& generator) {
  const std::uint64_t bits = generator() >> 11;
  return (static_cast<double>(bits) + 0.5) / 9007199254740992.0;
}

double reference_normal(std::uint64_t seed, const std::string& stream, int sample) {
  std::mt19937_64 generator(reference_splitmix64(seed ^ reference_hash_text(stream))
                            ^ reference_splitmix64(static_cast<std::uint64_t>(sample)));
  const double u = reference_unit_open(generator);
  const double v = reference_unit_open(generator);
  return std::sqrt(-2.0 * std::log(u)) * std::cos(6.2831853071795864769 * v);
}

}  // namespace

TEST(DeterministicSensor, IndependentReferenceStreamsAreStableAndClocked) {
  galata::sim::SensorConfiguration configuration;
  configuration.name = "reference";
  configuration.channel_names = {"x", "y"};
  configuration.seed = 123456;
  configuration.sample_period_s = 0.2;
  configuration.bias = Eigen::Vector2d(0.1, -0.2);
  configuration.white_noise_stddev = Eigen::Vector2d(0.5, 0.75);

  galata::sim::DeterministicSensor sensor(configuration, 0.1);
  const Eigen::Vector2d truth(1.0, 2.0);
  for (int sample = 0; sample < 4; ++sample) {
    const int integration_step = sample * 2;
    const auto reading =
        sensor.sample_if_due(integration_step, static_cast<double>(integration_step) * 0.1, truth);
    ASSERT_TRUE(reading.available);
    EXPECT_DOUBLE_EQ(reading.values(0),
                     truth(0) + configuration.bias(0)
                         + configuration.white_noise_stddev(0)
                               * reference_normal(configuration.seed, "reference/x", sample));
    EXPECT_DOUBLE_EQ(reading.values(1),
                     truth(1) + configuration.bias(1)
                         + configuration.white_noise_stddev(1)
                               * reference_normal(configuration.seed, "reference/y", sample));
    if (sample + 1 < 4) {
      const auto extra = sensor.sample_if_due(
          integration_step + 1, static_cast<double>(integration_step + 1) * 0.1, truth);
      EXPECT_EQ(extra.sample_index, sample);
      EXPECT_DOUBLE_EQ(extra.values(0), reading.values(0));
      EXPECT_DOUBLE_EQ(extra.values(1), reading.values(1));
    }
  }
}

TEST(DeterministicSensor, NoiseBudgetAndSeparateStreamsAreMeasured) {
  galata::sim::SensorConfiguration configuration;
  configuration.name = "statistics";
  configuration.channel_names = {"x"};
  configuration.seed = 77;
  configuration.sample_period_s = 0.01;
  configuration.white_noise_stddev = Eigen::VectorXd::Ones(1);
  galata::sim::DeterministicSensor sensor(configuration, 0.01);
  double mean = 0.0;
  double squared = 0.0;
  constexpr int kSamples = 4096;
  for (int sample = 0; sample < kSamples; ++sample) {
    const double value =
        sensor.sample_if_due(sample, sample * 0.01, Eigen::VectorXd::Zero(1)).values(0);
    mean += value;
    squared += value * value;
  }
  mean /= kSamples;
  const double variance = squared / kSamples - mean * mean;
  // A priori statistical budget for 4096 N(0,1) draws: the mean is allowed
  // 0.10 and the variance 20%; this checks the declared noise law, not a
  // regression value captured from the implementation.
  EXPECT_LT(std::fabs(mean), 0.10);
  EXPECT_GT(variance, 0.80);
  EXPECT_LT(variance, 1.20);
}

TEST(DeterministicSensor, DropoutHoldLastAndSaturationPoliciesAreObservable) {
  galata::sim::SensorConfiguration configuration;
  configuration.name = "policies";
  configuration.channel_names = {"x"};
  configuration.sample_period_s = 0.1;
  configuration.quantization_step = 0.5;
  configuration.saturation_min = -2.0;
  configuration.saturation_max = 2.0;
  configuration.dropout_samples = {1};
  configuration.dropout_policy = "hold_last";
  galata::sim::DeterministicSensor sensor(configuration, 0.1);
  const auto first = sensor.sample_if_due(0, 0.0, Eigen::VectorXd::Constant(1, 1.1));
  ASSERT_TRUE(first.available);
  EXPECT_DOUBLE_EQ(first.values(0), 1.0);
  const auto dropped = sensor.sample_if_due(1, 0.1, Eigen::VectorXd::Constant(1, 10.0));
  EXPECT_TRUE(dropped.available);
  EXPECT_TRUE(dropped.stale);
  EXPECT_DOUBLE_EQ(dropped.values(0), 1.0);
  const auto recovered = sensor.sample_if_due(2, 0.2, Eigen::VectorXd::Constant(1, 10.0));
  EXPECT_TRUE(recovered.available);
  EXPECT_FALSE(recovered.stale);
  EXPECT_DOUBLE_EQ(recovered.values(0), 2.0);
}

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
