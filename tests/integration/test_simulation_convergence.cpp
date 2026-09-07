// SPDX-License-Identifier: Apache-2.0
//
// Public workflow contracts: a smooth local nonlinear trajectory must converge
// under step halving and approach its full linearisation as disturbances shrink.
// References: Stevens, Lewis & Johnson, Aircraft Control and Simulation,
// Wiley, 2016, ch. 3; Hairer, Norsett & Wanner, Solving Ordinary Differential
// Equations I, Springer, 1993. The NT-33A coefficient provenance remains NASA
// CR-2144; these tests establish numerical consistency, not hardware validation.
#include "galata/linearize/finite_difference.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/pipeline.hpp"
#include "galata/sim/linear.hpp"
#include "galata/sim/nonlinear.hpp"
#include "galata/synth/control.hpp"

#include "integration_config.hpp"
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace {

struct Run {
  galata::pipeline::TrimArtifact trim;
  galata::synth::LqrDesign law;
  galata::sim::NonlinearResult response;
  std::array<double, galata::model::Controls::kSize> actuator_lags;
};

Run run_design(double step, int steps, double pitch_perturbation, const std::string& suffix) {
  const auto base = std::filesystem::path(GALATA_EXAMPLES_DIR) / "nt33a-control-design";
  auto pipeline = galata::pipeline::load_pipeline((base / "study.yaml").string());
  const std::vector<std::string> needed = {
      "aircraft", "trim", "full_plant", "plant", "law", "nonlinear_response"};
  std::erase_if(pipeline.stages, [&needed](const auto& stage) {
    return std::find(needed.begin(), needed.end(), stage.id) == needed.end();
  });
  std::array<double, galata::model::Controls::kSize> lags{};
  for (auto& stage : pipeline.stages) {
    if (stage.id != "nonlinear_response") {
      continue;
    }
    auto input = stage.input->as_map();
    input["step_s"] = galata::pipeline::Value::number(step);
    input["steps"] = galata::pipeline::Value::number(steps);
    input["sample_stride"] = galata::pipeline::Value::number(steps);
    input["initial_perturbation"] = galata::pipeline::Value::map(
        {{"theta", galata::pipeline::Value::number(pitch_perturbation)}});
    const auto names = galata::linearize::control_names();
    for (std::size_t channel = 0; channel < names.size(); ++channel) {
      lags[channel] = input.at("actuators")->get(names[channel])->number_at("time_constant_s");
    }
    stage.input = galata::pipeline::Value::map(std::move(input));
  }
  const auto output =
      std::filesystem::path(GALATA_INTEGRATION_SCRATCH_DIR) / ("convergence-" + suffix);
  const auto run = galata::pipeline::run_pipeline(pipeline,
                                                  galata::pipeline::builtin_registry(),
                                                  base.string(),
                                                  output.string(),
                                                  nullptr,
                                                  galata::pipeline::RunOptions{.overwrite = true});
  const auto* trim = run.find("trim");
  const auto* law = run.find("law");
  const auto* response = run.find("nonlinear_response");
  if (!trim || !law || !response) {
    throw std::runtime_error("the control-design example is missing a required workflow stage");
  }
  return {trim->payload_as<galata::pipeline::TrimArtifact>("trim_point"),
          law->payload_as<galata::synth::LqrDesign>("control_law"),
          response->payload_as<galata::sim::NonlinearResult>("nonlinear_trajectory"),
          lags};
}

Eigen::Vector4d scaled_longitudinal_deviation(const Run& run) {
  const auto& state = run.response.samples.back().state;
  const auto& trim = run.trim.point;
  const double speed = trim.airspeed_m_s;
  const double chord = run.trim.aircraft.geometry.mean_aerodynamic_chord_m;
  Eigen::Vector4d deviation;
  deviation << (state.velocity_body_m_s.x() - trim.state.velocity_body_m_s.x()) / speed,
      (state.velocity_body_m_s.z() - trim.state.velocity_body_m_s.z()) / speed,
      (state.angular_rate_body_rad_s.y() - trim.state.angular_rate_body_rad_s.y()) * chord
          / (2.0 * speed),
      galata::core::euler_from_quaternion(state.attitude_body_to_ned).pitch_rad
          - trim.pitch_attitude_rad;
  return deviation;
}

::testing::AssertionResult smooth_complete_run(const Run& run) {
  if (!run.response.completed || run.response.samples.empty()
      || run.response.outside_envelope_encountered) {
    return ::testing::AssertionFailure() << run.response.termination_reason;
  }
  for (std::size_t channel = 0; channel < run.actuator_lags.size(); ++channel) {
    if (run.response.position_limited_steps[channel] != 0
        || run.response.rate_limited_steps[channel] != 0) {
      return ::testing::AssertionFailure() << "actuator " << channel << " hit a limit";
    }
  }
  return ::testing::AssertionSuccess();
}

TEST(SimulationConvergence, NonlinearPipelineConvergesUnderStepHalving) {
  const auto coarse = run_design(0.01, 100, 0.001, "coarse");
  const auto medium = run_design(0.005, 200, 0.001, "medium");
  const auto fine = run_design(0.0025, 400, 0.001, "fine");
  ASSERT_TRUE(smooth_complete_run(coarse));
  ASSERT_TRUE(smooth_complete_run(medium));
  ASSERT_TRUE(smooth_complete_run(fine));
  // Dimensionless velocity departures, q*c/(2V), and pitch: avoid a norm
  // whose answer changes merely because one physical state uses different units.
  const double first =
      (scaled_longitudinal_deviation(coarse) - scaled_longitudinal_deviation(medium)).norm();
  const double second =
      (scaled_longitudinal_deviation(medium) - scaled_longitudinal_deviation(fine)).norm();
  ASSERT_GT(second, 0.0);
  const double observed_order = std::log2(first / second);
  // RK4 predicts fourth order while smooth. A band of one order covers the
  // higher-order remainder without permitting second-order or divergent runs.
  EXPECT_GT(observed_order, 3.0);
  EXPECT_LT(observed_order, 5.0);
}

TEST(SimulationConvergence, SmallDisturbancePipelineApproachesTheAugmentedLinearClosedLoop) {
  const auto larger = run_design(0.001, 1000, 0.001, "linear-limit-large");
  const auto smaller = run_design(0.001, 1000, 0.0005, "linear-limit-small");
  ASSERT_TRUE(smooth_complete_run(larger));
  ASSERT_TRUE(smooth_complete_run(smaller));
  const auto full =
      galata::linearize::linearize_finite_difference(larger.trim.aircraft, larger.trim.point);
  const Eigen::Index n = full.a.rows();
  const Eigen::Index m = full.b.cols();
  galata::model::LinearSystem augmented;
  augmented.a = Eigen::MatrixXd::Zero(n + m, n + m);
  augmented.a.topLeftCorner(n, n) = full.a;
  augmented.a.topRightCorner(n, m) = full.b;
  augmented.state_names = full.state_names;
  for (Eigen::Index channel = 0; channel < m; ++channel) {
    augmented.a(n + channel, n + channel) =
        -1.0 / larger.actuator_lags[static_cast<std::size_t>(channel)];
    augmented.state_names.push_back("actuator_"
                                    + full.input_names[static_cast<std::size_t>(channel)]);
  }
  for (std::size_t row = 0; row < larger.law.plant.input_names.size(); ++row) {
    const auto input = std::find(
        full.input_names.begin(), full.input_names.end(), larger.law.plant.input_names[row]);
    ASSERT_NE(input, full.input_names.end());
    const auto channel = static_cast<Eigen::Index>(input - full.input_names.begin());
    for (std::size_t column = 0; column < larger.law.plant.state_names.size(); ++column) {
      const auto state = std::find(
          full.state_names.begin(), full.state_names.end(), larger.law.plant.state_names[column]);
      ASSERT_NE(state, full.state_names.end());
      const auto state_index = static_cast<Eigen::Index>(state - full.state_names.begin());
      augmented.a(n + channel, state_index) =
          -larger.law.riccati.k(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column))
          / larger.actuator_lags[static_cast<std::size_t>(channel)];
    }
  }
  Eigen::VectorXd initial = Eigen::VectorXd::Zero(n + m);
  initial(galata::linearize::kPitch) = 1.0;
  const auto linear =
      galata::sim::simulate_linear(augmented, initial, Eigen::VectorXd{}, 0.001, 1000);
  const auto& final = linear.states.back();
  const double speed = larger.trim.point.airspeed_m_s;
  const double chord = larger.trim.aircraft.geometry.mean_aerodynamic_chord_m;
  Eigen::Vector4d reference;
  reference << final(galata::linearize::kVelocityU) / speed,
      final(galata::linearize::kVelocityW) / speed,
      final(galata::linearize::kRateQ) * chord / (2.0 * speed), final(galata::linearize::kPitch);
  const double large_error = (scaled_longitudinal_deviation(larger) / 0.001 - reference).norm();
  const double small_error = (scaled_longitudinal_deviation(smaller) / 0.0005 - reference).norm();
  // The first neglected term is O(epsilon^2), hence the normalized discrepancy
  // is O(epsilon): halving the perturbation must reduce it. Including all twelve
  // states and the actuator lags avoids treating omitted physics as nonlinearity.
  EXPECT_LT(small_error, 0.7 * large_error);
}

}  // namespace
