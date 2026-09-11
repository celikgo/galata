// SPDX-License-Identifier: Apache-2.0
//
// `sim.linear` driven by a declared input history, through the study schema,
// against its frozen contract.
//
// The references are not the routine under test. A double integrator with
// output feedthrough, y = x + 0.5 u, whose response to a held or linearly
// varying input is a polynomial of degree at most three, which RK4 integrates
// exactly — so the tolerance is round-off, and an event handled one step early
// or late shows up at the size of a step. And the constant-input run itself,
// which a history that never changes must reproduce bit for bit, with the CSV a
// constant run always wrote left exactly as it was.
//
// Written from the capability schema and the headers, not from the
// implementation (docs/TESTING.md).

#include "galata/model/linear_system.hpp"
#include "galata/pipeline/pipeline.hpp"

#include "integration_config.hpp"
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {
namespace fs = std::filesystem;
using namespace galata::pipeline;

class LinearHistoryWorkflow : public ::testing::Test {
 protected:
  fs::path root;
  fs::path output;

  void SetUp() override {
    static std::atomic<unsigned> sequence{0};
    const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
    root = fs::temp_directory_path()
           / ("galata-linear-history-" + std::to_string(tick) + "-"
              + std::to_string(sequence.fetch_add(1)));
    ASSERT_TRUE(fs::create_directory(root));
    output = root / "output";
    galata::model::LinearSystem system;
    system.a = Eigen::MatrixXd::Zero(2, 2);
    system.a(0, 1) = 1.0;
    system.b = Eigen::MatrixXd::Zero(2, 1);
    system.b(1, 0) = 1.0;
    system.c = Eigen::MatrixXd::Zero(1, 2);
    system.c(0, 0) = 1.0;
    system.d = Eigen::MatrixXd::Constant(1, 1, 0.5);
    system.state_names = {"position_m", "velocity_m_s"};
    system.input_names = {"acceleration_m_s2"};
    system.output_names = {"blended"};
    system.description = "Double integrator with feedthrough";
    system.citation = "Analytic test plant";
    system.units = "SI";
    std::ofstream(root / "plant.yaml") << galata::model::serialize_linear_system(system);
  }

  void TearDown() override {
    std::error_code error;
    fs::remove_all(root, error);
  }

  RunResult run(const std::string& simulate_input, const std::string& csv = "run.csv") {
    std::ofstream(root / "study.yaml")
        << "version: 1\nstages:\n"
           "  - id: plant\n    capability: model.linear.statespace\n    input: {path: plant.yaml}\n"
           "  - id: response\n    capability: sim.linear\n    input:\n"
           "      system: {from: plant}\n"
        << simulate_input
        << "  - id: csv\n    capability: report.csv\n    input: {trajectory: {from: response}, "
           "path: "
        << csv << "}\n";
    return run_pipeline(load_pipeline((root / "study.yaml").string()),
                        builtin_registry(),
                        root.string(),
                        output.string(),
                        nullptr,
                        RunOptions{.overwrite = true, .write_manifest = false});
  }

  std::vector<std::vector<std::string>> csv(const std::string& name) {
    std::ifstream file(output / name);
    std::vector<std::vector<std::string>> rows;
    for (std::string line; std::getline(file, line);) {
      std::vector<std::string> cells;
      std::stringstream stream(line);
      for (std::string cell; std::getline(stream, cell, ',');) {
        cells.push_back(cell);
      }
      rows.push_back(cells);
    }
    return rows;
  }

  void expect_refusal(const std::string& simulate_input, const std::string& expected) {
    try {
      (void)run(simulate_input);
      ADD_FAILURE() << "ran, and should have been refused:\n" << simulate_input;
    } catch (const std::runtime_error& error) {
      EXPECT_NE(std::string(error.what()).find(expected), std::string::npos)
          << "refused, but the message does not say '" << expected << "': " << error.what();
    }
  }
};

constexpr const char* kZeroOrder =
    "      step_s: 0.01\n      steps: 100\n      initial_state: [0.1, -0.2]\n"
    "      input_schedule:\n"
    "        hold: zero_order\n        extrapolation: hold\n        samples:\n"
    "          - {time_s: 0.0, values: [2.0]}\n"
    "          - {time_s: 0.25, values: [-1.0]}\n"
    "          - {time_s: 0.5, values: [0.0]}\n"
    "          - {time_s: 0.75, values: [3.0]}\n";

}  // namespace

// The states against the polynomial closed form at every sample; the output
// against y = x + 0.5 u(t) with the input IN FORCE at that sample; and the input
// column the CSV now carries because the input changes.
TEST_F(LinearHistoryWorkflow, AZeroOrderHistoryIsExactOnADoubleIntegratorThroughTheSchema) {
  (void)run(kZeroOrder);
  const auto rows = csv("run.csv");
  ASSERT_EQ(rows.size(), 102U) << "header plus 101 samples";
  EXPECT_EQ(rows[0].back(), "\"input:acceleration_m_s2\"");
  const auto held = [](double t) { return t < 0.25 ? 2.0 : t < 0.5 ? -1.0 : t < 0.75 ? 0.0 : 3.0; };
  double position = 0.1;
  double velocity = -0.2;
  double previous = 0.0;
  for (std::size_t k = 1; k < rows.size(); ++k) {
    const double t = std::stod(rows[k][0]);
    const double u = held(previous + 1e-12);
    const double d = t - previous;
    if (k > 1) {
      position += velocity * d + 0.5 * u * d * d;
      velocity += u * d;
    }
    previous = t;
    EXPECT_NEAR(std::stod(rows[k][1]), position, 1e-12) << "t = " << t;
    EXPECT_NEAR(std::stod(rows[k][2]), velocity, 1e-12) << "t = " << t;
    const double in_force = held(t + 1e-12);
    EXPECT_EQ(std::stod(rows[k][4]), in_force) << "t = " << t;
    EXPECT_NEAR(std::stod(rows[k][3]), position + 0.5 * in_force, 1e-12)
        << "the output must use the input in force at t = " << t;
  }
}

// COMPATIBILITY. A constant run is unchanged — the same states and the same CSV
// header it always wrote — and a history that holds that constant throughout
// reproduces it bit for bit.
TEST_F(LinearHistoryWorkflow, AConstantRunIsUnchangedAndAOneValueHistoryReproducesItBitForBit) {
  const std::string common =
      "      step_s: 0.01\n      steps: 50\n      initial_state: [0.3, 0.1]\n";
  const RunResult constant = run(common + "      constant_input: [0.7]\n", "constant.csv");
  const RunResult history = run(common
                                    + "      input_schedule:\n        hold: zero_order\n"
                                      "        extrapolation: hold\n        samples:\n"
                                      "          - {time_s: 0.0, values: [0.7]}\n",
                                "history.csv");
  const auto constant_rows = csv("constant.csv");
  const auto history_rows = csv("history.csv");
  EXPECT_EQ(constant_rows[0],
            (std::vector<std::string>{
                "time_s", "\"state:position_m\"", "\"state:velocity_m_s\"", "\"output:blended\""}))
      << "a constant run's CSV header must be the one it always was";
  ASSERT_EQ(constant_rows.size(), history_rows.size());
  for (std::size_t k = 1; k < constant_rows.size(); ++k) {
    for (std::size_t j = 0; j < constant_rows[k].size(); ++j) {
      EXPECT_EQ(constant_rows[k][j], history_rows[k][j]) << "sample " << k << ", column " << j;
    }
  }
}

TEST_F(LinearHistoryWorkflow, WhatAHistoryCannotDefineIsRefusedByName) {
  const std::string head = "      step_s: 0.01\n      steps: 100\n";
  const std::string samples =
      "        samples:\n          - {time_s: 0.0, values: [1.0]}\n"
      "          - {time_s: 0.305, values: [2.0]}\n";
  expect_refusal(head + "      constant_input: [1.0]\n      input_schedule:\n"
                        "        hold: zero_order\n        extrapolation: hold\n"
                        "        samples:\n          - {time_s: 0.0, values: [1.0]}\n",
                 "not both");
  expect_refusal(head + "      input_schedule:\n        hold: zero_order\n"
                        "        extrapolation: hold\n"
                        + samples,
                 "not a whole number");
  expect_refusal(head + "      input_schedule:\n        hold: linear\n        extrapolation: refuse\n"
                        "        samples:\n          - {time_s: 0.0, values: [1.0]}\n"
                        "          - {time_s: 0.5, values: [2.0]}\n",
                 "outside the declared span");
  expect_refusal(head + "      input_schedule:\n        extrapolation: hold\n"
                        "        samples:\n          - {time_s: 0.0, values: [1.0]}\n",
                 "hold");
  expect_refusal(head + "      input_schedule:\n        hold: cubic\n        extrapolation: hold\n"
                        "        samples:\n          - {time_s: 0.0, values: [1.0]}\n",
                 "no default");
  expect_refusal(head + "      input_schedule:\n        hold: zero_order\n        extrapolation: hold\n"
                        "        samples:\n          - {time_s: 0.0, values: [1.0, 2.0]}\n",
                 "carries 2 channel(s)");
  expect_refusal(head + "      input_schedule:\n        hold: zero_order\n        extrapolation: hold\n"
                        "        interpolation: linear\n"
                        "        samples:\n          - {time_s: 0.0, values: [1.0]}\n",
                 "unknown key 'interpolation'");
  expect_refusal(head + "      input_schedule:\n        hold: zero_order\n        extrapolation: hold\n"
                        "        samples:\n          - {time_s: 0.0, values: [1.0], unit: m_s2}\n",
                 "unknown key 'unit'");
}
