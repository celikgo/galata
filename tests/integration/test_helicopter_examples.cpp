// SPDX-License-Identifier: Apache-2.0
//
// Every shipped helicopter example must run, and must produce what its own
// README says it produces.
//
// These are the tests that stop the READMEs becoming fiction. Each number a
// helicopter README quotes is asserted here, so a change to the rotor, the trim
// or the model file fails this suite and the README gets corrected rather than
// silently becoming wrong.

#include "galata/analyze/modes.hpp"
#include "galata/core/state.hpp"
#include "galata/model/linear_system.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/pipeline.hpp"
#include "galata/pipeline/registry.hpp"

#include "integration_config.hpp"
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

galata::pipeline::RunResult run_heli_example(const std::string& example) {
  const std::filesystem::path directory = std::filesystem::path(GALATA_EXAMPLES_DIR) / example;
  const std::filesystem::path output =
      std::filesystem::path(GALATA_INTEGRATION_SCRATCH_DIR) / example;
  std::filesystem::create_directories(output);
  const galata::pipeline::Pipeline pipeline =
      galata::pipeline::load_pipeline((directory / "study.yaml").string());
  return galata::pipeline::run_pipeline(pipeline,
                                        galata::pipeline::builtin_registry(),
                                        directory.string(),
                                        output.string(),
                                        nullptr,
                                        galata::pipeline::RunOptions{.overwrite = true});
}

std::string read_text(const std::filesystem::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot open " + path.string());
  }
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

galata::pipeline::RunResult run_heli_document(const std::string& test_name, std::string document) {
  const std::filesystem::path output =
      std::filesystem::path(GALATA_INTEGRATION_SCRATCH_DIR) / test_name;
  std::filesystem::create_directories(output);
  const std::string relative_model = "../../models/souxmar-heli/souxmar-heli.yaml";
  const std::string absolute_model =
      (std::filesystem::path(GALATA_MODELS_DIR) / "souxmar-heli/souxmar-heli.yaml").string();
  std::size_t at = 0;
  while ((at = document.find(relative_model, at)) != std::string::npos) {
    document.replace(at, relative_model.size(), absolute_model);
    at += absolute_model.size();
  }
  const auto pipeline = galata::pipeline::parse_pipeline(document);
  return galata::pipeline::run_pipeline(pipeline,
                                        galata::pipeline::builtin_registry(),
                                        output.string(),
                                        output.string(),
                                        nullptr,
                                        galata::pipeline::RunOptions{.overwrite = true});
}

const galata::pipeline::Artifact& stage_named(const galata::pipeline::RunResult& result,
                                              const std::string& id) {
  const galata::pipeline::Artifact* found = result.find(id);
  if (found == nullptr) {
    throw std::runtime_error("no stage named '" + id + "' in this run");
  }
  return *found;
}

}  // namespace

// ---------------------------------------------------------------------------
// heli-hover-trim
// ---------------------------------------------------------------------------

TEST(ExampleHeliHoverTrim, RunsEndToEnd) {
  const auto result = run_heli_example("heli-hover-trim");
  ASSERT_EQ(result.stages.size(), 5U);
  EXPECT_EQ(result.stages[0].capability, "model.helicopter");
  EXPECT_EQ(result.stages[1].capability, "trim.helicopter");
  EXPECT_EQ(result.stages[2].capability, "linearize.vehicle");
  EXPECT_EQ(result.stages[3].capability, "analyze.modes");
  EXPECT_EQ(result.stages[4].capability, "report.markdown");

  // Charter rule 9: every artefact carries the build that made it.
  for (const auto& stage : result.stages) {
    EXPECT_FALSE(stage.artifact.produced_by_capability.empty());
    EXPECT_NE(stage.artifact.produced_by_build.find("galata "), std::string::npos);
  }
}

TEST(ExampleHeliTailRotorFailure, HealthyAndFailedRunsAreBothRecorded) {
  const auto result = run_heli_example("heli-tail-rotor-failure");
  ASSERT_EQ(result.stages.size(), 7U);
  EXPECT_NE(stage_named(result, "healthy").summary.find("completed"), std::string::npos);
  const std::string failed = stage_named(result, "failed").summary;
  EXPECT_NE(failed.find("applied 1 scheduled failure event"), std::string::npos) << failed;
  EXPECT_NE(failed.find("tail_rotor fraction=0.000000"), std::string::npos) << failed;

  const std::filesystem::path events = std::filesystem::path(GALATA_INTEGRATION_SCRATCH_DIR)
                                       / "heli-tail-rotor-failure" / "failed-hover.csv.events.txt";
  ASSERT_TRUE(std::filesystem::exists(events)) << events;
  std::ifstream stream(events);
  const std::string contents((std::istreambuf_iterator<char>(stream)),
                             std::istreambuf_iterator<char>());
  EXPECT_NE(contents.find("t=1.000000 s: tail_rotor fraction=0.000000"), std::string::npos);
}

// ---------------------------------------------------------------------------
// Closed-loop and sensor evidence
// ---------------------------------------------------------------------------

TEST(ExampleHeliClosedLoop, AllRequestedWorkflowsProduceControllerEvidence) {
  struct Case {
    const char* example;
    const char* stage;
    const char* csv;
  };

  const std::vector<Case> cases = {
      {"heli-sas-design", "sas", "sas.csv.controller.csv"},
      {"heli-attitude-hold", "closed", "attitude-hold.csv.controller.csv"},
      {"heli-altitude-hold", "closed", "altitude-hold.csv.controller.csv"},
      {"heli-noisy-feedback", "noisy", "noisy-feedback.csv.controller.csv"},
      {"heli-actuator-jam", "jammed", "actuator-jam.csv.controller.csv"},
  };
  for (const auto& item : cases) {
    const auto result = run_heli_example(item.example);
    const std::string summary = stage_named(result, item.stage).summary;
    EXPECT_NE(summary.find("completed"), std::string::npos) << item.example << ": " << summary;
    EXPECT_EQ(summary.find("REFUSED"), std::string::npos) << item.example << ": " << summary;
    const auto path =
        std::filesystem::path(GALATA_INTEGRATION_SCRATCH_DIR) / item.example / item.csv;
    ASSERT_TRUE(std::filesystem::exists(path)) << path;
    const std::string controller = read_text(path);
    EXPECT_NE(controller.find("measurement_available"), std::string::npos) << path;
    EXPECT_NE(controller.find("requested_"), std::string::npos) << path;
  }
}

TEST(ExampleHeliClosedLoop, SensorRunIsBitStableAndRecordsItsProvenance) {
  const auto first = run_heli_example("heli-noisy-feedback");
  EXPECT_NE(stage_named(first, "noisy").summary.find("deterministic named sensor"),
            std::string::npos);
  const auto directory =
      std::filesystem::path(GALATA_INTEGRATION_SCRATCH_DIR) / "heli-noisy-feedback";
  const std::string before = read_text(directory / "noisy-feedback.csv.controller.csv");
  const std::string report = read_text(directory / "noisy-feedback.md");
  EXPECT_NE(report.find("sensor seed: 20260919"), std::string::npos);
  EXPECT_NE(report.find("sensor algorithm: mt19937_64_box_muller_v1"), std::string::npos);

  const auto second = run_heli_example("heli-noisy-feedback");
  EXPECT_NE(stage_named(second, "noisy").summary.find("completed"), std::string::npos);
  EXPECT_EQ(before, read_text(directory / "noisy-feedback.csv.controller.csv"));
}

TEST(ExampleHeliFailures, EngineLossAndActuatorJamHaveStableSidecars) {
  const auto engine = run_heli_example("heli-engine-degradation");
  const std::string engine_summary = stage_named(engine, "degraded").summary;
  EXPECT_NE(engine_summary.find("applied 2 scheduled failure event"), std::string::npos)
      << engine_summary;
  const auto engine_events = std::filesystem::path(GALATA_INTEGRATION_SCRATCH_DIR)
                             / "heli-engine-degradation/engine-degradation.csv.events.txt";
  ASSERT_TRUE(std::filesystem::exists(engine_events)) << engine_events;
  const std::string engine_log = read_text(engine_events);
  const auto degraded = engine_log.find("engine fraction=0.500000");
  const auto lost = engine_log.find("engine fraction=0.000000");
  EXPECT_NE(degraded, std::string::npos);
  EXPECT_NE(lost, std::string::npos);
  EXPECT_LT(degraded, lost);

  const auto jam = run_heli_example("heli-actuator-jam");
  EXPECT_NE(stage_named(jam, "jammed").summary.find("applied 1 scheduled failure event"),
            std::string::npos);
  const auto jam_events = std::filesystem::path(GALATA_INTEGRATION_SCRATCH_DIR)
                          / "heli-actuator-jam/actuator-jam.csv.events.txt";
  ASSERT_TRUE(std::filesystem::exists(jam_events)) << jam_events;
  EXPECT_NE(read_text(jam_events).find("actuator_jam lateral_cyclic_command_rad"),
            std::string::npos);
}

std::string failure_validation_study(const std::string& event) {
  return "version: 1\n"
         "stages:\n"
         "  - id: aircraft\n"
         "    capability: model.helicopter\n"
         "    input: {path: ../../models/souxmar-heli/souxmar-heli.yaml}\n"
         "  - id: hover\n"
         "    capability: trim.helicopter\n"
         "    input: {helicopter: {from: aircraft}, airspeed_m_s: 0, altitude_m: 100}\n"
         "  - id: failed\n"
         "    capability: sim.helicopter\n"
         "    input:\n"
         "      trim: {from: hover}\n"
         "      step_s: 0.002\n"
         "      steps: 500\n"
         "      failure_events:\n"
         "        - "
         + event + "\n";
}

TEST(ExampleHeliFailures, InvalidFailureInputsAreRefusedByName) {
  const std::vector<std::pair<std::string, std::string>> invalid = {
      {"{time_s: 0.5, component: engine, fraction: 1.5}", "fraction"},
      {"{time_s: 0.501, component: engine, fraction: 0.5}", "between integration steps"},
      {"{time_s: 0.5, component: engine, fraction: 0.5, unexpected: 1}", "unknown key"},
      {"{time_s: 0.5, component: actuator_jam, actuator: lateral_cyclic_command_rad, position_rad: "
       "4.0}",
       "outside the actuator's declared travel"},
  };
  for (std::size_t i = 0; i < invalid.size(); ++i) {
    try {
      (void)run_heli_document("heli-invalid-failure-" + std::to_string(i),
                              failure_validation_study(invalid[i].first));
      FAIL() << "invalid failure event was accepted: " << invalid[i].first;
    } catch (const std::exception& error) {
      EXPECT_NE(std::string(error.what()).find(invalid[i].second), std::string::npos)
          << error.what();
    }
  }
}

TEST(ExampleHeliHoverTrim, ProducesTheTrimItsReadmeClaims) {
  const auto result = run_heli_example("heli-hover-trim");
  const std::string summary = stage_named(result, "hover").summary;

  // The README says "about 3.5 degrees of right roll" and quotes a collective
  // near 15 degrees. Both are in the summary line, so both are asserted.
  EXPECT_NE(summary.find("collective 15.1"), std::string::npos) << summary;
  EXPECT_NE(summary.find("roll 3.5"), std::string::npos) << summary;

  // Nine unknowns, full rank, and a well-conditioned Jacobian. The README
  // explains why it is nine rather than six.
  EXPECT_NE(summary.find("Jacobian rank 9/9"), std::string::npos) << summary;

  // And the envelope warning the README tells the reader to expect: the rotor is
  // at zero altitude and this model has no ground effect.
  EXPECT_NE(summary.find("ENVELOPE"), std::string::npos) << summary;
  EXPECT_NE(summary.find("ground effect"), std::string::npos) << summary;
}

TEST(ExampleHeliHoverTrim, TheHoverIsUnstableAsItsReadmeSays) {
  const auto result = run_heli_example("heli-hover-trim");
  const auto& linear = stage_named(result, "linear");
  const auto& system = linear.payload_as<galata::model::LinearSystem>("linear_system");

  const auto roles = galata::analyze::StateRoles::from_names(system.state_names);
  ASSERT_TRUE(roles.has_rotorcraft())
      << "the helicopter linearisation must declare a rotor-speed role, or no rotorcraft label "
         "can be assigned";
  const auto modes = galata::analyze::analyze_modes(system.a, system.state_names, roles);

  // THE CLAIM THE README MAKES, ASSERTED. "The hover is unstable": there is an
  // oscillation with negative damping, a period of tens of seconds, doubling in
  // tens of seconds. A model that hovered stably would be wrong, and this is the
  // test that would catch it.
  bool found = false;
  for (const auto& mode : modes.modes) {
    if (mode.is_oscillatory && mode.eigenvalue.real() > 0.0) {
      found = true;
      EXPECT_LT(mode.damping_ratio, 0.0);
      // The README quotes about 0.155 rad/s and a 41-second period.
      EXPECT_NEAR(mode.natural_frequency_rad_s, 0.155, 0.03);
      EXPECT_GT(mode.time_to_double_amplitude_s, 10.0);
      EXPECT_LT(mode.time_to_double_amplitude_s, 60.0);
    }
  }
  EXPECT_TRUE(found) << "no unstable oscillation in the hover linearisation";
}

// ---------------------------------------------------------------------------
// heli-forward-flight-trim
// ---------------------------------------------------------------------------

TEST(ExampleHeliForwardFlightTrim, ProducesTheTableItsReadmeQuotes) {
  const auto hover = run_heli_example("heli-hover-trim");
  const auto cruise = run_heli_example("heli-forward-flight-trim");

  const std::string cruise_summary = stage_named(cruise, "cruise").summary;
  // The README's altitude-aware table: collective 12.45, cyclic +5.13, pedal
  // 3.63, pitch -0.58. The atmosphere is evaluated at the declared 500 m.
  EXPECT_NE(cruise_summary.find("collective 12.45"), std::string::npos) << cruise_summary;
  EXPECT_NE(cruise_summary.find("cyclic 5.13"), std::string::npos) << cruise_summary;
  EXPECT_NE(cruise_summary.find("pedal 3.63"), std::string::npos) << cruise_summary;
  EXPECT_NE(cruise_summary.find("pitch -0.58"), std::string::npos) << cruise_summary;
  EXPECT_NE(cruise_summary.find("atmosphere altitude 500.0 m, density 1.1673 kg/m^3"),
            std::string::npos)
      << cruise_summary;

  // EVERY DIRECTION IN THE README'S TABLE, asserted as a comparison rather than
  // as a pair of numbers, because the direction is the physics and the numbers
  // are the model.
  const std::string hover_summary = stage_named(hover, "hover").summary;
  const auto value_after = [](const std::string& text, const std::string& key) {
    const auto at = text.find(key);
    if (at == std::string::npos) {
      throw std::runtime_error("no '" + key + "' in: " + text);
    }
    return std::stod(text.substr(at + key.size()));
  };
  // Collective falls with airspeed: translational lift does part of the work.
  EXPECT_LT(value_after(cruise_summary, "collective "), value_after(hover_summary, "collective "));
  // Pedal falls: there is less main-rotor torque to oppose.
  EXPECT_LT(value_after(cruise_summary, "pedal "), value_after(hover_summary, "pedal "));
  // Power falls: 40 m/s is nearer the minimum-power speed than the hover.
  EXPECT_LT(value_after(cruise_summary, "power "), value_after(hover_summary, "power "));
}

TEST(ExampleHeliForwardFlightTrim, TheTruncationEstimateConfirmsForwardFlightIsSmooth) {
  const auto cruise = run_heli_example("heli-forward-flight-trim");
  const std::string summary = stage_named(cruise, "linear").summary;
  // The README quotes 2e-10 here against 3.3e-1 in the hover, as the other side
  // of the argument that the hover kink is real. A large truncation estimate in
  // forward flight would mean the smoothness claim is wrong.
  const auto at = summary.find("truncation ");
  ASSERT_NE(at, std::string::npos) << summary;
  const double truncation = std::stod(summary.substr(at + std::string("truncation ").size()));
  EXPECT_LT(truncation, 1.0e-6)
      << "forward flight should be smooth; a large truncation estimate here contradicts the "
         "hover-versus-cruise argument the READMEs make: "
      << summary;
}

// ---------------------------------------------------------------------------
// heli-control-power
// ---------------------------------------------------------------------------

TEST(ExampleHeliControlPower, EachStepMovesTheAxisItIsSupposedTo) {
  const auto result = run_heli_example("heli-control-power");
  for (const char* stage : {"collective_step", "cyclic_step", "pedal_step"}) {
    const std::string summary = stage_named(result, stage).summary;
    EXPECT_NE(summary.find("completed"), std::string::npos)
        << stage << " did not complete: " << summary;
    EXPECT_EQ(summary.find("REFUSED"), std::string::npos) << stage << ": " << summary;
  }

  // The collective step is the one the README says does three things at once:
  // climbs, droops the rotor, and yaws. Read from the CSV it writes.
  const std::filesystem::path csv = std::filesystem::path(GALATA_INTEGRATION_SCRATCH_DIR)
                                    / "heli-control-power" / "collective-step.csv";
  ASSERT_TRUE(std::filesystem::exists(csv)) << csv;
  std::ifstream file(csv);
  std::string header;
  std::getline(file, header);
  std::vector<std::string> columns;
  for (std::istringstream stream(header); std::getline(stream, header, ',');) {
    columns.push_back(header);
  }
  const auto column_of = [&columns](const std::string& name) {
    const auto it = std::find(columns.begin(), columns.end(), name);
    if (it == columns.end()) {
      throw std::runtime_error("no column '" + name + "' in the collective-step CSV");
    }
    return static_cast<std::size_t>(it - columns.begin());
  };
  const std::size_t altitude = column_of("output_altitude_m");
  const std::size_t speed = column_of("main_rotor_speed_rad_s");
  const std::size_t yaw_rate = column_of("yaw_rate_rad_s");

  std::vector<std::vector<double>> rows;
  for (std::string line; std::getline(file, line);) {
    std::vector<double> values;
    for (std::istringstream stream(line); std::getline(stream, line, ',');) {
      values.push_back(std::stod(line));
    }
    rows.push_back(std::move(values));
  }
  ASSERT_GT(rows.size(), 10U);

  // 1. It climbs.
  EXPECT_GT(rows.back()[altitude], rows.front()[altitude] + 5.0);
  // 2. The rotor droops before it recovers. The minimum is strictly below the
  //    reference, which is the whole point of the engine-torque state.
  double lowest = rows.front()[speed];
  for (const auto& row : rows) {
    lowest = std::min(lowest, row[speed]);
  }
  EXPECT_LT(lowest, rows.front()[speed] - 0.05)
      << "no rotor droop under a collective step; the governor lag is not reaching the rotor";
  // 3. And it yaws, because the extra torque is not yet opposed by pedal.
  EXPECT_GT(std::fabs(rows.back()[yaw_rate]), 0.05);
}

// ---------------------------------------------------------------------------
// heli-governor-droop
// ---------------------------------------------------------------------------

namespace {

struct DroopTrace {
  std::vector<double> time;
  std::vector<double> speed;
  std::vector<double> torque;
};

DroopTrace read_droop(const std::string& file) {
  const std::filesystem::path csv =
      std::filesystem::path(GALATA_INTEGRATION_SCRATCH_DIR) / "heli-governor-droop" / file;
  std::ifstream stream(csv);
  if (!stream) {
    throw std::runtime_error("cannot open " + csv.string());
  }
  std::string header;
  std::getline(stream, header);
  std::vector<std::string> columns;
  for (std::istringstream split(header); std::getline(split, header, ',');) {
    columns.push_back(header);
  }
  const auto index = [&columns](const std::string& name) {
    const auto it = std::find(columns.begin(), columns.end(), name);
    if (it == columns.end()) {
      throw std::runtime_error("no column '" + name + "'");
    }
    return static_cast<std::size_t>(it - columns.begin());
  };
  const std::size_t time = index("time_s");
  const std::size_t speed = index("main_rotor_speed_rad_s");
  const std::size_t torque = index("engine_torque_n_m");

  DroopTrace trace;
  for (std::string line; std::getline(stream, line);) {
    std::vector<double> values;
    for (std::istringstream split(line); std::getline(split, line, ',');) {
      values.push_back(std::stod(line));
    }
    trace.time.push_back(values[time]);
    trace.speed.push_back(values[speed]);
    trace.torque.push_back(values[torque]);
  }
  return trace;
}

}  // namespace

TEST(ExampleHeliGovernorDroop, OneDegreeDroopsAndRecoversAsItsReadmeTabulates) {
  const auto result = run_heli_example("heli-governor-droop");
  EXPECT_NE(stage_named(result, "droop_recovered").summary.find("completed"), std::string::npos);

  const auto trace = read_droop("droop-recovered.csv");
  ASSERT_GT(trace.speed.size(), 100U);
  const double reference = trace.speed.front();

  double worst = 0.0;
  double worst_time = 0.0;
  for (std::size_t i = 0; i < trace.speed.size(); ++i) {
    const double droop = (reference - trace.speed[i]) / reference;
    if (droop > worst) {
      worst = droop;
      worst_time = trace.time[i];
    }
  }
  // The README's first table: 0.87% worst, at about 0.5 s. A droop that vanished
  // would mean the engine-torque state had stopped mattering, which is exactly
  // the silence this example exists to break.
  EXPECT_NEAR(worst, 0.0087, 0.003) << "worst droop " << worst;
  EXPECT_GT(worst_time, 0.2);
  EXPECT_LT(worst_time, 1.5);

  // AND IT RECOVERS, to within a tenth of a percent.
  EXPECT_NEAR(trace.speed.back(), reference, 0.001 * reference);
  EXPECT_GT(trace.torque.back(), trace.torque.front());
}

TEST(ExampleHeliGovernorDroop, ThreeDegreesSaturatesTheDriveRatingAndCannotRecover) {
  const auto result = run_heli_example("heli-governor-droop");
  EXPECT_NE(stage_named(result, "droop_saturated").summary.find("completed"), std::string::npos);

  const auto trace = read_droop("droop-saturated.csv");
  ASSERT_GT(trace.speed.size(), 100U);
  const double reference = trace.speed.front();

  double worst = 0.0;
  for (const double speed : trace.speed) {
    worst = std::max(worst, (reference - speed) / reference);
  }
  // The README's second table: about 5.6% worst.
  EXPECT_GT(worst, 0.04) << "worst droop " << worst;
  EXPECT_LT(worst, 0.08) << "worst droop " << worst;

  // THE ENGINE TORQUE REACHES THE DRIVE RATING AND STAYS THERE. 20 461.5 N m is
  // derived from the design package's own 700 kW drive-input limit, NOT from the
  // two PW207D1 engines, which could supply more. PROVENANCE.md §2 records why
  // the drive rating is the binding one.
  const double rating = 20461.5;
  EXPECT_NEAR(trace.torque.back(), rating, 1.0)
      << "the engine torque should be pinned at the drive rating: " << trace.torque.back();

  // AND THE ROTOR DOES NOT RECOVER. It is still percent-level low at the end,
  // because the governor has no authority left. A test that expected recovery
  // here would be asserting a governor that can exceed its own drivetrain.
  EXPECT_LT(trace.speed.back(), reference - 0.01 * reference)
      << "the rotor recovered despite a saturated drivetrain, which would mean the torque limit "
         "is not binding: "
      << trace.speed.back();
}

// ---------------------------------------------------------------------------
// heli-nonlinear-vs-linear
// ---------------------------------------------------------------------------

TEST(ExampleHeliNonlinearVsLinear, CruiseIsSecondOrderAndHoverIsNot) {
  const auto result = run_heli_example("heli-nonlinear-vs-linear");

  const std::string cruise = stage_named(result, "cruise_agreement").summary;
  const std::string hover = stage_named(result, "hover_agreement").summary;

  // The README quotes the cruise orders converging on 2 and ending at 1.9998.
  EXPECT_NE(cruise.find("1.99"), std::string::npos)
      << "cruise should measure an order near 2: " << cruise;

  // And it quotes the hover orders degrading and going negative. Asserted as a
  // property rather than a string: the LAST hover order must be far below 2,
  // because the discrepancy has stopped falling.
  const auto orders_in = [](const std::string& text) {
    std::vector<double> out;
    const auto at = text.find("observed order");
    if (at == std::string::npos) {
      throw std::runtime_error("no orders in: " + text);
    }
    std::istringstream stream(text.substr(at + std::string("observed order").size()));
    for (std::string token; stream >> token;) {
      try {
        out.push_back(std::stod(token));
      } catch (const std::exception&) {
        break;  // reached the trailing prose
      }
    }
    return out;
  };
  const auto cruise_orders = orders_in(cruise);
  const auto hover_orders = orders_in(hover);
  ASSERT_GE(cruise_orders.size(), 2U);
  ASSERT_GE(hover_orders.size(), 2U);

  EXPECT_GT(cruise_orders.back(), 1.9) << "cruise order " << cruise_orders.back();
  EXPECT_LT(cruise_orders.back(), 2.1) << "cruise order " << cruise_orders.back();

  // TWO-SIDED, matching the unit-level gate: the hover order must stay well
  // below 2, because an order near 2 there would mean the advance ratio's kink
  // at V = 0 had been smoothed away — a better Jacobian of a worse model.
  EXPECT_LT(hover_orders.back(), 1.0)
      << "exact hover measured an order of " << hover_orders.back()
      << "; near 2 would mean |V| in the advance ratio has been smoothed";
}
