// SPDX-License-Identifier: Apache-2.0
// Independent continuous scalar graph conformance: MODEL_CONFORMANCE.md.
// References: Hairer, Norsett and Wanner, Solving Ordinary Differential
// Equations I, 2nd revised ed., Springer, 1993; Butcher, Numerical Methods for
// Ordinary Differential Equations, 3rd ed., Wiley, 2016. Expected trajectories
// use closed-form ODE solutions and independently derived RK4 amplification.
// These synthetic cases establish no aircraft-model or qualification claim.
#include "galata/modeling/model.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <complex>
#include <cstdint>
#include <functional>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace galata::modeling;

Dimension dimension(int length = 0, int time = 0, int angle = 0) {
  return Dimension{length, 0, time, 0, 0, 0, 0, angle};
}

SignalType signal(int length = 0, int time = 0, Frame frame = Frame::None, int angle = 0) {
  return SignalType{dimension(length, time, angle), frame};
}

Eigen::VectorXd scalar(double value) {
  Eigen::VectorXd state(1);
  state(0) = value;
  return state;
}

Block& named(Model& model, const std::string& id) {
  const auto found = std::find_if(model.blocks.begin(),
                                  model.blocks.end(),
                                  [&id](const auto& block) { return block.id == id; });
  if (found == model.blocks.end()) {
    throw std::logic_error("invalid test fixture block ID");
  }
  return *found;
}

Model constant_output(double value = 1.0) {
  Model model;
  model.blocks = {{"constant", signal(), Constant{value}}, {"output", signal(), Output{}}};
  model.connections = {{"constant", "output", 0}};
  return model;
}

Model decay(double initial = 1.0) {
  Model model;
  model.blocks = {{"state", signal(1), Integrator{initial}},
                  {"rate", signal(1, -1), Gain{-1.0, dimension(0, -1)}},
                  {"output", signal(1), Output{}}};
  model.connections = {{"state", "rate", 0}, {"rate", "state", 0}, {"state", "output", 0}};
  return model;
}

Model affine() {
  auto model = decay(0.0);
  model.blocks.push_back({"command", signal(1, -1), Constant{1.0}});
  model.blocks.push_back({"sum", signal(1, -1), Sum{{1, 1}}});
  model.connections = {{"state", "rate", 0},
                       {"rate", "sum", 0},
                       {"command", "sum", 1},
                       {"sum", "state", 0},
                       {"state", "output", 0}};
  return model;
}

Model oscillator() {
  Model model;
  model.blocks = {{"position", signal(1), Integrator{1.0}},
                  {"velocity", signal(1, -1), Integrator{0.0}},
                  {"restoring", signal(1, -2), Gain{-1.0, dimension(0, -2)}},
                  {"out_position", signal(1), Output{}},
                  {"out_velocity", signal(1, -1), Output{}}};
  model.connections = {{"velocity", "position", 0},
                       {"position", "restoring", 0},
                       {"restoring", "velocity", 0},
                       {"position", "out_position", 0},
                       {"velocity", "out_velocity", 0}};
  return model;
}

void expect_error(const std::function<void()>& operation, ErrorCode expected) {
  try {
    operation();
    FAIL() << "operation accepted a case requiring refusal";
  } catch (const Error& error) {
    EXPECT_EQ(error.code(), expected) << error.what();
    EXPECT_FALSE(std::string(error.what()).empty());
  } catch (const std::exception& error) {
    FAIL() << "untyped failure: " << error.what();
  }
}

void expect_invalid_model(const Model& model, ErrorCode expected) {
  expect_error([&] { validate_model(model); }, expected);
  expect_error([&] { (void)compile_model(model); }, expected);
}

// B01: these fixtures contain only short scalar sums/products. Sixty-four
// rounded operations per block per step covers all four RK stages, state
// assembly and output evaluation; 32 more per step covers the independent
// polynomial reference and its scalar/complex recurrence. The supplied scale
// bounds the sum of absolute contributions after propagation. Using epsilon
// rather than epsilon/2 is conservative. This is not a general graph estimator.
double arithmetic_budget(int steps, std::size_t blocks, double scale = 2.0) {
  const double operations =
      (64.0 * static_cast<double>(blocks) + 32.0) * static_cast<double>(steps + 1);
  const double accumulated = operations * std::numeric_limits<double>::epsilon();
  return scale * accumulated / (1.0 - accumulated);
}

double decay_amplification(double h) {
  // Independent stability polynomial in Horner form, not the RK stage code.
  return 1.0 - h * (1.0 - h * (0.5 - h * (1.0 / 6.0 - h / 24.0)));
}

template <typename Value>
Value integer_power(Value base, int exponent) {
  Value result{1.0};
  for (int index = 0; index < exponent; ++index) {
    result *= base;
  }
  return result;
}

void expect_same_bits(const SimulationResult& first, const SimulationResult& second) {
  EXPECT_EQ(first.semantic_sha256, second.semantic_sha256);
  EXPECT_EQ(first.state_ids, second.state_ids);
  EXPECT_EQ(first.output_ids, second.output_ids);
  ASSERT_EQ(first.times_s.size(), second.times_s.size());
  ASSERT_EQ(first.states.size(), second.states.size());
  ASSERT_EQ(first.outputs.size(), second.outputs.size());
  for (std::size_t sample = 0; sample < first.times_s.size(); ++sample) {
    EXPECT_EQ(std::bit_cast<std::uint64_t>(first.times_s[sample]),
              std::bit_cast<std::uint64_t>(second.times_s[sample]));
    ASSERT_EQ(first.states[sample].size(), second.states[sample].size());
    ASSERT_EQ(first.outputs[sample].size(), second.outputs[sample].size());
    for (Eigen::Index entry = 0; entry < first.states[sample].size(); ++entry) {
      EXPECT_EQ(std::bit_cast<std::uint64_t>(first.states[sample](entry)),
                std::bit_cast<std::uint64_t>(second.states[sample](entry)));
    }
    for (Eigen::Index entry = 0; entry < first.outputs[sample].size(); ++entry) {
      EXPECT_EQ(std::bit_cast<std::uint64_t>(first.outputs[sample](entry)),
                std::bit_cast<std::uint64_t>(second.outputs[sample](entry)));
    }
  }
}

TEST(Modeling, StatelessDyadicArithmeticHasNoArtificialContinuousState) {
  // MC01: 2 * (3 - 1) = 4, with every arithmetic result exactly representable.
  Model model;
  model.blocks = {{"three", signal(1), Constant{3.0}},
                  {"one", signal(1), Constant{1.0}},
                  {"sum", signal(1), Sum{{1, -1}}},
                  {"double", signal(1), Gain{2.0, dimension()}},
                  {"output", signal(1), Output{}}};
  model.connections = {
      {"three", "sum", 0}, {"one", "sum", 1}, {"sum", "double", 0}, {"double", "output", 0}};
  const auto compiled = compile_model(model);
  EXPECT_TRUE(compiled.state_ids().empty());
  EXPECT_EQ(compiled.initial_state().size(), 0);
  const auto evaluated = compiled.evaluate(0.0, Eigen::VectorXd{});
  EXPECT_EQ(evaluated.derivatives.size(), 0);
  ASSERT_EQ(evaluated.outputs.size(), 1);
  EXPECT_EQ(evaluated.outputs(0), 4.0);
  const auto run = simulate(compiled, {.step_s = 0.125, .step_count = 3, .sample_stride = 2});
  EXPECT_EQ(run.times_s, (std::vector<double>{0.0, 0.25, 0.375}));
  for (std::size_t sample = 0; sample < run.times_s.size(); ++sample) {
    EXPECT_EQ(run.states[sample].size(), 0);
    EXPECT_EQ(run.outputs[sample](0), 4.0);
  }
}

TEST(Modeling, ConstantRateIntegralUsesElapsedTimeAndDeclaredInitialCondition) {
  // MC02: x(t) = 1/2 + 3(t-t0)/4 metres, independent of the integrator code.
  Model model;
  model.blocks = {{"rate", signal(1, -1), Constant{0.75}},
                  {"state", signal(1), Integrator{0.5}},
                  {"output", signal(1), Output{}}};
  model.connections = {{"rate", "state", 0}, {"state", "output", 0}};
  const auto compiled = compile_model(model);
  const SimulationOptions options{.initial_time_s = 4.0, .step_s = 0.125, .step_count = 16};
  const auto run = simulate(compiled, options);
  ASSERT_EQ(run.times_s.size(), 17U);
  for (std::size_t sample = 0; sample < run.times_s.size(); ++sample) {
    const double elapsed = static_cast<double>(sample) * options.step_s;
    EXPECT_EQ(run.times_s[sample], options.initial_time_s + elapsed);
    EXPECT_NEAR(run.states[sample](0),
                0.5 + 0.75 * elapsed,
                arithmetic_budget(options.step_count, model.blocks.size(), 4.0));
    EXPECT_EQ(run.outputs[sample](0), run.states[sample](0));
  }
}

TEST(Modeling, ExponentialFeedbackMatchesIndependentRk4PolynomialAndContinuousBounds) {
  // MC03/B02/B03/B04. h=1/8 has the exact rational R(-h)=86753/98304.
  const auto model = decay();
  const auto compiled = compile_model(model);
  const auto direct = compiled.evaluate(9.0, scalar(2.0));
  ASSERT_EQ(direct.derivatives.size(), 1);
  EXPECT_EQ(direct.derivatives(0), -2.0);
  EXPECT_EQ(direct.outputs(0), 2.0);
  const auto one = simulate(compiled, {.step_s = 0.125, .step_count = 1});
  EXPECT_NEAR(one.states.back()(0), 86753.0 / 98304.0, arithmetic_budget(1, model.blocks.size()));
  for (int refinement = 0; refinement < 3; ++refinement) {
    const int steps = 8 * (1 << refinement);
    const double h = 1.0 / static_cast<double>(steps);
    const auto run = simulate(compiled, {.step_s = h, .step_count = steps});
    const double discrete = integer_power(decay_amplification(h), steps);
    const double floating = arithmetic_budget(steps, model.blocks.size());
    const double truncation = static_cast<double>(steps) * std::pow(h, 5) / 120.0;
    // The B01 allowance also exceeds the small libm rounding allowance here;
    // the rational/polynomial check remains independent of exp's last bits.
    EXPECT_NEAR(run.states.back()(0), discrete, floating);
    EXPECT_NEAR(run.states.back()(0), std::exp(-1.0), truncation + floating);
    for (std::size_t sample = 0; sample < run.times_s.size(); ++sample) {
      EXPECT_EQ(run.outputs[sample](0), run.states[sample](0));
    }
  }
}

TEST(Modeling, AffineFeedbackReevaluatesBothCommandAndTemporaryStateAtEveryRkStage) {
  // MC04: xdot=1-x, x0=0 -> x(t)=1-exp(-t), x_N=1-R(-h)^N.
  const auto model = affine();
  const auto compiled = compile_model(model);
  const auto direct = compiled.evaluate(0.0, scalar(0.25));
  EXPECT_EQ(direct.derivatives(0), 0.75);
  EXPECT_EQ(direct.outputs(0), 0.25);
  constexpr int steps = 8;
  constexpr double h = 0.125;
  const auto run = simulate(compiled, {.step_s = h, .step_count = steps});
  const double floating = arithmetic_budget(steps, model.blocks.size(), 4.0);
  EXPECT_NEAR(run.states.back()(0), 1.0 - integer_power(86753.0 / 98304.0, steps), floating);
  EXPECT_NEAR(run.states.back()(0),
              1.0 - std::exp(-1.0),
              static_cast<double>(steps) * std::pow(h, 5) / 120.0 + floating);
}

TEST(Modeling, CoupledOscillatorUsesOneTemporaryStateForAllDerivatives) {
  // MC05: normalized q'=w, w'=-q. q_N=Re R(ih)^N; w_N=-Im R(ih)^N.
  // Position is scaled by 1 m, velocity by 1 m/s, time by 1 s before the norm.
  const auto model = oscillator();
  const auto compiled = compile_model(model);
  ASSERT_EQ(compiled.state_ids(), (std::vector<std::string>{"position", "velocity"}));
  ASSERT_EQ(compiled.output_ids(), (std::vector<std::string>{"out_position", "out_velocity"}));
  Eigen::VectorXd supplied(2);
  supplied << 2.0, 3.0;
  const auto evaluated = compiled.evaluate(7.0, supplied);
  EXPECT_EQ(evaluated.derivatives(0), 3.0);
  EXPECT_EQ(evaluated.derivatives(1), -2.0);
  EXPECT_EQ(evaluated.outputs(0), 2.0);
  EXPECT_EQ(evaluated.outputs(1), 3.0);
  for (int refinement = 0; refinement < 3; ++refinement) {
    const int steps = 8 * (1 << refinement);
    const double h = 1.0 / static_cast<double>(steps);
    const std::complex<double> amplification{1.0 - h * h / 2.0 + std::pow(h, 4) / 24.0,
                                             h - h * h * h / 6.0};
    const auto discrete = integer_power(amplification, steps);
    const auto run = simulate(compiled, {.step_s = h, .step_count = steps});
    const double floating = arithmetic_budget(steps, model.blocks.size(), 4.0);
    EXPECT_NEAR(run.states.back()(0), discrete.real(), floating);
    EXPECT_NEAR(run.states.back()(1), -discrete.imag(), floating);
    const double continuous_error =
        std::hypot(run.states.back()(0) - std::cos(1.0), run.states.back()(1) + std::sin(1.0));
    EXPECT_LE(continuous_error,
              static_cast<double>(steps) * std::exp(h) * std::pow(h, 5) / 120.0 + 2.0 * floating);
  }
}

TEST(Modeling, CompiledSourceSnapshotAndIndependentIntegratorStatesCannotAlias) {
  // MC06: two distinct state owners, with x0=1 and z0=2, obey the same decay.
  auto source = decay();
  source.blocks.push_back({"z_state", signal(1), Integrator{2.0}});
  source.blocks.push_back({"z_rate", signal(1, -1), Gain{-1.0, dimension(0, -1)}});
  source.blocks.push_back({"z_output", signal(1), Output{}});
  source.connections.insert(
      source.connections.end(),
      {{"z_state", "z_rate", 0}, {"z_rate", "z_state", 0}, {"z_state", "z_output", 0}});
  const auto compiled = compile_model(source);
  const auto hash = compiled.semantic_sha256();
  source.blocks.clear();
  source.connections.clear();
  EXPECT_EQ(compiled.source_model().blocks.size(), 6U);
  ASSERT_EQ(compiled.state_ids(), (std::vector<std::string>{"state", "z_state"}));
  EXPECT_EQ(compiled.initial_state()(0), 1.0);
  EXPECT_EQ(compiled.initial_state()(1), 2.0);
  Eigen::VectorXd state_a(2);
  state_a << 3.0, -4.0;
  const auto first = compiled.evaluate(0.0, state_a);
  const Eigen::VectorXd state_b = -state_a;
  (void)compiled.evaluate(1.0, state_b);
  const auto again = compiled.evaluate(0.0, state_a);
  EXPECT_EQ(first.derivatives(0), -3.0);
  EXPECT_EQ(first.derivatives(1), 4.0);
  EXPECT_EQ(first.derivatives, again.derivatives);
  EXPECT_EQ(first.outputs, again.outputs);
  const auto run = simulate(compiled, {.step_s = 0.125, .step_count = 8});
  EXPECT_EQ(run.semantic_sha256, hash);
  EXPECT_NEAR(run.states.back()(0), integer_power(86753.0 / 98304.0, 8), arithmetic_budget(8, 6));
  EXPECT_NEAR(run.states.back()(1),
              2.0 * integer_power(86753.0 / 98304.0, 8),
              arithmetic_budget(8, 6, 4.0));
  expect_same_bits(run, simulate(compiled, {.step_s = 0.125, .step_count = 8}));
}

TEST(Modeling, CanonicalOrderingDoesNotDependOnSourceInsertionOrder) {
  // MC10: preserve semantic IDs/ports while reversing all declarations.
  auto model = oscillator();
  const auto first = compile_model(model);
  std::reverse(model.blocks.begin(), model.blocks.end());
  std::reverse(model.connections.begin(), model.connections.end());
  const auto second = compile_model(model);
  EXPECT_EQ(first.semantic_sha256(), second.semantic_sha256());
  EXPECT_EQ(first.state_ids(), second.state_ids());
  EXPECT_EQ(first.output_ids(), second.output_ids());
  EXPECT_EQ(first.schedule_ids(), second.schedule_ids());
  expect_same_bits(simulate(first, {.step_s = 0.125, .step_count = 8}),
                   simulate(second, {.step_s = 0.125, .step_count = 8}));
  std::get<Integrator>(named(model, "position").parameters).initial_value = 2.0;
  EXPECT_NE(compile_model(model).semantic_sha256(), first.semantic_sha256());
}

TEST(Modeling, OrderedSumPortsPreserveFloatingReductionAndRepeatedInputs) {
  // MC11: ((1e16 + -1e16) + 1) is 1; (1e16 + ( -1e16 + 1)) is 0.
  Model model;
  model.blocks = {{"positive", signal(), Constant{1e16}},
                  {"negative", signal(), Constant{-1e16}},
                  {"small", signal(), Constant{1.0}},
                  {"sum", signal(), Sum{{1, 1, 1}}},
                  {"output", signal(), Output{}}};
  model.connections = {
      {"positive", "sum", 0}, {"negative", "sum", 1}, {"small", "sum", 2}, {"sum", "output", 0}};
  const auto first = compile_model(model);
  EXPECT_EQ(first.evaluate(0.0, Eigen::VectorXd{}).outputs(0), 1.0);
  std::reverse(model.blocks.begin(), model.blocks.end());
  std::reverse(model.connections.begin(), model.connections.end());
  const auto reordered = compile_model(model);
  EXPECT_EQ(first.semantic_sha256(), reordered.semantic_sha256());
  EXPECT_EQ(reordered.evaluate(0.0, Eigen::VectorXd{}).outputs(0), 1.0);
  model.connections = {
      {"positive", "sum", 0}, {"small", "sum", 1}, {"negative", "sum", 2}, {"sum", "output", 0}};
  const auto changed_ports = compile_model(model);
  EXPECT_NE(changed_ports.semantic_sha256(), first.semantic_sha256());
  EXPECT_EQ(changed_ports.evaluate(0.0, Eigen::VectorXd{}).outputs(0), 0.0);
  named(model, "sum").parameters = Sum{{1, 1, 1}};
  model.connections = {
      {"small", "sum", 0}, {"small", "sum", 1}, {"small", "sum", 2}, {"sum", "output", 0}};
  EXPECT_EQ(compile_model(model).evaluate(0.0, Eigen::VectorXd{}).outputs(0), 3.0);
}

TEST(Modeling, RecordingIncludesInitialAndFinalWithoutChangingDynamics) {
  // MC12: integer-derived times, zero duration, final sample off stride.
  const auto compiled = compile_model(decay());
  const auto zero = simulate(compiled, {.initial_time_s = 2.0, .step_s = 0.125, .step_count = 0});
  ASSERT_EQ(zero.times_s, (std::vector<double>{2.0}));
  EXPECT_EQ(zero.states.front()(0), 1.0);
  EXPECT_EQ(zero.outputs.front()(0), 1.0);
  const auto dense = simulate(compiled, {.initial_time_s = 2.0, .step_s = 0.125, .step_count = 5});
  const auto sparse = simulate(
      compiled, {.initial_time_s = 2.0, .step_s = 0.125, .step_count = 5, .sample_stride = 2});
  EXPECT_EQ(sparse.times_s, (std::vector<double>{2.0, 2.25, 2.5, 2.625}));
  EXPECT_EQ(std::bit_cast<std::uint64_t>(dense.states.back()(0)),
            std::bit_cast<std::uint64_t>(sparse.states.back()(0)));
  const auto dividing = simulate(compiled, {.step_s = 0.125, .step_count = 4, .sample_stride = 2});
  EXPECT_EQ(dividing.times_s, (std::vector<double>{0.0, 0.25, 0.5}));
  const auto longer = simulate(compiled, {.step_s = 0.125, .step_count = 1, .sample_stride = 7});
  EXPECT_EQ(longer.times_s, (std::vector<double>{0.0, 0.125}));
  expect_same_bits(dense, simulate(compiled, dense.options));
}

TEST(Modeling, MalformedDirectSourceCannotBypassCompilationValidation) {
  // MC20/MC23: parser validation is not the only semantic boundary.
  expect_invalid_model(Model{}, ErrorCode::InvalidModel);
  for (const std::string id : {"", "1starts_with_digit", "with space", "-prefix"}) {
    auto model = constant_output();
    model.blocks.front().id = id;
    model.connections.front().source = id;
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
  {
    auto model = constant_output();
    model.blocks.push_back(model.blocks.front());
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
  {
    auto model = constant_output();
    model.connections.front().source = "missing";
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
  {
    auto model = constant_output();
    model.connections.front().target = "missing";
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
  {
    auto model = constant_output();
    model.connections.front().input = 1;
    expect_invalid_model(model, ErrorCode::InvalidModel);
    model.connections.front().input = std::numeric_limits<std::size_t>::max();
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
  {
    auto model = constant_output();
    model.connections.clear();
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
  {
    auto model = constant_output();
    model.connections.push_back(model.connections.front());
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
  for (const auto unsupported : {true, false}) {
    auto model = constant_output();
    (unsupported ? model.schema : model.profile) = "future.unsupported.v9";
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
  {
    auto model = constant_output();
    model.blocks.pop_back();
    model.connections.clear();
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
}

TEST(Modeling, UnitsFramesAndAngleSemanticsAreCheckedBeforeEvaluation) {
  // MC21: rate dimensions belong to the gain and differ from state dimensions.
  {
    auto model = decay();
    named(model, "rate").output = signal(1);  // m cannot integrate directly to m.
    expect_invalid_model(model, ErrorCode::TypeMismatch);
  }
  {
    auto model = decay();
    std::get<Gain>(named(model, "rate").parameters).dimension = dimension();
    expect_invalid_model(model, ErrorCode::TypeMismatch);
  }
  {
    auto model = affine();
    named(model, "command").output = signal(0, -1);
    expect_invalid_model(model, ErrorCode::TypeMismatch);
  }
  {
    auto model = constant_output();
    named(model, "constant").output.frame = Frame::Body;
    named(model, "output").output.frame = Frame::Ned;
    expect_invalid_model(model, ErrorCode::TypeMismatch);
  }
  {
    auto model = constant_output();
    named(model, "constant").output = signal(0, 0, Frame::None, 1);
    expect_invalid_model(model, ErrorCode::TypeMismatch);  // Angle is not unitless.
  }
  {
    auto model = decay();
    for (auto& block : model.blocks) {
      block.output.frame = Frame::Body;
    }
    EXPECT_EQ(compile_model(model).evaluate(0.0, scalar(2.0)).derivatives(0), -2.0);
  }
}

TEST(Modeling, InvalidParameterDomainsAndEnumValuesAreRejected) {
  // MC20/MC21: direct aggregate construction can bypass parser spelling checks.
  for (const int sign : {0, 2, -2}) {
    auto model = affine();
    std::get<Sum>(named(model, "sum").parameters).signs[0] = sign;
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
  {
    auto model = affine();
    std::get<Sum>(named(model, "sum").parameters).signs.clear();
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
  {
    auto model = constant_output();
    model.blocks.front().output.dimension[0] = 17;
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
  {
    auto model = decay();
    std::get<Gain>(named(model, "rate").parameters).dimension[0] = -17;
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
  {
    auto model = constant_output();
    model.blocks.front().output.frame = static_cast<Frame>(99);
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
}

TEST(Modeling, EveryInstantaneousCycleIsRejectedEvenWhenItsGainIsZeroOrUnobserved) {
  // MC22: an integrator breaks feedthrough; numerical zero or unused output does not.
  for (const double coefficient : {0.0, 1.0}) {
    Model model;
    model.blocks = {{"gain", signal(), Gain{coefficient, dimension()}},
                    {"output", signal(), Output{}}};
    model.connections = {{"gain", "gain", 0}, {"gain", "output", 0}};
    expect_invalid_model(model, ErrorCode::AlgebraicLoop);
  }
  {
    auto model = constant_output();
    model.blocks.push_back({"loop_a", signal(), Gain{0.0, dimension()}});
    model.blocks.push_back({"loop_b", signal(), Gain{1.0, dimension()}});
    model.connections.push_back({"loop_a", "loop_b", 0});
    model.connections.push_back({"loop_b", "loop_a", 0});
    expect_invalid_model(model, ErrorCode::AlgebraicLoop);
  }
  EXPECT_NO_THROW((void)compile_model(decay()));
  EXPECT_NO_THROW((void)compile_model(affine()));
}

TEST(Modeling, NonfiniteDeclarationsAreInvalidAtEveryDirectSourceBoundary) {
  // MC24: retain actual nonfinite values; do not serialize or replace them.
  for (const double value : {std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity()}) {
    expect_invalid_model(constant_output(value), ErrorCode::InvalidModel);
    expect_invalid_model(decay(value), ErrorCode::InvalidModel);
    auto model = decay();
    std::get<Gain>(named(model, "rate").parameters).value = value;
    expect_invalid_model(model, ErrorCode::InvalidModel);
  }
}

TEST(Modeling, FiniteInputsThatOverflowOutputsDerivativesOrRkStagesNeverComplete) {
  // MC24: one refusal each for a purely observed branch, derivative, and stage.
  const double largest = std::numeric_limits<double>::max();
  {
    auto model = constant_output(largest);
    model.blocks.push_back({"gain", signal(), Gain{2.0, dimension()}});
    model.connections = {{"constant", "gain", 0}, {"gain", "output", 0}};
    const auto compiled = compile_model(model);
    expect_error([&] { (void)compiled.evaluate(0.0, Eigen::VectorXd{}); },
                 ErrorCode::NonFiniteEvaluation);
    expect_error([&] { (void)simulate(compiled, {.step_count = 0}); },
                 ErrorCode::NonFiniteEvaluation);
  }
  {
    auto model = decay(2.0);
    std::get<Gain>(named(model, "rate").parameters).value = largest;
    const auto compiled = compile_model(model);
    expect_error([&] { (void)compiled.evaluate(0.0, scalar(2.0)); },
                 ErrorCode::NonFiniteEvaluation);
  }
  {
    // A branch not contributing to any declared Output still belongs to the
    // accepted model. Dead-branch pruning must not hide an invalid operation.
    auto model = decay();
    model.blocks.push_back({"unused_constant", signal(), Constant{largest}});
    model.blocks.push_back({"unused_gain", signal(), Gain{2.0, dimension()}});
    model.connections.push_back({"unused_constant", "unused_gain", 0});
    const auto compiled = compile_model(model);
    expect_error([&] { (void)compiled.evaluate(0.0, scalar(1.0)); },
                 ErrorCode::NonFiniteEvaluation);
  }
  {
    Model model;
    model.blocks = {{"constant", signal(1, -1), Constant{largest / 2.0}},
                    {"state", signal(1), Integrator{largest / 2.0}},
                    {"output", signal(1), Output{}}};
    model.connections = {{"constant", "state", 0}, {"state", "output", 0}};
    const auto compiled = compile_model(model);
    expect_error([&] { (void)simulate(compiled, {.step_s = 4.0, .step_count = 1}); },
                 ErrorCode::NonFiniteEvaluation);
  }
}

TEST(Modeling, InvalidSimulationInputsAndUnrepresentableTimeAreRejected) {
  // MC25: the declared count/time grid must be representable, including RK stages.
  const auto compiled = compile_model(decay());
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const double step : {0.0, -0.125, infinity, nan}) {
    expect_error([&] { (void)simulate(compiled, {.step_s = step, .step_count = 1}); },
                 ErrorCode::InvalidSimulation);
  }
  expect_error([&] { (void)simulate(compiled, {.step_count = -1}); }, ErrorCode::InvalidSimulation);
  expect_error([&] { (void)simulate(compiled, {.sample_stride = 0}); },
               ErrorCode::InvalidSimulation);
  expect_error([&] { (void)simulate(compiled, {.initial_time_s = infinity}); },
               ErrorCode::InvalidSimulation);
  expect_error(
      [&] { (void)simulate(compiled, {.initial_time_s = 1e300, .step_s = 1.0, .step_count = 1}); },
      ErrorCode::InvalidSimulation);
  expect_error(
      [&] {
        (void)simulate(compiled, {.step_s = std::numeric_limits<double>::max(), .step_count = 2});
      },
      ErrorCode::InvalidSimulation);
  expect_error(
      [&] {
        (void)simulate(
            compiled, {.step_s = 2.0 * std::numeric_limits<double>::denorm_min(), .step_count = 1});
      },
      ErrorCode::InvalidSimulation);
  expect_error([&] { (void)compiled.evaluate(nan, scalar(1.0)); }, ErrorCode::InvalidSimulation);
  expect_error([&] { (void)compiled.evaluate(0.0, Eigen::VectorXd{}); },
               ErrorCode::InvalidSimulation);
  expect_error([&] { (void)compiled.evaluate(0.0, scalar(nan)); }, ErrorCode::NonFiniteEvaluation);
}

TEST(Modeling, OrderedSumIntermediateOverflowIsRefusedDespiteAFiniteExactSum) {
  // MC24: M + M - M is M in real arithmetic, but the declared left fold
  // overflows at the second addition. Reassociation would conceal the failure.
  Model model;
  model.blocks = {{"largest", signal(), Constant{std::numeric_limits<double>::max()}},
                  {"sum", signal(), Sum{{1, 1, -1}}},
                  {"output", signal(), Output{}}};
  model.connections = {
      {"largest", "sum", 0}, {"largest", "sum", 1}, {"largest", "sum", 2}, {"sum", "output", 0}};
  const auto compiled = compile_model(model);
  expect_error([&] { (void)compiled.evaluate(0.0, Eigen::VectorXd{}); },
               ErrorCode::NonFiniteEvaluation);
  expect_error([&] { (void)simulate(compiled, {.step_count = 0}); },
               ErrorCode::NonFiniteEvaluation);
}

TEST(Modeling, MaximumDepthUnityChainEvaluatesExactlyAndItsDeepCycleIsRejected) {
  // MC26/MC22: maximum admitted block count arranged as one dependency chain,
  // rather than the wide graphs in the count-limit fixtures. Every product is
  // exactly 1*1, so no numerical tolerance is needed. Closing the final gain
  // back to the first creates a long instantaneous cycle, never a hidden delay.
  auto model = constant_output();
  model.connections.clear();
  std::string previous = "constant";
  for (std::size_t index = model.blocks.size(); index < kMaxBlocks; ++index) {
    const auto id = "gain_" + std::to_string(index);
    model.blocks.push_back({id, signal(), Gain{1.0, dimension()}});
    model.connections.push_back({previous, id, 0});
    previous = id;
  }
  model.connections.push_back({previous, "output", 0});
  ASSERT_EQ(model.blocks.size(), kMaxBlocks);
  const auto compiled = compile_model(model);
  EXPECT_EQ(compiled.schedule_ids().size(), kMaxBlocks);
  EXPECT_TRUE(compiled.state_ids().empty());
  EXPECT_EQ(compiled.evaluate(0.0, Eigen::VectorXd{}).outputs(0), 1.0);
  const auto run = simulate(compiled, {.step_count = 0});
  ASSERT_EQ(run.outputs.size(), 1U);
  EXPECT_EQ(run.outputs.front()(0), 1.0);
  model.connections.front().source = previous;
  expect_invalid_model(model, ErrorCode::AlgebraicLoop);
}

TEST(Modeling, GraphAndSumPortLimitsAcceptPracticalBoundaryAndRefuseFirstExcess) {
  // MC26: finite source fixtures exercise exact caps without exhausting memory.
  auto model = constant_output();
  for (std::size_t index = model.blocks.size(); index < kMaxBlocks; ++index) {
    model.blocks.push_back({"extra_" + std::to_string(index), signal(), Constant{0.0}});
  }
  EXPECT_NO_THROW((void)compile_model(model));
  model.blocks.push_back({"beyond_limit", signal(), Constant{0.0}});
  expect_invalid_model(model, ErrorCode::ResourceLimit);

  Model summed;
  summed.blocks = {{"constant", signal(), Constant{1.0}},
                   {"sum", signal(), Sum{std::vector<int>(kMaxSumInputs, 1)}},
                   {"output", signal(), Output{}}};
  for (std::size_t port = 0; port < kMaxSumInputs; ++port) {
    summed.connections.push_back({"constant", "sum", port});
  }
  summed.connections.push_back({"sum", "output", 0});
  EXPECT_EQ(compile_model(summed).evaluate(0.0, Eigen::VectorXd{}).outputs(0),
            static_cast<double>(kMaxSumInputs));
  std::get<Sum>(named(summed, "sum").parameters).signs.push_back(1);
  summed.connections.push_back({"constant", "sum", kMaxSumInputs});
  expect_invalid_model(summed, ErrorCode::ResourceLimit);
}

TEST(Modeling, ConnectionLimitCountsAllPortsIncludingFanoutAndUnobservedBranches) {
  // MC26: 127*64 + 63 sum inputs + one output connection = 8192.
  Model model;
  model.blocks = {{"constant", signal(), Constant{1.0}}, {"output", signal(), Output{}}};
  for (std::size_t index = 0; index < 128; ++index) {
    const auto id = "sum_" + std::to_string(index);
    const std::size_t inputs = index == 127 ? 63 : 64;
    model.blocks.push_back({id, signal(), Sum{std::vector<int>(inputs, 1)}});
    for (std::size_t port = 0; port < inputs; ++port) {
      model.connections.push_back({"constant", id, port});
    }
  }
  model.connections.push_back({"sum_127", "output", 0});
  ASSERT_EQ(model.connections.size(), kMaxConnections);
  EXPECT_EQ(compile_model(model).evaluate(0.0, Eigen::VectorXd{}).outputs(0), 63.0);
  std::get<Sum>(named(model, "sum_127").parameters).signs.push_back(1);
  model.connections.push_back({"constant", "sum_127", 63});
  expect_invalid_model(model, ErrorCode::ResourceLimit);
}

TEST(Modeling, RunBudgetsRejectOversizeCountsStorageAndWorkBeforeExecutingThem) {
  // MC26: no case below runs its large requested workload.
  const auto compiled = compile_model(decay());
  expect_error([&] { (void)simulate(compiled, {.step_count = kMaxSteps + 1}); },
               ErrorCode::ResourceLimit);
  expect_error([&] { (void)simulate(compiled, {.step_count = std::numeric_limits<int>::max()}); },
               ErrorCode::ResourceLimit);
  // Initial + every step, with time/state/output = three scalars per sample.
  const int steps = static_cast<int>(kMaxRecordedScalars / 3);
  expect_error([&] { (void)simulate(compiled, {.step_count = steps}); }, ErrorCode::ResourceLimit);
  // Stateless trajectories still retain a time and an output at every sample.
  const auto stateless = compile_model(constant_output());
  const int stateless_steps = static_cast<int>(kMaxRecordedScalars / 2);
  expect_error([&] { (void)simulate(stateless, {.step_count = stateless_steps}); },
               ErrorCode::ResourceLimit);
  auto source = decay();
  for (std::size_t index = source.blocks.size(); index < kMaxBlocks; ++index) {
    source.blocks.push_back({"extra_" + std::to_string(index), signal(), Constant{0.0}});
  }
  const auto costly = compile_model(source);
  expect_error(
      [&] {
        (void)simulate(costly, {.step_s = 0.125, .step_count = 30000, .sample_stride = 30000});
      },
      ErrorCode::ResourceLimit);
}

TEST(Modeling, CanonicalEvidenceExpansionIsRefusedBeforeACompiledModelCanRun) {
  // MC26: a compact document can fit the input cap while the required canonical
  // YAML evidence expands past it. Compilation must establish serializability,
  // so an otherwise valid long run cannot first discover this after executing.
  const auto id = [](const std::string& prefix) {
    return prefix + std::string(42 - prefix.size(), 'x');
  };
  const auto constant = id("constant");
  const auto output = id("output");
  const std::string type = "output: {dimension: [0,0,0,0,0,0,0,0], frame: none}";
  std::ostringstream document;
  document << "schema: galata.model.v1\nprofile: continuous-scalar.v1\nblocks:\n"
           << "  - {id: " << constant << ", kind: constant, " << type << ", value: 1}\n"
           << "  - {id: " << output << ", kind: output, " << type << "}\n";
  for (std::size_t index = 0; index < 128; ++index) {
    const std::size_t inputs = index == 127 ? 63 : 64;
    document << "  - {id: " << id("sum_" + std::to_string(index)) << ", kind: sum, " << type
             << ", signs: [";
    for (std::size_t port = 0; port < inputs; ++port) {
      if (port != 0)
        document << ',';
      document << '1';
    }
    document << "]}\n";
  }
  document << "connections:\n";
  for (std::size_t index = 0; index < 128; ++index) {
    const std::size_t inputs = index == 127 ? 63 : 64;
    for (std::size_t port = 0; port < inputs; ++port) {
      document << "  - {source: " << constant << ", target: " << id("sum_" + std::to_string(index))
               << ", input: " << port << "}\n";
    }
  }
  document << "  - {source: " << id("sum_127") << ", target: " << output << ", input: 0}\n";
  const auto bytes = document.str();
  ASSERT_LE(bytes.size(), kMaxSourceBytes);
  const auto parsed = parse_model_yaml(bytes);
  ASSERT_EQ(parsed.connections.size(), kMaxConnections);
  EXPECT_NO_THROW(validate_model(parsed));
  expect_error([&] { (void)write_model_yaml(parsed); }, ErrorCode::ResourceLimit);
  expect_error([&] { (void)compile_model(parsed); }, ErrorCode::ResourceLimit);
}

TEST(Modeling, CancellationAndCallbackFailureCannotReturnACompletedTrajectory) {
  const auto compiled = compile_model(decay());
  expect_error([&] { (void)simulate(compiled, {.step_count = 4}, [] { return true; }); },
               ErrorCode::Cancelled);
  int calls = 0;
  expect_error([&] { (void)simulate(compiled, {.step_count = 4}, [&] { return ++calls >= 3; }); },
               ErrorCode::Cancelled);
  EXPECT_EQ(calls, 3);
  EXPECT_THROW((void)simulate(
                   compiled,
                   {.step_count = 4},
                   []() -> bool { throw std::logic_error("test cancellation callback failure"); }),
               std::logic_error);
}
}  // namespace
