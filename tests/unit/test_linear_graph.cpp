// SPDX-License-Identifier: Apache-2.0
// Independent ADR-0013 fixtures, written from the public contract without
// reading the adapter implementation. Exact dyadic matrix equations and the
// classical RK4 stability polynomial are the numerical references.
#include "galata/modeling/linear_adapter.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace galata::modeling;

Dimension dims(int length = 0, int time = 0, int angle = 0) {
  return {length, 0, time, 0, 0, 0, 0, angle};
}

SignalType type(int length = 0, int time = 0, int angle = 0, Frame frame = Frame::None) {
  return {dims(length, time, angle), frame};
}

Block& block_at(Model& model, const std::string& id) {
  const auto found = std::find_if(
      model.blocks.begin(), model.blocks.end(), [&](const auto& block) { return block.id == id; });
  if (found == model.blocks.end()) {
    throw std::logic_error("test fixture block is absent");
  }
  return *found;
}

Eigen::Index index_of(const std::vector<std::string>& ids, const std::string& id) {
  const auto found = std::find(ids.begin(), ids.end(), id);
  if (found == ids.end()) {
    throw std::logic_error("test fixture source map is absent");
  }
  return std::distance(ids.begin(), found);
}

void refuses(const std::function<void()>& action, ErrorCode code) {
  try {
    action();
    FAIL() << "accepted an invalid linear graph";
  } catch (const Error& error) {
    EXPECT_EQ(error.code(), code) << error.what();
    EXPECT_FALSE(std::string(error.what()).empty());
  }
}

Model mixed_row() {
  Model model;
  model.profile = kLinearProfile;
  const auto length = type(1, 0, 0, Frame::Body);
  const auto angle = type(0, 0, 1);
  const auto output = type(0, 0, 1, Frame::Ned);
  model.blocks = {
      {"z_length", length, Constant{2.0}},
      {"a_angle", angle, Constant{-1.0}},
      {"row",
       output,
       LinearCombination{{{length, Gain{0.5, dims(-1, 0, 1)}}, {angle, Gain{2.0, dims()}}}}},
      {"output", output, Output{}}};
  model.connections = {{"z_length", "row", 0}, {"a_angle", "row", 1}, {"row", "output", 0}};
  return model;
}

struct Fixture {
  galata::model::LinearSystem system;
  LinearChannels channels;
  LinearGraphOptions options;

  Fixture() {
    system.a.resize(2, 2);
    system.a << -1.0, 0.5, 2.0, -2.0;
    system.b.resize(2, 2);
    system.b << 2.0, -1.0, 0.5, 1.0;
    system.c.resize(2, 2);
    system.c << 2.0, -1.0, -0.5, 1.0;
    system.d.resize(2, 2);
    system.d << 1.0, 0.5, 2.0, -1.0;
    // Alphabetic label order deliberately differs from matrix order.
    system.state_names = {"z_velocity", "a_angle"};
    system.input_names = {"z_acceleration", "a_deflection"};
    system.output_names = {"z_distance", "a_ratio"};
    channels.states = {type(1, -1, 0, Frame::Body), type(0, 0, 1)};
    channels.inputs = {type(1, -2, 0, Frame::Ned), type(0, 0, 1, Frame::Body)};
    channels.outputs = {type(1), type(0, 0, 0, Frame::Ned)};
    options.initial_state.resize(2);
    options.initial_state << 0.25, 0.5;
    options.command.resize(2);
    options.command << 0.5, 2.0;
    options.feedback_gain.resize(2, 2);
    options.feedback_gain << 0.5, -1.0, 0.25, 0.5;
  }
};

TEST(LinearGraph, OrderedRowExplicitlyCouplesMixedDimensionsAndFrames) {
  const auto model = mixed_row();
  const auto compiled = compile_model(model);
  EXPECT_TRUE(compiled.state_ids().empty());
  EXPECT_EQ(compiled.evaluate(7.0, Eigen::VectorXd{}).outputs(0), -1.0);
  const auto restored = parse_model_yaml(write_model_yaml(model));
  EXPECT_EQ(canonical_model(restored), canonical_model(model));
  auto reordered = model;
  std::reverse(reordered.blocks.begin(), reordered.blocks.end());
  std::reverse(reordered.connections.begin(), reordered.connections.end());
  EXPECT_EQ(compile_model(reordered).semantic_sha256(), compiled.semantic_sha256());
  EXPECT_EQ(compile_model(reordered).evaluate(0.0, Eigen::VectorXd{}).outputs(0), -1.0);
}

TEST(LinearGraph, DyadicAdapterPreservesSourceOrderAndHandDerivedEquations) {
  const Fixture fixture;
  const auto graph = lower_linear_system(fixture.system, fixture.channels, fixture.options);
  EXPECT_EQ(graph.model.profile, kLinearProfile);
  EXPECT_EQ(graph.state_ids, (std::vector<std::string>{"state_000", "state_001"}));
  EXPECT_EQ(graph.command_ids, (std::vector<std::string>{"command_000", "command_001"}));
  EXPECT_EQ(graph.control_ids, (std::vector<std::string>{"control_000", "control_001"}));
  EXPECT_EQ(graph.output_ids, (std::vector<std::string>{"output_000", "output_001"}));
  EXPECT_EQ(graph.control_output_ids,
            (std::vector<std::string>{"control_output_000", "control_output_001"}));
  const auto compiled = compile_model(graph.model);
  Eigen::VectorXd state(2);
  state(index_of(compiled.state_ids(), graph.state_ids[0])) = 2.0;
  state(index_of(compiled.state_ids(), graph.state_ids[1])) = -1.0;
  // At x=[2,-1], c=[1/2,2], K=[[1/2,-1],[1/4,1/2]]:
  // u=[-3/2,2], Ax+Bu=[-15/2,29/4], Cx+Du=[9/2,-7].
  const auto evaluated = compiled.evaluate(3.0, state);
  EXPECT_EQ(evaluated.derivatives(index_of(compiled.state_ids(), graph.state_ids[0])), -7.5);
  EXPECT_EQ(evaluated.derivatives(index_of(compiled.state_ids(), graph.state_ids[1])), 7.25);
  EXPECT_EQ(evaluated.outputs(index_of(compiled.output_ids(), graph.output_ids[0])), 4.5);
  EXPECT_EQ(evaluated.outputs(index_of(compiled.output_ids(), graph.output_ids[1])), -7.0);
  EXPECT_EQ(evaluated.outputs(index_of(compiled.output_ids(), graph.control_output_ids[0])), -1.5);
  EXPECT_EQ(evaluated.outputs(index_of(compiled.output_ids(), graph.control_output_ids[1])), 2.0);
  EXPECT_EQ(compiled.initial_state()(index_of(compiled.state_ids(), graph.state_ids[0])), 0.25);
  EXPECT_EQ(compiled.initial_state()(index_of(compiled.state_ids(), graph.state_ids[1])), 0.5);

  auto reordered = graph.model;
  std::reverse(reordered.blocks.begin(), reordered.blocks.end());
  std::reverse(reordered.connections.begin(), reordered.connections.end());
  const auto permuted = compile_model(reordered);
  EXPECT_EQ(permuted.semantic_sha256(), compiled.semantic_sha256());
  EXPECT_EQ(permuted.state_ids(), compiled.state_ids());
  EXPECT_EQ(permuted.evaluate(3.0, state).derivatives, evaluated.derivatives);
  EXPECT_EQ(permuted.evaluate(3.0, state).outputs, evaluated.outputs);

  auto edited = graph.model;
  std::get<Constant>(block_at(edited, graph.command_ids[0]).parameters).value = 1.5;
  const auto modified = compile_model(edited);
  EXPECT_NE(modified.semantic_sha256(), compiled.semantic_sha256());
  EXPECT_EQ(modified.evaluate(3.0, state).derivatives(0), -5.5);
  EXPECT_EQ(compiled.evaluate(3.0, state).derivatives(0), -7.5);
}

TEST(LinearGraph, ScalarAffineAdapterMatchesIndependentRk4Polynomial) {
  // xdot=-x+1, x0=0. For h=1/8, R(-h)=86753/98304 exactly as
  // predeclared in MC04/B03. This uses no production integrator as oracle.
  galata::model::LinearSystem system;
  system.a = Eigen::MatrixXd::Constant(1, 1, -1.0);
  system.b = Eigen::MatrixXd::Ones(1, 1);
  system.state_names = {"distance"};
  system.input_names = {"speed_command"};
  LinearChannels channels{{type(1)}, {type(1, -1)}, {type(1)}};
  LinearGraphOptions options;
  options.initial_state = Eigen::VectorXd::Zero(1);
  options.command = Eigen::VectorXd::Ones(1);
  const auto graph = lower_linear_system(system, channels, options);
  const auto compiled = compile_model(graph.model);
  const auto result = simulate(compiled, {.step_s = 0.125, .step_count = 8});
  const double count = (64.0 * static_cast<double>(graph.model.blocks.size()) + 32.0) * 9.0;
  const double accumulated = count * std::numeric_limits<double>::epsilon();
  const double budget = 4.0 * accumulated / (1.0 - accumulated);
  double power = 1.0;
  ASSERT_EQ(result.states.size(), 9U);
  for (std::size_t sample = 0; sample < result.states.size(); ++sample) {
    EXPECT_NEAR(result.states[sample](0), 1.0 - power, budget);
    EXPECT_EQ(result.outputs[sample](index_of(result.output_ids, graph.output_ids[0])),
              result.states[sample](0));
    power *= 86753.0 / 98304.0;
  }
}

TEST(LinearGraph, RowRefusesLegacyProfileAndMismatchedDeclaredTypes) {
  auto changed = mixed_row();
  auto legacy_source = write_model_yaml(changed);
  const auto profile_at = legacy_source.find(kLinearProfile);
  ASSERT_NE(profile_at, std::string::npos);
  legacy_source.replace(profile_at, kLinearProfile.size(), kProfile);
  refuses([&] { (void)parse_model_yaml(legacy_source); }, ErrorCode::InvalidDocument);
  changed.profile = kProfile;
  EXPECT_THROW((void)compile_model(changed), Error);
  changed = mixed_row();
  std::get<LinearCombination>(block_at(changed, "row").parameters).terms[0].input.frame =
      Frame::None;
  refuses([&] { (void)compile_model(changed); }, ErrorCode::TypeMismatch);
  changed = mixed_row();
  std::get<LinearCombination>(block_at(changed, "row").parameters).terms[0].coefficient.dimension =
      dims();
  refuses([&] { (void)compile_model(changed); }, ErrorCode::TypeMismatch);
  changed = mixed_row();
  std::get<LinearCombination>(block_at(changed, "row").parameters).terms[0].input.frame =
      static_cast<Frame>(999);
  EXPECT_THROW((void)compile_model(changed), Error);
  changed = mixed_row();
  changed.connections.push_back(changed.connections.front());
  refuses([&] { (void)compile_model(changed); }, ErrorCode::InvalidModel);
}

TEST(LinearGraph, NewProfileDoesNotRelaxExistingGainFrameEquality) {
  Model model;
  model.profile = kLinearProfile;
  model.blocks = {{"source", type(1, 0, 0, Frame::Body), Constant{1.0}},
                  {"gain", type(1, 0, 0, Frame::Ned), Gain{1.0, dims()}},
                  {"output", type(1, 0, 0, Frame::Ned), Output{}}};
  model.connections = {{"source", "gain", 0}, {"gain", "output", 0}};
  refuses([&] { (void)compile_model(model); }, ErrorCode::TypeMismatch);
}

Model repeated_row(std::size_t count, double value = 1.0, double coefficient = 1.0) {
  Model model;
  model.profile = kLinearProfile;
  LinearCombination row;
  for (std::size_t port = 0; port < count; ++port) {
    row.terms.push_back({type(), Gain{coefficient, dims()}});
    model.connections.push_back({"source", "row", port});
  }
  model.blocks = {
      {"source", type(), Constant{value}}, {"row", type(), row}, {"output", type(), Output{}}};
  model.connections.push_back({"row", "output", 0});
  return model;
}

TEST(LinearGraph, RowOrderIsSemanticAndSourceDeclarationOrderIsNot) {
  auto model = repeated_row(3);
  model.blocks.push_back({"negative", type(), Constant{-1e16}});
  model.blocks.push_back({"unit", type(), Constant{1.0}});
  std::get<Constant>(block_at(model, "source").parameters).value = 1e16;
  model.connections[1].source = "negative";
  model.connections[2].source = "unit";
  const auto first = compile_model(model);
  EXPECT_EQ(first.evaluate(0.0, Eigen::VectorXd{}).outputs(0), 1.0);
  std::swap(model.connections[1].source, model.connections[2].source);
  const auto reordered = compile_model(model);
  EXPECT_NE(reordered.semantic_sha256(), first.semantic_sha256());
  EXPECT_EQ(reordered.evaluate(0.0, Eigen::VectorXd{}).outputs(0), 0.0);
}

TEST(LinearGraph, UserAuthoredZeroTermStillCreatesAnInstantaneousDependency) {
  auto model = repeated_row(1, 1.0, 0.0);
  model.connections[0].source = "row";
  refuses([&] { (void)compile_model(model); }, ErrorCode::AlgebraicLoop);
}

TEST(LinearGraph, RowRejectsProductAndIntermediateSumOverflow) {
  for (const auto& model : {repeated_row(1, std::numeric_limits<double>::max(), 2.0),
                            repeated_row(2, std::numeric_limits<double>::max())}) {
    const auto compiled = compile_model(model);
    refuses([&] { (void)compiled.evaluate(0.0, Eigen::VectorXd{}); },
            ErrorCode::NonFiniteEvaluation);
  }
  auto invalid = repeated_row(1);
  std::get<LinearCombination>(block_at(invalid, "row").parameters).terms[0].coefficient.value =
      std::numeric_limits<double>::infinity();
  refuses([&] { (void)compile_model(invalid); }, ErrorCode::InvalidModel);
}

TEST(LinearGraph, RowArityBoundaryIsBoundedAndEmptyRowsAreNotAuthoredBlocks) {
  EXPECT_EQ(compile_model(repeated_row(64)).evaluate(0.0, Eigen::VectorXd{}).outputs(0), 64.0);
  refuses([] { (void)compile_model(repeated_row(65)); }, ErrorCode::ResourceLimit);
  EXPECT_THROW((void)compile_model(repeated_row(0)), Error);
}

TEST(LinearGraph, ZeroMatricesAndDefaultOutputsRemainRunnableWithFeedback) {
  Fixture fixture;
  fixture.system.a.setZero();
  fixture.system.b.setZero();
  fixture.system.c.resize(0, 0);
  fixture.system.d.resize(0, 0);
  fixture.system.output_names.clear();
  fixture.channels.outputs = fixture.channels.states;
  const auto graph = lower_linear_system(fixture.system, fixture.channels, fixture.options);
  const auto compiled = compile_model(graph.model);
  const auto value = compiled.evaluate(0.0, compiled.initial_state());
  EXPECT_EQ(value.derivatives, Eigen::VectorXd::Zero(2));
  EXPECT_EQ(value.outputs(index_of(compiled.output_ids(), graph.output_ids[0])), 0.25);
  EXPECT_EQ(value.outputs(index_of(compiled.output_ids(), graph.output_ids[1])), 0.5);
  for (auto model_block : graph.model.blocks) {
    if (model_block.id == "derivative_000" || model_block.id == "derivative_001") {
      EXPECT_TRUE(std::holds_alternative<Constant>(model_block.parameters));
    }
  }
}

TEST(LinearGraph, AdapterRejectsWrongShapesMissingTypesAndNonfiniteParameters) {
  Fixture fixture;
  auto types = fixture.channels;
  types.states.pop_back();
  EXPECT_THROW((void)lower_linear_system(fixture.system, types, fixture.options), std::exception);
  auto options = fixture.options;
  options.command.resize(1);
  EXPECT_THROW((void)lower_linear_system(fixture.system, fixture.channels, options),
               std::exception);
  options = fixture.options;
  options.feedback_gain.resize(1, 2);
  EXPECT_THROW((void)lower_linear_system(fixture.system, fixture.channels, options),
               std::exception);
  options = fixture.options;
  options.initial_state(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)lower_linear_system(fixture.system, fixture.channels, options),
               std::exception);
  fixture.system.a(0, 0) = std::numeric_limits<double>::infinity();
  EXPECT_THROW((void)lower_linear_system(fixture.system, fixture.channels, fixture.options),
               std::exception);
}

TEST(LinearGraph, AdapterChannelBoundaryIsCheckedBeforeGraphExpansion) {
  for (const int count : {16, 17}) {
    galata::model::LinearSystem system;
    system.a = Eigen::MatrixXd::Zero(count, count);
    system.b = Eigen::MatrixXd::Zero(count, 1);
    for (int state = 0; state < count; ++state) {
      system.state_names.push_back("x" + std::to_string(state));
    }
    system.input_names = {"command"};
    LinearChannels channels;
    channels.states.assign(static_cast<std::size_t>(count), type());
    channels.inputs = {type()};
    channels.outputs = channels.states;
    LinearGraphOptions options;
    options.initial_state = Eigen::VectorXd::Zero(count);
    options.command = Eigen::VectorXd::Zero(1);
    if (count == 16) {
      EXPECT_NO_THROW((void)compile_model(lower_linear_system(system, channels, options).model));
    } else {
      EXPECT_THROW((void)lower_linear_system(system, channels, options), std::exception);
    }
  }
}

TEST(LinearGraph, ScalarCanonicalCompatibilityRegressionLockAnchoredToModelIoByteContract) {
  // Anchored to ModelIo.CanonicalVersionHasAHandSpecifiedByteContract and MC10,
  // whose expected bytes are hand-derived, not captured implementation output.
  Model model;
  model.blocks = {{"source", type(), Constant{0.25}}, {"result", type(), Output{}}};
  model.connections = {{"source", "result", 0}};
  constexpr auto expected =
      "galata.model.canonical.v1\n"
      "15:galata.model.v1\n20:continuous-scalar.v1\n6:blocks\n1:2\n"
      "6:result\n6:output\n"
      "1:0\n1:0\n1:0\n1:0\n1:0\n1:0\n1:0\n1:0\n4:none\n"
      "6:source\n8:constant\n"
      "1:0\n1:0\n1:0\n1:0\n1:0\n1:0\n1:0\n1:0\n4:none\n"
      "16:3fd0000000000000\n"
      "11:connections\n1:1\n6:source\n6:result\n1:0\n";
  EXPECT_EQ(canonical_model(model), expected);
}
}  // namespace
