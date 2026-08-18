// SPDX-License-Identifier: Apache-2.0
//
// The pipeline's contract: what it accepts, what it rejects, and in what order
// it runs things.
//
// The rejection tests matter more than the acceptance ones. A pipeline runner
// that accepts a cyclic graph, or silently skips a stage whose dependency is
// missing, produces a result that looks complete and is not.

#include "galata/pipeline/pipeline.hpp"
#include "galata/pipeline/registry.hpp"

#include <gtest/gtest.h>

#include <string>

namespace {

using galata::pipeline::parse_pipeline;
using galata::pipeline::Pipeline;

TEST(PipelineParse, ReadsStagesAndWiring) {
  const Pipeline pipeline = parse_pipeline(R"(
version: 1
stages:
  - id: first
    capability: model.linear.statespace
    input:
      path: model.yaml
  - id: second
    capability: analyze.modes
    input:
      system: { from: first }
      classify: true
)");
  ASSERT_EQ(pipeline.stages.size(), 2U);
  EXPECT_EQ(pipeline.version, 1);
  EXPECT_EQ(pipeline.stages[0].id, "first");
  EXPECT_EQ(pipeline.stages[1].capability, "analyze.modes");

  const auto references = pipeline.stages[1].input->referenced_stages();
  ASSERT_EQ(references.size(), 1U);
  EXPECT_EQ(references[0], "first");
}

TEST(PipelineParse, StageReferenceIsWiringNotData) {
  // `{from: x}` must never reach a capability as a one-entry map, or a
  // capability could read the wiring as a value and get a plausible wrong
  // answer instead of an error.
  const Pipeline pipeline = parse_pipeline(R"(
version: 1
stages:
  - id: a
    capability: analyze.modes
    input:
      system: { from: b }
  - id: b
    capability: model.linear.statespace
    input: { path: m.yaml }
)");
  const auto system = pipeline.stages[0].input->get("system");
  ASSERT_NE(system, nullptr);
  EXPECT_EQ(system->kind(), galata::pipeline::Value::Kind::StageReference);
  EXPECT_EQ(system->as_stage_reference(), "b");
}

TEST(PipelineParse, ScalarsKeepTheirTypes) {
  const Pipeline pipeline = parse_pipeline(R"(
version: 1
stages:
  - id: only
    capability: analyze.modes
    input:
      classify: true
      count: 42
      ratio: 0.5
      name: hello
      nothing: ~
)");
  const auto& input = pipeline.stages[0].input;
  EXPECT_EQ(input->get("classify")->kind(), galata::pipeline::Value::Kind::Bool);
  EXPECT_EQ(input->get("count")->kind(), galata::pipeline::Value::Kind::Number);
  EXPECT_EQ(input->get("ratio")->kind(), galata::pipeline::Value::Kind::Number);
  EXPECT_EQ(input->get("name")->kind(), galata::pipeline::Value::Kind::String);
  EXPECT_EQ(input->get("nothing")->kind(), galata::pipeline::Value::Kind::Null);
  EXPECT_TRUE(input->get("classify")->as_bool());
  EXPECT_DOUBLE_EQ(input->get("ratio")->as_number(), 0.5);
}

TEST(PipelineOrder, RunsDependenciesFirstAndKeepsDeclarationOrderOtherwise) {
  // `late` is declared first but depends on `early`, so it must run second.
  // `independent` depends on nothing and is declared last, so among the ready
  // stages it goes when its turn comes rather than jumping the queue.
  const Pipeline pipeline = parse_pipeline(R"(
version: 1
stages:
  - id: late
    capability: analyze.modes
    input: { system: { from: early } }
  - id: early
    capability: model.linear.statespace
    input: { path: m.yaml }
  - id: independent
    capability: model.linear.statespace
    input: { path: n.yaml }
)");
  const std::vector<std::string> order = pipeline.execution_order();
  ASSERT_EQ(order.size(), 3U);
  EXPECT_EQ(order[0], "early");
  EXPECT_EQ(order[1], "late");
  EXPECT_EQ(order[2], "independent");
}

TEST(PipelineOrder, IsDeterministicAcrossRepeatedCalls) {
  const Pipeline pipeline = parse_pipeline(R"(
version: 1
stages:
  - id: a
    capability: model.linear.statespace
    input: { path: a.yaml }
  - id: b
    capability: model.linear.statespace
    input: { path: b.yaml }
  - id: c
    capability: analyze.modes
    input: { system: { from: a } }
  - id: d
    capability: analyze.modes
    input: { system: { from: b } }
)");
  const std::vector<std::string> first = pipeline.execution_order();
  for (int repeat = 0; repeat < 20; ++repeat) {
    EXPECT_EQ(pipeline.execution_order(), first);
  }
}

TEST(PipelineParse, RejectsACycleAndNamesTheStagesInIt) {
  try {
    (void)parse_pipeline(R"(
version: 1
stages:
  - id: a
    capability: analyze.modes
    input: { system: { from: b } }
  - id: b
    capability: analyze.modes
    input: { system: { from: a } }
)");
    FAIL() << "a cyclic pipeline was accepted";
  } catch (const std::exception& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("cycle"), std::string::npos) << message;
    // "there is a cycle" is not actionable; the stages must be named.
    EXPECT_NE(message.find("a"), std::string::npos) << message;
    EXPECT_NE(message.find("b"), std::string::npos) << message;
  }
}

TEST(PipelineParse, RejectsAReferenceToAStageThatDoesNotExist) {
  EXPECT_THROW((void)parse_pipeline(R"(
version: 1
stages:
  - id: only
    capability: analyze.modes
    input: { system: { from: ghost } }
)"),
               std::runtime_error);
}

TEST(PipelineParse, RejectsSelfReference) {
  EXPECT_THROW((void)parse_pipeline(R"(
version: 1
stages:
  - id: loop
    capability: analyze.modes
    input: { system: { from: loop } }
)"),
               std::runtime_error);
}

TEST(PipelineParse, RejectsDuplicateStageIds) {
  EXPECT_THROW((void)parse_pipeline(R"(
version: 1
stages:
  - id: same
    capability: model.linear.statespace
    input: { path: a.yaml }
  - id: same
    capability: model.linear.statespace
    input: { path: b.yaml }
)"),
               std::runtime_error);
}

TEST(PipelineParse, RejectsAnUnsupportedVersion) {
  // A future format must fail loudly on an old binary rather than being
  // half-understood.
  try {
    (void)parse_pipeline("version: 2\nstages: [{id: a, capability: x}]\n");
    FAIL() << "a version 2 pipeline was accepted by a build that understands version 1";
  } catch (const std::exception& error) {
    EXPECT_NE(std::string(error.what()).find("version"), std::string::npos);
  }
}

TEST(PipelineParse, RejectsStructurallyIncompleteDocuments) {
  EXPECT_THROW((void)parse_pipeline("stages: []\n"), std::runtime_error);  // no version
  EXPECT_THROW((void)parse_pipeline("version: 1\n"), std::runtime_error);  // no stages
  EXPECT_THROW((void)parse_pipeline("version: 1\nstages: []\n"), std::runtime_error);
  EXPECT_THROW((void)parse_pipeline("version: 1\nstages: [{capability: x}]\n"),
               std::runtime_error);  // no id
  EXPECT_THROW((void)parse_pipeline("version: 1\nstages: [{id: a}]\n"),
               std::runtime_error);  // no capability
  EXPECT_THROW((void)parse_pipeline("::not yaml::\n"), std::runtime_error);
}

TEST(Registry, DispatchesByPrefix) {
  // Adding a whole vertical must need no dispatcher change: capabilities are
  // found by their dotted prefix. This is the property the plugin ABI will
  // depend on, asserted rather than assumed.
  const auto& registry = galata::pipeline::builtin_registry();
  EXPECT_FALSE(registry.with_prefix("analyze").empty());
  EXPECT_FALSE(registry.with_prefix("model").empty());
  EXPECT_FALSE(registry.with_prefix("report").empty());
  EXPECT_TRUE(registry.with_prefix("nonexistent").empty());

  // A prefix must match on a dot boundary, never on a bare string prefix:
  // "model" must not match a hypothetical "modelling.something".
  for (const auto* capability : registry.with_prefix("model")) {
    EXPECT_EQ(capability->id.compare(0, 6, "model."), 0) << capability->id;
  }
}

TEST(Registry, EveryCapabilityDeclaresItsStateAndWhatItProduces) {
  // The README's status table is generated from this. A capability with an
  // empty summary or an unset state would produce a row that says nothing.
  for (const auto* capability : galata::pipeline::builtin_registry().all()) {
    EXPECT_FALSE(capability->id.empty());
    EXPECT_FALSE(capability->summary.empty()) << capability->id;
    EXPECT_FALSE(capability->produces.empty()) << capability->id;
    EXPECT_TRUE(capability->run) << capability->id;
  }
}

TEST(Registry, ListingOrderIsStable) {
  const auto first = galata::pipeline::builtin_registry().all();
  const auto second = galata::pipeline::builtin_registry().all();
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_EQ(first[i]->id, second[i]->id);
  }
}

TEST(Registry, RefusesDuplicateRegistration) {
  galata::pipeline::Registry registry;
  galata::pipeline::Capability capability{
      "test.thing",
      "does a thing",
      "thing",
      galata::pipeline::Capability::State::Stub,
      [](const galata::pipeline::StageContext&) { return galata::pipeline::Artifact{}; }};
  registry.add(capability);
  EXPECT_THROW(registry.add(capability), std::invalid_argument);
}

}  // namespace
