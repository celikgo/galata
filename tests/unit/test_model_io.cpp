// SPDX-License-Identifier: Apache-2.0
// MC10/MC20/MC23/MC24/MC26: source-format conformance, independently specified
// binary64 encodings and refusal boundaries. No aircraft validation claim.
#include "galata/modeling/model.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <locale>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace galata::modeling;

constexpr std::string_view kMinimal = R"(schema: galata.model.v1
profile: continuous-scalar.v1
blocks:
  - id: source
    kind: constant
    output: {dimension: [0, 0, 0, 0, 0, 0, 0, 0], frame: none}
    value: 0.25
  - id: result
    kind: output
    output: {dimension: [0, 0, 0, 0, 0, 0, 0, 0], frame: none}
connections:
  - {source: source, target: result, input: 0}
)";

constexpr std::string_view kAffine = R"(schema: galata.model.v1
profile: continuous-scalar.v1
blocks:
  - id: bias
    kind: constant
    output: {dimension: [1, 0, -1, 0, 0, 0, 0, 0], frame: body}
    value: 1
  - id: decay
    kind: gain
    output: {dimension: [1, 0, -1, 0, 0, 0, 0, 0], frame: body}
    coefficient: {value: 1, dimension: [0, 0, -1, 0, 0, 0, 0, 0]}
  - id: derivative
    kind: sum
    output: {dimension: [1, 0, -1, 0, 0, 0, 0, 0], frame: body}
    signs: [1, -1]
  - id: state
    kind: integrator
    output: {dimension: [1, 0, 0, 0, 0, 0, 0, 0], frame: body}
    initial_value: 0.5
  - id: result
    kind: output
    output: {dimension: [1, 0, 0, 0, 0, 0, 0, 0], frame: body}
connections:
  - {source: bias, target: derivative, input: 0}
  - {source: decay, target: derivative, input: 1}
  - {source: state, target: decay, input: 0}
  - {source: derivative, target: state, input: 0}
  - {source: state, target: result, input: 0}
)";

std::string replaced(std::string_view text, std::string_view from, std::string_view to) {
  std::string result(text);
  const auto offset = result.find(from);
  if (offset == std::string::npos) {
    throw std::logic_error("test fixture replacement did not match");
  }
  result.replace(offset, from.size(), to);
  return result;
}

void expect_error(const std::function<void()>& action, ErrorCode expected) {
  try {
    action();
    FAIL() << "expected typed model error";
  } catch (const Error& error) {
    EXPECT_EQ(error.code(), expected) << error.what();
    EXPECT_FALSE(std::string(error.what()).empty());
  }
}

void expect_document_error(std::string_view text) {
  expect_error([text] { (void)parse_model_yaml(text); }, ErrorCode::InvalidDocument);
  expect_error([text] { (void)parse_model_draft_yaml(text); }, ErrorCode::InvalidDocument);
}

class TestFloatingEnvironment final {
 public:
  TestFloatingEnvironment() {
    if (std::feholdexcept(&saved_) != 0) {
      throw std::runtime_error("cannot save the test floating-point environment");
    }
  }

  ~TestFloatingEnvironment() {
    (void)std::fesetenv(&saved_);
  }

  void restore() {
    if (std::fesetenv(&saved_) != 0) {
      throw std::runtime_error("cannot restore the test floating-point environment");
    }
  }

 private:
  std::fenv_t saved_{};
};

TEST(ModelIo, ParsesAllFiveKindsWithExplicitDimensionsFramesAndOrderedPorts) {
  const Model model = parse_model_yaml(kAffine);
  ASSERT_EQ(model.blocks.size(), 5U);
  ASSERT_EQ(model.connections.size(), 5U);
  EXPECT_EQ(model.schema, kSchema);
  EXPECT_EQ(model.profile, kProfile);
  EXPECT_EQ(std::get<Constant>(model.blocks[0].parameters).value, 1.0);
  EXPECT_EQ(std::get<Gain>(model.blocks[1].parameters).dimension,
            (Dimension{0, 0, -1, 0, 0, 0, 0, 0}));
  EXPECT_EQ(std::get<Sum>(model.blocks[2].parameters).signs, (std::vector<int>{1, -1}));
  EXPECT_EQ(std::get<Integrator>(model.blocks[3].parameters).initial_value, 0.5);
  EXPECT_TRUE(std::holds_alternative<Output>(model.blocks[4].parameters));
  EXPECT_EQ(model.blocks[3].output.dimension, (Dimension{1, 0, 0, 0, 0, 0, 0, 0}));
  EXPECT_EQ(model.blocks[3].output.frame, Frame::Body);
  EXPECT_EQ(model.connections[1].input, 1U);
  EXPECT_EQ(model.connections[1].target, "derivative");
}

TEST(ModelDraft, AcceptsIncompleteGraphsWithoutGrantingExecutableOrCanonicalSemantics) {
  // ADR-0012 permits these editor states; every executable entry point retains
  // the model.v1 graph contract. Input indices remain syntactically size_t.
  struct Case {
    std::string source;
    ErrorCode executable_error;
  };

  const std::vector<Case> cases = {
      {"schema: galata.model.v1\nprofile: continuous-scalar.v1\nblocks: []\nconnections: []\n",
       ErrorCode::InvalidModel},
      {replaced(kMinimal,
                "kind: output",
                "kind: gain\n    coefficient: {value: 1, dimension: [0, 0, 0, 0, 0, 0, 0, 0]}"),
       ErrorCode::InvalidModel},
      {replaced(kMinimal,
                "connections:\n  - {source: source, target: result, input: 0}",
                "connections: []"),
       ErrorCode::InvalidModel},
      {replaced(kMinimal, "source: source", "source: deleted_source"), ErrorCode::InvalidModel},
      {replaced(kMinimal, "target: result", "target: future_target"), ErrorCode::InvalidModel},
      {replaced(kMinimal, "input: 0", "input: 4096"), ErrorCode::InvalidModel},
      {std::string(kMinimal) + "  - {source: source, target: result, input: 0}\n",
       ErrorCode::InvalidModel},
      {replaced(kMinimal, "frame: none", "frame: body"), ErrorCode::TypeMismatch},
      {replaced(kMinimal, "source: source", "source: result"), ErrorCode::AlgebraicLoop}};
  for (const auto& value : cases) {
    SCOPED_TRACE(value.source);
    const Model draft = parse_model_draft_yaml(value.source);
    EXPECT_NO_THROW(validate_model_draft(draft));
    expect_error([&] { validate_model(draft); }, value.executable_error);
    expect_error([&] { (void)parse_model_yaml(value.source); }, value.executable_error);
    expect_error([&] { (void)compile_model(draft); }, value.executable_error);
    expect_error([&] { (void)write_model_yaml(draft); }, value.executable_error);
    expect_error([&] { (void)canonical_model(draft); }, value.executable_error);
  }
}

TEST(ModelDraft, ValidDraftCompilesWithTheSameIdentityAsTheOrdinaryParser) {
  const auto draft = parse_model_draft_yaml(kAffine);
  EXPECT_EQ(compile_model(draft).semantic_sha256(),
            compile_model(parse_model_yaml(kAffine)).semantic_sha256());
  EXPECT_EQ(write_model_yaml(draft), write_model_yaml(parse_model_yaml(kAffine)));
}

TEST(ModelDraft, RefusesInvalidMetadataEvenWhenConnectionsAreIncomplete) {
  const Model valid = parse_model_yaml(kAffine);
  const auto refused = [&](const std::function<void(Model&)>& mutate,
                           ErrorCode code = ErrorCode::InvalidModel) {
    Model draft = valid;
    draft.connections.clear();
    mutate(draft);
    expect_error([&] { validate_model_draft(draft); }, code);
  };
  refused([](Model& model) { model.schema = "galata.model.v2"; });
  refused([](Model& model) { model.profile = "sampled-scalar.v1"; });
  refused([](Model& model) { model.blocks[1].id = model.blocks[0].id; });
  refused([](Model& model) { model.blocks[0].id = "not an ID"; });
  refused([](Model& model) { model.blocks[0].output.dimension[0] = 17; });
  refused([](Model& model) { model.blocks[0].output.frame = static_cast<Frame>(99); });
  refused([](Model& model) {
    std::get<Constant>(model.blocks[0].parameters).value = std::numeric_limits<double>::infinity();
  });
  refused([](Model& model) {
    std::get<Gain>(model.blocks[1].parameters).value = std::numeric_limits<double>::quiet_NaN();
  });
  refused([](Model& model) { std::get<Gain>(model.blocks[1].parameters).dimension[2] = -17; });
  refused([](Model& model) { std::get<Sum>(model.blocks[2].parameters).signs = {}; });
  refused([](Model& model) { std::get<Sum>(model.blocks[2].parameters).signs = {1, 0}; });
  refused([](Model& model) {
    std::get<Integrator>(model.blocks[3].parameters).initial_value =
        -std::numeric_limits<double>::infinity();
  });
  refused([](Model& model) { model.connections = {{"bad.id", "future_target", 0}}; });
  refused([](Model& model) { model.connections = {{"future_source", "", 0}}; });
  refused([](Model& model) { model.blocks.resize(kMaxBlocks + 1); }, ErrorCode::ResourceLimit);
  refused([](Model& model) { model.connections.resize(kMaxConnections + 1); },
          ErrorCode::ResourceLimit);
  refused(
      [](Model& model) {
        std::get<Sum>(model.blocks[2].parameters).signs.assign(kMaxSumInputs + 1, 1);
      },
      ErrorCode::ResourceLimit);
}

TEST(ModelDraft, ParserRetainsMetadataAndResourceRefusals) {
  for (const std::string& document :
       {replaced(kMinimal, "galata.model.v1", "galata.model.v2"),
        replaced(kMinimal, "continuous-scalar.v1", "continuous-vector.v1"),
        replaced(kMinimal, "id: result", "id: source"),
        replaced(kMinimal, "id: result", "id: 'bad.id'"),
        replaced(kMinimal, "source: source", "source: 'bad.id'"),
        replaced(kMinimal, "[0, 0, 0, 0, 0, 0, 0, 0]", "[17, 0, 0, 0, 0, 0, 0, 0]"),
        replaced(kAffine, "signs: [1, -1]", "signs: []"),
        replaced(kAffine, "signs: [1, -1]", "signs: [1, 0]")}) {
    SCOPED_TRACE(document);
    expect_error([&] { (void)parse_model_draft_yaml(document); }, ErrorCode::InvalidModel);
  }
  const std::string oversized(kMaxSourceBytes + 1, '[');
  expect_error([&] { (void)parse_model_draft_yaml(oversized); }, ErrorCode::ResourceLimit);
}

TEST(ModelIo, RejectsMalformedDuplicateAliasedTaggedAndMultipleDocuments) {
  const std::vector<std::string> documents = {
      "",
      "[]",
      "null",
      "{blocks: [}",
      std::string(kMinimal) + "schema: galata.model.v1\n",
      replaced(kMinimal, "    value: 0.25", "    value: 0.25\n    value: 0.5"),
      replaced(kMinimal, "frame: none}", "frame: none, frame: body}"),
      replaced(kMinimal, "{source: source,", "{source: source, source: source,"),
      replaced(kMinimal, "value: 0.25", "value: &number 0.25"),
      replaced(kMinimal, "value: 0.25", "value: *number"),
      replaced(kMinimal, "value: 0.25", "value: !!float 0.25"),
      replaced(kMinimal, "value: 0.25", "value: !custom 0.25"),
      replaced(kMinimal, "  - id: source", "  - &recursive {id: source, value: *recursive}"),
      std::string(kMinimal) + "---\n" + std::string(kMinimal),
      "? [compound, key]\n: value\n"};
  for (const std::string& document : documents) {
    SCOPED_TRACE(document);
    expect_document_error(document);
  }
}

TEST(ModelIo, RejectsUnknownFieldsUnsupportedBlocksAndIrrelevantParameters) {
  const std::vector<std::string> documents = {
      std::string(kMinimal) + "presentation: {}\n",
      std::string(kMinimal) + "clocks: []\n",
      replaced(kMinimal, "kind: constant", "kind: unit_delay"),
      replaced(kMinimal, "kind: constant", "kind: constant\n    sample_time: 0.1"),
      replaced(kMinimal, "kind: output", "kind: output\n    initial_value: 0"),
      replaced(kMinimal, "frame: none}", "frame: none, shape: [1]}"),
      replaced(kMinimal, "frame: none}", "frame: none, unit: dimensionless}"),
      replaced(kMinimal, "input: 0}", "input: 0, clock: continuous}"),
      replaced(kAffine, "kind: gain", "kind: gain\n    value: 1"),
      replaced(kAffine, "coefficient: {value: 1,", "coefficient: {scale: 1, value: 1,"),
      replaced(kAffine, "kind: sum", "kind: sum\n    coefficient: 1"),
      replaced(kAffine, "kind: integrator", "kind: integrator\n    signs: [1]")};
  for (const std::string& document : documents) {
    SCOPED_TRACE(document);
    expect_document_error(document);
  }
}

TEST(ModelIo, RequiresEveryEffectiveFieldAndExactScalarShapes) {
  const std::vector<std::string> documents = {
      replaced(kMinimal, "schema: galata.model.v1\n", ""),
      replaced(kMinimal, "profile: continuous-scalar.v1\n", ""),
      "schema: galata.model.v1\nprofile: continuous-scalar.v1\nblocks: []\n",
      replaced(kMinimal, "    kind: constant\n", ""),
      replaced(kMinimal, "  - id: source", "  - id: null"),
      replaced(kMinimal, "  - id: source", "  - id: [source]"),
      replaced(kMinimal, "    value: 0.25\n", ""),
      replaced(kMinimal, "value: 0.25", "value: [0.25]"),
      replaced(kMinimal, "frame: none", "frame: enu"),
      replaced(kMinimal, ", frame: none", ""),
      replaced(kMinimal, "dimension: [0, 0, 0, 0, 0, 0, 0, 0], ", ""),
      replaced(kMinimal, "[0, 0, 0, 0, 0, 0, 0, 0]", "[0, 0, 0]"),
      replaced(kMinimal, "[0, 0, 0, 0, 0, 0, 0, 0]", "dimensionless"),
      replaced(kMinimal, ", input: 0", ""),
      replaced(kMinimal, "input: 0", "input: -1"),
      replaced(kMinimal, "input: 0", "input: 0.0"),
      replaced(kMinimal, "input: 0", "input: 184467440737095516160"),
      replaced(kAffine, "coefficient: {value: 1,", "coefficient: {"),
      replaced(kAffine, "    initial_value: 0.5\n", ""),
      replaced(kAffine, "    signs: [1, -1]\n", ""),
      replaced(kAffine, "signs: [1, -1]", "signs: [true, -1]")};
  for (const std::string& document : documents) {
    SCOPED_TRACE(document);
    expect_document_error(document);
  }
  for (std::string_view exponent : {"0.5", "1e0", "true", "'0'", "999999999999999999999"}) {
    expect_document_error(replaced(kMinimal,
                                   "[0, 0, 0, 0, 0, 0, 0, 0]",
                                   "[" + std::string(exponent) + ", 0, 0, 0, 0, 0, 0, 0]"));
  }
}

TEST(ModelIo, RejectsNonFiniteNonDecimalAndOutOfRangeNumericTokens) {
  for (std::string_view number : {".nan",
                                  ".inf",
                                  "-.inf",
                                  "NaN",
                                  "Infinity",
                                  "true",
                                  "0x1p0",
                                  "0x10",
                                  "1_000",
                                  "1:20",
                                  "1e9999",
                                  "1e-9999",
                                  "'0.25'",
                                  "\"0.25\"",
                                  ".",
                                  "+",
                                  "1e",
                                  "null"}) {
    SCOPED_TRACE(number);
    expect_document_error(replaced(kMinimal, "value: 0.25", "value: " + std::string(number)));
  }
}

TEST(ModelIo, DecimalBoundarySpellingsHaveIndependentlySpecifiedBinary64Bits) {
  struct Case {
    std::string_view text;
    std::uint64_t bits;
  };

  // Binary64 sign/exponent/fraction encodings, not values emitted by Galata.
  const std::array<Case, 8> cases{{{"+1.25", 0x3ff4000000000000ULL},
                                   {".125", 0x3fc0000000000000ULL},
                                   {"1.", 0x3ff0000000000000ULL},
                                   {"-0", 0x8000000000000000ULL},
                                   {"4.9406564584124654e-324", 0x0000000000000001ULL},
                                   {"2.2250738585072014e-308", 0x0010000000000000ULL},
                                   {"1.7976931348623157e308", 0x7fefffffffffffffULL},
                                   {"-1.7976931348623157e308", 0xffefffffffffffffULL}}};
  for (const Case& value : cases) {
    SCOPED_TRACE(value.text);
    const auto model =
        parse_model_yaml(replaced(kMinimal, "value: 0.25", "value: " + std::string(value.text)));
    EXPECT_EQ(std::bit_cast<std::uint64_t>(std::get<Constant>(model.blocks[0].parameters).value),
              value.bits);
  }
}

TEST(ModelIo, WritesAndParsesExactFiniteParameterBitsIncludingSignedZeroAndSubnormals) {
  // Fixed bit corpus covers both signs, adjacent representable numbers, normal
  // boundaries, the smallest/largest subnormal and the finite range boundary.
  const std::array<std::uint64_t, 12> bits = {0,
                                              0x8000000000000000ULL,
                                              1,
                                              0x8000000000000001ULL,
                                              0x000fffffffffffffULL,
                                              0x0010000000000000ULL,
                                              0x3fb999999999999aULL,
                                              0x3fefffffffffffffULL,
                                              0x3ff0000000000000ULL,
                                              0x3ff0000000000001ULL,
                                              0x7fefffffffffffffULL,
                                              0xffefffffffffffffULL};
  for (std::uint64_t value : bits) {
    SCOPED_TRACE(value);
    Model model = parse_model_yaml(kAffine);
    std::get<Constant>(model.blocks[0].parameters).value = std::bit_cast<double>(value);
    std::get<Gain>(model.blocks[1].parameters).value = std::bit_cast<double>(value);
    std::get<Integrator>(model.blocks[3].parameters).initial_value = std::bit_cast<double>(value);
    const std::string source = write_model_yaml(model);
    const Model restored = parse_model_yaml(source);
    EXPECT_EQ(canonical_model(restored), canonical_model(model));
    EXPECT_EQ(write_model_yaml(restored), source);
    for (const Block& item : restored.blocks) {
      if (item.id == "bias") {
        EXPECT_EQ(std::bit_cast<std::uint64_t>(std::get<Constant>(item.parameters).value), value);
      } else if (item.id == "decay") {
        EXPECT_EQ(std::bit_cast<std::uint64_t>(std::get<Gain>(item.parameters).value), value);
      } else if (item.id == "state") {
        EXPECT_EQ(std::bit_cast<std::uint64_t>(std::get<Integrator>(item.parameters).initial_value),
                  value);
      }
    }
  }
}

TEST(ModelIo, DecimalConversionUsesNearestEvenAndRestoresCallerRoundingAndExceptionFlags) {
  struct Case {
    std::string_view text;
    std::uint64_t bits;
  };

  // Exact binary64 encodings include tie-to-even in both directions, tiny
  // values rounding to min-normal, and the explicit sign of a zero significand.
  const std::array<Case, 13> cases{
      {{"0.1", 0x3fb999999999999aULL},
       {"4.9406564584124654e-324", 0x0000000000000001ULL},
       {"-4.9406564584124654e-324", 0x8000000000000001ULL},
       {"2.2250738585072014e-308", 0x0010000000000000ULL},
       {"-2.2250738585072012e-308", 0x8010000000000000ULL},
       {"1.7976931348623157e308", 0x7fefffffffffffffULL},
       {"-1.7976931348623157e308", 0xffefffffffffffffULL},
       {"0", 0},
       {"-0", 0x8000000000000000ULL},
       {"-0.0e-9999", 0x8000000000000000ULL},
       {"0e9999", 0},
       {"1.00000000000000011102230246251565404236316680908203125", 0x3ff0000000000000ULL},
       {"1.00000000000000033306690738754696212708950042724609375", 0x3ff0000000000002ULL}}};
  for (int mode : {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO}) {
    SCOPED_TRACE(mode);
    for (const Case& value : cases) {
      SCOPED_TRACE(value.text);
      const std::string text =
          replaced(kMinimal, "value: 0.25", "value: " + std::string(value.text));
      TestFloatingEnvironment environment;
      ASSERT_EQ(std::fesetround(mode), 0);
      ASSERT_EQ(std::feraiseexcept(FE_DIVBYZERO | FE_INVALID), 0);
      const int original_flags = std::fetestexcept(FE_ALL_EXCEPT);
      const Model model = parse_model_yaml(text);
      const auto bits =
          std::bit_cast<std::uint64_t>(std::get<Constant>(model.blocks[0].parameters).value);
      const int restored_mode = std::fegetround();
      const int restored_flags = std::fetestexcept(FE_ALL_EXCEPT);
      environment.restore();
      EXPECT_EQ(bits, value.bits);
      EXPECT_EQ(restored_mode, mode);
      EXPECT_EQ(restored_flags, original_flags);
    }
  }
}

TEST(ModelIo, DecimalRefusalsRestoreTheCallerEnvironmentWithoutClampingOverflowOrUnderflow) {
  for (int mode : {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO}) {
    SCOPED_TRACE(mode);
    for (std::string_view number : {"1e9999",
                                    "-1e9999",
                                    "1.7976931348623159e308",
                                    "1e-9999",
                                    "-1e-9999",
                                    "1e-324",
                                    "1.0junk",
                                    "1e",
                                    "'0.1'"}) {
      SCOPED_TRACE(number);
      const std::string text = replaced(kMinimal, "value: 0.25", "value: " + std::string(number));
      TestFloatingEnvironment environment;
      ASSERT_EQ(std::fesetround(mode), 0);
      ASSERT_EQ(std::feraiseexcept(FE_DIVBYZERO | FE_INVALID), 0);
      const int original_flags = std::fetestexcept(FE_ALL_EXCEPT);
      bool refused = false;
      ErrorCode code = ErrorCode::InvalidModel;
      try {
        (void)parse_model_yaml(text);
      } catch (const Error& error) {
        refused = true;
        code = error.code();
      }
      const int restored_mode = std::fegetround();
      const int restored_flags = std::fetestexcept(FE_ALL_EXCEPT);
      environment.restore();
      EXPECT_TRUE(refused);
      EXPECT_EQ(code, ErrorCode::InvalidDocument);
      EXPECT_EQ(restored_mode, mode);
      EXPECT_EQ(restored_flags, original_flags);
    }
  }
}

TEST(ModelIo, CanonicalIdentityIgnoresDeclarationsAndPresentationFreeSourceSpelling) {
  Model model = parse_model_yaml(kAffine);
  const std::string canonical = canonical_model(model);
  const std::string source = write_model_yaml(model);
  std::reverse(model.blocks.begin(), model.blocks.end());
  std::reverse(model.connections.begin(), model.connections.end());
  EXPECT_EQ(canonical_model(model), canonical);
  EXPECT_EQ(write_model_yaml(model), source);
  EXPECT_EQ(
      canonical_model(parse_model_yaml("# layout belongs to a future project document\n"
                                       + replaced(kAffine, "value: 1\n", "value: +1.000e0\n"))),
      canonical);
}

TEST(ModelIo, WriterQuotesValidIdentifiersThatYamlWouldInterpretAsNull) {
  for (std::string_view id : {"null", "Null", "NULL"}) {
    Model model = parse_model_yaml(kMinimal);
    model.blocks[0].id = id;
    model.connections[0].source = id;
    EXPECT_EQ(canonical_model(parse_model_yaml(write_model_yaml(model))), canonical_model(model));
    model = parse_model_yaml(kMinimal);
    model.blocks[1].id = id;
    model.connections[0].target = id;
    EXPECT_EQ(canonical_model(parse_model_yaml(write_model_yaml(model))), canonical_model(model));
  }
}

TEST(ModelIo, CanonicalIdentityContainsExactBitsDimensionsFramesSignsAndConnections) {
  const Model model = parse_model_yaml(kAffine);
  const std::string original = canonical_model(model);
  Model changed = model;
  std::get<Constant>(changed.blocks[0].parameters).value = std::nextafter(1.0, 2.0);
  EXPECT_NE(canonical_model(changed), original);
  EXPECT_NE(canonical_model(changed).find("16:3ff0000000000001\n"), std::string::npos);
  changed = model;
  std::get<Gain>(changed.blocks[1].parameters).value = 2.0;
  EXPECT_NE(canonical_model(changed), original);
  changed = model;
  std::get<Integrator>(changed.blocks[3].parameters).initial_value = 0.25;
  EXPECT_NE(canonical_model(changed), original);
  changed = model;
  std::get<Sum>(changed.blocks[2].parameters).signs = {-1, 1};
  EXPECT_NE(canonical_model(changed), original);
  changed = model;
  std::swap(changed.connections[0].input, changed.connections[1].input);
  EXPECT_NE(canonical_model(changed), original);
  changed = model;
  for (Block& item : changed.blocks) {
    item.output.frame = Frame::Ned;
  }
  EXPECT_NE(canonical_model(changed), original);
  changed = model;
  for (Block& item : changed.blocks) {
    item.output.dimension[0] = 2;
  }
  EXPECT_NE(canonical_model(changed), original);
  changed = model;
  std::get<Integrator>(changed.blocks[3].parameters).initial_value = 0.0;
  const std::string positive_zero = canonical_model(changed);
  std::get<Integrator>(changed.blocks[3].parameters).initial_value = -0.0;
  EXPECT_NE(canonical_model(changed), positive_zero);
  EXPECT_NE(canonical_model(changed).find("16:8000000000000000\n"), std::string::npos);
}

TEST(ModelIo, CanonicalVersionHasAHandSpecifiedByteContract) {
  // The specified canonical grammar uses length-prefixed ASCII fields and
  // network-order hexadecimal binary64 bits. 0.25 has exponent -2 and zero
  // fraction, hence the independent 3fd0000000000000 encoding below.
  constexpr std::string_view expected =
      "galata.model.canonical.v1\n"
      "15:galata.model.v1\n20:continuous-scalar.v1\n6:blocks\n1:2\n"
      "6:result\n6:output\n"
      "1:0\n1:0\n1:0\n1:0\n1:0\n1:0\n1:0\n1:0\n4:none\n"
      "6:source\n8:constant\n"
      "1:0\n1:0\n1:0\n1:0\n1:0\n1:0\n1:0\n1:0\n4:none\n"
      "16:3fd0000000000000\n"
      "11:connections\n1:1\n6:source\n6:result\n1:0\n";
  EXPECT_EQ(canonical_model(parse_model_yaml(kMinimal)), expected);
}

TEST(ModelIo, PreservesSemanticValidationErrorsAndRejectsInvalidDirectModels) {
  Model model = parse_model_yaml(kMinimal);
  model.blocks[1].output.frame = Frame::Body;
  expect_error([&model] { validate_model(model); }, ErrorCode::TypeMismatch);
  expect_error([&model] { (void)write_model_yaml(model); }, ErrorCode::TypeMismatch);
  expect_error([&model] { (void)canonical_model(model); }, ErrorCode::TypeMismatch);
  const auto invalid_text =
      replaced(kMinimal,
               "kind: output\n    output: {dimension: [0, 0, 0, 0, 0, 0, 0, 0], frame: none}",
               "kind: output\n    output: {dimension: [0, 0, 0, 0, 0, 0, 0, 0], frame: body}");
  expect_error([&invalid_text] { (void)parse_model_yaml(invalid_text); }, ErrorCode::TypeMismatch);
  for (double value :
       {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    model = parse_model_yaml(kMinimal);
    std::get<Constant>(model.blocks[0].parameters).value = value;
    expect_error([&model] { (void)write_model_yaml(model); }, ErrorCode::InvalidModel);
    expect_error([&model] { (void)canonical_model(model); }, ErrorCode::InvalidModel);
  }
}

TEST(ModelIo, RefusesOversizedInputBeforeParsingAndBoundsNestingAndCollections) {
  std::string oversized(kMaxSourceBytes + 1, '[');
  expect_error([&oversized] { (void)parse_model_yaml(oversized); }, ErrorCode::ResourceLimit);
  std::string boundary(kMinimal);
  boundary += '#';
  boundary.append(kMaxSourceBytes - boundary.size(), 'x');
  EXPECT_NO_THROW((void)parse_model_yaml(boundary));
  expect_document_error(std::string(65, '[') + "0" + std::string(65, ']'));
  std::string too_many_blocks = "schema: galata.model.v1\nprofile: continuous-scalar.v1\nblocks:\n";
  for (std::size_t i = 0; i <= kMaxBlocks; ++i) {
    too_many_blocks += "  - {}\n";
  }
  too_many_blocks += "connections: []\n";
  expect_error([&too_many_blocks] { (void)parse_model_yaml(too_many_blocks); },
               ErrorCode::ResourceLimit);
  std::string too_many_connections =
      replaced(kMinimal, "  - {source: source, target: result, input: 0}\n", "");
  for (std::size_t i = 0; i <= kMaxConnections; ++i) {
    too_many_connections += "  - {}\n";
  }
  expect_error([&too_many_connections] { (void)parse_model_yaml(too_many_connections); },
               ErrorCode::ResourceLimit);
  std::string signs = "[";
  for (std::size_t i = 0; i <= kMaxSumInputs; ++i) {
    if (i != 0) {
      signs += ',';
    }
    signs += '1';
  }
  signs += ']';
  const std::string too_many_signs = replaced(kAffine, "[1, -1]", signs);
  expect_error([&too_many_signs] { (void)parse_model_yaml(too_many_signs); },
               ErrorCode::ResourceLimit);
}

class CommaDecimal final : public std::numpunct<char> {
 protected:
  char do_decimal_point() const override {
    return ',';
  }

  char do_thousands_sep() const override {
    return '.';
  }

  std::string do_grouping() const override {
    return "\3";
  }
};

TEST(ModelIo, ParsingAndBothSerializationsIgnoreTheEmbeddingProcessCppLocale) {
  const Model model = parse_model_yaml(kAffine);
  const std::string source = write_model_yaml(model);
  const std::string canonical = canonical_model(model);

  struct RestoreLocale {
    std::locale previous;

    ~RestoreLocale() {
      std::locale::global(previous);
    }
  } restore{std::locale()};

  std::locale::global(std::locale(restore.previous, new CommaDecimal));
  EXPECT_EQ(write_model_yaml(model), source);
  EXPECT_EQ(canonical_model(model), canonical);
  EXPECT_EQ(canonical_model(parse_model_yaml(source)), canonical);
}
}  // namespace
