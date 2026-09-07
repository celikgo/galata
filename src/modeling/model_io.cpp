// SPDX-License-Identifier: Apache-2.0
// Continuous scalar source adapter; see ADR-0010 and MODEL_CONFORMANCE.md.
// WHAT THIS IS NOT: general YAML, implicit unit conversion, an editor document,
// or an interchange format for sampled, hybrid or arbitrary executable blocks.
#include "galata/modeling/model.hpp"

#include "../io/strict_yaml.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cfenv>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>

namespace galata::modeling {
namespace {

[[noreturn]] void invalid(const std::string& path, const std::string& message) {
  throw Error(ErrorCode::InvalidDocument, path + ": " + message);
}

YAML::Node required(const YAML::Node& node, const std::string& key, const std::string& path) {
  const YAML::Node value = node[key];
  if (!value || value.IsNull()) {
    invalid(path, "missing required '" + key + "'");
  }
  return value;
}

std::string scalar(const YAML::Node& node, const std::string& path) {
  if (!node.IsScalar()) {
    invalid(path, "expected a scalar");
  }
  return node.Scalar();
}

void sequence(const YAML::Node& node, const std::string& path, std::size_t maximum) {
  if (!node.IsSequence()) {
    invalid(path, "expected a sequence");
  }
  if (node.size() > maximum) {
    throw Error(ErrorCode::ResourceLimit, path + ": collection exceeds its declared limit");
  }
}

// The field contract supplies the numeric type: YAML booleans, hexadecimal,
// sexagesimal forms and quoted strings are not alternate numeric spellings.
// Decimal syntax is [+-]?(digits[.digits*]|.digits+)([eE][+-]?digits+)? .
bool decimal(std::string_view text) {
  std::size_t offset = 0;
  if (!text.empty() && (text.front() == '+' || text.front() == '-')) {
    ++offset;
  }
  const auto digits = [&text, &offset]() {
    const std::size_t begin = offset;
    while (offset < text.size() && text[offset] >= '0' && text[offset] <= '9') {
      ++offset;
    }
    return offset != begin;
  };
  const bool whole = digits();
  bool fraction = false;
  if (offset < text.size() && text[offset] == '.') {
    ++offset;
    fraction = digits();
  }
  if (!whole && !fraction) {
    return false;
  }
  if (offset < text.size() && (text[offset] == 'e' || text[offset] == 'E')) {
    ++offset;
    if (offset < text.size() && (text[offset] == '+' || text[offset] == '-')) {
      ++offset;
    }
    if (!digits()) {
      return false;
    }
  }
  return offset == text.size();
}

// Floating from_chars requires newer libc++/macOS runtimes than the existing
// macOS CI environment (libc++ C++17 status, P0067R5). Classic-locale num_get is
// portable, but its strtod conversion follows the caller's rounding mode and
// may raise floating exceptions. Keep nearest-even source semantics without
// changing the embedding thread's saved rounding mode or exception flags.
class DecimalEnvironment final {
 public:
  explicit DecimalEnvironment(const std::string& path) {
    if (std::feholdexcept(&saved_) != 0) {
      invalid(path, "cannot save the floating-point environment for decimal conversion");
    }
    active_ = true;
    if (std::fesetround(FE_TONEAREST) != 0) {
      restore(path);
      invalid(path, "cannot select nearest-even decimal conversion");
    }
  }

  ~DecimalEnvironment() {
    if (active_) {
      (void)std::fesetenv(&saved_);
    }
  }

  DecimalEnvironment(const DecimalEnvironment&) = delete;
  DecimalEnvironment& operator=(const DecimalEnvironment&) = delete;

  void restore(const std::string& path) {
    if (std::fesetenv(&saved_) != 0) {
      invalid(path, "cannot restore the floating-point environment after decimal conversion");
    }
    active_ = false;
  }

 private:
  std::fenv_t saved_{};
  bool active_ = false;
};

double number(const YAML::Node& node, const std::string& path) {
  const std::string text = scalar(node, path);
  if (node.Tag() == "!" || !decimal(text)) {
    invalid(path, "expected an unquoted finite decimal number");
  }
  DecimalEnvironment environment(path);
  std::istringstream stream(text);
  stream.imbue(std::locale::classic());
  double value = 0.0;
  stream >> value;
  const bool complete = stream.rdbuf()->sgetc() == std::char_traits<char>::eof();
  // libc++ sets failbit on ERANGE even when strtod returned a representable
  // subnormal, or rounded a tiny value to exactly min-normal. The strict
  // grammar and complete consumption rule out syntax failure here. Accept
  // only that tiny nonzero range result; normal-range failure/overflow refuses.
  const bool representable_underflow =
      value != 0.0 && std::abs(value) <= std::numeric_limits<double>::min();
  // libstdc++ can instead report success when a nonzero decimal underflows to
  // zero. Look only at significand digits: the exponent in -0e-9999 is not a
  // nonzero significand, and its signed zero must survive a round trip.
  const auto significand_end = text.find_first_of("eE");
  const std::string_view significand(
      text.data(), significand_end == std::string::npos ? text.size() : significand_end);
  const bool nonzero_significand =
      std::any_of(significand.begin(), significand.end(), [](char digit) {
        return digit >= '1' && digit <= '9';
      });
  if (!complete || stream.bad() || (stream.fail() && !representable_underflow)
      || !std::isfinite(value) || (value == 0.0 && nonzero_significand)) {
    invalid(path, "decimal number is outside finite binary64 range");
  }
  environment.restore(path);
  return value;
}

template <typename Integer>
Integer integer(const YAML::Node& node, const std::string& path) {
  const std::string text = scalar(node, path);
  if (node.Tag() == "!") {
    invalid(path, "expected an unquoted decimal integer");
  }
  Integer value{};
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value, 10);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    invalid(path, "expected a representable decimal integer");
  }
  return value;
}

Dimension dimension(const YAML::Node& node, const std::string& path) {
  if (!node.IsSequence() || node.size() != Dimension{}.size()) {
    invalid(path, "dimension requires exactly eight integer SI and angle exponents");
  }
  Dimension value{};
  for (std::size_t i = 0; i < value.size(); ++i) {
    value[i] = integer<int>(node[i], path + "[" + std::to_string(i) + "]");
  }
  return value;
}

SignalType signal_type(const YAML::Node& node, const std::string& path) {
  io::yaml_keys(node, path, {"dimension", "frame"});
  SignalType result;
  result.dimension = dimension(required(node, "dimension", path), path + ".dimension");
  const std::string frame = scalar(required(node, "frame", path), path + ".frame");
  if (frame == "none") {
    result.frame = Frame::None;
  } else if (frame == "body") {
    result.frame = Frame::Body;
  } else if (frame == "ned") {
    result.frame = Frame::Ned;
  } else {
    invalid(path + ".frame", "unsupported frame '" + frame + "'");
  }
  return result;
}

Block block(const YAML::Node& node, const std::string& path, std::string_view profile) {
  if (!node.IsMap()) {
    invalid(path, "expected a block map");
  }
  Block result;
  result.id = scalar(required(node, "id", path), path + ".id");
  const std::string kind = scalar(required(node, "kind", path), path + ".kind");
  const std::string location = path + " ('" + result.id + "')";
  result.output = signal_type(required(node, "output", location), location + ".output");
  if (kind == "constant") {
    io::yaml_keys(node, location, {"id", "kind", "output", "value"});
    result.parameters = Constant{number(required(node, "value", location), location + ".value")};
  } else if (kind == "gain") {
    io::yaml_keys(node, location, {"id", "kind", "output", "coefficient"});
    const YAML::Node coefficient = required(node, "coefficient", location);
    const std::string coefficient_path = location + ".coefficient";
    io::yaml_keys(coefficient, coefficient_path, {"value", "dimension"});
    result.parameters =
        Gain{number(required(coefficient, "value", coefficient_path), coefficient_path + ".value"),
             dimension(required(coefficient, "dimension", coefficient_path),
                       coefficient_path + ".dimension")};
  } else if (kind == "sum") {
    io::yaml_keys(node, location, {"id", "kind", "output", "signs"});
    const YAML::Node signs = required(node, "signs", location);
    sequence(signs, location + ".signs", kMaxSumInputs);
    Sum sum;
    sum.signs.reserve(signs.size());
    for (std::size_t i = 0; i < signs.size(); ++i) {
      sum.signs.push_back(integer<int>(signs[i], location + ".signs[" + std::to_string(i) + "]"));
    }
    result.parameters = std::move(sum);
  } else if (kind == "integrator") {
    io::yaml_keys(node, location, {"id", "kind", "output", "initial_value"});
    result.parameters =
        Integrator{number(required(node, "initial_value", location), location + ".initial_value")};
  } else if (kind == "output") {
    io::yaml_keys(node, location, {"id", "kind", "output"});
    result.parameters = Output{};
  } else if (kind == "linear_combination" && profile == kLinearProfile) {
    io::yaml_keys(node, location, {"id", "kind", "output", "terms"});
    const YAML::Node terms = required(node, "terms", location);
    sequence(terms, location + ".terms", kMaxLinearTerms);
    LinearCombination combination;
    combination.terms.reserve(terms.size());
    for (std::size_t i = 0; i < terms.size(); ++i) {
      const YAML::Node term = terms[i];
      const std::string term_path = location + ".terms[" + std::to_string(i) + "]";
      io::yaml_keys(term, term_path, {"input", "coefficient"});
      const YAML::Node coefficient = required(term, "coefficient", term_path);
      const std::string coefficient_path = term_path + ".coefficient";
      io::yaml_keys(coefficient, coefficient_path, {"value", "dimension"});
      combination.terms.push_back(
          {signal_type(required(term, "input", term_path), term_path + ".input"),
           Gain{number(required(coefficient, "value", coefficient_path),
                       coefficient_path + ".value"),
                dimension(required(coefficient, "dimension", coefficient_path),
                          coefficient_path + ".dimension")}});
    }
    result.parameters = std::move(combination);
  } else {
    invalid(location + ".kind", "unsupported block '" + kind + "' in " + std::string(profile));
  }
  return result;
}

template <typename Integer>
std::string integer_text(Integer value) {
  std::array<char, 64> buffer{};
  const auto result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
  if (result.ec != std::errc{}) {
    throw Error(ErrorCode::InvalidModel, "integer cannot be serialized");
  }
  return {buffer.data(), result.ptr};
}

std::string number_text(double value) {
  std::array<char, 64> buffer{};
  const auto result = std::to_chars(buffer.data(),
                                    buffer.data() + buffer.size(),
                                    value,
                                    std::chars_format::general,
                                    std::numeric_limits<double>::max_digits10);
  if (result.ec != std::errc{}) {
    throw Error(ErrorCode::InvalidModel, "binary64 value cannot be serialized");
  }
  return {buffer.data(), result.ptr};
}

std::string_view frame_name(Frame frame) {
  switch (frame) {
    case Frame::None:
      return "none";
    case Frame::Body:
      return "body";
    case Frame::Ned:
      return "ned";
  }
  throw Error(ErrorCode::InvalidModel, "unsupported frame cannot be serialized");
}

std::string_view block_kind(const Block& value) {
  if (std::holds_alternative<Constant>(value.parameters)) {
    return "constant";
  }
  if (std::holds_alternative<Gain>(value.parameters)) {
    return "gain";
  }
  if (std::holds_alternative<Sum>(value.parameters)) {
    return "sum";
  }
  if (std::holds_alternative<Integrator>(value.parameters)) {
    return "integrator";
  }
  if (std::holds_alternative<LinearCombination>(value.parameters)) {
    return "linear_combination";
  }
  return "output";
}

std::vector<const Block*> ordered_blocks(const Model& model) {
  std::vector<const Block*> result;
  result.reserve(model.blocks.size());
  for (const Block& value : model.blocks) {
    result.push_back(&value);
  }
  std::sort(result.begin(), result.end(), [](const Block* first, const Block* second) {
    return first->id < second->id;
  });
  return result;
}

std::vector<Connection> ordered_connections(const Model& model) {
  auto result = model.connections;
  std::sort(result.begin(), result.end(), [](const Connection& first, const Connection& second) {
    return std::tie(first.target, first.input, first.source)
           < std::tie(second.target, second.input, second.source);
  });
  return result;
}

template <typename Integers>
std::string integer_list(const Integers& values) {
  std::string result = "[";
  bool first = true;
  for (int value : values) {
    if (!first) {
      result += ", ";
    }
    result += integer_text(value);
    first = false;
  }
  return result + "]";
}

// All canonical fields are length-prefixed ASCII tokens. Neither delimiters in
// future field values nor native endianness can change record boundaries.
void token(std::string& target, std::string_view value) {
  target += integer_text(value.size()) + ":";
  target.append(value);
  target += '\n';
}

void dimension_tokens(std::string& target, const Dimension& value) {
  for (int exponent : value) {
    token(target, integer_text(exponent));
  }
}

std::string binary64_bits(double value) {
  static_assert(sizeof(double) == sizeof(std::uint64_t));
  static_assert(std::numeric_limits<double>::is_iec559
                && std::numeric_limits<double>::digits == 53);
  constexpr std::string_view digits = "0123456789abcdef";
  const auto bits = std::bit_cast<std::uint64_t>(value);
  std::string result(16, '0');
  for (std::size_t i = 0; i < result.size(); ++i) {
    const auto shift = static_cast<unsigned int>(4 * (15 - i));
    result[i] = digits[static_cast<std::size_t>((bits >> shift) & 0xfU)];
  }
  return result;
}

}  // namespace

Model parse_model_draft_yaml(std::string_view source) {
  if (source.size() > kMaxSourceBytes) {
    throw Error(ErrorCode::ResourceLimit, "model: source exceeds the one-MiB byte limit");
  }
  try {
    const YAML::Node root = io::load_yaml(std::string(source), "model");
    io::yaml_keys(root, "model", {"schema", "profile", "blocks", "connections"});
    Model result;
    result.schema = scalar(required(root, "schema", "model"), "model.schema");
    result.profile = scalar(required(root, "profile", "model"), "model.profile");
    const YAML::Node blocks = required(root, "blocks", "model");
    sequence(blocks, "model.blocks", kMaxBlocks);
    result.blocks.reserve(blocks.size());
    for (std::size_t i = 0; i < blocks.size(); ++i) {
      result.blocks.push_back(
          block(blocks[i], "model.blocks[" + std::to_string(i) + "]", result.profile));
    }
    const YAML::Node connections = required(root, "connections", "model");
    sequence(connections, "model.connections", kMaxConnections);
    result.connections.reserve(connections.size());
    for (std::size_t i = 0; i < connections.size(); ++i) {
      const YAML::Node node = connections[i];
      const std::string path = "model.connections[" + std::to_string(i) + "]";
      io::yaml_keys(node, path, {"source", "target", "input"});
      result.connections.push_back(
          Connection{scalar(required(node, "source", path), path + ".source"),
                     scalar(required(node, "target", path), path + ".target"),
                     integer<std::size_t>(required(node, "input", path), path + ".input")});
    }
    validate_model_draft(result);
    return result;
  } catch (const Error&) {
    throw;
  } catch (const std::exception& error) {
    throw Error(ErrorCode::InvalidDocument, std::string("model: ") + error.what());
  }
}

Model parse_model_yaml(std::string_view source) {
  auto model = parse_model_draft_yaml(source);
  validate_model(model);
  return model;
}

std::string write_model_yaml(const Model& model) {
  validate_model(model);
  std::string result = "schema: " + model.schema + "\nprofile: " + model.profile + "\nblocks:\n";
  for (const Block* value : ordered_blocks(model)) {
    // IDs are bounded ASCII, so quoting requires no escaping and keeps valid
    // identifiers such as "null" from becoming YAML null values on reparse.
    result += "  - id: \"" + value->id + "\"\n    kind: ";
    result += block_kind(*value);
    result += "\n    output:\n      dimension: " + integer_list(value->output.dimension);
    result += "\n      frame: ";
    result += frame_name(value->output.frame);
    result += '\n';
    if (const auto* constant = std::get_if<Constant>(&value->parameters)) {
      result += "    value: " + number_text(constant->value) + "\n";
    } else if (const auto* gain = std::get_if<Gain>(&value->parameters)) {
      result += "    coefficient:\n      value: " + number_text(gain->value);
      result += "\n      dimension: " + integer_list(gain->dimension) + "\n";
    } else if (const auto* sum = std::get_if<Sum>(&value->parameters)) {
      result += "    signs: " + integer_list(sum->signs) + "\n";
    } else if (const auto* integrator = std::get_if<Integrator>(&value->parameters)) {
      result += "    initial_value: " + number_text(integrator->initial_value) + "\n";
    } else if (const auto* combination = std::get_if<LinearCombination>(&value->parameters)) {
      result += "    terms:\n";
      for (const auto& term : combination->terms) {
        result += "      - input:\n          dimension: " + integer_list(term.input.dimension);
        result += "\n          frame: ";
        result += frame_name(term.input.frame);
        result += "\n        coefficient:\n          value: " + number_text(term.coefficient.value);
        result += "\n          dimension: " + integer_list(term.coefficient.dimension) + "\n";
      }
    }
  }
  const auto connections = ordered_connections(model);
  result += connections.empty() ? "connections: []\n" : "connections:\n";
  for (const Connection& connection : connections) {
    result +=
        "  - source: \"" + connection.source + "\"\n    target: \"" + connection.target + "\"";
    result += "\n    input: " + integer_text(connection.input) + "\n";
  }
  if (result.size() > kMaxSourceBytes) {
    throw Error(ErrorCode::ResourceLimit,
                "model: serialized source exceeds the one-MiB byte limit");
  }
  return result;
}

std::string canonical_model(const Model& model) {
  validate_model(model);
  std::string result = "galata.model.canonical.v1\n";
  token(result, model.schema);
  token(result, model.profile);
  token(result, "blocks");
  token(result, integer_text(model.blocks.size()));
  for (const Block* value : ordered_blocks(model)) {
    token(result, value->id);
    token(result, block_kind(*value));
    dimension_tokens(result, value->output.dimension);
    token(result, frame_name(value->output.frame));
    if (const auto* constant = std::get_if<Constant>(&value->parameters)) {
      token(result, binary64_bits(constant->value));
    } else if (const auto* gain = std::get_if<Gain>(&value->parameters)) {
      token(result, binary64_bits(gain->value));
      dimension_tokens(result, gain->dimension);
    } else if (const auto* sum = std::get_if<Sum>(&value->parameters)) {
      token(result, integer_text(sum->signs.size()));
      for (int sign : sum->signs) {
        token(result, integer_text(sign));
      }
    } else if (const auto* integrator = std::get_if<Integrator>(&value->parameters)) {
      token(result, binary64_bits(integrator->initial_value));
    } else if (const auto* combination = std::get_if<LinearCombination>(&value->parameters)) {
      token(result, integer_text(combination->terms.size()));
      for (const auto& term : combination->terms) {
        dimension_tokens(result, term.input.dimension);
        token(result, frame_name(term.input.frame));
        token(result, binary64_bits(term.coefficient.value));
        dimension_tokens(result, term.coefficient.dimension);
      }
    }
  }
  token(result, "connections");
  token(result, integer_text(model.connections.size()));
  for (const Connection& connection : ordered_connections(model)) {
    token(result, connection.source);
    token(result, connection.target);
    token(result, integer_text(connection.input));
  }
  return result;
}

}  // namespace galata::modeling
