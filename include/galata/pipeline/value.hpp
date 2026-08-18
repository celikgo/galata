// SPDX-License-Identifier: Apache-2.0
//
// The typed input tree a pipeline stage receives.
//
// Parsed from the pipeline YAML once, then handed to capabilities as a
// read-only tree. Capabilities never see YAML: keeping the parser on one side
// of this type means the pipeline format can gain a second serialisation
// without touching a single capability.
//
// WHAT THIS IS NOT: not a general YAML document model. It carries what a stage
// input can be — scalars, lists, maps and references to other stages — and
// nothing else. YAML anchors, tags and multi-document streams are not
// represented, and the parser rejects rather than ignores them.

#ifndef GALATA_PIPELINE_VALUE_HPP
#define GALATA_PIPELINE_VALUE_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace galata::pipeline {

class Value;
using ValuePtr = std::shared_ptr<const Value>;

class Value {
 public:
  enum class Kind {
    Null,
    Bool,
    Number,
    String,
    // A reference to another stage's output, written `{from: stage_id}` in
    // YAML. Kept as its own kind rather than as a map so that the DAG can be
    // built by walking the tree, and so a capability cannot mistake a wiring
    // instruction for data.
    StageReference,
    List,
    Map,
  };

  static ValuePtr null();
  static ValuePtr boolean(bool value);
  static ValuePtr number(double value);
  static ValuePtr string(std::string value);
  static ValuePtr stage_reference(std::string stage_id);
  static ValuePtr list(std::vector<ValuePtr> items);
  static ValuePtr map(std::map<std::string, ValuePtr> entries);

  [[nodiscard]] Kind kind() const noexcept {
    return kind_;
  }

  [[nodiscard]] static std::string kind_name(Kind kind);

  [[nodiscard]] bool as_bool() const;
  [[nodiscard]] double as_number() const;
  [[nodiscard]] const std::string& as_string() const;
  [[nodiscard]] const std::string& as_stage_reference() const;
  [[nodiscard]] const std::vector<ValuePtr>& as_list() const;
  [[nodiscard]] const std::map<std::string, ValuePtr>& as_map() const;

  // Map lookup. Returns nullptr when absent, so a caller can distinguish
  // "missing" from "present and null".
  [[nodiscard]] ValuePtr get(const std::string& key) const;

  // Typed lookups that throw a message naming the stage's own vocabulary
  // rather than a type id. A capability's error should read like a
  // specification, not like a cast failure.
  [[nodiscard]] double number_at(const std::string& key) const;
  [[nodiscard]] double number_at(const std::string& key, double fallback) const;
  [[nodiscard]] std::string string_at(const std::string& key) const;
  [[nodiscard]] std::string string_at(const std::string& key, const std::string& fallback) const;
  [[nodiscard]] bool bool_at(const std::string& key, bool fallback) const;

  // Every stage id this subtree refers to, in the order encountered.
  [[nodiscard]] std::vector<std::string> referenced_stages() const;

 private:
  Kind kind_ = Kind::Null;
  bool bool_ = false;
  double number_ = 0.0;
  std::string string_;
  std::vector<ValuePtr> list_;
  std::map<std::string, ValuePtr> map_;

  friend struct ValueFactory;
};

}  // namespace galata::pipeline

#endif  // GALATA_PIPELINE_VALUE_HPP
