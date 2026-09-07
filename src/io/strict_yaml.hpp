// SPDX-License-Identifier: Apache-2.0
// File-format validation shared by studies and model loaders. Validate the event
// stream before building a tree: aliases can otherwise make recursive validation
// revisit the same node forever. This is deliberately a small YAML subset.
#ifndef GALATA_IO_STRICT_YAML_HPP
#define GALATA_IO_STRICT_YAML_HPP

#include <yaml-cpp/eventhandler.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <initializer_list>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

namespace galata::io {

class YamlContract final : public YAML::EventHandler {
 public:
  void OnDocumentStart(const YAML::Mark&) override {
    if (++documents_ != 1) {
      throw std::invalid_argument("exactly one YAML document is required");
    }
  }

  void OnDocumentEnd() override {}

  void OnNull(const YAML::Mark&, YAML::anchor_t anchor) override {
    check("?", anchor);
  }

  void OnAlias(const YAML::Mark&, YAML::anchor_t) override {
    throw std::invalid_argument("YAML aliases are not supported");
  }

  void OnAnchor(const YAML::Mark&, const std::string&) override {
    throw std::invalid_argument("YAML anchors are not supported");
  }

  void OnScalar(const YAML::Mark&,
                const std::string& tag,
                YAML::anchor_t anchor,
                const std::string&) override {
    check(tag, anchor);
  }

  void OnSequenceStart(const YAML::Mark&,
                       const std::string& tag,
                       YAML::anchor_t anchor,
                       YAML::EmitterStyle::value) override {
    enter(tag, anchor);
  }

  void OnSequenceEnd() override {
    --depth_;
  }

  void OnMapStart(const YAML::Mark&,
                  const std::string& tag,
                  YAML::anchor_t anchor,
                  YAML::EmitterStyle::value) override {
    enter(tag, anchor);
  }

  void OnMapEnd() override {
    --depth_;
  }

 private:
  int documents_ = 0;
  int depth_ = 0;

  static void check(const std::string& tag, YAML::anchor_t anchor) {
    if (anchor != 0 || (tag != "?" && tag != "!" && !tag.empty())) {
      throw std::invalid_argument("YAML anchors and explicit tags are not supported");
    }
  }

  void enter(const std::string& tag, YAML::anchor_t anchor) {
    check(tag, anchor);
    if (++depth_ > 64) {
      throw std::invalid_argument("YAML nesting exceeds 64 levels");
    }
  }
};

inline void unique_yaml_keys(const YAML::Node& node, const std::string& path) {
  if (node.IsMap()) {
    std::set<std::string> seen;
    for (const auto& entry : node) {
      if (!entry.first.IsScalar()) {
        throw std::invalid_argument(path + ": map keys must be strings");
      }
      const std::string key = entry.first.Scalar();
      if (!seen.insert(key).second) {
        throw std::invalid_argument(path + ": duplicate key '" + key + "'");
      }
      unique_yaml_keys(entry.second, path + "." + key);
    }
  } else if (node.IsSequence()) {
    for (std::size_t i = 0; i < node.size(); ++i) {
      unique_yaml_keys(node[i], path + "[" + std::to_string(i) + "]");
    }
  }
}

inline YAML::Node load_yaml(const std::string& bytes, const std::string& path) {
  try {
    std::istringstream stream(bytes);
    YAML::Parser parser(stream);
    YamlContract contract;
    while (parser.HandleNextDocument(contract)) {}
    YAML::Node root = YAML::Load(bytes);
    unique_yaml_keys(root, path);
    return root;
  } catch (const std::exception& error) {
    throw std::invalid_argument(path + ": " + error.what());
  }
}

inline void yaml_keys(const YAML::Node& node,
                      const std::string& path,
                      std::initializer_list<std::string> allowed) {
  if (!node.IsMap()) {
    throw std::invalid_argument(path + ": expected a map");
  }
  for (const auto& entry : node) {
    const std::string key = entry.first.Scalar();
    if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
      throw std::invalid_argument(path + ": unknown key '" + key + "'");
    }
  }
}

}  // namespace galata::io
#endif
