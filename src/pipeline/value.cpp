// SPDX-License-Identifier: Apache-2.0

#include "galata/pipeline/value.hpp"

#include <sstream>
#include <stdexcept>
#include <utility>

namespace galata::pipeline {
namespace {

[[noreturn]] void wrong_kind(Value::Kind actual, Value::Kind expected) {
  std::ostringstream message;
  message << "pipeline value is a " << Value::kind_name(actual) << ", expected a "
          << Value::kind_name(expected);
  throw std::runtime_error(message.str());
}

}  // namespace

struct ValueFactory {
  template <typename Configure>
  static ValuePtr make(Value::Kind kind, Configure configure) {
    auto value = std::make_shared<Value>();
    value->kind_ = kind;
    configure(*value);
    return value;
  }
};

std::string Value::kind_name(Kind kind) {
  switch (kind) {
    case Kind::Null:
      return "null";
    case Kind::Bool:
      return "boolean";
    case Kind::Number:
      return "number";
    case Kind::String:
      return "string";
    case Kind::StageReference:
      return "stage reference";
    case Kind::List:
      return "list";
    case Kind::Map:
      return "map";
  }
  return "unknown";
}

ValuePtr Value::null() {
  return ValueFactory::make(Kind::Null, [](Value&) {});
}

ValuePtr Value::boolean(bool value) {
  return ValueFactory::make(Kind::Bool, [value](Value& v) { v.bool_ = value; });
}

ValuePtr Value::number(double value) {
  return ValueFactory::make(Kind::Number, [value](Value& v) { v.number_ = value; });
}

ValuePtr Value::string(std::string value) {
  return ValueFactory::make(Kind::String, [&value](Value& v) { v.string_ = std::move(value); });
}

ValuePtr Value::stage_reference(std::string stage_id) {
  return ValueFactory::make(Kind::StageReference,
                            [&stage_id](Value& v) { v.string_ = std::move(stage_id); });
}

ValuePtr Value::list(std::vector<ValuePtr> items) {
  return ValueFactory::make(Kind::List, [&items](Value& v) { v.list_ = std::move(items); });
}

ValuePtr Value::map(std::map<std::string, ValuePtr> entries) {
  return ValueFactory::make(Kind::Map, [&entries](Value& v) { v.map_ = std::move(entries); });
}

bool Value::as_bool() const {
  if (kind_ != Kind::Bool) {
    wrong_kind(kind_, Kind::Bool);
  }
  return bool_;
}

double Value::as_number() const {
  if (kind_ != Kind::Number) {
    wrong_kind(kind_, Kind::Number);
  }
  return number_;
}

const std::string& Value::as_string() const {
  if (kind_ != Kind::String) {
    wrong_kind(kind_, Kind::String);
  }
  return string_;
}

const std::string& Value::as_stage_reference() const {
  if (kind_ != Kind::StageReference) {
    wrong_kind(kind_, Kind::StageReference);
  }
  return string_;
}

const std::vector<ValuePtr>& Value::as_list() const {
  if (kind_ != Kind::List) {
    wrong_kind(kind_, Kind::List);
  }
  return list_;
}

const std::map<std::string, ValuePtr>& Value::as_map() const {
  if (kind_ != Kind::Map) {
    wrong_kind(kind_, Kind::Map);
  }
  return map_;
}

ValuePtr Value::get(const std::string& key) const {
  if (kind_ != Kind::Map) {
    return nullptr;
  }
  const auto found = map_.find(key);
  return (found == map_.end()) ? nullptr : found->second;
}

double Value::number_at(const std::string& key) const {
  const ValuePtr found = get(key);
  if (!found) {
    throw std::runtime_error("required input '" + key + "' is missing");
  }
  if (found->kind() != Kind::Number) {
    throw std::runtime_error("input '" + key + "' is a " + kind_name(found->kind())
                             + ", expected a number");
  }
  return found->as_number();
}

double Value::number_at(const std::string& key, double fallback) const {
  const ValuePtr found = get(key);
  return found ? found->as_number() : fallback;
}

std::string Value::string_at(const std::string& key) const {
  const ValuePtr found = get(key);
  if (!found) {
    throw std::runtime_error("required input '" + key + "' is missing");
  }
  if (found->kind() != Kind::String) {
    throw std::runtime_error("input '" + key + "' is a " + kind_name(found->kind())
                             + ", expected a string");
  }
  return found->as_string();
}

std::string Value::string_at(const std::string& key, const std::string& fallback) const {
  const ValuePtr found = get(key);
  return found ? found->as_string() : fallback;
}

bool Value::bool_at(const std::string& key, bool fallback) const {
  const ValuePtr found = get(key);
  return found ? found->as_bool() : fallback;
}

std::vector<std::string> Value::referenced_stages() const {
  std::vector<std::string> references;
  switch (kind_) {
    case Kind::StageReference:
      references.push_back(string_);
      break;
    case Kind::List:
      for (const ValuePtr& item : list_) {
        for (std::string& id : item->referenced_stages()) {
          references.push_back(std::move(id));
        }
      }
      break;
    case Kind::Map:
      for (const auto& [key, item] : map_) {
        (void)key;
        for (std::string& id : item->referenced_stages()) {
          references.push_back(std::move(id));
        }
      }
      break;
    default:
      break;
  }
  return references;
}

}  // namespace galata::pipeline
