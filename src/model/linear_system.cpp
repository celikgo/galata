// SPDX-License-Identifier: Apache-2.0

#include "galata/model/linear_system.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace galata::model {
namespace {

Eigen::MatrixXd read_matrix(const YAML::Node& node,
                            const std::string& name,
                            const std::string& path) {
  if (!node.IsSequence() || node.size() == 0) {
    throw std::invalid_argument(path + ": '" + name + "' must be a non-empty sequence of rows");
  }
  const std::size_t rows = node.size();
  std::size_t columns = 0;
  std::vector<std::vector<double>> values;
  values.reserve(rows);

  for (std::size_t r = 0; r < rows; ++r) {
    const YAML::Node& row = node[r];
    if (!row.IsSequence()) {
      throw std::invalid_argument(path + ": '" + name + "' row " + std::to_string(r)
                                  + " is not a sequence");
    }
    if (r == 0) {
      columns = row.size();
    } else if (row.size() != columns) {
      throw std::invalid_argument(path + ": '" + name + "' row " + std::to_string(r) + " has " +
                                  std::to_string(row.size()) + " entries, row 0 has " +
                                  std::to_string(columns) + ". A ragged matrix is a typo, not a "
                                  "shape.");
    }
    std::vector<double> parsed;
    parsed.reserve(columns);
    for (std::size_t c = 0; c < columns; ++c) {
      parsed.push_back(row[c].as<double>());
    }
    values.push_back(std::move(parsed));
  }

  Eigen::MatrixXd matrix(static_cast<Eigen::Index>(rows), static_cast<Eigen::Index>(columns));
  for (std::size_t r = 0; r < rows; ++r) {
    for (std::size_t c = 0; c < columns; ++c) {
      matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = values[r][c];
    }
  }
  return matrix;
}

std::vector<std::string> read_names(const YAML::Node& node) {
  std::vector<std::string> names;
  if (!node) {
    return names;
  }
  for (const YAML::Node& entry : node) {
    names.push_back(entry.Scalar());
  }
  return names;
}

}  // namespace

Eigen::MatrixXd LinearSystem::output_matrix() const {
  if (c.size() != 0) {
    return c;
  }
  return Eigen::MatrixXd::Identity(a.rows(), a.rows());
}

Eigen::MatrixXd LinearSystem::feedthrough_matrix() const {
  if (d.size() != 0) {
    return d;
  }
  return Eigen::MatrixXd::Zero(output_count(), input_count());
}

std::vector<std::string> LinearSystem::output_labels() const {
  if (!output_names.empty()) {
    return output_names;
  }
  if (c.size() == 0) {
    return state_names;
  }
  std::vector<std::string> generated;
  generated.reserve(static_cast<std::size_t>(c.rows()));
  for (Eigen::Index row = 0; row < c.rows(); ++row) {
    generated.push_back("y" + std::to_string(row));
  }
  return generated;
}

void LinearSystem::validate() const {
  if (a.rows() == 0) {
    throw std::invalid_argument("linear system: A is empty");
  }
  if (a.rows() != a.cols()) {
    std::ostringstream message;
    message << "linear system: A is " << a.rows() << "x" << a.cols() << ", must be square";
    throw std::invalid_argument(message.str());
  }
  if (static_cast<Eigen::Index>(state_names.size()) != a.rows()) {
    std::ostringstream message;
    message << "linear system: " << state_names.size() << " state names for a " << a.rows()
            << "-state model. Every state must be named — the names are what the modal "
               "classifier reads.";
    throw std::invalid_argument(message.str());
  }
  std::set<std::string> unique(state_names.begin(), state_names.end());
  if (unique.size() != state_names.size()) {
    throw std::invalid_argument("linear system: state names are not unique");
  }
  if (b.size() != 0) {
    if (b.rows() != a.rows()) {
      std::ostringstream message;
      message << "linear system: B has " << b.rows() << " rows, A has " << a.rows();
      throw std::invalid_argument(message.str());
    }
    if (static_cast<Eigen::Index>(input_names.size()) != b.cols()) {
      std::ostringstream message;
      message << "linear system: " << input_names.size() << " input names for " << b.cols()
              << " columns of B";
      throw std::invalid_argument(message.str());
    }
  }
  if (c.size() != 0) {
    if (c.cols() != a.rows()) {
      std::ostringstream message;
      message << "linear system: C has " << c.cols() << " columns, A has " << a.rows() << " states";
      throw std::invalid_argument(message.str());
    }
    if (!output_names.empty() && static_cast<Eigen::Index>(output_names.size()) != c.rows()) {
      std::ostringstream message;
      message << "linear system: " << output_names.size() << " output names for " << c.rows()
              << " rows of C";
      throw std::invalid_argument(message.str());
    }
  } else if (!output_names.empty() && static_cast<Eigen::Index>(output_names.size()) != a.rows()) {
    std::ostringstream message;
    message << "linear system: " << output_names.size()
            << " output names but no C, which "
               "means the outputs are the "
            << a.rows() << " states";
    throw std::invalid_argument(message.str());
  }
  if (d.size() != 0) {
    if (d.rows() != output_count()) {
      std::ostringstream message;
      message << "linear system: D has " << d.rows() << " rows, there are " << output_count()
              << " outputs";
      throw std::invalid_argument(message.str());
    }
    if (d.cols() != input_count()) {
      std::ostringstream message;
      message << "linear system: D has " << d.cols() << " columns, there are " << input_count()
              << " inputs";
      throw std::invalid_argument(message.str());
    }
  }
  if (!a.allFinite()) {
    throw std::invalid_argument("linear system: A contains a non-finite entry");
  }
  if (b.size() != 0 && !b.allFinite()) {
    throw std::invalid_argument("linear system: B contains a non-finite entry");
  }
  if (c.size() != 0 && !c.allFinite()) {
    throw std::invalid_argument("linear system: C contains a non-finite entry");
  }
  if (d.size() != 0 && !d.allFinite()) {
    throw std::invalid_argument("linear system: D contains a non-finite entry");
  }
}

LinearSystem load_linear_system(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::invalid_argument("cannot open linear system file: " + path);
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();

  YAML::Node root;
  try {
    root = YAML::Load(buffer.str());
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument(path + ": not valid YAML: " + error.what());
  }
  if (!root.IsMap()) {
    throw std::invalid_argument(path + ": the document must be a map");
  }
  if (!root["a"]) {
    throw std::invalid_argument(path + ": missing 'a' (the state matrix)");
  }

  LinearSystem system;
  system.a = read_matrix(root["a"], "a", path);
  if (root["b"]) {
    system.b = read_matrix(root["b"], "b", path);
  }
  if (root["c"]) {
    system.c = read_matrix(root["c"], "c", path);
  }
  if (root["d"]) {
    system.d = read_matrix(root["d"], "d", path);
  }
  system.state_names = read_names(root["states"]);
  system.input_names = read_names(root["inputs"]);
  system.output_names = read_names(root["outputs"]);
  system.description = root["description"] ? root["description"].Scalar() : std::string{};
  system.citation = root["citation"] ? root["citation"].Scalar() : std::string{};
  system.units = root["units"] ? root["units"].Scalar() : std::string{};

  system.validate();
  return system;
}

}  // namespace galata::model
