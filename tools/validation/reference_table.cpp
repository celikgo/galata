// SPDX-License-Identifier: Apache-2.0

#include "reference_table.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace galata::testing {
namespace {

std::vector<std::string> split_commas(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

}  // namespace

double ReferenceTable::at(std::size_t row, const std::string& column) const {
  if (row >= rows.size()) {
    throw std::out_of_range("reference row " + std::to_string(row) + " out of range");
  }
  const auto found = rows[row].find(column);
  if (found == rows[row].end()) {
    throw std::out_of_range("reference column '" + column + "' not present");
  }
  return found->second;
}

const std::string& ReferenceTable::text(std::size_t row, const std::string& column) const {
  if (row >= text_rows.size()) {
    throw std::out_of_range("reference row " + std::to_string(row) + " out of range");
  }
  const auto found = text_rows[row].find(column);
  if (found == text_rows[row].end()) {
    throw std::out_of_range("reference column '" + column + "' not present");
  }
  return found->second;
}

std::map<std::string, double> ReferenceTable::as_lookup(const std::string& key_column,
                                                        const std::string& value_column) const {
  std::map<std::string, double> lookup;
  for (std::size_t row = 0; row < text_rows.size(); ++row) {
    const std::string& key = text(row, key_column);
    if (lookup.count(key) != 0) {
      throw std::runtime_error("reference table has a duplicate key '" + key + "' in column '"
                               + key_column
                               + "'. A repeated key means the transcription is wrong.");
    }
    lookup[key] = at(row, value_column);
  }
  return lookup;
}

ReferenceTable load_reference(const std::string& directory, const std::string& name) {
  const std::string path = directory + "/" + name;
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot open reference data file: " + path);
  }

  ReferenceTable table;
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();  // tolerate a CRLF checkout even though .gitattributes forbids one
    }
    if (line.empty()) {
      continue;
    }
    if (line[0] == '#') {
      table.citation += line;
      table.citation += '\n';
      continue;
    }
    if (table.columns.empty()) {
      table.columns = split_commas(line);
      continue;
    }
    const std::vector<std::string> fields = split_commas(line);
    if (fields.size() != table.columns.size()) {
      throw std::runtime_error("reference file " + name + ": row has "
                               + std::to_string(fields.size()) + " fields, header has "
                               + std::to_string(table.columns.size()));
    }
    std::map<std::string, double> numeric;
    std::map<std::string, std::string> text;
    for (std::size_t i = 0; i < fields.size(); ++i) {
      text[table.columns[i]] = fields[i];
      // A field that is not a number is not an error: long-format tables carry
      // names and unit strings alongside values.
      try {
        std::size_t consumed = 0;
        const double value = std::stod(fields[i], &consumed);
        if (consumed == fields[i].size()) {
          numeric[table.columns[i]] = value;
        }
      } catch (const std::exception&) {
        // Not numeric; the text view already has it.
      }
    }
    table.rows.push_back(numeric);
    table.text_rows.push_back(text);
  }

  // An uncited reference table is the failure mode charter rule 8 exists to
  // prevent, so it is an error rather than a warning.
  if (table.citation.empty()) {
    throw std::runtime_error("reference file " + name +
                             " has no '#' citation header. Every reference file must name "
                             "the document its values came from.");
  }
  if (table.rows.empty()) {
    throw std::runtime_error("reference file " + name + " contains no data rows");
  }
  return table;
}

double printed_precision_tolerance(double printed,
                                   int significant_figures,
                                   double units_in_last_place) {
  if (printed == 0.0) {
    return units_in_last_place * 2.0 * std::pow(10.0, -significant_figures);
  }
  const double magnitude = std::floor(std::log10(std::fabs(printed)));
  return units_in_last_place * std::pow(10.0, magnitude - (significant_figures - 1));
}

}  // namespace galata::testing
