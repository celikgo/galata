// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the delimited-text reader declared in
// include/galata/data/csv.hpp.

#include "galata/data/csv.hpp"

#include "galata/core/sha256.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <set>
#include <sstream>
#include <stdexcept>

namespace galata::data {
namespace {

// Splits on the delimiter, keeping EMPTY fields including a trailing one.
// "a,b," is three fields, the last empty — a getline-based loop silently drops
// it and turns an ordinary row with an empty last column into a ragged one.
std::vector<std::string> split(const std::string& line, char delimiter) {
  std::vector<std::string> fields;
  std::string field;
  std::size_t start = 0;
  while (true) {
    const std::size_t next = line.find(delimiter, start);
    field = line.substr(start, next == std::string::npos ? std::string::npos : next - start);
    // Trim spaces and quotes; a header written by a spreadsheet often carries
    // both, and a column that differs from its declaration only by a quote is a
    // refusal nobody can act on.
    const auto first = field.find_first_not_of(" \t\r\"");
    const auto last = field.find_last_not_of(" \t\r\"");
    fields.push_back(first == std::string::npos ? std::string()
                                                : field.substr(first, last - first + 1));
    if (next == std::string::npos) {
      break;
    }
    start = next + 1;
  }
  return fields;
}

// Parses a full field as a double. Rejects trailing rubbish: "1.0kg" is not a
// number, and reading it as 1.0 would silently drop a unit the study never
// declared.
bool parse_number(const std::string& text, double& out) {
  if (text.empty()) {
    return false;
  }
  const char* begin = text.c_str();
  char* end = nullptr;
  const double value = std::strtod(begin, &end);
  if (end == begin) {
    return false;
  }
  while (*end == ' ' || *end == '\t' || *end == '\r') {
    ++end;
  }
  if (*end != '\0') {
    return false;
  }
  out = value;
  return true;
}

}  // namespace

Record read_csv(const std::string& contents,
                const std::string& source_path,
                const CsvImportRequest& request) {
  if (!(request.time_scale_to_seconds > 0.0) || !std::isfinite(request.time_scale_to_seconds)) {
    throw std::invalid_argument(
        "data.import.csv: time_scale_to_seconds must be positive and finite. It is required "
        "because a column called `t` or `timestamp` is as often microseconds as seconds, and "
        "reading one as the other rescales every rate in the record by a million");
  }

  std::istringstream stream(contents);
  std::string line;
  if (!std::getline(stream, line)) {
    throw std::invalid_argument("data.import.csv: the file is empty");
  }
  const std::vector<std::string> header = split(line, ',');
  if (header.empty()) {
    throw std::invalid_argument("data.import.csv: the file has no header row");
  }
  {
    std::set<std::string> seen;
    for (const std::string& name : header) {
      if (!seen.insert(name).second) {
        throw std::invalid_argument("data.import.csv: the header repeats the column '" + name
                                    + "'; which one a mapping meant is not decidable");
      }
    }
  }

  const auto index_of = [&](const std::string& column) {
    const auto found = std::find(header.begin(), header.end(), column);
    if (found == header.end()) {
      throw std::invalid_argument("data.import.csv: the file has no column '" + column + "'");
    }
    return static_cast<std::size_t>(found - header.begin());
  };

  const std::size_t time_index = index_of(request.time_column);
  std::vector<std::size_t> channel_index;
  channel_index.reserve(request.channels.size());
  for (const ColumnMapping& mapping : request.channels) {
    if (mapping.unit.empty()) {
      throw std::invalid_argument("data.import.csv: channel '" + mapping.name
                                  + "' declares no unit. A number without a unit is not a "
                                    "measurement, and this reader will not guess one");
    }
    if (mapping.frame.empty()) {
      throw std::invalid_argument("data.import.csv: channel '" + mapping.name
                                  + "' declares no frame; use `none` for a scalar that has "
                                    "none, rather than leaving it unsaid");
    }
    channel_index.push_back(index_of(mapping.column));
  }

  // Every column must be accounted for. A file that gains a column should stop
  // the study rather than import as though it had not changed.
  {
    std::set<std::string> accounted{request.time_column};
    for (const ColumnMapping& mapping : request.channels) {
      accounted.insert(mapping.column);
    }
    for (const std::string& ignored : request.ignore_columns) {
      if (std::find(header.begin(), header.end(), ignored) == header.end()) {
        throw std::invalid_argument("data.import.csv: the study ignores a column '" + ignored
                                    + "' that the file does not have");
      }
      accounted.insert(ignored);
    }
    for (const std::string& name : header) {
      if (accounted.count(name) == 0) {
        throw std::invalid_argument(
            "data.import.csv: the file has a column '" + name
            + "' that is neither mapped nor listed in `ignore_columns`. Every column must be "
              "accounted for, so a file that gains one stops the study rather than importing "
              "as though it had not");
      }
    }
  }

  Record record;
  record.source_path = source_path;
  record.source_sha256 = core::sha256(contents);
  record.description = request.description;
  record.timebase = Timebase::SourceTimestamps;
  record.channels.resize(request.channels.size());
  for (std::size_t c = 0; c < request.channels.size(); ++c) {
    const ColumnMapping& mapping = request.channels[c];
    Channel& channel = record.channels[c];
    channel.name = mapping.name;
    channel.source_name = mapping.column;
    channel.unit = mapping.unit;
    channel.frame = mapping.frame;
    channel.scale_applied = mapping.scale;
    channel.offset_applied = mapping.offset;
  }

  std::int64_t row_number = 1;
  while (std::getline(stream, line)) {
    ++row_number;
    if (line.empty()) {
      continue;
    }
    const std::vector<std::string> fields = split(line, ',');
    if (fields.size() != header.size()) {
      std::ostringstream message;
      message << "data.import.csv: row " << row_number << " has " << fields.size()
              << " field(s) against the header's " << header.size()
              << "; a ragged file is not repaired here";
      throw std::invalid_argument(message.str());
    }
    ++record.rows_read;

    double raw_time = 0.0;
    if (!parse_number(fields[time_index], raw_time)) {
      std::ostringstream message;
      message << "data.import.csv: row " << row_number << " has a non-numeric time '"
              << fields[time_index] << "'";
      throw std::invalid_argument(message.str());
    }
    const double time_s = raw_time * request.time_scale_to_seconds;

    std::vector<double> values(request.channels.size());
    bool row_finite = std::isfinite(time_s);
    for (std::size_t c = 0; c < request.channels.size() && row_finite; ++c) {
      double raw = 0.0;
      if (!parse_number(fields[channel_index[c]], raw)) {
        if (request.missing == MissingPolicy::DropRow) {
          row_finite = false;
          break;
        }
        std::ostringstream message;
        message << "data.import.csv: row " << row_number << ", column '"
                << request.channels[c].column << "' holds '" << fields[channel_index[c]]
                << "', which is not a number. Declare `missing: drop_row` if the source "
                   "genuinely drops samples; the count is then reported";
        throw std::invalid_argument(message.str());
      }
      const double si = request.channels[c].scale * raw + request.channels[c].offset;
      if (!std::isfinite(si)) {
        row_finite = false;
        break;
      }
      values[c] = si;
      if (request.channels[c].scale != 1.0 || request.channels[c].offset != 0.0) {
        ++record.samples_altered;
      }
    }
    if (!row_finite) {
      if (request.missing == MissingPolicy::Refuse) {
        std::ostringstream message;
        message << "data.import.csv: row " << row_number
                << " holds a non-finite value. Declare `missing: drop_row` to skip such rows "
                   "and have the count reported";
        throw std::invalid_argument(message.str());
      }
      ++record.rows_dropped_nonfinite;
      continue;
    }

    if (!record.times_s.empty() && !(time_s > record.times_s.back())) {
      if (request.missing == MissingPolicy::DropRow && time_s == record.times_s.back()) {
        ++record.rows_dropped_duplicate_time;
        continue;
      }
      std::ostringstream message;
      message << "data.import.csv: row " << row_number << " is at t = " << time_s
              << " s, which does not follow t = " << record.times_s.back()
              << " s. Rows are not sorted and timestamps are not repaired here: a record whose "
                 "order was invented is not a measurement";
      throw std::invalid_argument(message.str());
    }
    record.times_s.push_back(time_s);
    for (std::size_t c = 0; c < values.size(); ++c) {
      record.channels[c].samples.push_back(values[c]);
    }
  }

  if (record.times_s.empty()) {
    throw std::invalid_argument("data.import.csv: the file has a header and no usable rows");
  }
  return record;
}

}  // namespace galata::data
