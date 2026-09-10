// SPDX-License-Identifier: Apache-2.0
//
// Reading a measured record from delimited text.
//
// A CSV carries column names and nothing else: no units, no frames, no
// statement of what its time column means. Everything this reader needs beyond
// the numbers is therefore DECLARED by the caller, and a declaration that does
// not match the file is an error rather than a best effort.
#pragma once

#include "galata/data/record.hpp"

#include <string>
#include <vector>

namespace galata::data {

// What one column of the file becomes.
struct ColumnMapping {
  std::string column;  // the header in the file
  std::string name;    // what galata will call it
  std::string unit;    // SI unit AFTER scale and offset
  std::string frame = "none";
  // Applied as `si = scale * raw + offset`, in that order. Both default to the
  // identity; a file already in SI needs neither. This is the ONE place a
  // conversion happens, and what it was applied to is recorded on the channel.
  double scale = 1.0;
  double offset = 0.0;
};

// How to handle a row this reader cannot use as it stands.
enum class MissingPolicy {
  // Refuse the whole import. The default, and the right choice when a gap means
  // the run was not what the study thinks it was.
  Refuse,
  // Drop the row and count it. Legitimate when a sensor genuinely drops
  // samples, and honest only because the count is reported.
  DropRow,
};

struct CsvImportRequest {
  std::string time_column;
  // Seconds per unit of the time column: 1.0 for seconds, 1e-6 for
  // microseconds. Declared, because a column called `t` or `timestamp` is as
  // often microseconds as seconds and reading one as the other silently
  // rescales every rate in the record by a million.
  double time_scale_to_seconds = 1.0;
  std::vector<ColumnMapping> channels;
  MissingPolicy missing = MissingPolicy::Refuse;
  // Columns present in the file that the study deliberately ignores. Listing
  // them is required: an unmapped, unignored column is refused, so a file that
  // gains a column does not quietly import as though it had not.
  std::vector<std::string> ignore_columns;
  std::string description;
};

// Refuses, by name: a missing or duplicated header, a declared column the file
// does not have, a file column that is neither mapped nor ignored, a
// non-numeric field, a non-finite value or a time that does not strictly
// increase under `MissingPolicy::Refuse`, an empty file, and a time scale that
// is not positive and finite.
//
// Never repairs. It does not sort rows, does not interpolate across a gap, does
// not extrapolate beyond the ends and does not invent a timestamp.
[[nodiscard]] Record read_csv(const std::string& contents,
                              const std::string& source_path,
                              const CsvImportRequest& request);

}  // namespace galata::data
