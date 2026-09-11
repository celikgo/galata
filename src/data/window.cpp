// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the record window declared in
// include/galata/data/window.hpp.

#include "galata/data/window.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace galata::data {

Record window_record(const Record& record, double start_s, double end_s) {
  if (!std::isfinite(start_s) || !std::isfinite(end_s)) {
    throw std::invalid_argument("data.window: both bounds must be finite");
  }
  if (!(end_s > start_s)) {
    throw std::invalid_argument(
        "data.window: the window does not bracket — `end_time_s` must "
        "exceed `start_time_s`");
  }

  Record out;
  // The FILE's identity travels unchanged. Two windows of one import agree
  // about where their observations came from, and that agreement is what lets
  // a validation prove disjointness rather than declare it.
  out.source_path = record.source_path;
  out.source_sha256 = record.source_sha256;
  out.description = record.description;
  out.timebase = record.timebase;
  out.sample_rate_hz = record.sample_rate_hz;
  // The import's own counts are the parent's and are carried rather than
  // recomputed: this routine dropped nothing non-finite and repaired nothing,
  // and zeroing them here would hide what the import had to do.
  out.rows_read = record.rows_read;
  out.rows_dropped_nonfinite = record.rows_dropped_nonfinite;
  out.rows_dropped_duplicate_time = record.rows_dropped_duplicate_time;
  out.samples_altered = record.samples_altered;

  out.is_window = true;
  out.window_start_s = start_s;
  out.window_end_s = end_s;

  out.channels.resize(record.channels.size());
  for (std::size_t c = 0; c < record.channels.size(); ++c) {
    const Channel& source = record.channels[c];
    Channel& kept = out.channels[c];
    kept.name = source.name;
    kept.source_name = source.source_name;
    kept.unit = source.unit;
    kept.frame = source.frame;
    kept.source_unit = source.source_unit;
    kept.scale_applied = source.scale_applied;
    kept.offset_applied = source.offset_applied;
  }

  for (std::size_t k = 0; k < record.times_s.size(); ++k) {
    const double time_s = record.times_s[k];
    if (time_s < start_s || time_s >= end_s) {
      ++out.samples_outside_window;
      continue;
    }
    out.times_s.push_back(time_s);
    for (std::size_t c = 0; c < record.channels.size(); ++c) {
      out.channels[c].samples.push_back(record.channels[c].samples[k]);
    }
  }

  if (out.times_s.size() < 2) {
    std::ostringstream message;
    message << "data.window: [" << start_s << ", " << end_s << ") s selects " << out.times_s.size()
            << " of the record's " << record.sample_count()
            << " sample(s), which is too few to simulate through. The record covers ["
            << record.first_time_s() << ", " << record.last_time_s()
            << "] s; the usual cause is a window written in a different unit from the "
               "record's timebase";
    throw std::invalid_argument(message.str());
  }
  return out;
}

}  // namespace galata::data
