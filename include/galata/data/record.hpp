// SPDX-License-Identifier: Apache-2.0
//
// A measured record: what a bench run or a flight log becomes once it is inside
// galata, and the contract every fitting capability reads.
//
// WHY THIS EXISTS BEFORE ANY FITTING API. A fit is only as good as its inputs'
// provenance, and the questions that matter — which file, which bytes, in what
// units, in which frame, resampled how, with how many samples dropped — are all
// unanswerable once the numbers have been separated from their source. They are
// carried here so that a coefficient can be traced back to the exact bytes that
// produced it.
//
// WHAT THIS IS NOT. Not a time-series library. It does not smooth, filter,
// interpolate on demand or repair anything. A record is what a file said, plus
// an explicit statement of every transformation applied on the way in.
//
// THE UNIT AND THE FRAME ARE DECLARED BY THE CALLER, NOT INFERRED. A CSV column
// called `thrust` might be newtons, gram-force or a raw count; a column called
// `ax` might be NED, FRD or the sensor's own axes. Guessing is how a fit
// acquires a factor of 9.80665 that nothing downstream can see. Every channel
// therefore carries a unit and a frame that a human wrote down, and a channel
// whose study does not declare them is refused rather than defaulted.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace galata::data {

// One measured signal, already converted to SI and already in the declared
// frame — because the conversion happens once, at the boundary, and what it was
// converted FROM is recorded rather than reapplied downstream (ADR-0003).
struct Channel {
  std::string name;         // the name galata knows it by
  std::string source_name;  // the column or topic field it came from
  std::string unit;         // SI unit of `samples`, e.g. "rad/s", "N", "m/s^2"
  // The frame the samples are expressed in: "ned", "frd", "enu", "flu", or
  // "none" for a scalar that has no frame. Recorded, never converted silently:
  // a record in FLU stays in FLU and says so.
  std::string frame;
  // What the source held before conversion, kept so a reader can check the
  // arithmetic rather than trust it. Empty when no conversion was applied.
  std::string source_unit;
  double scale_applied = 1.0;
  double offset_applied = 0.0;
  std::vector<double> samples;
};

// How a record's timebase was produced.
enum class Timebase {
  // Times are the source's own, unchanged.
  SourceTimestamps,
  // Times were resampled onto a uniform grid at `sample_rate_hz`.
  UniformResampled,
};

struct Record {
  std::string source_path;
  std::string source_sha256;  // the bytes, not the path — a path is not an identity
  std::string description;

  // WHEN THIS RECORD IS A CUT OF ANOTHER. `source_path` and `source_sha256`
  // still name the FILE, unchanged, so two windows of one import agree about
  // where their observations came from; these three say which stretch of it
  // this record is.
  //
  // This exists so that "held out" can be a checked statement rather than a
  // hopeful one. Two records with different file digests may hold the same
  // observations — a file reformatted, a segment copied, two exports of one
  // flight — and unequal hashes prove nothing about the DATA. Two windows of
  // ONE file whose intervals do not overlap are disjoint by construction, and
  // that is the one case an estimation/validation split can be VERIFIED rather
  // than declared. See `include/galata/identify/validate.hpp`.
  //
  // The interval is half-open, [start, end): a sample exactly at `window_end_s`
  // belongs to the next window and to no other, so adjacent windows share
  // nothing and no sample is lost between them.
  bool is_window = false;
  double window_start_s = 0.0;
  double window_end_s = 0.0;
  // Samples the parent held that fell outside the window. Reported rather than
  // absorbed, for the reason every other count here is.
  std::int64_t samples_outside_window = 0;

  std::vector<double> times_s;  // strictly increasing, seconds
  std::vector<Channel> channels;

  Timebase timebase = Timebase::SourceTimestamps;
  double sample_rate_hz = 0.0;  // meaningful only when resampled

  // What the import had to do to the data, reported rather than absorbed. A fit
  // whose record dropped a third of its samples is a different fit from one
  // whose record dropped none, and the difference must be visible.
  std::int64_t rows_read = 0;
  std::int64_t rows_dropped_nonfinite = 0;
  std::int64_t rows_dropped_duplicate_time = 0;
  std::int64_t samples_altered = 0;

  [[nodiscard]] std::size_t sample_count() const noexcept {
    return times_s.size();
  }

  [[nodiscard]] const Channel* find(const std::string& name) const;
  [[nodiscard]] double duration_s() const;

  // The stretch of time the samples actually cover, which is what a reader
  // comparing two records for overlap needs. Not the declared window: a window
  // may be wider than the samples that fell inside it, and a claim about
  // independence must rest on the observations, not on the request.
  [[nodiscard]] double first_time_s() const;
  [[nodiscard]] double last_time_s() const;
};

}  // namespace galata::data
