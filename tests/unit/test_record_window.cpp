// SPDX-License-Identifier: Apache-2.0
//
// Cutting a record into a window of itself.
//
// WHAT IS ACTUALLY BEING PROTECTED HERE. The window exists so that an
// estimation/validation split can be CHECKED rather than believed — see
// include/galata/identify/validate.hpp. Two properties carry that, and neither
// is obvious from the numbers: the file's identity must survive the cut, so two
// windows agree about where their observations came from; and the interval must
// be half-open, so a sample on a boundary belongs to exactly one of two adjacent
// windows. A closed interval would put the boundary sample in both, and a
// "verified disjoint" split would then share an observation.

#include "galata/data/window.hpp"

#include <gtest/gtest.h>

// <cmath> for std::nan. libc++ pulls it in through gtest; libstdc++ does not,
// so this file compiled on AppleClang and failed on GCC. A file includes what it
// uses.
#include <cmath>
#include <string>

namespace {

using galata::data::Channel;
using galata::data::Record;
using galata::data::window_record;

Record ramp() {
  Record record;
  record.source_path = "flight.csv";
  record.source_sha256 = std::string(64, 'a');
  record.description = "a ramp";
  record.rows_read = 11;
  record.rows_dropped_nonfinite = 2;
  record.samples_altered = 3;
  Channel value;
  value.name = "height_m";
  value.source_name = "h";
  value.unit = "m";
  value.frame = "ned";
  value.source_unit = "ft";
  value.scale_applied = 0.3048;
  for (int k = 0; k <= 10; ++k) {
    record.times_s.push_back(0.1 * static_cast<double>(k));
    value.samples.push_back(static_cast<double>(k));
  }
  record.channels.push_back(value);
  return record;
}

}  // namespace

TEST(RecordWindow, TheFilesIdentitySurvivesTheCut) {
  const Record cut = window_record(ramp(), 0.2, 0.6);
  // The whole point: two windows of one import must agree about the bytes their
  // observations came from, because that agreement is what lets a validation
  // prove disjointness instead of declaring it.
  EXPECT_EQ(cut.source_sha256, std::string(64, 'a'));
  EXPECT_EQ(cut.source_path, "flight.csv");
  EXPECT_TRUE(cut.is_window);
  EXPECT_EQ(cut.window_start_s, 0.2);
  EXPECT_EQ(cut.window_end_s, 0.6);
}

TEST(RecordWindow, TheIntervalIsHalfOpenSoAdjacentWindowsShareNoSample) {
  const Record whole = ramp();
  const Record first = window_record(whole, 0.0, 0.5);
  const Record second = window_record(whole, 0.5, 1.01);
  EXPECT_EQ(first.sample_count(), 5u);
  EXPECT_EQ(second.sample_count(), 6u);
  EXPECT_EQ(first.sample_count() + second.sample_count(), whole.sample_count());
  // The boundary sample belongs to the second window and to no other.
  EXPECT_LT(first.last_time_s(), 0.5);
  EXPECT_DOUBLE_EQ(second.first_time_s(), 0.5);
}

TEST(RecordWindow, ChannelMetadataAndTheImportsOwnCountsAreCarried) {
  const Record cut = window_record(ramp(), 0.2, 0.6);
  ASSERT_EQ(cut.channels.size(), 1u);
  const Channel& kept = cut.channels.front();
  EXPECT_EQ(kept.name, "height_m");
  EXPECT_EQ(kept.unit, "m");
  EXPECT_EQ(kept.frame, "ned");
  EXPECT_EQ(kept.source_unit, "ft");
  EXPECT_EQ(kept.scale_applied, 0.3048);
  // The import's counts are the parent's and are not reset: this routine
  // repaired nothing, and zeroing them would hide what the import had to do.
  EXPECT_EQ(cut.rows_dropped_nonfinite, 2);
  EXPECT_EQ(cut.samples_altered, 3);
  EXPECT_EQ(cut.samples_outside_window, 7);
  EXPECT_EQ(cut.channels.front().samples.front(), 2.0);
  EXPECT_EQ(cut.channels.front().samples.back(), 5.0);
}

TEST(RecordWindow, AWindowThatDoesNotBracketIsRefused) {
  EXPECT_THROW((void)window_record(ramp(), 0.6, 0.6), std::invalid_argument);
  EXPECT_THROW((void)window_record(ramp(), 0.6, 0.2), std::invalid_argument);
  EXPECT_THROW((void)window_record(ramp(), 0.0, std::nan("")), std::invalid_argument);
}

// The usual cause of an empty window is a bound written in the wrong unit —
// milliseconds against a record in seconds. The refusal has to say what the
// record actually covers, or the author has no way to see it.
TEST(RecordWindow, AWindowSelectingTooFewSamplesNamesTheIntervalTheRecordCovers) {
  try {
    (void)window_record(ramp(), 200.0, 600.0);
    FAIL() << "a window outside the record entirely must be refused";
  } catch (const std::invalid_argument& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("0 of the record's 11 sample(s)"), std::string::npos) << message;
    EXPECT_NE(message.find("[0, 1] s"), std::string::npos) << message;
    EXPECT_NE(message.find("different unit"), std::string::npos) << message;
  }
  // One sample is still too few: nothing downstream can simulate through it.
  EXPECT_THROW((void)window_record(ramp(), 0.25, 0.35), std::invalid_argument);
}
