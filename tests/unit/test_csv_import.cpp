// SPDX-License-Identifier: Apache-2.0
//
// The measured-record contract: what a delimited file becomes, and what it is
// refused for.
//
// Every refusal here exists because the alternative is a number that reaches a
// fitted coefficient carrying an error nothing downstream can see. A unit
// guessed wrong is a factor; a frame guessed wrong is a sign; a timebase
// guessed wrong is a factor of a million.

#include "galata/data/csv.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace {

using galata::data::ColumnMapping;
using galata::data::CsvImportRequest;
using galata::data::MissingPolicy;
using galata::data::read_csv;

const char* kFile =
    "t_us,omega_rpm,thrust_gf\n"
    "0,0,0\n"
    "100000,3000,510\n"
    "200000,4000,905\n";

CsvImportRequest mapping() {
  CsvImportRequest request;
  request.time_column = "t_us";
  request.time_scale_to_seconds = 1e-6;
  request.channels = {
      {"omega_rpm", "rotor_speed_rad_s", "rad/s", "none", 0.10471975511965977, 0.0},
      {"thrust_gf", "thrust_n", "N", "frd", 0.00980665, 0.0},
  };
  return request;
}

}  // namespace

TEST(CsvImport, ConvertsAtTheBoundaryAndRecordsWhatItDid) {
  const auto record = read_csv(kFile, "bench.csv", mapping());
  ASSERT_EQ(record.sample_count(), 3u);
  // Microseconds declared, seconds stored.
  EXPECT_DOUBLE_EQ(record.times_s[1], 0.1);
  EXPECT_DOUBLE_EQ(record.duration_s(), 0.2);

  const auto* speed = record.find("rotor_speed_rad_s");
  ASSERT_NE(speed, nullptr);
  // 3000 rpm = 3000 * 2 pi / 60 = 100 pi rad/s. Written down, not read off.
  EXPECT_NEAR(speed->samples[1], 3000.0 * 0.10471975511965977, 1e-12);
  EXPECT_EQ(speed->unit, "rad/s");
  EXPECT_EQ(speed->source_name, "omega_rpm");
  // The conversion is recorded rather than only applied, so a reader can check
  // the arithmetic instead of trusting it.
  EXPECT_DOUBLE_EQ(speed->scale_applied, 0.10471975511965977);

  const auto* thrust = record.find("thrust_n");
  ASSERT_NE(thrust, nullptr);
  EXPECT_EQ(thrust->frame, "frd");
  EXPECT_NEAR(thrust->samples[2], 905.0 * 0.00980665, 1e-12);

  // The bytes, not the path: two runs citing one path can have read different
  // files.
  EXPECT_EQ(record.source_sha256.size(), 64u);
  EXPECT_EQ(record.source_path, "bench.csv");
}

TEST(CsvImport, EveryColumnMustBeAccountedFor) {
  CsvImportRequest request = mapping();
  const char* with_extra =
      "t_us,omega_rpm,thrust_gf,notes\n"
      "0,0,0,idle\n";
  // A file that gains a column stops the study rather than importing as though
  // it had not changed.
  EXPECT_THROW((void)read_csv(with_extra, "x.csv", request), std::invalid_argument);
  request.ignore_columns = {"notes"};
  EXPECT_NO_THROW((void)read_csv(with_extra, "x.csv", request));
  // Ignoring a column the file does not have is also a mistake worth hearing
  // about: the study and the file disagree either way.
  request.ignore_columns = {"absent"};
  EXPECT_THROW((void)read_csv(with_extra, "x.csv", request), std::invalid_argument);
}

TEST(CsvImport, AmbiguousOrUndeclaredMetadataIsRefused) {
  CsvImportRequest request = mapping();
  request.channels[0].unit.clear();
  EXPECT_THROW((void)read_csv(kFile, "x.csv", request), std::invalid_argument)
      << "a number without a unit is not a measurement";

  request = mapping();
  request.channels[0].frame.clear();
  EXPECT_THROW((void)read_csv(kFile, "x.csv", request), std::invalid_argument)
      << "`none` must be written, not left unsaid";

  request = mapping();
  request.time_scale_to_seconds = 0.0;
  EXPECT_THROW((void)read_csv(kFile, "x.csv", request), std::invalid_argument);

  request = mapping();
  request.channels[0].column = "no_such_column";
  EXPECT_THROW((void)read_csv(kFile, "x.csv", request), std::invalid_argument);
}

TEST(CsvImport, MalformedDataIsRefusedRatherThanRepaired) {
  // A field that is a number with something after it is not a number: reading
  // "1.0kg" as 1.0 silently drops a unit the study never declared.
  EXPECT_THROW((void)read_csv("t_us,omega_rpm,thrust_gf\n0,1.0kg,0\n", "x.csv", mapping()),
               std::invalid_argument);
  // Time that does not advance. Rows are not sorted here.
  EXPECT_THROW((void)read_csv("t_us,omega_rpm,thrust_gf\n100,0,0\n50,1,1\n", "x.csv", mapping()),
               std::invalid_argument);
  // A ragged row.
  EXPECT_THROW((void)read_csv("t_us,omega_rpm,thrust_gf\n0,1\n", "x.csv", mapping()),
               std::invalid_argument);
  // A repeated header.
  CsvImportRequest request = mapping();
  EXPECT_THROW((void)read_csv("t_us,omega_rpm,omega_rpm\n0,1,2\n", "x.csv", request),
               std::invalid_argument);
  // A header and nothing else.
  EXPECT_THROW((void)read_csv("t_us,omega_rpm,thrust_gf\n", "x.csv", mapping()),
               std::invalid_argument);
}

TEST(CsvImport, MissingSamplesAreRefusedByDefaultAndCountedWhenDeclared) {
  const char* gappy =
      "t_us,omega_rpm,thrust_gf\n"
      "0,0,0\n"
      "100000,,510\n"
      "200000,4000,905\n";
  EXPECT_THROW((void)read_csv(gappy, "x.csv", mapping()), std::invalid_argument)
      << "a gap must stop the import unless the study says the source drops samples";

  CsvImportRequest request = mapping();
  request.missing = MissingPolicy::DropRow;
  const auto record = read_csv(gappy, "x.csv", request);
  EXPECT_EQ(record.sample_count(), 2u);
  // Honest only because the count is reported.
  EXPECT_EQ(record.rows_dropped_nonfinite, 1);
  EXPECT_EQ(record.rows_read, 3);
}

// A trailing empty field is an ordinary empty last column, not a ragged row.
// A getline-based split drops it and turns every such file into a refusal.
TEST(CsvImport, ATrailingEmptyFieldIsStillAField) {
  CsvImportRequest request = mapping();
  request.ignore_columns = {"notes"};
  const char* trailing =
      "t_us,omega_rpm,thrust_gf,notes\n"
      "0,0,0,idle\n"
      "100000,3000,510,\n";
  const auto record = read_csv(trailing, "x.csv", request);
  EXPECT_EQ(record.sample_count(), 2u);
}
