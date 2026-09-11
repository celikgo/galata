// SPDX-License-Identifier: Apache-2.0
//
// The PX4 ULog reader (ADR-0016).
//
// THE FIXTURE IS GENERATED, NOT COMMITTED. `tests/data/make_ulog_fixture.py`
// writes it and the values below are the ones that script writes, in the open,
// where a reviewer who doubts one edits the line that produced it. The same
// script hands the fixture to `pyulog` — PX4's own tooling, an implementation
// that is not ours — and refuses to emit a fixture pyulog cannot read or reads
// differently. So a value agreed here has been agreed by two readers.
//
// SIGNS ARE THE POINT. Every axis in the fixture carries a distinct magnitude
// AND a distinct sign, because the failures worth catching in a binary reader
// are a transposed axis and a flipped sign, and both survive any check that
// only looks at magnitudes.

#include "galata/data/ulog.hpp"

#include "ulog_fixture.hpp"
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace {

using galata::data::read_ulog;
using galata::data::UlogChannel;
using galata::data::UlogImportRequest;

UlogImportRequest rates_and_attitude() {
  UlogImportRequest request;
  request.resample_hz = 250.0;
  request.channels = {
      {"vehicle_angular_velocity", "xyz[0]", "gyro_p_rad_s", "rad/s", "frd", 1.0, 0.0, 0},
      {"vehicle_angular_velocity", "xyz[1]", "gyro_q_rad_s", "rad/s", "frd", 1.0, 0.0, 0},
      {"vehicle_angular_velocity", "xyz[2]", "gyro_r_rad_s", "rad/s", "frd", 1.0, 0.0, 0},
      {"vehicle_attitude", "q[0]", "q_w", "1", "none", 1.0, 0.0, 0},
      {"vehicle_attitude", "q[1]", "q_x", "1", "none", 1.0, 0.0, 0},
      {"vehicle_attitude", "q[2]", "q_y", "1", "none", 1.0, 0.0, 0},
      {"vehicle_attitude", "q[3]", "q_z", "1", "none", 1.0, 0.0, 0},
  };
  return request;
}

}  // namespace

TEST(UlogImport, ReadsEachAxisWithItsOwnSign) {
  const auto record = read_ulog(galata::test::ulog_fixture(), "sample.ulg", rates_and_attitude());
  ASSERT_GT(record.sample_count(), 1u);

  // The generator writes, at sample k: p = 0.10 + k, q = -0.20 - k, r = 0.30 + k.
  // Three distinct magnitudes and a distinct sign on the middle one, so a
  // reader that swapped two axes or dropped a sign cannot pass.
  const auto* p = record.find("gyro_p_rad_s");
  const auto* q = record.find("gyro_q_rad_s");
  const auto* r = record.find("gyro_r_rad_s");
  ASSERT_NE(p, nullptr);
  ASSERT_NE(q, nullptr);
  ASSERT_NE(r, nullptr);
  EXPECT_NEAR(p->samples.front(), 0.10, 1e-6);
  EXPECT_NEAR(q->samples.front(), -0.20, 1e-6) << "the roll-rate axis must keep its sign";
  EXPECT_NEAR(r->samples.front(), 0.30, 1e-6);
  EXPECT_GT(p->samples.back(), p->samples.front()) << "p rises through the fixture";
  EXPECT_LT(q->samples.back(), q->samples.front()) << "q falls through the fixture";
}

// PX4's quaternion is stored w, x, y, z — scalar FIRST. A reader that assumed
// the other convention reads the scalar as a vector component and every
// attitude in the record is wrong by a rotation nobody can see in a magnitude.
TEST(UlogImport, TheQuaternionIsScalarFirstAndInOrder) {
  const auto record = read_ulog(galata::test::ulog_fixture(), "sample.ulg", rates_and_attitude());
  const auto* w = record.find("q_w");
  const auto* x = record.find("q_x");
  const auto* y = record.find("q_y");
  const auto* z = record.find("q_z");
  ASSERT_NE(w, nullptr);
  ASSERT_NE(z, nullptr);
  // The generator writes q = (1.0, 0.1a, 0.2a, 0.3a) with a = 0.02 at k = 0.
  EXPECT_NEAR(w->samples.front(), 1.0, 1e-6) << "the scalar part is first";
  EXPECT_NEAR(x->samples.front(), 0.1 * 0.02, 1e-9);
  EXPECT_NEAR(y->samples.front(), 0.2 * 0.02, 1e-9);
  EXPECT_NEAR(z->samples.front(), 0.3 * 0.02, 1e-9);
  // Strictly increasing magnitudes across the three vector components is what
  // distinguishes an in-order read from a permuted one.
  EXPECT_LT(x->samples.front(), y->samples.front());
  EXPECT_LT(y->samples.front(), z->samples.front());
}

// A DUPLICATED TOPIC IS TWO SERIES, NOT ONE. PX4 logs a second sensor as the
// same message format with a new message id and multi_id 1. A reader that
// ignored multi_id would merge the two into a channel that is neither — and
// would pass every other test in this file, because every other test reads
// instance 0 of a topic logged once.
//
// The fixture's second `sensor_combined` is offset by 100 in every component,
// so a merge or a wrong selection is out by exactly that and cannot be mistaken
// for a rounding difference.
TEST(UlogImport, ASecondInstanceOfATopicIsItsOwnSeries) {
  UlogImportRequest request;
  request.resample_hz = 250.0;
  request.channels = {
      {"sensor_combined", "gyro_rad[0]", "gyro_x_0", "rad/s", "frd", 1.0, 0.0, 0},
      {"sensor_combined", "gyro_rad[0]", "gyro_x_1", "rad/s", "frd", 1.0, 0.0, 1},
      {"sensor_combined", "accelerometer_m_s2[2]", "accel_z_0", "m/s^2", "frd", 1.0, 0.0, 0},
      {"sensor_combined", "accelerometer_m_s2[2]", "accel_z_1", "m/s^2", "frd", 1.0, 0.0, 1},
  };
  const auto record = read_ulog(galata::test::ulog_fixture(), "sample.ulg", request);

  const auto* gyro0 = record.find("gyro_x_0");
  const auto* gyro1 = record.find("gyro_x_1");
  const auto* accel0 = record.find("accel_z_0");
  const auto* accel1 = record.find("accel_z_1");
  ASSERT_NE(gyro0, nullptr);
  ASSERT_NE(gyro1, nullptr);
  ASSERT_NE(accel0, nullptr);
  ASSERT_NE(accel1, nullptr);

  // The values the fixture wrote: instance 0 at 0.01 + k, instance 1 offset by
  // exactly 100. Asserted against the generator rather than against each other,
  // so a reader that returned instance 0 twice fails on the second channel.
  ASSERT_FALSE(gyro0->samples.empty());
  EXPECT_NEAR(gyro0->samples.front(), 0.01, 1e-6);
  EXPECT_NEAR(gyro1->samples.front(), 100.01, 1e-4);
  EXPECT_NEAR(accel0->samples.front(), -9.81, 1e-4);
  EXPECT_NEAR(accel1->samples.front(), 90.19, 1e-4);
  for (std::size_t k = 0; k < gyro0->samples.size(); ++k) {
    EXPECT_NEAR(gyro1->samples[k] - gyro0->samples[k], 100.0, 1e-3)
        << "instance 1 must stay exactly 100 from instance 0 at sample " << k
        << "; a merged or misselected instance breaks this";
  }
}

// And an instance the log does not carry is refused by name rather than
// silently falling back to instance 0, which would return a channel of the
// wrong sensor under the name the study chose.
TEST(UlogImport, AnInstanceTheLogDoesNotCarryIsRefused) {
  UlogImportRequest request;
  request.resample_hz = 250.0;
  request.channels = {
      {"sensor_combined", "gyro_rad[0]", "gyro", "rad/s", "frd", 1.0, 0.0, 7},
  };
  EXPECT_THROW((void)read_ulog(galata::test::ulog_fixture(), "sample.ulg", request),
               std::invalid_argument);
}

TEST(UlogImport, ProvenanceAndTimebaseAreRecorded) {
  const auto record = read_ulog(galata::test::ulog_fixture(), "sample.ulg", rates_and_attitude());
  EXPECT_EQ(record.source_sha256.size(), 64u);
  EXPECT_EQ(record.timebase, galata::data::Timebase::UniformResampled);
  EXPECT_DOUBLE_EQ(record.sample_rate_hz, 250.0);
  // The grid is uniform at the declared rate, and does not run past the data.
  ASSERT_GT(record.times_s.size(), 2u);
  EXPECT_NEAR(record.times_s[1] - record.times_s[0], 1.0 / 250.0, 1e-12);
  const auto* channel = record.find("gyro_p_rad_s");
  ASSERT_NE(channel, nullptr);
  EXPECT_EQ(channel->samples.size(), record.times_s.size());
  EXPECT_EQ(channel->source_name, "vehicle_angular_velocity.xyz[0]");
}

TEST(UlogImport, WhatTheLogDoesNotHaveIsRefusedByName) {
  auto request = rates_and_attitude();
  request.channels[0].topic = "no_such_topic";
  EXPECT_THROW((void)read_ulog(galata::test::ulog_fixture(), "x.ulg", request),
               std::invalid_argument)
      << "a topic quietly absent is a fit performed on less data than the study thinks";

  request = rates_and_attitude();
  request.channels[0].field = "no_such_field";
  EXPECT_THROW((void)read_ulog(galata::test::ulog_fixture(), "x.ulg", request),
               std::invalid_argument);

  request = rates_and_attitude();
  request.channels[0].field = "xyz[7]";
  EXPECT_THROW((void)read_ulog(galata::test::ulog_fixture(), "x.ulg", request),
               std::invalid_argument)
      << "an index past the end of an array must be refused, not read from the next field";

  request = rates_and_attitude();
  request.channels[0].unit.clear();
  EXPECT_THROW((void)read_ulog(galata::test::ulog_fixture(), "x.ulg", request),
               std::invalid_argument);

  request = rates_and_attitude();
  request.resample_hz = 0.0;
  EXPECT_THROW((void)read_ulog(galata::test::ulog_fixture(), "x.ulg", request),
               std::invalid_argument)
      << "channels that do not share a timebase are not a record a fit can read";
}

TEST(UlogImport, AFileThatIsNotAUlogOrIsTruncatedIsRefused) {
  EXPECT_THROW((void)read_ulog("not a ulog at all", "x.ulg", rates_and_attitude()),
               std::invalid_argument);
  // A real header, then nothing: refused rather than returning an empty record
  // that a study would read as a flight during which nothing happened.
  const std::string whole = galata::test::ulog_fixture();
  EXPECT_THROW((void)read_ulog(whole.substr(0, 40), "x.ulg", rates_and_attitude()),
               std::invalid_argument);
}

// PX4 actuator outputs are normalised commands, not rotor speeds. The reader
// carries them as what they are, and any conversion is the study's declared
// scale — never an assumption made here.
TEST(UlogImport, ActuatorOutputsAreCarriedAsCommandsNotRotorSpeeds) {
  UlogImportRequest request;
  request.resample_hz = 100.0;
  request.channels = {
      {"actuator_motors", "control[0]", "motor_0_normalised", "1", "none", 1.0, 0.0, 0},
  };
  const auto record = read_ulog(galata::test::ulog_fixture(), "sample.ulg", request);
  const auto* motor = record.find("motor_0_normalised");
  ASSERT_NE(motor, nullptr);
  EXPECT_EQ(motor->unit, "1") << "a normalised command is dimensionless, not rad/s";
  EXPECT_NEAR(motor->samples.front(), 0.5, 1e-6);
  EXPECT_DOUBLE_EQ(motor->scale_applied, 1.0);
}
