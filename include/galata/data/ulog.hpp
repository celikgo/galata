// SPDX-License-Identifier: Apache-2.0
//
// Reading a measured record from a PX4 ULog file (ADR-0016).
//
// WHAT IS SUPPORTED, declared rather than discovered. The container's
// definitions and data sections; the scalar field types PX4 emits and
// one-dimensional arrays of them. Message types this reader does not use are
// skipped by their own length, which the format permits; what is NOT permitted
// to pass silently is a requested topic or field the log does not contain, or a
// field type this reader cannot size. Both are refused by name, because a topic
// quietly absent from a record is a fit performed on less data than the study
// thinks it used.
//
// WHAT IS NOT SUPPORTED, and says so: appended data — a non-zero incompat flag,
// which is what a log truncated by a crash carries — and any format version
// other than 1.
//
// PX4 ACTUATOR OUTPUTS ARE NOT ROTOR SPEEDS. `actuator_motors.control` is a
// normalised command, not rad/s, and this reader will not pretend otherwise. A
// channel drawn from it declares its own scale and offset like any other, and
// turning it into a rotor speed needs a calibration nobody here has measured.
#pragma once

#include "galata/data/record.hpp"

#include <string>
#include <vector>

namespace galata::data {

// One field of one topic, and what galata will call it.
struct UlogChannel {
  std::string topic;  // e.g. "vehicle_angular_velocity"
  std::string field;  // e.g. "xyz[0]" — an array element is named, never flattened
  std::string name;   // the record's channel name
  std::string unit;   // required: a number without a unit is not a measurement
  std::string frame;  // required; "none" for a scalar that has none
  double scale = 1.0;
  double offset = 0.0;
  int multi_id = 0;  // which instance, when a topic is logged more than once
};

struct UlogImportRequest {
  std::vector<UlogChannel> channels;
  // The uniform grid every channel is placed on. Required: topics arrive on
  // their own timestamps at their own rates, and a record whose channels do not
  // share a timebase is not one a fit can read.
  double resample_hz = 0.0;
  // Between grid points the last sample AT OR BEFORE the grid time is used —
  // zero-order hold, no interpolation. The grid is clipped to the span every
  // requested channel covers, so no channel is asked for a value it never had.
  std::string description;
};

[[nodiscard]] Record read_ulog(const std::string& bytes,
                               const std::string& source_path,
                               const UlogImportRequest& request);

}  // namespace galata::data
