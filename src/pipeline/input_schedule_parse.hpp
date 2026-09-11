// SPDX-License-Identifier: Apache-2.0
//
// The study schema for a declared input history, shared by every capability
// that integrates one — `sim.plant` for the nonlinear plant and `sim.linear` for
// a linear model — so that a schedule's timing means the same thing on either
// path. Its values are each model's own inputs, which for a linearisation are
// deviations from the trim.
//
//   input_schedule:
//     hold: zero_order          # or linear; required, no default
//     extrapolation: refuse     # or hold; required, no default
//     samples:
//       - {time_s: 0.0, values: [626.3, 626.3, 626.3, 626.3]}
//       - {time_s: 1.0, values: [645.1, 607.5, 607.5, 645.1]}
//
// The semantics — timestamps from the start of the run, what a hold means, which
// changes are events, what happens outside the span — are the integrators', and
// are written where they are implemented: include/galata/sim/schedule.hpp and
// include/galata/sim/linear.hpp.
#ifndef GALATA_PIPELINE_INPUT_SCHEDULE_PARSE_HPP
#define GALATA_PIPELINE_INPUT_SCHEDULE_PARSE_HPP

#include "galata/pipeline/registry.hpp"
#include "galata/sim/schedule.hpp"

#include <string>

namespace galata::pipeline {

// Absent or null returns an empty schedule. Refuses a missing or unknown hold or
// extrapolation, a missing `samples`, a sample without `values`, a width other
// than `expected_width`, and any key the schema does not name, at either level.
[[nodiscard]] sim::InputSchedule schedule_at(const StageContext& context,
                                             const std::string& key,
                                             int expected_width);

}  // namespace galata::pipeline

#endif  // GALATA_PIPELINE_INPUT_SCHEDULE_PARSE_HPP
