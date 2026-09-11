// SPDX-License-Identifier: Apache-2.0
//
// Cutting a measured record into a window of itself.
//
// WHY THIS IS A CAPABILITY AND NOT A READER OPTION. An estimation/validation
// split is a statement about which observations trained a model and which did
// not, and the only way that statement can be CHECKED rather than believed is
// if both records demonstrably come from one file and cover stretches of it
// that do not overlap. A window declared inside `data.import.csv` would give the
// same numbers and would give them a second file digest, which proves nothing:
// see `include/galata/identify/validate.hpp` for what unequal digests do and do
// not establish. Cutting AFTER the import keeps one source identity and adds an
// interval to it, so the split carries its own evidence.
//
// It also means one implementation serves every reader. A ULog and a CSV are cut
// the same way, because by this point they are the same object.
//
// THE INTERVAL IS HALF-OPEN, [start, end). A sample exactly at `end_s` belongs to
// the next window and to no other. Adjacent windows therefore share no sample
// and lose none between them, which is the property that makes two of them
// provably disjoint.
//
// WHAT THIS IS NOT. Not a resampler and not a filter. It selects samples the
// record already holds; it does not interpolate the ends onto the window
// boundary, because a sample that was never measured is not evidence about
// anything. A window containing fewer than two samples is refused rather than
// returned, since nothing downstream can simulate through it.
#pragma once

#include "galata/data/record.hpp"

namespace galata::data {

// Refuses: a window that does not bracket, a non-finite bound, and a window
// that selects fewer than two of the record's samples — naming, in that last
// case, the interval the record actually covers, because the usual cause is a
// window written in the wrong units.
[[nodiscard]] Record window_record(const Record& record, double start_s, double end_s);

}  // namespace galata::data
