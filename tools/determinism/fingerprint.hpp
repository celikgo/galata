// SPDX-License-Identifier: Apache-2.0
//
// The determinism fingerprint battery, as a library.
//
// Extracted from the tool's main() so that two callers can share one
// definition of "what is fingerprinted": the `galata-determinism` binary,
// which prints it, and the V&V report generator, which needs to state how many
// values the determinism gate covers without that number being typed by hand.
//
// A second implementation of the battery would be a second answer to the same
// question, and the report would be describing a fingerprint nobody takes.

#ifndef GALATA_TOOLS_DETERMINISM_FINGERPRINT_HPP
#define GALATA_TOOLS_DETERMINISM_FINGERPRINT_HPP

#include <functional>
#include <string>

namespace galata::determinism {

// Called once per fingerprinted value, in a fixed order.
using Emit = std::function<void(const std::string& key, double value)>;

// Runs the whole battery. `model_path` is the aircraft the trim and
// linearisation cases use.
void fingerprint(const std::string& model_path, const Emit& emit);

// How many values the battery emits, and how many of those are excluded from
// the cross-platform tier because they sit downstream of a finite difference.
struct Counts {
  int total = 0;
  int tier1_only = 0;

  [[nodiscard]] int cross_platform() const {
    return total - tier1_only;
  }
};

[[nodiscard]] Counts fingerprint_counts(const std::string& model_path);

}  // namespace galata::determinism

#endif  // GALATA_TOOLS_DETERMINISM_FINGERPRINT_HPP
