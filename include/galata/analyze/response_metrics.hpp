// SPDX-License-Identifier: Apache-2.0
//
// Response metrics independent of a particular vehicle or pipeline adapter.
// The caller supplies a sampled signal and its sampled reference; this module
// deliberately does not infer units or controller intent from a channel name.

#ifndef GALATA_ANALYZE_RESPONSE_METRICS_HPP
#define GALATA_ANALYZE_RESPONSE_METRICS_HPP

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace galata::analyze {

enum class SettlingStatus {
  DemonstratedRecovery,
  AlreadyWithinBand,
  NotSettledWithinObservationWindow,
};

struct ResponseMetricOptions {
  // The event time is the disturbance or reference-change boundary. Settling
  // duration is reported relative to this time, never as an absolute clock.
  double event_time_s = 0.0;
  // A dedicated absolute settling band, independent of peak-error criteria.
  double settling_band = 0.0;
  // The signal must remain in the band for this long before it is settled.
  double settling_dwell_s = 0.0;
  // When present, overshoot is measured relative to this step, not relative to
  // the uncontrolled response. A zero-amplitude step is inapplicable.
  double reference_before = 0.0;
  double reference_after = 0.0;
  bool has_reference_step = false;
};

struct ResponseMetricResult {
  double peak_tracking_error = 0.0;
  double final_tracking_error = 0.0;
  double rms_tracking_error = 0.0;
  double settling_duration_s = std::numeric_limits<double>::infinity();
  SettlingStatus settling_status = SettlingStatus::NotSettledWithinObservationWindow;
  bool overshoot_applicable = false;
  double overshoot_fraction = std::numeric_limits<double>::quiet_NaN();
};

// Analyze one constant-reference observation segment. The vectors must share
// a strictly non-decreasing time lattice and contain finite values. The
// segment starts at event_time_s (the first sample at or after it) and ends at
// the final supplied sample. A caller handling repeated reference changes
// invokes this once per segment.
[[nodiscard]] ResponseMetricResult analyze_response_segment(const std::vector<double>& times_s,
                                                            const std::vector<double>& reference,
                                                            const std::vector<double>& measured,
                                                            const ResponseMetricOptions& options);

[[nodiscard]] const char* to_string(SettlingStatus status) noexcept;

}  // namespace galata::analyze

#endif  // GALATA_ANALYZE_RESPONSE_METRICS_HPP
