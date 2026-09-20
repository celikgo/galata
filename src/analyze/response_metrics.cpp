// SPDX-License-Identifier: Apache-2.0

#include "galata/analyze/response_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace galata::analyze {
namespace {

std::size_t event_index(const std::vector<double>& times_s, double event_time_s) {
  const auto found = std::lower_bound(times_s.begin(), times_s.end(), event_time_s);
  return static_cast<std::size_t>(found - times_s.begin());
}

void validate(const std::vector<double>& times_s,
              const std::vector<double>& reference,
              const std::vector<double>& measured,
              const ResponseMetricOptions& options) {
  if (times_s.empty() || times_s.size() != reference.size() || times_s.size() != measured.size()) {
    throw std::invalid_argument(
        "analyze_response_segment requires equal, non-empty time/reference/measured vectors");
  }
  if (!std::isfinite(options.event_time_s) || !std::isfinite(options.settling_band)
      || options.settling_band < 0.0 || !std::isfinite(options.settling_dwell_s)
      || options.settling_dwell_s < 0.0 || !std::isfinite(options.reference_before)
      || !std::isfinite(options.reference_after)) {
    throw std::invalid_argument(
        "analyze_response_segment options must be finite and non-negative where declared");
  }
  for (std::size_t i = 0; i < times_s.size(); ++i) {
    if (!std::isfinite(times_s[i]) || !std::isfinite(reference[i]) || !std::isfinite(measured[i])) {
      throw std::invalid_argument("analyze_response_segment rejects missing or non-finite samples");
    }
    if (i != 0 && times_s[i] < times_s[i - 1]) {
      throw std::invalid_argument("analyze_response_segment requires non-decreasing time samples");
    }
  }
}

}  // namespace

ResponseMetricResult analyze_response_segment(const std::vector<double>& times_s,
                                              const std::vector<double>& reference,
                                              const std::vector<double>& measured,
                                              const ResponseMetricOptions& options) {
  validate(times_s, reference, measured, options);
  const std::size_t first = event_index(times_s, options.event_time_s);
  if (first >= times_s.size()) {
    throw std::invalid_argument(
        "analyze_response_segment event_time_s is after the observation window");
  }

  ResponseMetricResult result;
  double squared_error = 0.0;
  std::size_t sample_count = 0;
  for (std::size_t i = first; i < times_s.size(); ++i) {
    const double error = measured[i] - reference[i];
    result.peak_tracking_error = std::max(result.peak_tracking_error, std::fabs(error));
    squared_error += error * error;
    ++sample_count;
  }
  result.final_tracking_error = std::fabs(measured.back() - reference.back());
  result.rms_tracking_error = std::sqrt(squared_error / static_cast<double>(sample_count));

  const double initial_error = std::fabs(measured[first] - reference[first]);
  if (initial_error <= options.settling_band) {
    result.settling_duration_s = 0.0;
    result.settling_status = SettlingStatus::AlreadyWithinBand;
  } else {
    for (std::size_t i = first; i < times_s.size(); ++i) {
      if (std::fabs(measured[i] - reference[i]) > options.settling_band) {
        continue;
      }
      const double required_end = times_s[i] + options.settling_dwell_s;
      const auto end = std::upper_bound(
          times_s.begin() + static_cast<std::ptrdiff_t>(i), times_s.end(), required_end);
      const std::size_t end_index = static_cast<std::size_t>(end - times_s.begin());
      if (end_index == 0 || end_index > times_s.size()) {
        continue;
      }
      bool remains_in_band = true;
      for (std::size_t j = i; j < end_index; ++j) {
        if (std::fabs(measured[j] - reference[j]) > options.settling_band) {
          remains_in_band = false;
          break;
        }
      }
      if (remains_in_band
          && (end_index == times_s.size() || times_s[end_index - 1] >= required_end)) {
        result.settling_duration_s = times_s[i] - options.event_time_s;
        result.settling_status = SettlingStatus::DemonstratedRecovery;
        break;
      }
    }
  }

  if (options.has_reference_step) {
    const double amplitude = options.reference_after - options.reference_before;
    if (std::fabs(amplitude) > 0.0) {
      result.overshoot_applicable = true;
      double signed_excursion = 0.0;
      for (std::size_t i = first; i < times_s.size(); ++i) {
        const double signed_response = measured[i] - options.reference_after;
        signed_excursion =
            std::max(signed_excursion, (amplitude > 0.0 ? signed_response : -signed_response));
      }
      result.overshoot_fraction = std::max(0.0, signed_excursion) / std::fabs(amplitude);
    }
  }
  return result;
}

const char* to_string(SettlingStatus status) noexcept {
  switch (status) {
    case SettlingStatus::DemonstratedRecovery:
      return "demonstrated_recovery";
    case SettlingStatus::AlreadyWithinBand:
      return "already_within_band";
    case SettlingStatus::NotSettledWithinObservationWindow:
      return "not_settled_within_observation_window";
  }
  return "unknown";
}

}  // namespace galata::analyze
