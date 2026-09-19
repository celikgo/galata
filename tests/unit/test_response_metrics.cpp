// SPDX-License-Identifier: Apache-2.0

#include "galata/analyze/response_metrics.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

using galata::analyze::ResponseMetricOptions;
using galata::analyze::SettlingStatus;
using galata::analyze::analyze_response_segment;

TEST(ResponseMetrics, KnownStepHasIndependentRmsSettlingAndPositiveOvershoot) {
  const std::vector<double> time{0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  const std::vector<double> reference{0.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  const std::vector<double> measured{0.0, 0.4, 1.2, 1.04, 1.0, 1.0};
  ResponseMetricOptions options;
  options.event_time_s = 1.0;
  options.settling_band = 0.05;
  options.settling_dwell_s = 1.0;
  options.reference_before = 0.0;
  options.reference_after = 1.0;
  options.has_reference_step = true;

  const auto result = analyze_response_segment(time, reference, measured, options);
  EXPECT_NEAR(result.peak_tracking_error, 0.6, 1.0e-12);
  EXPECT_NEAR(result.final_tracking_error, 0.0, 1.0e-12);
  EXPECT_NEAR(result.rms_tracking_error, std::sqrt((0.6 * 0.6 + 0.2 * 0.2 + 0.04 * 0.04) / 5.0),
              1.0e-12);
  EXPECT_EQ(result.settling_status, SettlingStatus::DemonstratedRecovery);
  EXPECT_NEAR(result.settling_duration_s, 2.0, 1.0e-12);
  EXPECT_TRUE(result.overshoot_applicable);
  EXPECT_NEAR(result.overshoot_fraction, 0.2, 1.0e-12);
}
// repeated-step metric coverage follows in the integration contract
TEST(ResponseMetrics, RepeatedStepsAreMeasuredPerSegmentAndOscillationSettlesAfterDwell) {
  ResponseMetricOptions first;
  first.event_time_s = 1.0;
  first.settling_band = 0.1;
  first.settling_dwell_s = 1.0;
  first.reference_before = 0.0;
  first.reference_after = 1.0;
  first.has_reference_step = true;
  const std::vector<double> first_time{1.0, 2.0, 3.0};
  const std::vector<double> first_reference{1.0, 1.0, 1.0};
  const std::vector<double> first_measured{0.2, 1.3, 0.0};
  const auto first_result = analyze_response_segment(first_time, first_reference, first_measured, first);
  EXPECT_EQ(first_result.settling_status, SettlingStatus::NotSettledWithinObservationWindow);
  EXPECT_NEAR(first_result.overshoot_fraction, 0.3, 1.0e-12);
  ResponseMetricOptions second = first;
  second.event_time_s = 3.0;
  second.reference_before = 1.0;
  second.reference_after = -1.0;
  const std::vector<double> second_time{3.0, 4.0, 5.0, 6.0};
  const std::vector<double> second_reference{-1.0, -1.0, -1.0, -1.0};
  const std::vector<double> second_measured{0.0, -1.4, -1.04, -1.0};
  const auto second_result = analyze_response_segment(second_time, second_reference, second_measured, second);
  EXPECT_EQ(second_result.settling_status, SettlingStatus::DemonstratedRecovery);
  EXPECT_NEAR(second_result.settling_duration_s, 2.0, 1.0e-12);
  EXPECT_NEAR(second_result.overshoot_fraction, 0.2, 1.0e-12);
}

TEST(ResponseMetrics, NegativeAndZeroStepsAreHandledExplicitly) {
  const std::vector<double> time{0.0, 1.0, 2.0};
  const std::vector<double> reference{0.0, -2.0, -2.0};
  const std::vector<double> measured{0.0, -2.5, -2.0};
  ResponseMetricOptions negative;
  negative.event_time_s = 1.0;
  negative.settling_band = 0.1;
  negative.settling_dwell_s = 0.0;
  negative.reference_before = 0.0;
  negative.reference_after = -2.0;
  negative.has_reference_step = true;
  const auto negative_result = analyze_response_segment(time, reference, measured, negative);
  EXPECT_NEAR(negative_result.overshoot_fraction, 0.25, 1.0e-12);

  ResponseMetricOptions zero = negative;
  zero.reference_before = -2.0;
  zero.reference_after = -2.0;
  const auto zero_result = analyze_response_segment(time, reference, measured, zero);
  EXPECT_FALSE(zero_result.overshoot_applicable);
  EXPECT_TRUE(std::isnan(zero_result.overshoot_fraction));
}

TEST(ResponseMetrics, AlreadyWithinBandIsNotClaimedAsRecovery) {
  const std::vector<double> time{0.0, 1.0, 2.0};
  const std::vector<double> reference{0.0, 0.0, 0.0};
  const std::vector<double> measured{0.01, 0.01, 0.01};
  ResponseMetricOptions options;
  options.settling_band = 0.02;
  options.settling_dwell_s = 1.0;
  const auto result = analyze_response_segment(time, reference, measured, options);
  EXPECT_EQ(result.settling_status, SettlingStatus::AlreadyWithinBand);
  EXPECT_DOUBLE_EQ(result.settling_duration_s, 0.0);
}

TEST(ResponseMetrics, MissingAndNeverSettledSamplesAreRejectedOrReported) {
  const std::vector<double> time{0.0, 1.0, 2.0};
  const std::vector<double> reference{0.0, 0.0, 0.0};
  const std::vector<double> never{1.0, 0.8, 0.7};
  ResponseMetricOptions options;
  options.settling_band = 0.1;
  options.settling_dwell_s = 0.0;
  const auto result = analyze_response_segment(time, reference, never, options);
  EXPECT_EQ(result.settling_status, SettlingStatus::NotSettledWithinObservationWindow);
  EXPECT_TRUE(std::isinf(result.settling_duration_s));

  auto missing = never;
  missing[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)analyze_response_segment(time, reference, missing, options),
               std::invalid_argument);
}
