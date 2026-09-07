// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_PIPELINE_CHARTS_HPP
#define GALATA_PIPELINE_CHARTS_HPP
#include <cstddef>
#include <string>
#include <vector>

namespace galata::pipeline {
struct Artifact;

struct TimeSeries {
  std::string label;
  std::vector<double> values;
};

// Group only quantities sharing a physical unit. Values are displayed as
// provided; no normalization, unit conversion or stability inference is made.
struct TimeSeriesChart {
  std::string title;
  std::string y_label;
  std::vector<double> times_s;
  std::vector<TimeSeries> series;
};

[[nodiscard]] std::vector<TimeSeriesChart> design_charts(const Artifact& artifact);

// Escaped, accessible standalone SVG figure. Long histories retain bucket
// minima/maxima for display; the chart caption identifies this reduction.
[[nodiscard]] std::string render_timeseries_svg(const TimeSeriesChart& chart,
                                                std::size_t chart_index = 0);
}  // namespace galata::pipeline
#endif
