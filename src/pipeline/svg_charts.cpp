// SPDX-License-Identifier: Apache-2.0
#include "galata/pipeline/charts.hpp"

#include "html_report.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace galata::pipeline {
namespace {
constexpr std::array<const char*, 6> colors{"#1768a6",
                                            "#bf5b17",
                                            "#27805d",
                                            "#884baa",
                                            "#ac3d64",
                                            "#657a17"};

std::string number(double value) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(4) << (value == 0 ? 0 : value);
  return out.str();
}

std::vector<std::size_t> display_indices(const std::vector<double>& values) {
  std::vector<std::size_t> result;
  if (values.size() <= 2002) {
    result.resize(values.size());
    std::iota(result.begin(), result.end(), std::size_t{0});
    return result;
  }
  result.push_back(0);
  const auto width = (values.size() - 2 + 499) / 500;
  for (std::size_t first = 1; first < values.size() - 1; first += width) {
    const auto last = std::min(first + width, values.size() - 1);
    auto minimum = first;
    auto maximum = first;
    for (auto i = first; i < last; ++i) {
      if (values[i] < values[minimum]) {
        minimum = i;
      }
      if (values[i] > values[maximum]) {
        maximum = i;
      }
    }
    std::array<std::size_t, 4> bucket{first, minimum, maximum, last - 1};
    std::sort(bucket.begin(), bucket.end());
    for (const auto i : bucket) {
      if (result.back() != i) {
        result.push_back(i);
      }
    }
  }
  result.push_back(values.size() - 1);
  return result;
}
}  // namespace

std::string render_timeseries_svg(const TimeSeriesChart& chart, std::size_t chart_index) {
  if (chart.times_s.empty() || chart.series.empty()) {
    throw std::invalid_argument("time-history chart requires samples and at least one series");
  }
  double y_min = std::numeric_limits<double>::infinity();
  double y_max = -y_min;
  for (std::size_t i = 0; i < chart.times_s.size(); ++i) {
    if (!std::isfinite(chart.times_s[i]) || (i > 0 && chart.times_s[i] <= chart.times_s[i - 1])) {
      throw std::invalid_argument("time-history chart requires finite, increasing sample times");
    }
  }
  for (const auto& series : chart.series) {
    if (series.values.size() != chart.times_s.size()) {
      throw std::invalid_argument("time-history chart sample dimensions disagree");
    }
    for (const auto value : series.values) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("time-history chart has a non-finite sample");
      }
      y_min = std::min(y_min, value);
      y_max = std::max(y_max, value);
    }
  }
  // Normalize before differencing to avoid overflow even where long double has
  // the same range as double. Constant traces receive a visible axis span.
  const long double y_scale =
      static_cast<long double>(std::max({1.0, std::abs(y_min), std::abs(y_max)}));
  long double low = static_cast<long double>(y_min) / y_scale;
  long double high = static_cast<long double>(y_max) / y_scale;
  const auto pad =
      high == low ? std::max(1.0L / y_scale, std::abs(low) * 0.05L) : (high - low) * 0.05L;
  low -= pad;
  high += pad;
  const long double t_scale = static_cast<long double>(
      std::max({1.0, std::abs(chart.times_s.front()), std::abs(chart.times_s.back())}));
  long double start = static_cast<long double>(chart.times_s.front()) / t_scale;
  long double end = static_cast<long double>(chart.times_s.back()) / t_scale;
  if (start == end) {
    start -= 0.5L;
    end += 0.5L;
  }
  constexpr double left = 78, right = 548, top = 24, bottom = 242;
  const auto x = [&](double time) {
    return left
           + static_cast<double>((static_cast<long double>(time) / t_scale - start) / (end - start))
                 * (right - left);
  };
  const auto y = [&](double value) {
    return bottom
           - static_cast<double>((static_cast<long double>(value) / y_scale - low) / (high - low))
                 * (bottom - top);
  };
  const auto axis_value = [](long double normalized, long double scale) {
    const long double limit = static_cast<long double>(std::numeric_limits<double>::max());
    return static_cast<double>(std::clamp(normalized, -limit / scale, limit / scale) * scale);
  };
  const auto id = "history-" + std::to_string(chart_index);
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "<figure class=\"chart\"><h3>" << escape_html(chart.title)
      << "</h3><ul class=\"chart-legend\">";
  for (std::size_t i = 0; i < chart.series.size(); ++i) {
    out << "<li><span style=\"border-color:" << colors[i % colors.size()] << "\"></span>"
        << escape_html(chart.series[i].label) << "</li>";
  }
  out << "</ul><svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 580 292\" "
         "role=\"img\" aria-labelledby=\""
      << id << "-title " << id << "-description\"><title id=\"" << id << "-title\">"
      << escape_html(chart.title) << "</title><desc id=\"" << id << "-description\">"
      << chart.times_s.size() << " recorded samples from " << number(chart.times_s.front())
      << " to " << number(chart.times_s.back())
      << " seconds. Vertical axis: " << escape_html(chart.y_label) << ".</desc>\n";
  out << "<g font-family=\"system-ui,sans-serif\" font-size=\"12\" fill=\"#516379\">\n";
  for (int i = 0; i <= 4; ++i) {
    const double fraction = static_cast<double>(i) / 4.0;
    const auto xx = left + fraction * (right - left);
    const auto yy = bottom - fraction * (bottom - top);
    out << "<path d=\"M" << left << ',' << yy << "H" << right << "\" stroke=\"#dce5ed\"/><text x=\""
        << left - 8 << "\" y=\"" << yy + 4 << "\" text-anchor=\"end\">"
        << number(axis_value(low + static_cast<long double>(fraction) * (high - low), y_scale))
        << "</text>\n"
        << "<text x=\"" << xx << "\" y=\"" << bottom + 22 << "\" text-anchor=\"middle\">"
        << number(axis_value(start + static_cast<long double>(fraction) * (end - start), t_scale))
        << "</text>\n";
  }
  out << "<text x=\"313\" y=\"286\" text-anchor=\"middle\">Time (s)</text>"
         "<text transform=\"translate(16 133) rotate(-90)\" text-anchor=\"middle\">"
      << escape_html(chart.y_label) << "</text></g>\n"
      << "<path d=\"M" << left << ',' << top << 'V' << bottom << 'H' << right
      << "\" fill=\"none\" stroke=\"#879caf\"/>\n";
  for (std::size_t i = 0; i < chart.series.size(); ++i) {
    const auto& series = chart.series[i];
    out << "<polyline fill=\"none\" stroke=\"" << colors[i % colors.size()]
        << "\" stroke-width=\"1.8\" stroke-linejoin=\"round\" points=\"";
    out << std::fixed << std::setprecision(2);
    for (const auto index : display_indices(series.values)) {
      out << x(chart.times_s[index]) << ',' << y(series.values[index]) << ' ';
    }
    out << "\"/>\n";
    if (chart.times_s.size() == 1) {
      out << "<circle cx=\"" << x(chart.times_s.front()) << "\" cy=\"" << y(series.values.front())
          << "\" r=\"3\" fill=\"" << colors[i % colors.size()] << "\"/>\n";
    }
  }
  out << "</svg><figcaption>" << chart.times_s.size() << " recorded samples. ";
  if (chart.times_s.size() > 2002) {
    out << "Display reduced to at most 2,002 points per series, preserving each bucket's "
           "minimum and maximum. ";
  }
  out << "Each vertical axis is scaled independently.</figcaption></figure>\n";
  return out.str();
}
}  // namespace galata::pipeline
