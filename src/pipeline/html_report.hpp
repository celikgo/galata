// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_PIPELINE_HTML_REPORT_HPP
#define GALATA_PIPELINE_HTML_REPORT_HPP
#include "galata/pipeline/charts.hpp"

#include <string>
#include <string_view>

namespace galata::pipeline {
// Deliberately small Markdown subset used by our own report writers. All input
// text is escaped; HTML, scripts, images and remote embeds are never interpreted.
[[nodiscard]] std::string escape_html(std::string_view text);
[[nodiscard]] std::string standalone_html(std::string_view title,
                                          const std::string& markdown,
                                          const std::vector<TimeSeriesChart>& charts = {});
}  // namespace galata::pipeline
#endif
