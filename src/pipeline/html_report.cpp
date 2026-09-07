// SPDX-License-Identifier: Apache-2.0
#include "html_report.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

namespace galata::pipeline {
std::string escape_html(std::string_view text) {
  std::string out;
  for (const char c : text) {
    switch (c) {
      case '&':
        out += "&amp;";
        break;
      case '<':
        out += "&lt;";
        break;
      case '>':
        out += "&gt;";
        break;
      case '"':
        out += "&quot;";
        break;
      case '\'':
        out += "&#39;";
        break;
      default:
        out += c;
    }
  }
  return out;
}

namespace {
std::string inline_text(std::string_view text, int depth = 0) {
  if (depth > 16) {
    return escape_html(text);
  }
  std::string out;
  for (std::size_t i = 0; i < text.size();) {
    if (text[i] == '\\' && i + 1 < text.size()) {
      out += escape_html(text.substr(i + 1, 1));
      i += 2;
      continue;
    }
    const bool bold = text.substr(i, 2) == "**";
    const bool code = text[i] == '`';
    const bool italic =
        !bold && (text[i] == '*' || text[i] == '_') && (i == 0 || text[i - 1] == ' ');
    if (bold || code || italic) {
      const std::size_t width = bold ? 2 : 1;
      const auto marker = text.substr(i, width);
      const auto end = text.find(marker, i + width);
      if (end != std::string_view::npos) {
        const char* tag = bold ? "strong" : (code ? "code" : "em");
        out += '<';
        out += tag;
        out += '>';
        // Escape even within emphasis/code; no raw markup is trusted.
        const auto content = text.substr(i + width, end - i - width);
        out += code ? escape_html(content) : inline_text(content, depth + 1);
        out += "</";
        out += tag;
        out += '>';
        i = end + width;
        continue;
      }
    }
    out += escape_html(text.substr(i, 1));
    ++i;
  }
  return out;
}

std::vector<std::string> cells(const std::string& line) {
  std::vector<std::string> result;
  std::string cell;
  bool escaped = false;
  for (std::size_t i = 1; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '|' && !escaped) {
      result.push_back(cell);
      cell.clear();
    } else {
      cell += c;
    }
    escaped = c == '\\' && !escaped;
  }
  if (!cell.empty()) {
    result.push_back(cell);
  }
  return result;
}

bool separator(const std::string& line) {
  const auto parsed = cells(line);
  return !parsed.empty() && std::all_of(parsed.begin(), parsed.end(), [](const auto& cell) {
    return cell.find('-') != std::string::npos
           && cell.find_first_not_of(" -:\t\r") == std::string::npos;
  });
}

void row(std::ostream& out, const std::string& line, const char* tag) {
  out << "<tr>";
  for (const auto& cell : cells(line)) {
    out << '<' << tag << '>' << inline_text(cell) << "</" << tag << '>';
  }
  out << "</tr>\n";
}
}  // namespace

std::string standalone_html(std::string_view title,
                            const std::string& markdown,
                            const std::vector<TimeSeriesChart>& charts) {
  std::vector<std::string> lines;
  std::istringstream input(markdown);
  for (std::string line; std::getline(input, line);) {
    lines.push_back(std::move(line));
  }
  std::ostringstream out;
  out << "<!doctype html>\n<html lang=\"en\"><head><meta charset=\"utf-8\">\n"
         "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
         "<meta http-equiv=\"Content-Security-Policy\" content=\"default-src 'none'; "
         "style-src 'unsafe-inline'; base-uri 'none'; form-action 'none'\">\n"
         "<title>"
      << escape_html(title)
      << "</title>\n<style>\n"
         ":root{color-scheme:light;font:16px/1.6 system-ui,sans-serif;color:#172b40;"
         "background:#f0f4f8}*{box-sizing:border-box}body{margin:0;padding:40px 20px}"
         "main{max-width:1120px;margin:auto;background:#fff;padding:40px 48px;"
         "border:1px solid #d9e2ed;border-radius:12px}h1{line-height:1.2;font-size:2rem;"
         "letter-spacing:-.03em;margin:0 0 32px}h2{font-size:1.3rem;margin:40px 0 16px;"
         "padding-top:20px;border-top:2px solid #dae5ef}h3{font-size:1.05rem;margin:24px 0 12px}"
         "p{max-width:90ch}code{font-size:.9em;"
         "background:#edf3f8;padding:.1em .3em;border-radius:3px;overflow-wrap:anywhere}"
         "pre{overflow-x:auto;background:#edf3f8;padding:16px;border-radius:6px;line-height:1.5}"
         "pre code{padding:0;white-space:pre;background:none}blockquote{margin:20px 0;"
         "padding:8px 20px;border-left:4px solid #ba6b22;background:#fff7ed}"
         ".charts{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,420px),1fr));"
         "gap:18px}.chart{margin:0;border:1px solid #dce5ed;border-radius:8px;padding:16px;"
         "min-width:0;break-inside:avoid}.chart h3{margin:0 0 8px;font-size:1rem}"
         ".chart svg{display:block;width:100%;height:auto}.chart figcaption{font-size:.75rem;"
         "color:#516379}.chart-legend{display:flex;flex-wrap:wrap;gap:5px 16px;list-style:none;"
         "padding:0;margin:0 0 8px;font-size:.8rem}.chart-legend span{display:inline-block;"
         "width:18px;border-top:3px solid;margin:0 5px 3px 0}"
         ".table{overflow-x:auto;margin:20px 0}table{border-collapse:collapse;min-width:65%;"
         "font-size:.875rem;font-variant-numeric:tabular-nums}th,td{padding:9px 13px;"
         "border-bottom:1px solid #dce5ed;text-align:right;white-space:nowrap}th:first-child,"
         "td:first-child{text-align:left}th{background:#eaf1f7;font-weight:650}"
         "tbody tr:nth-child(even){background:#f7fafc}em{color:#516379}hr{border:0;"
         "border-top:1px solid #dce5ed;margin:32px 0}footer{font-size:.8rem;color:#60758a;"
         "margin-top:32px}@media(max-width:640px){body{padding:12px}main{padding:24px 18px}"
         "h1{font-size:1.6rem}table{min-width:100%}}@media print{body{padding:0;background:#fff}"
         "main{padding:0;border:0;max-width:none}h2{break-after:avoid}tr{break-inside:avoid}"
         ".table{overflow:visible}th,td{white-space:normal;padding:5px;font-size:9pt}}\n"
         "</style></head><body><main>\n";
  for (std::size_t i = 0; i < lines.size();) {
    const auto& line = lines[i];
    if (line.empty()) {
      ++i;
      continue;
    }
    if (line.rfind("```", 0) == 0) {
      out << "<pre><code>";
      ++i;
      while (i < lines.size() && lines[i].rfind("```", 0) != 0) {
        out << escape_html(lines[i++]) << '\n';
      }
      if (i < lines.size()) {
        ++i;
      }
      out << "</code></pre>\n";
      continue;
    }
    if (line.front() == '>') {
      std::string quote;
      while (i < lines.size() && !lines[i].empty() && lines[i].front() == '>') {
        if (!quote.empty()) {
          quote += ' ';
        }
        quote += lines[i++].substr(1);
      }
      out << "<blockquote><p>" << inline_text(quote) << "</p></blockquote>\n";
      continue;
    }
    if (line.front() == '|' && i + 1 < lines.size() && separator(lines[i + 1])) {
      out << "<div class=\"table\"><table><thead>";
      row(out, line, "th");
      out << "</thead><tbody>\n";
      i += 2;
      while (i < lines.size() && !lines[i].empty() && lines[i].front() == '|') {
        row(out, lines[i++], "td");
      }
      out << "</tbody></table></div>\n";
      continue;
    }
    if (line == "---") {
      out << "<hr>\n";
      ++i;
      continue;
    }
    const auto level = line.find_first_not_of('#');
    if (level >= 1 && level <= 6 && level < line.size() && line[level] == ' ') {
      out << "<h" << level << '>' << inline_text(line.substr(level + 1)) << "</h" << level << ">\n";
      ++i;
      continue;
    }
    std::string paragraph = line;
    ++i;
    while (i < lines.size() && !lines[i].empty() && lines[i].front() != '|'
           && lines[i].front() != '#' && lines[i].front() != '>' && lines[i].rfind("```", 0) != 0
           && lines[i] != "---") {
      paragraph += ' ';
      paragraph += lines[i++];
    }
    out << "<p>" << inline_text(paragraph) << "</p>\n";
  }
  if (!charts.empty()) {
    out << "<h2>Time histories</h2>\n"
           "<p>Each plot uses the recorded simulation samples, with values and units from its "
           "source artifact. Consult the corresponding study sections and CSV files for "
           "initial conditions, termination status and complete sampled histories.</p>\n"
           "<div class=\"charts\">\n";
    for (std::size_t i = 0; i < charts.size(); ++i) {
      out << render_timeseries_svg(charts[i], i);
    }
    out << "</div>\n";
  }
  out << "<footer>Self-contained offline report. A completed run writes a manifest recording "
         "exact inputs, output hashes and the executable that produced this study.</footer>\n"
         "</main></body></html>\n";
  return out.str();
}
}  // namespace galata::pipeline
