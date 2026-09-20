// SPDX-License-Identifier: Apache-2.0
//
// Piecewise-linear and piecewise-constant table lookup on a rectangular grid.
//
// Reference:
//   C. de Boor, "A Practical Guide to Splines", revised ed., Springer, 2001.
//   W. H. Press et al., "Numerical Recipes", 3rd ed., Cambridge, 2007, §3.6.
//
// Validity envelope and known error behaviour are in the header's
// "WHAT THIS IS NOT" block. The interpolation is exact at every breakpoint and
// second-order accurate between them, with error bounded by |f''| h^2 / 8 on
// each interval; the second derivative is the table author's problem, not this
// file's.
//
// DETERMINISM. The bracket search is a binary search over a strictly increasing
// vector, so it visits the same indices in the same order for the same
// argument. The interpolation is written as a single fused expression with a
// fixed grouping, because floating-point addition is not associative and
// ADR-0004's bit-identity tier is a claim about this expression.

#include "galata/numerics/table.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace galata::numerics {
namespace {

std::string number(double value) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(6) << value;
  return out.str();
}

void validate_axis(const std::string& table_name,
                   const char* axis,
                   const std::vector<double>& breakpoints) {
  if (breakpoints.size() < 2) {
    throw std::invalid_argument("table '" + table_name + "': the " + axis + " axis has "
                                + std::to_string(breakpoints.size())
                                + " breakpoints; at least two are needed to interpolate between");
  }
  for (std::size_t i = 0; i < breakpoints.size(); ++i) {
    if (!std::isfinite(breakpoints[i])) {
      throw std::invalid_argument("table '" + table_name + "': " + axis + " breakpoint "
                                  + std::to_string(i) + " is not finite");
    }
    if (i > 0 && !(breakpoints[i] > breakpoints[i - 1])) {
      throw std::invalid_argument(
          "table '" + table_name + "': " + axis + " breakpoints must strictly increase, but entry "
          + std::to_string(i) + " (" + number(breakpoints[i]) + ") does not exceed entry "
          + std::to_string(i - 1) + " (" + number(breakpoints[i - 1])
          + "). A repeated breakpoint is ambiguous rather than redundant: it is how a step gets "
            "written, and reading it as either value picks a different function");
    }
  }
}

// Index of the interval containing `argument`, and the fraction along it.
// Precondition: argument is inside [front, back] and the axis is valid.
struct Bracket {
  std::size_t lower = 0;
  double fraction = 0.0;  // in [0, 1]
};

Bracket bracket_of(const std::vector<double>& breakpoints, double argument) {
  // upper_bound gives the first breakpoint strictly greater than the argument,
  // so the interval below it is the one that contains it. At the last
  // breakpoint exactly, this lands one past the end and the clamp below puts
  // the query on the final interval at fraction 1 — which is the same value,
  // reached without a special case.
  const auto it = std::upper_bound(breakpoints.begin(), breakpoints.end(), argument);
  std::size_t upper = static_cast<std::size_t>(it - breakpoints.begin());
  upper = std::clamp<std::size_t>(upper, 1, breakpoints.size() - 1);
  const std::size_t lower = upper - 1;
  const double span = breakpoints[upper] - breakpoints[lower];
  // span > 0 is guaranteed by validate_axis.
  return {lower, (argument - breakpoints[lower]) / span};
}

// Applies the declared outside rule. Returns the argument to interpolate at,
// having updated the counters. `Refuse` throws here.
double apply_outside_rule(const std::string& table_name,
                          const char* axis,
                          const std::vector<double>& breakpoints,
                          double argument,
                          TableExtrapolation rule,
                          std::uint64_t& out_of_range,
                          double& worst_excursion) {
  const double low = breakpoints.front();
  const double high = breakpoints.back();
  if (argument >= low && argument <= high) {
    return argument;
  }
  const double excursion = argument < low ? low - argument : argument - high;
  ++out_of_range;
  worst_excursion = std::max(worst_excursion, excursion);

  switch (rule) {
    case TableExtrapolation::Refuse:
      throw std::out_of_range(
          "table '" + table_name + "': " + axis + " argument " + number(argument)
          + " is outside the measured span [" + number(low) + ", " + number(high) + "] by "
          + number(excursion)
          + ", and this table declares extrapolation: refuse. The data does not say what the "
            "value is there; clamping would be a claim the measurement never made");
    case TableExtrapolation::Hold:
      return argument < low ? low : high;
    case TableExtrapolation::Linear:
      return argument;  // the interpolator's bracket clamp continues the end slope
  }
  return argument;
}

}  // namespace

std::string to_string(TableInterpolation rule) {
  switch (rule) {
    case TableInterpolation::Linear:
      return "linear";
    case TableInterpolation::Nearest_Low:
      return "nearest_low";
  }
  return "unknown";
}

std::string to_string(TableExtrapolation rule) {
  switch (rule) {
    case TableExtrapolation::Hold:
      return "hold";
    case TableExtrapolation::Refuse:
      return "refuse";
    case TableExtrapolation::Linear:
      return "linear";
  }
  return "unknown";
}

// --------------------------------------------------------------------------
// Table1D
// --------------------------------------------------------------------------

Table1D::Table1D(std::string name,
                 std::vector<double> breakpoints,
                 std::vector<double> values,
                 TableInterpolation interpolation,
                 TableExtrapolation extrapolation)
    : name_(std::move(name)), breakpoints_(std::move(breakpoints)), values_(std::move(values)),
      interpolation_(interpolation), extrapolation_(extrapolation) {
  if (name_.empty()) {
    throw std::invalid_argument(
        "table: a table needs a name; it is what an out-of-range refusal reports");
  }
  validate_axis(name_, "argument", breakpoints_);
  if (values_.size() != breakpoints_.size()) {
    throw std::invalid_argument("table '" + name_ + "': has " + std::to_string(breakpoints_.size())
                                + " breakpoints and " + std::to_string(values_.size())
                                + " values; they must match");
  }
  for (std::size_t i = 0; i < values_.size(); ++i) {
    if (!std::isfinite(values_[i])) {
      throw std::invalid_argument("table '" + name_ + "': value " + std::to_string(i)
                                  + " is not finite");
    }
  }
}

double Table1D::first_breakpoint() const {
  if (breakpoints_.empty()) {
    throw std::logic_error("table: first_breakpoint() on an empty table");
  }
  return breakpoints_.front();
}

double Table1D::last_breakpoint() const {
  if (breakpoints_.empty()) {
    throw std::logic_error("table: last_breakpoint() on an empty table");
  }
  return breakpoints_.back();
}

void Table1D::reset_counts() const noexcept {
  out_of_range_ = 0;
  worst_excursion_ = 0.0;
}

double Table1D::at(double argument) const {
  if (breakpoints_.empty()) {
    throw std::logic_error("table: at() on an empty table");
  }
  if (!std::isfinite(argument)) {
    throw std::invalid_argument("table '" + name_ + "': the argument is not finite");
  }
  const double effective = apply_outside_rule(
      name_, "argument", breakpoints_, argument, extrapolation_, out_of_range_, worst_excursion_);
  const Bracket bracket = bracket_of(breakpoints_, effective);
  const double low = values_[bracket.lower];
  if (interpolation_ == TableInterpolation::Nearest_Low) {
    // Exactly at the upper breakpoint the fraction is 1 and the value is the
    // upper one; anywhere inside the interval it is the lower. This keeps the
    // rule "the value at the last breakpoint not after the query" exact.
    return bracket.fraction >= 1.0 ? values_[bracket.lower + 1] : low;
  }
  const double high = values_[bracket.lower + 1];
  // Written as low + f*(high - low), not (1-f)*low + f*high: the first is exact
  // at f = 0 and f = 1 in floating point, and the second is not at f = 1.
  return low + bracket.fraction * (high - low);
}

bool Table1D::breakpoint_near(double argument, double tolerance) const {
  if (breakpoints_.size() < 3 || !std::isfinite(argument) || !(tolerance >= 0.0)) {
    return false;
  }
  for (std::size_t i = 1; i + 1 < breakpoints_.size(); ++i) {
    if (std::fabs(argument - breakpoints_[i]) <= tolerance) {
      return true;
    }
  }
  return false;
}

// --------------------------------------------------------------------------
// Table2D
// --------------------------------------------------------------------------

Table2D::Table2D(std::string name,
                 std::vector<double> rows,
                 std::vector<double> columns,
                 Eigen::MatrixXd values,
                 TableInterpolation interpolation,
                 TableExtrapolation extrapolation)
    : name_(std::move(name)), rows_(std::move(rows)), columns_(std::move(columns)),
      values_(std::move(values)), interpolation_(interpolation), extrapolation_(extrapolation) {
  if (name_.empty()) {
    throw std::invalid_argument("table: a table needs a name");
  }
  validate_axis(name_, "row", rows_);
  validate_axis(name_, "column", columns_);
  if (values_.rows() != static_cast<Eigen::Index>(rows_.size())
      || values_.cols() != static_cast<Eigen::Index>(columns_.size())) {
    throw std::invalid_argument(
        "table '" + name_ + "': values are " + std::to_string(values_.rows()) + "x"
        + std::to_string(values_.cols()) + " but the axes are " + std::to_string(rows_.size())
        + " rows by " + std::to_string(columns_.size()) + " columns");
  }
  if (!values_.allFinite()) {
    throw std::invalid_argument("table '" + name_ + "': a value is not finite");
  }
}

void Table2D::reset_counts() const noexcept {
  out_of_range_ = 0;
  worst_excursion_ = 0.0;
}

double Table2D::at(double row_argument, double column_argument) const {
  if (rows_.empty()) {
    throw std::logic_error("table: at() on an empty table");
  }
  if (!std::isfinite(row_argument) || !std::isfinite(column_argument)) {
    throw std::invalid_argument("table '" + name_ + "': an argument is not finite");
  }
  // Both axes are checked before either is used, so a query outside both
  // reports the row axis first and deterministically, rather than whichever
  // the evaluation order reached.
  const double row_effective = apply_outside_rule(
      name_, "row", rows_, row_argument, extrapolation_, out_of_range_, worst_excursion_);
  const double column_effective = apply_outside_rule(
      name_, "column", columns_, column_argument, extrapolation_, out_of_range_, worst_excursion_);

  const Bracket row = bracket_of(rows_, row_effective);
  const Bracket column = bracket_of(columns_, column_effective);
  const auto i = static_cast<Eigen::Index>(row.lower);
  const auto j = static_cast<Eigen::Index>(column.lower);

  if (interpolation_ == TableInterpolation::Nearest_Low) {
    const Eigen::Index ii = row.fraction >= 1.0 ? i + 1 : i;
    const Eigen::Index jj = column.fraction >= 1.0 ? j + 1 : j;
    return values_(ii, jj);
  }

  // Bilinear, as two exact 1-D interpolations along the columns followed by one
  // along the rows. The order is fixed and written out because a different
  // grouping gives a different last bit.
  const double low = values_(i, j) + column.fraction * (values_(i, j + 1) - values_(i, j));
  const double high =
      values_(i + 1, j) + column.fraction * (values_(i + 1, j + 1) - values_(i + 1, j));
  return low + row.fraction * (high - low);
}

}  // namespace galata::numerics
