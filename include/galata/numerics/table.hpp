// SPDX-License-Identifier: Apache-2.0
//
// Deterministic lookup tables with a DECLARED interpolation rule and a DECLARED
// extrapolation rule.
//
// Reference:
//   C. de Boor, "A Practical Guide to Splines", revised ed., Springer, 2001,
//   chapter 2 — piecewise-linear interpolation and the breakpoint conventions
//   used here.
//   W. H. Press, S. A. Teukolsky, W. T. Vetterling and B. P. Flannery,
//   "Numerical Recipes", 3rd ed., Cambridge, 2007, section 3.6 — bilinear
//   interpolation on a rectangular grid.
//
// WHY BOTH RULES ARE THE CALLER'S. There is no default that is right for every
// table. A fuselage drag coefficient measured between -20 and +20 degrees of
// incidence has no defensible value at 60 degrees, and returning the endpoint
// there is a claim the wind tunnel never made. An engine power schedule, by
// contrast, is genuinely flat above its rating. Guessing produces a number that
// is smooth, plausible and outside the data — which is the specific way a
// component build-up produces a confidently wrong helicopter.
//
// This follows the pattern `galata/sim/schedule.hpp` already sets for input
// histories: the caller declares the hold and the outside rule, and `Refuse` is
// available and meant to be used.
//
// OUT-OF-RANGE QUERIES ARE COUNTED EVEN WHEN THEY ARE ALLOWED. A run that
// clamped four hundred times at the edge of its drag table produced a valid
// trajectory of an aircraft whose aerodynamics were never measured there, and
// the count is the only way the report can say so. `out_of_range_count()` is
// mutable state on an otherwise pure object for exactly that reason, and it is
// the ONLY mutable state: two identical query sequences produce identical
// values and identical counts.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not a spline, and not smooth. Piecewise-linear interpolation has a
//   discontinuous first derivative at every breakpoint. A finite-difference
//   Jacobian taken ACROSS a breakpoint therefore sees a kink, and its Richardson
//   truncation estimate cannot detect it — the estimate assumes an error
//   expansion the kink does not have. `breakpoint_near()` exists so a
//   linearisation can report that it straddled one rather than discover it
//   later; `numerics/jacobian.hpp` already warns about the same hazard for rate
//   limits.
//
// * Not a fitter. A table is the data somebody measured, plus the two rules for
//   reading between and beyond it. It does not smooth, regularise or repair a
//   non-monotonic axis — it refuses one.
//
// * Not sparse, and not large. The intended size is an aerodynamic or engine
//   table: tens to hundreds of breakpoints per axis, held densely. A million-
//   point table will work and will be the wrong data structure.
//
// * Not higher-dimensional. One and two axes only. A three-axis table is a
//   different interpolation problem and pretending otherwise by nesting these
//   would give a result that depends on nesting order.

#ifndef GALATA_NUMERICS_TABLE_HPP
#define GALATA_NUMERICS_TABLE_HPP

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace galata::numerics {

// How to read a value BETWEEN breakpoints.
enum class TableInterpolation {
  // Piecewise linear. Exact at every breakpoint.
  Linear,
  // Piecewise constant, taking the value at the last breakpoint not after the
  // query. A schedule that switches rather than blends.
  Nearest_Low,
};

// How to read a value OUTSIDE the breakpoint span.
enum class TableExtrapolation {
  // Clamp to the nearest edge value. Declared, never assumed, and counted.
  Hold,
  // Refuse. The query left the data, and a silent clamp would make the answer
  // mean something the measurement does not say.
  Refuse,
  // Continue the end segment's slope. Available because it is occasionally
  // correct — a linear-in-Mach correction outside its fitted band — and
  // dangerous everywhere else. Counted like Hold.
  Linear,
};

[[nodiscard]] std::string to_string(TableInterpolation rule);
[[nodiscard]] std::string to_string(TableExtrapolation rule);

// A table of one argument.
class Table1D {
 public:
  Table1D() = default;

  // Refuses: fewer than two breakpoints, a size mismatch, a non-finite entry,
  // and — the one that matters most — breakpoints that do not strictly
  // increase. A repeated breakpoint is ambiguous rather than redundant: it is
  // exactly how a step is written by somebody who has not read this header, and
  // reading it as either value picks a different function.
  Table1D(std::string name,
          std::vector<double> breakpoints,
          std::vector<double> values,
          TableInterpolation interpolation,
          TableExtrapolation extrapolation);

  [[nodiscard]] double at(double argument) const;

  [[nodiscard]] const std::string& name() const noexcept {
    return name_;
  }
  [[nodiscard]] std::size_t size() const noexcept {
    return breakpoints_.size();
  }
  [[nodiscard]] bool empty() const noexcept {
    return breakpoints_.empty();
  }
  [[nodiscard]] double first_breakpoint() const;
  [[nodiscard]] double last_breakpoint() const;
  [[nodiscard]] TableInterpolation interpolation() const noexcept {
    return interpolation_;
  }
  [[nodiscard]] TableExtrapolation extrapolation() const noexcept {
    return extrapolation_;
  }

  // How many queries fell outside the span. Reset by `reset_counts()`.
  [[nodiscard]] std::uint64_t out_of_range_count() const noexcept {
    return out_of_range_;
  }
  // The furthest any query strayed, in the argument's own units. Zero when none did.
  [[nodiscard]] double worst_excursion() const noexcept {
    return worst_excursion_;
  }
  void reset_counts() const noexcept;

  // True when `argument` is within `tolerance` of an interior breakpoint, so a
  // caller taking a finite difference can report that it straddled a kink.
  [[nodiscard]] bool breakpoint_near(double argument, double tolerance) const;

 private:
  std::string name_;
  std::vector<double> breakpoints_;
  std::vector<double> values_;
  TableInterpolation interpolation_ = TableInterpolation::Linear;
  TableExtrapolation extrapolation_ = TableExtrapolation::Refuse;
  mutable std::uint64_t out_of_range_ = 0;
  mutable double worst_excursion_ = 0.0;
};

// A table of two arguments on a rectangular grid.
//
// `values(i, j)` is the value at `rows[i]`, `columns[j]`. Row-major in the
// mathematical sense, not the storage sense: the storage is Eigen's and the
// accessor is explicit so no caller has to know which.
class Table2D {
 public:
  Table2D() = default;

  // Refuses the same things Table1D refuses, on both axes, plus a value matrix
  // whose shape does not match the two breakpoint vectors.
  Table2D(std::string name,
          std::vector<double> rows,
          std::vector<double> columns,
          Eigen::MatrixXd values,
          TableInterpolation interpolation,
          TableExtrapolation extrapolation);

  [[nodiscard]] double at(double row_argument, double column_argument) const;

  [[nodiscard]] const std::string& name() const noexcept {
    return name_;
  }
  [[nodiscard]] bool empty() const noexcept {
    return rows_.empty();
  }
  [[nodiscard]] std::size_t row_count() const noexcept {
    return rows_.size();
  }
  [[nodiscard]] std::size_t column_count() const noexcept {
    return columns_.size();
  }
  [[nodiscard]] TableInterpolation interpolation() const noexcept {
    return interpolation_;
  }
  [[nodiscard]] TableExtrapolation extrapolation() const noexcept {
    return extrapolation_;
  }
  [[nodiscard]] std::uint64_t out_of_range_count() const noexcept {
    return out_of_range_;
  }
  [[nodiscard]] double worst_excursion() const noexcept {
    return worst_excursion_;
  }
  void reset_counts() const noexcept;

 private:
  std::string name_;
  std::vector<double> rows_;
  std::vector<double> columns_;
  Eigen::MatrixXd values_;
  TableInterpolation interpolation_ = TableInterpolation::Linear;
  TableExtrapolation extrapolation_ = TableExtrapolation::Refuse;
  mutable std::uint64_t out_of_range_ = 0;
  mutable double worst_excursion_ = 0.0;
};

}  // namespace galata::numerics

#endif  // GALATA_NUMERICS_TABLE_HPP
