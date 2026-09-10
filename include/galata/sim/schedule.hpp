// SPDX-License-Identifier: Apache-2.0
//
// A declared time history for a simulation input.
//
// WHAT THIS IS. A finite list of (time, value) samples plus a stated rule for
// reading a value between them and a stated rule for reading one outside them.
// Both rules are the CALLER's, declared in the study, because there is no
// default that is right for every channel: a rotor command that steps is held,
// a wind that builds is interpolated, and guessing which one a caller meant
// produces a plausible trajectory that answers a question nobody asked.
//
// WHAT THIS IS NOT. Not a signal generator, not a spline, and not a resampler.
// It does not smooth, it does not filter, and it does not repair a history that
// arrives out of order — it refuses one.
//
// ORDER LOSS AT A DISCONTINUITY, stated because it is easy to lose silently.
// RK4 evaluates its derivative four times inside one step. If a zero-order-hold
// jump falls strictly inside a step, two of those evaluations see the old value
// and two the new, and the method is no longer fourth order across that step —
// it is first order there, and the Richardson estimate cannot see it for the
// same reason it cannot see a kink. The result is still DETERMINISTIC, which is
// what ADR-0004 requires, and it is still wrong to more than the step's usual
// error. `discontinuities()` exposes the jump times so a caller can align the
// step to them, and `sim.plant` reports how many did not align.
#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

namespace galata::sim {

// How to read a value BETWEEN two samples.
enum class HoldPolicy {
  // Piecewise constant, value(t) = value at the last sample not after t. This
  // is what a commanded input does when something writes it at a fixed rate,
  // and it is what PX4 logs record.
  ZeroOrder,
  // Piecewise linear between samples. A ramp is exactly representable; a step
  // is not, and asking for one here gives a ramp over the whole segment.
  Linear,
};

// How to read a value OUTSIDE the sampled span.
enum class Extrapolation {
  // Clamp to the first or last sample. Declared, never assumed.
  Hold,
  // Refuse. The horizon asked for more history than the caller supplied, and a
  // silent clamp would make the tail of the run mean something the study does
  // not say.
  Refuse,
};

class InputSchedule {
 public:
  InputSchedule() = default;

  // Refuses: fewer than one sample, mismatched widths, a non-finite time or
  // value, and — the one that matters most — times that do not strictly
  // increase. A repeated timestamp is ambiguous rather than redundant, because
  // it is exactly how a step is written by someone who has not read this
  // header, and reading it as either value silently picks a different
  // trajectory.
  InputSchedule(std::vector<double> times_s,
                std::vector<Eigen::VectorXd> values,
                HoldPolicy hold,
                Extrapolation outside);

  [[nodiscard]] bool empty() const noexcept {
    return times_s_.empty();
  }

  [[nodiscard]] int width() const noexcept {
    return width_;
  }

  [[nodiscard]] HoldPolicy hold() const noexcept {
    return hold_;
  }

  [[nodiscard]] double first_time_s() const;
  [[nodiscard]] double last_time_s() const;

  // The value at t.
  [[nodiscard]] Eigen::VectorXd at(double time_s) const;

  // The time derivative at t. Zero inside a zero-order-hold segment and at the
  // ends; the segment slope under Linear. At a ZeroOrder jump the true
  // derivative is not a function, and this returns zero rather than a large
  // number: the jump is handled as an EVENT by the caller, not as a rate. See
  // `discontinuities()`.
  [[nodiscard]] Eigen::VectorXd rate_at(double time_s) const;

  // Sample times at which a zero-order-hold value actually changes. Empty for
  // Linear, and empty for a ZeroOrder history whose value never moves.
  [[nodiscard]] const std::vector<double>& discontinuities() const noexcept {
    return discontinuities_;
  }

  // The jump applied at a discontinuity time: value just after minus value just
  // before. Throws if `time_s` is not one of `discontinuities()`.
  [[nodiscard]] Eigen::VectorXd jump_at(double time_s) const;

 private:
  [[nodiscard]] std::size_t segment_for(double time_s) const;

  std::vector<double> times_s_;
  std::vector<Eigen::VectorXd> values_;
  std::vector<double> discontinuities_;
  HoldPolicy hold_ = HoldPolicy::ZeroOrder;
  Extrapolation outside_ = Extrapolation::Refuse;
  int width_ = 0;
};

}  // namespace galata::sim
