// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the declared input history in include/galata/sim/schedule.hpp.

#include "galata/sim/schedule.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace galata::sim {
namespace {

void require_finite(double value, const char* what) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string("input schedule: ") + what + " is not finite");
  }
}

}  // namespace

InputSchedule::InputSchedule(std::vector<double> times_s,
                             std::vector<Eigen::VectorXd> values,
                             HoldPolicy hold,
                             Extrapolation outside)
    : times_s_(std::move(times_s)), values_(std::move(values)), hold_(hold), outside_(outside) {
  if (times_s_.empty()) {
    throw std::invalid_argument("input schedule: at least one sample is required");
  }
  if (times_s_.size() != values_.size()) {
    std::ostringstream message;
    message << "input schedule: " << times_s_.size() << " time(s) and " << values_.size()
            << " value(s); every sample needs both";
    throw std::invalid_argument(message.str());
  }
  width_ = static_cast<int>(values_.front().size());
  if (width_ < 1) {
    throw std::invalid_argument("input schedule: a sample must carry at least one channel");
  }
  for (std::size_t i = 0; i < times_s_.size(); ++i) {
    require_finite(times_s_[i], "a sample time");
    if (static_cast<int>(values_[i].size()) != width_) {
      std::ostringstream message;
      message << "input schedule: sample " << i << " carries " << values_[i].size()
              << " channel(s) but the first carries " << width_
              << "; a history whose width changes is not a history of one signal";
      throw std::invalid_argument(message.str());
    }
    if (!values_[i].allFinite()) {
      std::ostringstream message;
      message << "input schedule: sample " << i << " carries a non-finite value";
      throw std::invalid_argument(message.str());
    }
    // STRICTLY increasing. A repeated timestamp is how a step gets written by
    // someone who has not read the header, and reading it as either value picks
    // a different trajectory without saying so.
    if (i > 0 && !(times_s_[i] > times_s_[i - 1])) {
      std::ostringstream message;
      message << "input schedule: sample times must strictly increase; sample " << i << " at "
              << times_s_[i] << " s does not follow sample " << (i - 1) << " at " << times_s_[i - 1]
              << " s. A repeated time is ambiguous rather than redundant — "
              << "write a step as two samples at distinct times under a zero-order hold";
      throw std::invalid_argument(message.str());
    }
  }
  if (hold_ == HoldPolicy::ZeroOrder) {
    for (std::size_t i = 1; i < times_s_.size(); ++i) {
      if (values_[i] != values_[i - 1]) {
        discontinuities_.push_back(times_s_[i]);
      }
    }
  }
}

double InputSchedule::first_time_s() const {
  if (times_s_.empty()) {
    throw std::logic_error("input schedule: empty schedule has no first time");
  }
  return times_s_.front();
}

double InputSchedule::last_time_s() const {
  if (times_s_.empty()) {
    throw std::logic_error("input schedule: empty schedule has no last time");
  }
  return times_s_.back();
}

std::size_t InputSchedule::segment_for(double time_s) const {
  // Index of the last sample whose time is <= time_s. Callers have already
  // resolved the outside-the-span case.
  const auto upper = std::upper_bound(times_s_.begin(), times_s_.end(), time_s);
  const auto index = static_cast<std::size_t>(upper - times_s_.begin());
  return index == 0 ? 0 : index - 1;
}

Eigen::VectorXd InputSchedule::at(double time_s) const {
  require_finite(time_s, "the evaluation time");
  if (times_s_.empty()) {
    throw std::logic_error("input schedule: empty schedule has no value");
  }
  if (time_s < times_s_.front() || time_s > times_s_.back()) {
    if (outside_ == Extrapolation::Refuse) {
      std::ostringstream message;
      message << "input schedule: asked for t = " << time_s << " s, outside the declared span ["
              << times_s_.front() << ", " << times_s_.back()
              << "] s. Extend the history, shorten the horizon, or declare "
                 "`extrapolation: hold` — a silent clamp would make the tail of the run mean "
                 "something this study does not say";
      throw std::invalid_argument(message.str());
    }
    return time_s < times_s_.front() ? values_.front() : values_.back();
  }
  const std::size_t index = segment_for(time_s);
  if (hold_ == HoldPolicy::ZeroOrder || index + 1 >= times_s_.size()) {
    return values_[index];
  }
  const double span = times_s_[index + 1] - times_s_[index];
  const double fraction = (time_s - times_s_[index]) / span;
  return values_[index] + fraction * (values_[index + 1] - values_[index]);
}

Eigen::VectorXd InputSchedule::rate_at(double time_s) const {
  require_finite(time_s, "the evaluation time");
  if (times_s_.empty()) {
    throw std::logic_error("input schedule: empty schedule has no rate");
  }
  // Held outside the span, and constant within a zero-order segment: both are
  // genuinely zero rate. At a ZeroOrder jump the derivative is not a function
  // and the jump is the caller's event to apply, not a rate to integrate.
  if (hold_ == HoldPolicy::ZeroOrder || time_s < times_s_.front() || time_s >= times_s_.back()) {
    return Eigen::VectorXd::Zero(width_);
  }
  const std::size_t index = segment_for(time_s);
  if (index + 1 >= times_s_.size()) {
    return Eigen::VectorXd::Zero(width_);
  }
  const double span = times_s_[index + 1] - times_s_[index];
  return (values_[index + 1] - values_[index]) / span;
}

Eigen::VectorXd InputSchedule::jump_at(double time_s) const {
  const auto found = std::find(discontinuities_.begin(), discontinuities_.end(), time_s);
  if (found == discontinuities_.end()) {
    std::ostringstream message;
    message << "input schedule: " << time_s << " s is not one of this history's discontinuities";
    throw std::invalid_argument(message.str());
  }
  const auto at_time = std::lower_bound(times_s_.begin(), times_s_.end(), time_s);
  const auto index = static_cast<std::size_t>(at_time - times_s_.begin());
  if (index == 0) {
    throw std::logic_error("input schedule: a discontinuity cannot be the first sample");
  }
  return values_[index] - values_[index - 1];
}

}  // namespace galata::sim
