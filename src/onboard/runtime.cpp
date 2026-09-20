// SPDX-License-Identifier: Apache-2.0
#include "galata/onboard/runtime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace galata::onboard {

std::string to_string(RuntimeState state) {
  switch (state) {
    case RuntimeState::Stopped:
      return "stopped";
    case RuntimeState::Ready:
      return "ready";
    case RuntimeState::Armed:
      return "armed";
    case RuntimeState::Faulted:
      return "faulted";
  }
  return "unknown";
}

Runtime::Runtime(hardware::Transport& transport,
                 hardware::ArmingInterlock& interlock,
                 RuntimeConfiguration configuration)
    : interlock_(interlock), guarded_transport_(transport, interlock),
      configuration_(std::move(configuration)) {
  hardware::validate_interface(configuration_.interface);
  if (!(configuration_.max_controller_time_s > 0.0)
      || !std::isfinite(configuration_.max_controller_time_s)) {
    throw std::invalid_argument("onboard: max_controller_time_s must be positive and finite");
  }
  if (!(configuration_.watchdog_timeout_s >= configuration_.interface.sample_period_s)
      || !std::isfinite(configuration_.watchdog_timeout_s)) {
    throw std::invalid_argument(
        "onboard: watchdog_timeout_s must be finite and at least one sample period");
  }
  if (configuration_.max_controller_time_s > configuration_.watchdog_timeout_s) {
    throw std::invalid_argument(
        "onboard: controller execution budget cannot exceed the watchdog timeout");
  }
}

void Runtime::connect() {
  if (state_ == RuntimeState::Armed) {
    throw std::logic_error("onboard: cannot reconnect an armed runtime");
  }
  fault_reason_.clear();
  completed_cycles_ = 0;
  metrics_ = {};
  try {
    guarded_transport_.connect(configuration_.interface);
    if (guarded_transport_.state() != hardware::LinkState::Ready) {
      throw std::runtime_error("onboard: transport did not become ready");
    }
    state_ = RuntimeState::Ready;
  } catch (...) {
    state_ = RuntimeState::Faulted;
    interlock_.disarm();
    throw;
  }
}

void Runtime::arm(const std::string& operator_id, const std::string& confirmation) {
  if (state_ != RuntimeState::Ready) {
    throw std::logic_error("onboard: runtime must be connected and ready before arming");
  }
  if (guarded_transport_.state() != hardware::LinkState::Ready) {
    fault("transport stopped being ready before arming");
    throw std::runtime_error("onboard: transport stopped being ready before arming");
  }
  interlock_.arm(operator_id, confirmation);
  state_ = RuntimeState::Armed;
}

void Runtime::disarm() noexcept {
  interlock_.disarm();
  if (state_ == RuntimeState::Armed) {
    state_ = RuntimeState::Ready;
  }
}

void Runtime::stop() noexcept {
  interlock_.disarm();
  guarded_transport_.disconnect();
  state_ = RuntimeState::Stopped;
}

bool Runtime::step(const Controller& controller) {
  if (state_ != RuntimeState::Armed) {
    throw std::logic_error("onboard: runtime step requires an armed runtime");
  }
  if (!controller) {
    fault("controller callback is empty");
    throw std::invalid_argument("onboard: controller callback is empty");
  }

  hardware::Frame sensor;
  try {
    const auto cycle_started = std::chrono::steady_clock::now();
    if (!guarded_transport_.receive(sensor)) {
      fault("sensor receive timed out before a complete cycle");
      return false;
    }
    const auto controller_started = std::chrono::steady_clock::now();
    hardware::Frame actuator = controller(sensor);
    const std::chrono::duration<double> controller_elapsed =
        std::chrono::steady_clock::now() - controller_started;
    metrics_.controller_worst_case_s =
        std::max(metrics_.controller_worst_case_s, controller_elapsed.count());
    const std::chrono::duration<double> observed_cycle_elapsed =
        std::chrono::steady_clock::now() - cycle_started;
    metrics_.cycle_worst_case_s =
        std::max(metrics_.cycle_worst_case_s, observed_cycle_elapsed.count());
    ++metrics_.observed_cycles;
    if (controller_elapsed.count() > configuration_.max_controller_time_s) {
      fault("controller callback exceeded its declared execution budget");
      throw std::runtime_error(
          "onboard: controller callback exceeded its declared execution budget");
    }
    const std::chrono::duration<double> cycle_elapsed =
        std::chrono::steady_clock::now() - cycle_started;
    if (cycle_elapsed.count() > configuration_.watchdog_timeout_s) {
      fault("one onboard cycle exceeded its watchdog timeout");
      throw std::runtime_error("onboard: one cycle exceeded its watchdog timeout");
    }
    hardware::validate_frame(actuator, configuration_.interface.actuator_channels, "actuator");
    if (actuator.sequence != sensor.sequence) {
      fault("controller output sequence does not match its sensor frame");
      throw std::invalid_argument(
          "onboard: controller output sequence does not match its sensor frame");
    }
    if (actuator.timestamp_s != sensor.timestamp_s) {
      fault("controller output timestamp does not match its sensor frame");
      throw std::invalid_argument(
          "onboard: controller output timestamp does not match its sensor frame");
    }
    guarded_transport_.send(actuator);
    ++completed_cycles_;
    return true;
  } catch (...) {
    if (state_ != RuntimeState::Faulted) {
      fault("controller or transport failure during cycle");
    }
    throw;
  }
}

void Runtime::fault(std::string reason) noexcept {
  interlock_.disarm();
  guarded_transport_.disconnect();
  state_ = RuntimeState::Faulted;
  fault_reason_ = std::move(reason);
}

}  // namespace galata::onboard
