// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "galata/hardware/interface.hpp"

#include <cstdint>
#include <functional>
#include <limits>
#include <string>

namespace galata::onboard {

enum class RuntimeState {
  Stopped,
  Ready,
  Armed,
  Faulted,
};

[[nodiscard]] std::string to_string(RuntimeState state);

struct RuntimeConfiguration {
  hardware::InterfaceSpec interface;
  // The supervisor detects a controller callback that exceeds this budget
  // after it returns. It cannot preempt a wedged callback; a target OS/watchdog
  // remains responsible for hard real-time enforcement.
  double max_controller_time_s = std::numeric_limits<double>::quiet_NaN();
  // Maximum wall-clock time for one receive/compute cycle.  This is a
  // software observation point; it cannot preempt a blocked adapter or
  // callback, so a target watchdog remains mandatory for hard enforcement.
  double watchdog_timeout_s = std::numeric_limits<double>::quiet_NaN();
};

// Timing observed by this process while a cycle is supervised. These values
// are useful for SIL/bench diagnostics and evidence reports; they are not a
// hard-real-time or target-timing claim because this class cannot preempt a
// callback, transport or operating-system scheduler.
struct RuntimeMetrics {
  std::uint64_t observed_cycles = 0;
  double controller_worst_case_s = 0.0;
  double cycle_worst_case_s = 0.0;
};

// The controller is deliberately a narrow synchronous callback. A generated
// or hand-written target controller can be adapted to it, while the runtime
// remains responsible for channel widths, frame identity and the output gate.
using Controller = std::function<hardware::Frame(const hardware::Frame& sensor)>;

// One-cycle supervisor for SIL, HIL and a reviewed target adapter. It enforces
// the safety ordering around a controller:
//
//   receive one sensor frame -> compute one actuator frame -> send it
//
// Missing input, a controller exception, an invalid output, a mismatched
// sequence/timestamp or a transport exception disarms and disconnects before
// the error is reported. This is a fail-closed execution boundary, not a
// target-specific flight-computer implementation or a certification claim.
class Runtime final {
 public:
  Runtime(hardware::Transport& transport,
          hardware::ArmingInterlock& interlock,
          RuntimeConfiguration configuration);

  Runtime(const Runtime&) = delete;
  Runtime& operator=(const Runtime&) = delete;

  void connect();
  void arm(const std::string& operator_id, const std::string& confirmation);
  void disarm() noexcept;
  void stop() noexcept;

  // Returns true when one complete cycle was published. A bounded receive
  // timeout returns false and faults the runtime. Other failures fault and are
  // rethrown so the caller cannot mistake a refused cycle for a valid result.
  [[nodiscard]] bool step(const Controller& controller);

  [[nodiscard]] RuntimeState state() const noexcept {
    return state_;
  }

  [[nodiscard]] std::uint64_t completed_cycles() const noexcept {
    return completed_cycles_;
  }

  [[nodiscard]] const std::string& fault_reason() const noexcept {
    return fault_reason_;
  }

  [[nodiscard]] const RuntimeMetrics& metrics() const noexcept {
    return metrics_;
  }

 private:
  void fault(std::string reason) noexcept;

  hardware::ArmingInterlock& interlock_;
  hardware::GuardedTransport guarded_transport_;
  RuntimeConfiguration configuration_;
  RuntimeState state_ = RuntimeState::Stopped;
  std::uint64_t completed_cycles_ = 0;
  RuntimeMetrics metrics_;
  std::string fault_reason_;
};

}  // namespace galata::onboard
